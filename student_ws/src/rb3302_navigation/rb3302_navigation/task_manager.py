"""Safe non-blocking executor for the ordered RB3302 landmark task."""
import enum
import math
import yaml
import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from .landmark_converter import LandmarkConverter
from .task_validation import load_task, validate_landmark_config


class TaskState(enum.Enum):
    WAITING_FOR_NAV2 = enum.auto(); INITIALIZING_LOCALIZATION = enum.auto()
    WAITING_FOR_LOCALIZATION = enum.auto(); NAVIGATING = enum.auto()
    CANCELLING = enum.auto(); DWELLING = enum.auto(); COMPLETED = enum.auto(); FAILED = enum.auto()


class TaskManager(Node):
    """One-shot client of Nav2. FAILED tasks cannot resubmit goals."""
    def __init__(self):
        super().__init__('rb3302_task_manager')
        defaults = {'task_file': '', 'landmarks_file': '', 'dwell_seconds': 3.0,
          'goal_orientation': 'start_bearing', 'goal_acceptance_timeout_seconds': 5.0,
          'navigation_timeout_seconds': 40.0, 'cancellation_timeout_seconds': 5.0,
          'localization_timeout_seconds': 25.0, 'overall_timeout_seconds': 180.0,
          'localization_xy_variance_limit': 0.25, 'localization_yaw_variance_limit': 0.8,
          'stop_linear_tolerance': 0.02, 'stop_angular_tolerance': 0.05, 'goal_xy_tolerance': 0.18}
        for name, value in defaults.items(): self.declare_parameter(name, value)
        self.stop_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.initialpose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self._amcl_cb, 10)
        self.create_subscription(Odometry, '/odom', self._odom_cb, 10)
        self.tf_buffer = Buffer(); self.tf_listener = TransformListener(self.tf_buffer, self)
        self.state = TaskState.WAITING_FOR_NAV2; self.goal_index = 0; self.goal_handle = None; self.goal_pending = False
        self.current_goal_label = self.goal_target = self._last_amcl_pose = self._last_odom = None
        self._state_deadline = self._overall_deadline = self._dwell_deadline = None; self._last_initial_pose = 0.0
        try:
            self.task = load_task(self._param('task_file'))
            with open(self._param('landmarks_file'), encoding='utf-8') as stream: config = validate_landmark_config(yaml.safe_load(stream))
            self.converter = LandmarkConverter(config)
        except Exception as exc:
            self.task = None; self.get_logger().fatal(f'Task configuration rejected: {exc}'); self._fail('Invalid task or landmark calibration')
        self.create_timer(0.1, self.tick)

    def _now(self): return self.get_clock().now().nanoseconds / 1e9
    def _param(self, name): return self.get_parameter(name).value
    def _set_state(self, state, timeout=None): self.state = state; self._state_deadline = None if timeout is None else self._now() + timeout
    def stop(self): self.stop_pub.publish(Twist())
    def _amcl_cb(self, msg): self._last_amcl_pose = msg
    def _odom_cb(self, msg): self._last_odom = msg

    def _publish_initial_pose(self):
        start = self.task['start']; x, y = self.converter.coordinate(start['square']); yaw = self.converter.bearing_yaw(start['bearing'])
        msg = PoseWithCovarianceStamped(); msg.header.frame_id = 'map'; msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x, msg.pose.pose.position.y = x, y; msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        msg.pose.covariance[0] = msg.pose.covariance[7] = 0.09; msg.pose.covariance[35] = math.radians(45) ** 2
        self.initialpose_pub.publish(msg); self._last_initial_pose = self._now()

    def _localization_ready(self):
        msg = self._last_amcl_pose
        if msg is None or msg.header.frame_id != 'map': return False
        c = msg.pose.covariance
        if c[0] > self._param('localization_xy_variance_limit') or c[7] > self._param('localization_xy_variance_limit') or c[35] > self._param('localization_yaw_variance_limit'): return False
        try: return self.tf_buffer.can_transform('map', 'base_link', rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.0))
        except Exception: return False

    def _robot_stopped(self):
        if self._last_odom is None: return False
        t = self._last_odom.twist.twist
        return abs(t.linear.x) <= self._param('stop_linear_tolerance') and abs(t.angular.z) <= self._param('stop_angular_tolerance')

    def _fail(self, reason):
        if self.state in (TaskState.FAILED, TaskState.COMPLETED): return
        self.get_logger().error(reason); self.stop()
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async(); self._set_state(TaskState.CANCELLING, self._param('cancellation_timeout_seconds'))
        else: self._set_state(TaskState.FAILED)

    def _send_next_goal(self):
        if self.goal_index >= len(self.task['targets']): self.stop(); self._set_state(TaskState.COMPLETED); self.get_logger().info('Four-target task complete; robot stopped.'); return
        self.current_goal_label = self.task['targets'][self.goal_index]; x, y = self.converter.coordinate(self.current_goal_label); self.goal_target = (x, y)
        yaw = self.converter.bearing_yaw(self.task['start']['bearing']) if self._param('goal_orientation') == 'start_bearing' else 0.0
        goal = NavigateToPose.Goal(); goal.pose = PoseStamped(); goal.pose.header.frame_id = 'map'; goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x, goal.pose.pose.position.y = x, y; goal.pose.pose.orientation.z, goal.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        self.goal_pending = True; self._set_state(TaskState.NAVIGATING, self._param('goal_acceptance_timeout_seconds')); self.client.send_goal_async(goal).add_done_callback(self._goal_response)
        self.get_logger().info(f'Sending target {self.goal_index + 1}/4: {self.current_goal_label} ({x:.3f}, {y:.3f})')

    def _goal_response(self, future):
        if self.state is not TaskState.NAVIGATING or not self.goal_pending: return
        self.goal_pending = False
        try: self.goal_handle = future.result()
        except Exception as exc: self._fail(f'Goal request failed: {exc}'); return
        if self.goal_handle is None or not self.goal_handle.accepted: self._fail('Nav2 rejected goal'); return
        self._state_deadline = self._now() + self._param('navigation_timeout_seconds'); self.goal_handle.get_result_async().add_done_callback(self._goal_result)

    def _goal_result(self, future):
        if self.state not in (TaskState.NAVIGATING, TaskState.CANCELLING): return
        self.goal_handle = None
        try: status = future.result().status
        except Exception as exc: self._fail(f'Goal result failed: {exc}'); return
        if self.state is TaskState.CANCELLING: return
        if status != GoalStatus.STATUS_SUCCEEDED: self._fail(f'Navigation failed with action status {status}'); return
        p = self._last_amcl_pose.pose.pose.position if self._last_amcl_pose else None
        if p is None or math.hypot(p.x - self.goal_target[0], p.y - self.goal_target[1]) > self._param('goal_xy_tolerance'): self._fail('Nav2 reported success outside target tolerance'); return
        self.stop(); self._dwell_deadline = None; self._set_state(TaskState.DWELLING)

    def tick(self):
        now = self._now()
        if self.state in (TaskState.FAILED, TaskState.COMPLETED, TaskState.CANCELLING, TaskState.DWELLING): self.stop()
        if self.task is None: return
        if self._overall_deadline is not None and now >= self._overall_deadline and self.state not in (TaskState.COMPLETED, TaskState.FAILED): self._fail('Overall task limit expired'); return
        if self.state is TaskState.WAITING_FOR_NAV2:
            if self.client.wait_for_server(timeout_sec=0.0): self._overall_deadline = now + self._param('overall_timeout_seconds'); self._publish_initial_pose(); self._set_state(TaskState.INITIALIZING_LOCALIZATION, self._param('localization_timeout_seconds'))
        elif self.state in (TaskState.INITIALIZING_LOCALIZATION, TaskState.WAITING_FOR_LOCALIZATION):
            if self._localization_ready(): self._send_next_goal(); return
            if now >= self._state_deadline: self._fail('AMCL did not converge within localization timeout'); return
            if now - self._last_initial_pose >= 1.0: self._publish_initial_pose()
            self.state = TaskState.WAITING_FOR_LOCALIZATION
        elif self.state is TaskState.NAVIGATING and now >= self._state_deadline: self._fail('Navigation goal acceptance or execution timeout')
        elif self.state is TaskState.CANCELLING and now >= self._state_deadline: self.get_logger().error('Cancellation timeout; command stop remains active'); self._set_state(TaskState.FAILED)
        elif self.state is TaskState.DWELLING:
            if not self._robot_stopped(): return
            if self._dwell_deadline is None: self._dwell_deadline = now + self._param('dwell_seconds')
            elif now >= self._dwell_deadline: self.goal_index += 1; self._send_next_goal()


def main():
    rclpy.init(); node = TaskManager()
    try: rclpy.spin(node)
    finally: node.stop(); node.destroy_node(); rclpy.shutdown()
