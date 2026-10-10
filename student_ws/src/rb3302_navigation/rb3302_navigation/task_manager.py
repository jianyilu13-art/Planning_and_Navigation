"""Non-blocking sequential NavigateToPose client with safe failure behaviour."""
import math
import os
import yaml
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav2_msgs.action import NavigateToPose
from .landmark_converter import LandmarkConverter
from .task_validation import load_task


class TaskManager(Node):
    def __init__(self):
        super().__init__('rb3302_task_manager')
        for name, default in [('task_file', ''), ('landmarks_file', ''), ('map_ready', False),
                              ('dwell_seconds', 3.0), ('goal_timeout_seconds', 40.0)]: self.declare_parameter(name, default)
        self.stop_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.index = 0; self.goal_handle = None; self.deadline = None; self.dwell_until = None; self.initialized = False
        try:
            self.task = load_task(self.get_parameter('task_file').value)
            with open(self.get_parameter('landmarks_file').value, encoding='utf-8') as stream: config = yaml.safe_load(stream)
            if not self.get_parameter('map_ready').value: raise RuntimeError('map_ready is false: configure a measured map YAML before navigation')
            if not config.get('calibrated', False): raise RuntimeError('landmarks.yaml is not calibrated; navigation is locked for safety')
            self.converter = LandmarkConverter(config)
        except Exception as exc:
            self.get_logger().fatal(f'Task configuration rejected: {exc}'); self.stop(); self.create_timer(1.0, self.stop); return
        self.create_timer(0.1, self.tick)

    def stop(self): self.stop_pub.publish(Twist())

    def initial_pose(self):
        start = self.task['start']; x, y = self.converter.coordinate(start['square']); yaw = self.converter.bearing_yaw(start['bearing'])
        msg = PoseWithCovarianceStamped(); msg.header.frame_id = 'map'; msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = x; msg.pose.pose.position.y = y; msg.pose.pose.orientation.z = math.sin(yaw/2); msg.pose.pose.orientation.w = math.cos(yaw/2)
        msg.pose.covariance[0] = msg.pose.covariance[7] = 0.09; msg.pose.covariance[35] = math.radians(45) ** 2
        self.initialpose.publish(msg)

    def tick(self):
        now = self.get_clock().now()
        if not self.initialized:
            if not self.client.wait_for_server(timeout_sec=0.0): return
            self.initialpose = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
            self.initial_pose(); self.initialized = True; self.dwell_until = now.nanoseconds + 2_000_000_000
            self.get_logger().info('Nav2 available; initial pose sent, allowing AMCL two seconds to incorporate it.'); return
        if self.dwell_until:
            if now.nanoseconds < self.dwell_until: self.stop(); return
            self.dwell_until = None; self.send_next(); return
        if self.goal_handle and now.nanoseconds > self.deadline:
            self.get_logger().error('Navigation goal timed out; cancelling and stopping.'); self.goal_handle.cancel_goal_async(); self.stop(); self.goal_handle = None; return

    def send_next(self):
        if self.index >= len(self.task['targets']): self.stop(); self.get_logger().info('Four-target task complete; robot stopped.'); return
        label = self.task['targets'][self.index]; x, y = self.converter.coordinate(label)
        goal = NavigateToPose.Goal(); goal.pose = PoseStamped(); goal.pose.header.frame_id = 'map'; goal.pose.header.stamp = self.get_clock().now().to_msg(); goal.pose.pose.position.x = x; goal.pose.pose.position.y = y; goal.pose.pose.orientation.w = 1.0
        future = self.client.send_goal_async(goal); future.add_done_callback(self.goal_response)
        self.get_logger().info(f'Sending target {self.index + 1}/4: {label} ({x:.3f}, {y:.3f})')

    def goal_response(self, future):
        self.goal_handle = future.result()
        if not self.goal_handle.accepted: self.get_logger().error('Nav2 rejected goal; stopping.'); self.stop(); self.goal_handle = None; return
        self.deadline = self.get_clock().now().nanoseconds + int(self.get_parameter('goal_timeout_seconds').value * 1e9)
        self.goal_handle.get_result_async().add_done_callback(self.goal_result)

    def goal_result(self, future):
        result = future.result(); self.goal_handle = None; self.stop()
        if result.status != 4: self.get_logger().error(f'Navigation failed (action status {result.status}); task halted.'); return
        self.index += 1; self.dwell_until = self.get_clock().now().nanoseconds + int(self.get_parameter('dwell_seconds').value * 1e9)
        self.get_logger().info(f'Arrived; holding stopped for {self.get_parameter("dwell_seconds").value:.1f} seconds.')


def main():
    rclpy.init(); node = TaskManager()
    try: rclpy.spin(node)
    finally: node.stop(); node.destroy_node(); rclpy.shutdown()
