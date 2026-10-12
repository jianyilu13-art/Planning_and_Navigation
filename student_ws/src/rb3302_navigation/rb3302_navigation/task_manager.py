"""
RB3302 Navigation Task Manager

Purpose:
    Execute a navigation task consisting of four ordered landmarks.

Responsibilities:
    1. Load task.yaml and landmarks.yaml.
    2. Initialize robot localization using AMCL.
    3. Send navigation goals to the official Nav2 stack.
    4. Receive navigation results.
    5. Stop at each landmark for 3 seconds.
    6. Continue to the next landmark.
    7. Finish after four landmarks or handle failure.

IMPORTANT:
    This file does NOT implement A* or RPP.
    Actual navigation is handled by the official planning_rpp package.
"""

# ============================================================
# 1. IMPORTS
# ============================================================

import enum
import math
import yaml
import rclpy

# ROS 2 action status constants
from action_msgs.msg import GoalStatus

# ROS 2 message types
from geometry_msgs.msg import (
    PoseStamped,
    PoseWithCovarianceStamped,
    Twist,
)

# Nav2 navigation action interface
from nav2_msgs.action import NavigateToPose

# Robot odometry message
from nav_msgs.msg import Odometry

# ROS 2 action client and node
from rclpy.action import ActionClient
from rclpy.node import Node

# ROS 2 coordinate transformation system
from tf2_ros import Buffer, TransformListener

# Our own modules
from .landmark_converter import LandmarkConverter
from .task_validation import load_task, validate_landmark_config


# ============================================================
# 2. TASK STATES
# ============================================================

class TaskState(enum.Enum):
    """
    States of OUR task manager.

    These are NOT Nav2 internal states.
    """

    WAITING_FOR_NAV2 = enum.auto()

    INITIALIZING_LOCALIZATION = enum.auto()

    WAITING_FOR_LOCALIZATION = enum.auto()

    NAVIGATING = enum.auto()

    CANCELLING = enum.auto()

    DWELLING = enum.auto()

    COMPLETED = enum.auto()

    FAILED = enum.auto()


# ============================================================
# 3. TASK MANAGER NODE
# ============================================================

class TaskManager(Node):
    """
    ROS 2 node responsible for the four-landmark mission.

    Communication:

        TaskManager
            |
            | NavigateToPose action
            v
        Nav2 BT Navigator
            |
            v
        Official navigation stack
            |
            v
        Robot

    The task manager sends destinations.
    Nav2 performs the actual navigation.
    """

    # ========================================================
    # 3.1 INITIALIZATION
    # ========================================================

    def __init__(self):

        # Create ROS 2 node
        super().__init__('rb3302_task_manager')

        # ----------------------------------------------------
        # A. Declare ROS parameters
        # ----------------------------------------------------

        defaults = {

            # Paths to YAML configuration files
            'task_file': '',
            'landmarks_file': '',

            # Time robot must remain stopped at each landmark
            'dwell_seconds': 3.0,

            # Desired orientation at each navigation target
            'goal_orientation': 'start_bearing',

            # Maximum time waiting for Nav2 to accept a goal
            'goal_acceptance_timeout_seconds': 5.0,

            # Maximum navigation time for EACH landmark
            'navigation_timeout_seconds': 40.0,

            # Maximum time allowed for cancelling navigation
            'cancellation_timeout_seconds': 5.0,

            # Maximum time allowed for localization
            'localization_timeout_seconds': 25.0,

            # Maximum time for the complete mission
            'overall_timeout_seconds': 180.0,

            # Localization uncertainty limits
            'localization_xy_variance_limit': 0.25,
            'localization_yaw_variance_limit': 0.8,

            # Velocity thresholds for considering robot stopped
            'stop_linear_tolerance': 0.02,
            'stop_angular_tolerance': 0.05,

            # Maximum distance between robot and target
            'goal_xy_tolerance': 0.18,
        }

        # Register parameters with ROS 2
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        # ----------------------------------------------------
        # B. Create ROS publishers
        # ----------------------------------------------------

        # Publisher 1:
        # Send zero velocity to /cmd_vel to request robot stop.
        #
        # Communication:
        # TaskManager -> /cmd_vel -> Robot velocity interface

        self.stop_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        # Publisher 2:
        # Send initial robot pose to AMCL.
        #
        # Communication:
        # TaskManager -> /initialpose -> AMCL

        self.initialpose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            '/initialpose',
            10
        )

        # ----------------------------------------------------
        # C. Create Nav2 Action Client
        # ----------------------------------------------------

        # This is the MAIN connection to the official Nav2 stack.
        #
        # TaskManager sends a target pose.
        # Nav2 navigates to that target.
        # Nav2 returns success or failure.

        self.client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose'
        )

        # ----------------------------------------------------
        # D. Create ROS subscribers
        # ----------------------------------------------------

        # Receive robot position estimates from AMCL.
        #
        # Whenever /amcl_pose publishes a message,
        # ROS executes self._amcl_cb().

        self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self._amcl_cb,
            10
        )

        # Receive robot odometry.
        #
        # Whenever /odom publishes a message,
        # ROS executes self._odom_cb().

        self.create_subscription(
            Odometry,
            '/odom',
            self._odom_cb,
            10
        )

        # ----------------------------------------------------
        # E. Create TF listener
        # ----------------------------------------------------

        # Stores coordinate transformations such as:
        #
        # map -> odom -> base_link

        self.tf_buffer = Buffer()

        self.tf_listener = TransformListener(
            self.tf_buffer,
            self
        )

        # ----------------------------------------------------
        # F. Initialize task state variables
        # ----------------------------------------------------

        # Initial state: wait for Nav2
        self.state = TaskState.WAITING_FOR_NAV2

        # Index of current target:
        # 0 = first target
        # 1 = second target
        # 2 = third target
        # 3 = fourth target
        self.goal_index = 0

        # Handle for the current accepted Nav2 goal
        self.goal_handle = None

        # True while waiting for Nav2 to accept a goal
        self.goal_pending = False

        # Current landmark label, e.g. "G"
        self.current_goal_label = None

        # Current target coordinates, e.g. (1.2, 2.4)
        self.goal_target = None

        # Latest robot position received from AMCL
        self._last_amcl_pose = None

        # Latest robot odometry
        self._last_odom = None

        # Deadline for current state
        self._state_deadline = None

        # Deadline for entire mission
        self._overall_deadline = None

        # Deadline for 3-second stop
        self._dwell_deadline = None

        # Time when initial pose was last published
        self._last_initial_pose = 0.0

        # ----------------------------------------------------
        # G. Load task configuration
        # ----------------------------------------------------

        try:

            # Read task.yaml
            #
            # Example:
            #
            # start:
            #   square: A
            #   bearing: N
            #
            # targets: [G, M, T, X]

            self.task = load_task(
                self._param('task_file')
            )

            # Read landmarks.yaml
            #
            # Contains:
            # - Origin of square A
            # - Grid spacing
            # - Grid rotation
            # - Optional coordinate overrides

            with open(
                self._param('landmarks_file'),
                encoding='utf-8'
            ) as stream:

                config = validate_landmark_config(
                    yaml.safe_load(stream)
                )

            # Create landmark coordinate converter
            #
            # Example:
            # converter.coordinate('G') -> (x, y)

            self.converter = LandmarkConverter(config)

        except Exception as exc:

            # Configuration invalid: do not start mission
            self.task = None

            self.get_logger().fatal(
                f'Task configuration rejected: {exc}'
            )

            self._fail(
                'Invalid task or landmark calibration'
            )

        # ----------------------------------------------------
        # H. Create main timer
        # ----------------------------------------------------

        # ROS calls self.tick() every 0.1 seconds.
        #
        # This is OUR task manager's timer.
        # It is NOT Nav2's controller timer.

        self.create_timer(
            0.1,
            self.tick
        )

    # ========================================================
    # 4. HELPER FUNCTIONS
    # ========================================================

    def _now(self):
        """Return current ROS time in seconds."""

        return self.get_clock().now().nanoseconds / 1e9

    def _param(self, name):
        """Read a ROS parameter."""

        return self.get_parameter(name).value

    def _set_state(self, state, timeout=None):
        """
        Change task state.

        If timeout is provided, create a deadline.
        """

        self.state = state

        if timeout is None:
            self._state_deadline = None
        else:
            self._state_deadline = self._now() + timeout

    def stop(self):
        """
        Request robot stop.

        Publishes zero linear and angular velocity.

        TaskManager -> /cmd_vel -> Robot
        """

        self.stop_pub.publish(Twist())

    # ========================================================
    # 5. SUBSCRIBER CALLBACKS
    # ========================================================

    def _amcl_cb(self, msg):
        """
        Triggered whenever AMCL publishes /amcl_pose.

        Store the latest estimated robot position.
        """

        self._last_amcl_pose = msg

    def _odom_cb(self, msg):
        """
        Triggered whenever /odom publishes a message.

        Store the latest robot odometry.
        """

        self._last_odom = msg

    # ========================================================
    # 6. INITIALIZE ROBOT LOCALIZATION
    # ========================================================

    def _publish_initial_pose(self):
        """
        Convert starting square and bearing into a ROS pose.

        Publish that pose to AMCL through /initialpose.
        """

        # Read starting information from task.yaml
        start = self.task['start']

        # Convert starting landmark into map coordinates
        x, y = self.converter.coordinate(
            start['square']
        )

        # Convert compass bearing into ROS yaw
        yaw = self.converter.bearing_yaw(
            start['bearing']
        )

        # Create initial pose message
        msg = PoseWithCovarianceStamped()

        # Initial pose is expressed in the map frame
        msg.header.frame_id = 'map'

        # Attach current ROS timestamp
        msg.header.stamp = self.get_clock().now().to_msg()

        # Set initial position
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y

        # Convert yaw into quaternion orientation
        msg.pose.pose.orientation.z = math.sin(yaw / 2)
        msg.pose.pose.orientation.w = math.cos(yaw / 2)

        # Set position uncertainty
        msg.pose.covariance[0] = 0.09
        msg.pose.covariance[7] = 0.09

        # Set yaw uncertainty
        msg.pose.covariance[35] = math.radians(45) ** 2

        # COMMUNICATION WITH AMCL
        #
        # TaskManager -> /initialpose -> AMCL

        self.initialpose_pub.publish(msg)

        # Record publication time
        self._last_initial_pose = self._now()

    # ========================================================
    # 7. CHECK LOCALIZATION
    # ========================================================

    def _localization_ready(self):
        """
        Check whether AMCL localization is ready.

        Conditions:
        1. AMCL pose exists.
        2. Pose is in map frame.
        3. Reported covariance is acceptable.
        4. TF map -> base_link is available.
        """

        # Get latest AMCL pose
        msg = self._last_amcl_pose

        # No pose or incorrect frame
        if msg is None or msg.header.frame_id != 'map':
            return False

        # Get covariance
        c = msg.pose.covariance

        # Check position uncertainty
        if (
            c[0] > self._param('localization_xy_variance_limit')
            or
            c[7] > self._param('localization_xy_variance_limit')
            or
            c[35] > self._param('localization_yaw_variance_limit')
        ):
            return False

        # Check TF transformation availability
        try:

            return self.tf_buffer.can_transform(
                'map',
                'base_link',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.0)
            )

        except Exception:
            return False

    # ========================================================
    # 8. CHECK WHETHER ROBOT HAS STOPPED
    # ========================================================

    def _robot_stopped(self):
        """
        Determine whether the robot is stationary
        using odometry velocity.
        """

        # Cannot verify stopping without odometry
        if self._last_odom is None:
            return False

        # Extract velocity from odometry
        t = self._last_odom.twist.twist

        # Check linear and angular velocity
        return (
            abs(t.linear.x)
            <= self._param('stop_linear_tolerance')
            and
            abs(t.angular.z)
            <= self._param('stop_angular_tolerance')
        )

    # ========================================================
    # 9. FAILURE HANDLING
    # ========================================================

    def _fail(self, reason):
        """
        Handle task failure.

        1. Log failure.
        2. Request robot stop.
        3. Cancel active Nav2 goal if possible.
        4. Enter FAILED state.
        """

        # Do nothing if task already ended
        if self.state in (
            TaskState.FAILED,
            TaskState.COMPLETED
        ):
            return

        # Print failure reason
        self.get_logger().error(reason)

        # Request robot stop
        self.stop()

        # If an accepted navigation goal exists,
        # request cancellation from Nav2.
        if self.goal_handle is not None:

            self.goal_handle.cancel_goal_async()

            self._set_state(
                TaskState.CANCELLING,
                self._param('cancellation_timeout_seconds')
            )

        else:

            # No accepted goal: fail immediately
            self._set_state(TaskState.FAILED)

    # ========================================================
    # 10. SEND NEXT NAVIGATION GOAL
    # ========================================================

    def _send_next_goal(self):
        """
        Send the next landmark to Nav2.

        Example:
            targets = [G, M, T, X]

            goal_index = 0 -> G
            goal_index = 1 -> M
            goal_index = 2 -> T
            goal_index = 3 -> X
        """

        # ----------------------------------------------------
        # A. Check whether all four targets are complete
        # ----------------------------------------------------

        if self.goal_index >= len(self.task['targets']):

            # All targets completed
            self.stop()

            self._set_state(
                TaskState.COMPLETED
            )

            self.get_logger().info(
                'Four-target task complete; robot stopped.'
            )

            return

        # ----------------------------------------------------
        # B. Get current target
        # ----------------------------------------------------

        # Read current landmark from task.yaml
        self.current_goal_label = (
            self.task['targets'][self.goal_index]
        )

        # Convert landmark label into map coordinates
        x, y = self.converter.coordinate(
            self.current_goal_label
        )

        # Save target coordinates for later verification
        self.goal_target = (x, y)

        # ----------------------------------------------------
        # C. Determine desired target orientation
        # ----------------------------------------------------

        if self._param('goal_orientation') == 'start_bearing':

            yaw = self.converter.bearing_yaw(
                self.task['start']['bearing']
            )

        else:

            yaw = 0.0

        # ----------------------------------------------------
        # D. Create Nav2 navigation goal
        # ----------------------------------------------------

        goal = NavigateToPose.Goal()

        goal.pose = PoseStamped()

        # Target coordinates are in map frame
        goal.pose.header.frame_id = 'map'

        goal.pose.header.stamp = self.get_clock().now().to_msg()

        # Set target position
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y

        # Set target orientation
        goal.pose.pose.orientation.z = math.sin(yaw / 2)
        goal.pose.pose.orientation.w = math.cos(yaw / 2)

        # ----------------------------------------------------
        # E. Send goal to Nav2
        # ----------------------------------------------------

        # Mark goal as waiting for acceptance
        self.goal_pending = True

        # Enter NAVIGATING state with acceptance timeout
        self._set_state(
            TaskState.NAVIGATING,
            self._param('goal_acceptance_timeout_seconds')
        )

        # MAIN COMMUNICATION WITH NAV2
        #
        # TaskManager -> NavigateToPose -> Nav2 BT Navigator
        #
        # Nav2 performs navigation asynchronously.
        #
        # When Nav2 responds to the goal request,
        # _goal_response() will execute.

        self.client.send_goal_async(
            goal
        ).add_done_callback(
            self._goal_response
        )

        # Print current navigation target
        self.get_logger().info(
            f'Sending target {self.goal_index + 1}/4: '
            f'{self.current_goal_label} ({x:.3f}, {y:.3f})'
        )

    # ========================================================
    # 11. NAV2 GOAL ACCEPTANCE CALLBACK
    # ========================================================

    def _goal_response(self, future):
        """
        Triggered when Nav2 responds to the goal request.

        This does NOT mean the robot has reached the target.

        It only means Nav2 accepted or rejected the goal.
        """

        # Ignore response if task is no longer waiting
        if (
            self.state is not TaskState.NAVIGATING
            or not self.goal_pending
        ):
            return

        # Goal request is no longer pending
        self.goal_pending = False

        # Read Nav2 goal handle
        try:

            self.goal_handle = future.result()

        except Exception as exc:

            self._fail(
                f'Goal request failed: {exc}'
            )
            return

        # Check whether Nav2 accepted the goal
        if (
            self.goal_handle is None
            or not self.goal_handle.accepted
        ):

            self._fail('Nav2 rejected goal')
            return

        # Nav2 accepted the goal.
        # Start navigation execution timeout.
        self._state_deadline = (
            self._now()
            + self._param('navigation_timeout_seconds')
        )

        # Register callback for FINAL navigation result.
        #
        # When Nav2 finishes navigating,
        # ROS executes _goal_result().

        self.goal_handle.get_result_async().add_done_callback(
            self._goal_result
        )

    # ========================================================
    # 12. NAV2 NAVIGATION RESULT CALLBACK
    # ========================================================

    def _goal_result(self, future):
        """
        Triggered when Nav2 finishes navigation.

        Possible outcomes:
        - SUCCEEDED
        - ABORTED
        - CANCELED
        """

        # Only process results during navigation/cancellation
        if self.state not in (
            TaskState.NAVIGATING,
            TaskState.CANCELLING
        ):
            return

        # Clear active goal handle
        self.goal_handle = None

        # Read Nav2 result status
        try:

            status = future.result().status

        except Exception as exc:

            self._fail(
                f'Goal result failed: {exc}'
            )
            return

        # If cancellation was requested, ignore normal result
        if self.state is TaskState.CANCELLING:
            return

        # Check whether navigation succeeded
        if status != GoalStatus.STATUS_SUCCEEDED:

            self._fail(
                f'Navigation failed with action status {status}'
            )
            return

        # ----------------------------------------------------
        # A. Verify robot reached target
        # ----------------------------------------------------

        # Get latest AMCL estimated position
        if self._last_amcl_pose is not None:

            p = self._last_amcl_pose.pose.pose.position

        else:

            p = None

        # Check distance from robot to target
        if (
            p is None
            or math.hypot(
                p.x - self.goal_target[0],
                p.y - self.goal_target[1]
            ) > self._param('goal_xy_tolerance')
        ):

            self._fail(
                'Nav2 reported success outside target tolerance'
            )
            return

        # ----------------------------------------------------
        # B. Navigation succeeded
        # ----------------------------------------------------

        # Request robot stop
        self.stop()

        # Reset dwell timer
        self._dwell_deadline = None

        # Enter DWELLING state
        self._set_state(
            TaskState.DWELLING
        )

    # ========================================================
    # 13. MAIN TIMER CALLBACK — TASK STATE MACHINE
    # ========================================================

    def tick(self):
        """
        Main task manager timer callback.

        Called every 0.1 seconds by ROS 2.

        Responsibilities:
        - Wait for Nav2.
        - Initialize localization.
        - Check localization readiness.
        - Monitor navigation timeout.
        - Handle cancellation timeout.
        - Handle 3-second dwelling.
        - Start next navigation goal.
        """

        # Get current time
        now = self._now()

        # ----------------------------------------------------
        # A. Request stop in stationary states
        # ----------------------------------------------------

        if self.state in (
            TaskState.FAILED,
            TaskState.COMPLETED,
            TaskState.CANCELLING,
            TaskState.DWELLING
        ):

            self.stop()

        # ----------------------------------------------------
        # B. Stop processing if task configuration is invalid
        # ----------------------------------------------------

        if self.task is None:
            return

        # ----------------------------------------------------
        # C. Check overall mission timeout
        # ----------------------------------------------------

        if (
            self._overall_deadline is not None
            and now >= self._overall_deadline
            and self.state not in (
                TaskState.COMPLETED,
                TaskState.FAILED
            )
        ):

            self._fail(
                'Overall task limit expired'
            )
            return

        # ----------------------------------------------------
        # D. STATE: WAITING_FOR_NAV2
        # ----------------------------------------------------

        if self.state is TaskState.WAITING_FOR_NAV2:

            # Check whether Nav2 action server is available
            if self.client.wait_for_server(timeout_sec=0.0):

                # Start overall mission timer
                self._overall_deadline = (
                    now + self._param('overall_timeout_seconds')
                )

                # Initialize robot starting position
                self._publish_initial_pose()

                # Enter localization initialization state
                self._set_state(
                    TaskState.INITIALIZING_LOCALIZATION,
                    self._param('localization_timeout_seconds')
                )

        # ----------------------------------------------------
        # E. STATE: INITIALIZING / WAITING FOR LOCALIZATION
        # ----------------------------------------------------

        elif self.state in (
            TaskState.INITIALIZING_LOCALIZATION,
            TaskState.WAITING_FOR_LOCALIZATION
        ):

            # Check whether localization is ready
            if self._localization_ready():

                # Localization ready -> send first target
                self._send_next_goal()
                return

            # Check localization timeout
            if now >= self._state_deadline:

                self._fail(
                    'AMCL did not converge within localization timeout'
                )
                return

            # Republish initial pose approximately every second
            if now - self._last_initial_pose >= 1.0:

                self._publish_initial_pose()

            # Continue waiting for localization
            self.state = TaskState.WAITING_FOR_LOCALIZATION

        # ----------------------------------------------------
        # F. STATE: NAVIGATING
        # ----------------------------------------------------

        elif self.state is TaskState.NAVIGATING:

            # Check navigation timeout
            if now >= self._state_deadline:

                self._fail(
                    'Navigation goal acceptance or execution timeout'
                )

        # ----------------------------------------------------
        # G. STATE: CANCELLING
        # ----------------------------------------------------

        elif self.state is TaskState.CANCELLING:

            # Check cancellation timeout
            if now >= self._state_deadline:

                self.get_logger().error(
                    'Cancellation timeout; command stop remains active'
                )

                self._set_state(
                    TaskState.FAILED
                )

        # ----------------------------------------------------
        # H. STATE: DWELLING
        # ----------------------------------------------------

        elif self.state is TaskState.DWELLING:

            # Check whether robot has stopped
            if not self._robot_stopped():
                return

            # Start dwell timer if it has not started
            if self._dwell_deadline is None:

                self._dwell_deadline = (
                    now + self._param('dwell_seconds')
                )

            # Dwell period completed
            elif now >= self._dwell_deadline:

                # Move to next target index
                self.goal_index += 1

                # Send next goal or finish mission
                self._send_next_goal()


# ============================================================
# 14. PROGRAM ENTRY POINT
# ============================================================

def main():
    """
    Start ROS 2 and run the TaskManager node.
    """

    # Initialize ROS 2
    rclpy.init()

    # Create our task manager node
    node = TaskManager()

    try:

        # Keep ROS node running.
        #
        # Processes:
        # - Timer callback: tick()
        # - AMCL callback: _amcl_cb()
        # - Odometry callback: _odom_cb()
        # - Nav2 goal acceptance: _goal_response()
        # - Nav2 navigation result: _goal_result()

        rclpy.spin(node)

    finally:

        # Request robot stop before shutdown
        node.stop()

        # Destroy ROS node
        node.destroy_node()

        # Shut down ROS 2
        rclpy.shutdown()
