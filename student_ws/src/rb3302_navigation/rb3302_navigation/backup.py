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
from lifecycle_msgs.msg import State as LifecycleState
from lifecycle_msgs.srv import GetState

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

    WAITING_FOR_NAV2_LIFECYCLE = enum.auto()

    NAVIGATING = enum.auto()

    RECOVERY_WAIT = enum.auto()

    RECOVERY_CHECK = enum.auto()

    RECOVERY_CANCEL = enum.auto()

    # CANCELLING is reserved for terminal cancellation.  A retry uses
    # RECOVERY_CANCEL so an intentional stop can never start another goal.
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

            # Initial bearing is for AMCL only. PositionGoalChecker makes the
            # final heading irrelevant to graded-run completion.
            'goal_orientation': 'none',

            # Maximum time waiting for Nav2 to accept a goal
            'goal_acceptance_timeout_seconds': 5.0,

            # Maximum navigation time for EACH landmark
            'navigation_timeout_seconds': 120.0,

            # Maximum time allowed for cancelling navigation
            'cancellation_timeout_seconds': 5.0,

            # Maximum time allowed for localization
            'localization_timeout_seconds': 25.0,

            # Time to wait after localization for Nav2's managed navigation
            # nodes to finish their lifecycle activation.
            'nav2_activation_timeout_seconds': 45.0,

            # Lifecycle requests are asynchronous and intentionally sampled,
            # rather than blocking the executor during Nav2 startup.
            'lifecycle_check_period_seconds': 0.5,
            'lifecycle_request_timeout_seconds': 2.0,

            # A rejected startup goal is retried only a bounded number of
            # times, after lifecycle state has been checked again.
            'startup_goal_retries': 2,
            'startup_goal_retry_delay_seconds': 1.0,

            # General navigation recovery.  max_navigation_retries counts
            # additional submissions after the first attempt at a waypoint.
            'max_navigation_retries': 5,
            'retry_delay_seconds': 2.0,
            'recovery_readiness_timeout_seconds': 30.0,
            'recovery_tf_max_age_seconds': 2.0,
            # AMCL intentionally future-dates map -> odom by its transform
            # tolerance (1.0 s in nav2_real.yaml).
            'recovery_tf_future_tolerance_seconds': 1.5,
            'recovery_diagnostic_period_seconds': 2.0,

            # Maximum time for the complete mission
            'overall_timeout_seconds': 300.0,

            # Localization uncertainty limits
            'localization_xy_variance_limit': 0.25,
            'localization_yaw_variance_limit': 0.8,

            # Velocity thresholds for considering robot stopped
            'stop_linear_tolerance': 0.02,
            'stop_angular_tolerance': 0.05,

            # Maximum distance between robot and target
            'goal_xy_tolerance': 0.08,

            # Do not continually reset AMCL on real hardware. Enable only for
            # a deliberate recovery experiment.
            'republish_initial_pose': False,
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

        # These are the nodes managed by planning_rpp/planning.launch.py.
        # Keep the list configurable for a simulation that intentionally
        # launches a smaller Nav2 stack.
        self.declare_parameter(
            'required_nav2_lifecycle_nodes',
            [
                '/planner_server',
                '/controller_server',
                '/smoother_server',
                '/behavior_server',
                '/bt_navigator',
                '/waypoint_follower',
                '/velocity_smoother',
            ]
        )
        self._lifecycle_clients = {
            node_name: self.create_client(GetState, f'{node_name}/get_state')
            for node_name in self._param('required_nav2_lifecycle_nodes')
        }
        self._lifecycle_futures = {
            node_name: None for node_name in self._lifecycle_clients
        }
        self._lifecycle_states = {
            node_name: None for node_name in self._lifecycle_clients
        }
        # Record when each lifecycle request was started.  Recovery accepts
        # ACTIVE only from requests issued after recovery checking began.
        self._lifecycle_query_started = {
            node_name: float('-inf') for node_name in self._lifecycle_clients
        }
        self._lifecycle_state_query_started = {
            node_name: float('-inf') for node_name in self._lifecycle_clients
        }
        self._last_lifecycle_refresh = float('-inf')
        self._last_lifecycle_wait_log = float('-inf')

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

        # Submission and retry counts for the current target.  Retries count
        # additional submissions, not the initial attempt.
        self._goal_attempts = 0
        self._navigation_retries = 0
        self._goal_retry_at = None

        # Every asynchronous action callback carries this generation.  A new
        # generation is not created until the preceding one is fully settled.
        self._next_attempt_id = 0
        self._active_attempt_id = None
        self._goal_send_future = None
        self._goal_result_future = None
        self._cancel_future = None
        self._goal_terminal = False
        self._cancel_for_retry = False
        self._cancel_reason = None
        self._cancel_result_status = None
        self._stale_goal_handles = {}

        # Recovery readiness always uses fresh lifecycle samples and never
        # changes the mission-wide deadline.
        self._recovery_check_started_at = None
        self._recovery_reason = None
        self._recovery_blockers = []
        self._recovery_readiness_details = ''
        self._last_recovery_diagnostic_log = float('-inf')
        self._terminal_failure_reason = None
        self._shutdown_requested = False

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
        self._initial_pose_stamp_ns = None

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

            self.get_logger().info(
                f"[TASK] Loaded starting square {self.task['start']['square']}, "
                f"bearing {self.task['start']['bearing']}"
            )
            self.get_logger().info(
                '[TASK] Loaded targets: ' + ', '.join(self.task['targets'])
            )

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

        # Ctrl-C may invalidate the rcl context before `finally` executes.
        if rclpy.ok():
            try:
                self.stop_pub.publish(Twist())
            except Exception as exc:
                self.get_logger().debug(f'Unable to publish stop during shutdown: {exc}')

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
        self._initial_pose_stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        self.get_logger().info('[AMCL] Initial pose published')

    # ========================================================
    # 7. CHECK LOCALIZATION
    # ========================================================

    def _amcl_readiness(self, now):
        """Return AMCL validity, blockers, and values for diagnostics."""
        xy_limit = self._param('localization_xy_variance_limit')
        yaw_limit = self._param('localization_yaw_variance_limit')
        details = {
            'pose_age': None,
            'covariance': None,
            'xy_limit': xy_limit,
            'yaw_limit': yaw_limit,
        }
        blockers = []
        msg = self._last_amcl_pose
        if msg is None:
            return False, ['AMCL pose unavailable'], details

        if msg.header.frame_id != 'map':
            blockers.append(
                f'AMCL frame={msg.header.frame_id!r}, expected "map"'
            )

        pose_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        details['pose_age'] = now - pose_stamp
        pose_stamp_ns = (
            msg.header.stamp.sec * 1_000_000_000
            + msg.header.stamp.nanosec
        )
        if self._initial_pose_stamp_ns is None:
            blockers.append('initial pose publication is not recorded')
        elif pose_stamp_ns + 500_000_000 < self._initial_pose_stamp_ns:
            blockers.append(
                'AMCL pose predates initial pose by more than 0.5 s'
            )

        c = msg.pose.covariance
        covariance = (c[0], c[7], c[35])
        details['covariance'] = covariance
        labels_and_limits = (
            ('AMCL covariance x', covariance[0], xy_limit),
            ('AMCL covariance y', covariance[1], xy_limit),
            ('AMCL covariance yaw', covariance[2], yaw_limit),
        )
        for label, value, limit in labels_and_limits:
            if not math.isfinite(value) or value > limit:
                blockers.append(f'{label}={value:.6g} exceeds {limit:.6g}')

        # AMCL pose age is intentionally diagnostic-only.  A stationary robot
        # may not trigger a filter update or a new /amcl_pose publication.
        return not blockers, blockers, details

    def _localization_ready(self, require_recent=False):
        """
        Check whether AMCL localization is ready.

        Conditions:
        1. AMCL pose exists.
        2. Pose is in map frame.
        3. Reported covariance is acceptable.
        4. TF map -> base_link is available.
        """

        # `require_recent` remains accepted for compatibility, but AMCL pose
        # age is no longer a gate.  Recovery verifies live TF timestamps.
        del require_recent
        amcl_ready, _, _ = self._amcl_readiness(self._now())
        if not amcl_ready:
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

    def _odometry_tf_ready(self):
        """Require the EKF's live odom -> base_link before initializing AMCL."""
        if self._last_odom is None:
            return False
        try:
            return self.tf_buffer.can_transform(
                'odom', 'base_link', rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.0))
        except Exception:
            return False

    # ========================================================
    # 9. NAV2 LIFECYCLE READINESS
    # ========================================================

    @staticmethod
    def _lifecycle_state_name(state_id):
        """Return a useful, stable name for lifecycle logging."""
        names = {
            LifecycleState.PRIMARY_STATE_UNKNOWN: 'UNKNOWN',
            LifecycleState.PRIMARY_STATE_UNCONFIGURED: 'UNCONFIGURED',
            LifecycleState.PRIMARY_STATE_INACTIVE: 'INACTIVE',
            LifecycleState.PRIMARY_STATE_ACTIVE: 'ACTIVE',
            LifecycleState.PRIMARY_STATE_FINALIZED: 'FINALIZED',
        }
        return names.get(state_id, f'STATE_{state_id}')

    def _refresh_lifecycle_states(self, now):
        """Poll cached lifecycle clients without blocking the executor."""
        for node_name, future in self._lifecycle_futures.items():
            if (
                future is not None
                and not future.done()
                and now - self._lifecycle_query_started[node_name]
                >= self._param('lifecycle_request_timeout_seconds')
            ):
                # GetState is read-only.  Stop tracking a stuck request so a
                # new recovery-time sample can be issued.
                self._lifecycle_futures[node_name] = None
                self._lifecycle_states[node_name] = None
                self._lifecycle_state_query_started[node_name] = float('-inf')
                self.get_logger().warn(
                    f'[NAV2] {node_name}: lifecycle query timed out; retrying'
                )
                future = None
            if future is not None and future.done():
                self._lifecycle_futures[node_name] = None
                query_started = self._lifecycle_query_started[node_name]
                try:
                    state_id = future.result().current_state.id
                except Exception as exc:
                    self.get_logger().warn(
                        f'[NAV2] {node_name}: lifecycle query failed: {exc}'
                    )
                    state_id = None

                self._lifecycle_state_query_started[node_name] = query_started

                if self._lifecycle_states[node_name] != state_id:
                    self._lifecycle_states[node_name] = state_id
                    label = ('service unavailable' if state_id is None else
                             self._lifecycle_state_name(state_id))
                    self.get_logger().info(f'[NAV2] {node_name}: {label}')

        if now - self._last_lifecycle_refresh < self._param('lifecycle_check_period_seconds'):
            return

        self._last_lifecycle_refresh = now
        for node_name, client in self._lifecycle_clients.items():
            if self._lifecycle_futures[node_name] is not None:
                continue
            if not client.service_is_ready():
                if self._lifecycle_states[node_name] is not None:
                    self._lifecycle_states[node_name] = None
                    self.get_logger().info(f'[NAV2] {node_name}: service unavailable')
                continue
            try:
                self._lifecycle_query_started[node_name] = now
                self._lifecycle_futures[node_name] = client.call_async(GetState.Request())
            except Exception as exc:
                self.get_logger().warn(
                    f'[NAV2] {node_name}: unable to request lifecycle state: {exc}'
                )

    def _nav2_lifecycle_ready(self, now, fresh_since=None):
        """Return True only after every required navigation node is ACTIVE."""
        self._refresh_lifecycle_states(now)
        inactive = [
            node_name for node_name, state_id in self._lifecycle_states.items()
            if (
                state_id != LifecycleState.PRIMARY_STATE_ACTIVE
                or (
                    fresh_since is not None
                    and self._lifecycle_state_query_started[node_name] < fresh_since
                )
            )
        ]
        if not inactive:
            return True

        if now - self._last_lifecycle_wait_log >= 5.0:
            summary = ', '.join(
                f'{node_name}={self._lifecycle_state_name(self._lifecycle_states[node_name])}'
                if self._lifecycle_states[node_name] is not None
                else f'{node_name}=UNAVAILABLE'
                for node_name in inactive
            )
            self.get_logger().info(
                f'[NAV2] Waiting for lifecycle activation: {summary}'
            )
            self._last_lifecycle_wait_log = now
        return False

    def _begin_recovery_check(self, now):
        """Require new lifecycle replies before declaring recovery ready."""
        self._recovery_check_started_at = now
        self._last_lifecycle_refresh = float('-inf')
        self._last_lifecycle_wait_log = float('-inf')
        self._last_recovery_diagnostic_log = float('-inf')
        self._recovery_blockers = []
        self._recovery_readiness_details = ''
        for node_name in self._lifecycle_states:
            # Do not let a request issued before recovery suppress the fresh
            # GetState request required by recovery readiness.
            self._lifecycle_futures[node_name] = None
            self._lifecycle_states[node_name] = None
            self._lifecycle_query_started[node_name] = float('-inf')
            self._lifecycle_state_query_started[node_name] = float('-inf')
        self._set_state(
            TaskState.RECOVERY_CHECK,
            self._param('recovery_readiness_timeout_seconds')
        )
        self.get_logger().info('[RECOVERY] Checking Nav2 readiness')

    def _transform_readiness(self, target, source, now, require_fresh):
        """Return TF readiness plus timestamp diagnostics."""
        name = f'{target}->{source}'
        try:
            transform = self.tf_buffer.lookup_transform(
                target,
                source,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.0)
            )
        except Exception as exc:
            return False, f'{name}=unavailable ({exc})', f'{name} unavailable'

        stamp = (
            transform.header.stamp.sec
            + transform.header.stamp.nanosec / 1e9
        )
        age = now - stamp
        if not require_fresh:
            return True, f'{name}=available age={age:.3f}s', None

        max_age = self._param('recovery_tf_max_age_seconds')
        future_tolerance = self._param(
            'recovery_tf_future_tolerance_seconds'
        )
        detail = (
            f'{name}=available age={age:.3f}s '
            f'allowed=[-{future_tolerance:.3f},{max_age:.3f}]s'
        )
        if age > max_age:
            return (
                False,
                detail,
                f'{name} stale: age={age:.3f}s > {max_age:.3f}s'
            )
        if age < -future_tolerance:
            return (
                False,
                detail,
                f'{name} too far in future: age={age:.3f}s '
                f'< -{future_tolerance:.3f}s'
            )
        return True, detail, None

    def _log_recovery_diagnostics(self, now, force=False):
        """Log the complete blocker set at a bounded rate."""
        period = self._param('recovery_diagnostic_period_seconds')
        if not force and now - self._last_recovery_diagnostic_log < period:
            return
        self._last_recovery_diagnostic_log = now
        blockers = '; '.join(self._recovery_blockers) or 'none'
        self.get_logger().warn(f'[RECOVERY] Readiness blockers: {blockers}')
        self.get_logger().info(
            f'[RECOVERY] Readiness values: {self._recovery_readiness_details}'
        )

    def _recovery_ready(self, now):
        """Evaluate and report every dependency required before a retry."""
        blockers = []
        details = []

        action_ready = self.client.wait_for_server(timeout_sec=0.0)
        details.append(f'action_server={action_ready}')
        if not action_ready:
            blockers.append('NavigateToPose action server unavailable')

        lifecycle_ready = self._nav2_lifecycle_ready(
            now, fresh_since=self._recovery_check_started_at
        )
        lifecycle_values = []
        for node_name, state_id in self._lifecycle_states.items():
            label = (
                self._lifecycle_state_name(state_id)
                if state_id is not None else 'UNAVAILABLE'
            )
            fresh = (
                self._lifecycle_state_query_started[node_name]
                >= self._recovery_check_started_at
            )
            lifecycle_values.append(f'{node_name}={label},fresh={fresh}')
            if state_id != LifecycleState.PRIMARY_STATE_ACTIVE:
                blockers.append(f'{node_name} lifecycle={label}')
            elif not fresh:
                blockers.append(f'{node_name} lifecycle sample is stale')
        details.append('lifecycle=[' + ', '.join(lifecycle_values) + ']')

        amcl_ready, amcl_blockers, amcl = self._amcl_readiness(now)
        blockers.extend(amcl_blockers)
        pose_age = amcl['pose_age']
        covariance = amcl['covariance']
        details.append(
            'amcl_pose_age='
            + ('unavailable' if pose_age is None else f'{pose_age:.3f}s')
        )
        if covariance is None:
            details.append('amcl_covariance=unavailable')
        else:
            details.append(
                f'amcl_covariance=(x={covariance[0]:.6g}, '
                f'y={covariance[1]:.6g}, yaw={covariance[2]:.6g}) '
                f'limits=(xy={amcl["xy_limit"]:.6g}, '
                f'yaw={amcl["yaw_limit"]:.6g})'
            )

        if self._last_odom is None:
            blockers.append('odometry message unavailable')
            details.append('odometry_message=False')
        else:
            details.append('odometry_message=True')

        map_odom_ready, map_odom_detail, map_odom_blocker = (
            self._transform_readiness('map', 'odom', now, require_fresh=True)
        )
        odom_base_ready, odom_base_detail, odom_base_blocker = (
            self._transform_readiness(
                'odom', 'base_link', now, require_fresh=True
            )
        )
        map_base_ready, map_base_detail, map_base_blocker = (
            self._transform_readiness(
                'map', 'base_link', now, require_fresh=False
            )
        )
        details.extend((map_odom_detail, odom_base_detail, map_base_detail))
        for blocker in (
            map_odom_blocker, odom_base_blocker, map_base_blocker
        ):
            if blocker is not None:
                blockers.append(blocker)

        ready = all((
            action_ready,
            lifecycle_ready,
            amcl_ready,
            self._last_odom is not None,
            map_odom_ready,
            odom_base_ready,
            map_base_ready,
        ))
        self._recovery_blockers = blockers
        self._recovery_readiness_details = '; '.join(details)
        if not ready:
            self._log_recovery_diagnostics(now)
        return ready

    # ========================================================
    # 10. FAILURE HANDLING
    # ========================================================

    def _clear_attempt(self, attempt_id=None):
        """Forget an attempt only after every action operation is settled."""
        if attempt_id is not None and attempt_id != self._active_attempt_id:
            return
        self.goal_handle = None
        self.goal_pending = False
        self._goal_send_future = None
        self._goal_result_future = None
        self._cancel_future = None
        self._goal_terminal = False
        self._cancel_for_retry = False
        self._cancel_reason = None
        self._cancel_result_status = None
        self._active_attempt_id = None

    def _schedule_navigation_retry(self, reason, retry_delay=None):
        """Schedule, but do not yet submit, another attempt at this waypoint."""
        maximum = int(self._param('max_navigation_retries'))
        if self._navigation_retries >= maximum:
            self.get_logger().error(
                f'[RECOVERY] Goal {self.current_goal_label} failed after '
                f'{self._goal_attempts} attempts'
            )
            self.get_logger().error('[RECOVERY] Retry limit reached')
            self.get_logger().error(
                f'[TASK] Waypoint {self.current_goal_label} incomplete'
            )
            self._fail(reason)
            return

        delay = (
            self._param('retry_delay_seconds')
            if retry_delay is None else retry_delay
        )
        now = self._now()
        if (
            self._overall_deadline is not None
            and now + delay >= self._overall_deadline
        ):
            self._fail(
                'Overall task limit leaves insufficient time for another retry'
            )
            return

        self._recovery_reason = reason
        self._goal_retry_at = now + delay
        self._set_state(TaskState.RECOVERY_WAIT)
        self.get_logger().warn(
            f'[RECOVERY] Scheduling retry {self._navigation_retries + 1}/'
            f'{maximum} for goal {self.current_goal_label}: {reason}'
        )
        self.get_logger().info(f'[RECOVERY] Waiting {delay:.1f} seconds')

    def _handle_navigation_failure(
        self, reason, retryable=True, attempt_id=None, retry_delay=None
    ):
        """Centralize navigation failure classification and safe recovery."""
        if self.state in (TaskState.COMPLETED, TaskState.FAILED):
            return
        if self._shutdown_requested or not retryable:
            self._fail(reason)
            return
        if (
            attempt_id is not None
            and attempt_id != self._active_attempt_id
        ):
            self.get_logger().warn(
                f'[NAV] Ignoring failure from stale attempt {attempt_id}: {reason}'
            )
            return

        # An unresolved send, accepted goal, result, or cancellation must be
        # reconciled before a replacement can be submitted.
        send_unresolved = (
            self.goal_pending
            or (
                self._goal_send_future is not None
                and not self._goal_send_future.done()
            )
        )
        result_unresolved = (
            self._goal_result_future is not None
            and not self._goal_result_future.done()
        )
        cancel_unresolved = (
            self._cancel_future is not None
            and not self._cancel_future.done()
        )
        accepted_active = (
            self.goal_handle is not None
            and self.goal_handle.accepted
            and not self._goal_terminal
        )
        if send_unresolved or result_unresolved or cancel_unresolved or accepted_active:
            self._begin_cancellation(reason, for_retry=True)
            return

        completed_attempt = self._active_attempt_id
        self._clear_attempt(completed_attempt)
        self._schedule_navigation_retry(reason, retry_delay=retry_delay)

    def _register_result_callback(self, attempt_id):
        """Request a terminal result for the accepted current attempt."""
        try:
            self._goal_result_future = self.goal_handle.get_result_async()
            self._goal_result_future.add_done_callback(
                lambda future, aid=attempt_id: self._goal_result(future, aid)
            )
        except Exception as exc:
            self._goal_result_future = None
            self.get_logger().warn(
                f'[NAV] Unable to request goal result: {exc}'
            )
            return False
        return True

    def _issue_cancel_request(self, attempt_id):
        """Asynchronously request cancellation of an accepted current goal."""
        if attempt_id != self._active_attempt_id or self._cancel_future is not None:
            return
        try:
            self._cancel_future = self.goal_handle.cancel_goal_async()
            self._cancel_future.add_done_callback(
                lambda future, aid=attempt_id: self._cancel_response(future, aid)
            )
        except Exception as exc:
            self.get_logger().error(
                f'[NAV] Unable to request cancellation: {exc}'
            )
            # The old goal may still be active.  Creating a replacement would
            # be unsafe, so this is terminal even when recovery was intended.
            self._terminal_failure_reason = self._cancel_reason
            self._set_state(TaskState.FAILED)

    def _begin_cancellation(self, reason, for_retry):
        """Cancel for recovery or terminal failure without conflating them."""
        attempt_id = self._active_attempt_id
        self.stop()
        self._cancel_for_retry = for_retry
        self._cancel_reason = reason
        self._cancel_result_status = None
        self._set_state(
            TaskState.RECOVERY_CANCEL if for_retry else TaskState.CANCELLING,
            self._param('cancellation_timeout_seconds')
        )

        if self.goal_pending:
            self.get_logger().warn(
                '[NAV] Acceptance timed out; waiting for the outstanding '
                'goal response before cancellation'
            )
            return

        if self.goal_handle is None or not self.goal_handle.accepted:
            self._goal_terminal = True
            self._maybe_finish_cancellation(attempt_id)
            return

        if self._goal_result_future is None or self._goal_result_future.done():
            self._register_result_callback(attempt_id)
        self._issue_cancel_request(attempt_id)

    def _cancel_response(self, future, attempt_id):
        """Record cancel acknowledgment; terminal result is still required."""
        if attempt_id != self._active_attempt_id:
            return
        self._cancel_future = None
        try:
            response = future.result()
            if not response.goals_canceling:
                self.get_logger().warn(
                    '[NAV] Cancellation response contained no canceling goal; '
                    'waiting for terminal result'
                )
        except Exception as exc:
            self.get_logger().warn(
                f'[NAV] Cancellation acknowledgment failed: {exc}; '
                'waiting for terminal result'
            )
        self._maybe_finish_cancellation(attempt_id)

    def _maybe_finish_cancellation(self, attempt_id):
        """Finish cancellation only after acknowledgment and terminal result."""
        if attempt_id != self._active_attempt_id:
            return
        if not self._goal_terminal or self._cancel_future is not None:
            return

        status = self._cancel_result_status
        reason = self._cancel_reason
        retry = self._cancel_for_retry
        self._clear_attempt(attempt_id)

        if not retry:
            self._set_state(TaskState.FAILED)
            return

        # A timeout can race with a legitimate success.  A verified terminal
        # success advances normally instead of replaying the waypoint.
        if status == GoalStatus.STATUS_SUCCEEDED:
            self._finish_goal_success()
            return
        self._schedule_navigation_retry(reason)

    def _fail(self, reason):
        """
        Handle task failure.

        1. Log failure.
        2. Request robot stop.
        3. Cancel active Nav2 goal if possible.
        4. Enter FAILED state.
        """

        if self.state in (TaskState.FAILED, TaskState.COMPLETED):
            return
        if self.state is TaskState.CANCELLING:
            return

        self._terminal_failure_reason = reason
        self.get_logger().error(f'[TASK] {reason}')
        self.get_logger().error('[TASK] Stopping safely')
        self.stop()
        unresolved = (
            self.goal_pending
            or self._cancel_future is not None
            or (
                self.goal_handle is not None
                and self.goal_handle.accepted
                and not self._goal_terminal
            )
        )
        if unresolved:
            self._begin_cancellation(reason, for_retry=False)
            return
        self._clear_attempt(self._active_attempt_id)
        self._set_state(TaskState.FAILED)

    # ========================================================
    # 11. SEND NEXT NAVIGATION GOAL
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

        if self.goal_index >= len(self.task['targets']):
            self.stop()
            self._set_state(TaskState.COMPLETED)
            self.get_logger().info('[TASK] Four-target task complete; robot stopped')
            return

        # A new submission is forbidden while any operation from the previous
        # generation is unresolved.
        if (
            self._active_attempt_id is not None
            or self.goal_pending
            or self._cancel_future is not None
        ):
            self._fail('Internal safety check prevented overlapping navigation goals')
            return
        if self._overall_deadline is not None and self._now() >= self._overall_deadline:
            self._fail('Overall task limit expired before goal submission')
            return

        self.current_goal_label = self.task['targets'][self.goal_index]
        try:
            x, y = self.converter.coordinate(self.current_goal_label)
            x, y = float(x), float(y)
        except Exception as exc:
            self._handle_navigation_failure(
                f'Invalid waypoint {self.current_goal_label}: {exc}',
                retryable=False
            )
            return
        if not (math.isfinite(x) and math.isfinite(y)):
            self._handle_navigation_failure(
                f'Invalid waypoint coordinates for {self.current_goal_label}',
                retryable=False
            )
            return
        self.goal_target = (x, y)

        if self._param('goal_orientation') == 'start_bearing':
            yaw = self.converter.bearing_yaw(
                self.task['start']['bearing']
            )
        elif self._param('goal_orientation') == 'none':
            yaw = 0.0
        else:
            self._handle_navigation_failure(
                'goal_orientation must be none or start_bearing',
                retryable=False
            )
            return

        if not self.client.wait_for_server(timeout_sec=0.0):
            self._handle_navigation_failure(
                'NavigateToPose action server temporarily unavailable'
            )
            return

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.z = math.sin(yaw / 2)
        goal.pose.pose.orientation.w = math.cos(yaw / 2)

        if self._goal_attempts > 0:
            maximum = int(self._param('max_navigation_retries'))
            if self._navigation_retries >= maximum:
                self._fail(f'Retry limit reached for goal {self.current_goal_label}')
                return
            self._navigation_retries += 1

        self._next_attempt_id += 1
        attempt_id = self._next_attempt_id
        self._active_attempt_id = attempt_id
        self.goal_pending = True
        self._goal_terminal = False
        self._goal_attempts += 1
        self._set_state(
            TaskState.NAVIGATING,
            self._param('goal_acceptance_timeout_seconds')
        )

        try:
            self._goal_send_future = self.client.send_goal_async(goal)
            self._goal_send_future.add_done_callback(
                lambda future, aid=attempt_id: self._goal_response(future, aid)
            )
        except Exception as exc:
            self.goal_pending = False
            self._goal_terminal = True
            self._handle_navigation_failure(
                f'Goal submission failed: {exc}', attempt_id=attempt_id
            )
            return

        total = int(self._param('max_navigation_retries')) + 1
        verb = 'Resending' if self._goal_attempts > 1 else 'Sending'
        self.get_logger().info(
            f'[NAV] {verb} goal {self.current_goal_label} '
            f'({x:.3f}, {y:.3f}), attempt {self._goal_attempts}/{total}'
        )

    # ========================================================
    # 12. NAV2 GOAL ACCEPTANCE CALLBACK
    # ========================================================

    def _account_for_stale_goal_response(self, future, attempt_id):
        """Cancel and retain any goal accepted by a stale send callback."""
        try:
            handle = future.result()
        except Exception as exc:
            self.get_logger().warn(
                f'[NAV] Stale attempt {attempt_id} response failed: {exc}'
            )
            return
        if handle is None or not handle.accepted:
            return

        self.get_logger().error(
            f'[NAV] Late acceptance for stale attempt {attempt_id}; canceling it'
        )
        self._stale_goal_handles[attempt_id] = handle
        try:
            handle.get_result_async().add_done_callback(
                lambda _future, aid=attempt_id:
                self._stale_goal_handles.pop(aid, None)
            )
            handle.cancel_goal_async()
        except Exception as exc:
            self.get_logger().error(
                f'[NAV] Unable to reconcile stale attempt {attempt_id}: {exc}'
            )

    def _goal_response(self, future, attempt_id):
        """
        Triggered when Nav2 responds to the goal request.

        This does NOT mean the robot has reached the target.

        It only means Nav2 accepted or rejected the goal.
        """

        if attempt_id != self._active_attempt_id:
            self._account_for_stale_goal_response(future, attempt_id)
            return

        self.goal_pending = False
        self._goal_send_future = None

        try:
            self.goal_handle = future.result()
        except Exception as exc:
            self._goal_terminal = True
            if self.state in (TaskState.RECOVERY_CANCEL, TaskState.CANCELLING):
                self._maybe_finish_cancellation(attempt_id)
            else:
                self._handle_navigation_failure(
                    f'Goal request failed: {exc}', attempt_id=attempt_id
                )
            return

        if self.goal_handle is None or not self.goal_handle.accepted:
            self.goal_handle = None
            self._goal_terminal = True
            reason = f'Nav2 rejected goal {self.current_goal_label}'
            self.get_logger().warn(f'[NAV] {reason}')
            if self.state in (TaskState.RECOVERY_CANCEL, TaskState.CANCELLING):
                self._cancel_reason = self._cancel_reason or reason
                self._maybe_finish_cancellation(attempt_id)
                return

            startup_delay = None
            if (
                self.goal_index == 0
                and self._goal_attempts <= int(self._param('startup_goal_retries'))
            ):
                startup_delay = self._param('startup_goal_retry_delay_seconds')
            self._handle_navigation_failure(
                reason,
                attempt_id=attempt_id,
                retry_delay=startup_delay
            )
            return

        self.get_logger().info(
            f'[NAV] Goal {self.current_goal_label} accepted, attempt '
            f'{self._goal_attempts}/{int(self._param("max_navigation_retries")) + 1}'
        )
        result_registered = self._register_result_callback(attempt_id)
        if self.state in (TaskState.RECOVERY_CANCEL, TaskState.CANCELLING):
            self._issue_cancel_request(attempt_id)
            return
        if not result_registered:
            self._handle_navigation_failure(
                'Unable to retrieve goal result', attempt_id=attempt_id
            )
            return
        if self.state is not TaskState.NAVIGATING:
            self._begin_cancellation(
                'Goal accepted after task left navigation state', for_retry=False
            )
            return
        self._state_deadline = self._now() + self._param('navigation_timeout_seconds')

    # ========================================================
    # 13. NAV2 NAVIGATION RESULT CALLBACK
    # ========================================================

    def _finish_goal_success(self):
        """Verify Nav2 success before entering the existing dwell phase."""
        if self._overall_deadline is not None and self._now() >= self._overall_deadline:
            self._fail('Overall task limit expired before success verification')
            return
        p = (
            self._last_amcl_pose.pose.pose.position
            if self._last_amcl_pose is not None else None
        )
        if (
            p is None
            or math.hypot(
                p.x - self.goal_target[0], p.y - self.goal_target[1]
            ) > self._param('goal_xy_tolerance')
        ):
            self._handle_navigation_failure(
                'Nav2 reported success outside target tolerance'
            )
            return

        self.get_logger().info(f'[NAV] Goal {self.current_goal_label} succeeded')
        self.get_logger().info(f'[TASK] Dwell at {self.current_goal_label}')
        self.stop()
        self._dwell_deadline = None
        self._set_state(TaskState.DWELLING)

    def _goal_result(self, future, attempt_id):
        """
        Triggered when Nav2 finishes navigation.

        Possible outcomes:
        - SUCCEEDED
        - ABORTED
        - CANCELED
        """

        if attempt_id != self._active_attempt_id:
            self._stale_goal_handles.pop(attempt_id, None)
            return

        try:
            status = future.result().status
        except Exception as exc:
            self._goal_result_future = None
            if self.state in (TaskState.RECOVERY_CANCEL, TaskState.CANCELLING):
                self.get_logger().warn(
                    f'[NAV] Goal result unavailable during cancellation: {exc}'
                )
                return
            self._handle_navigation_failure(
                f'Goal result failed: {exc}', attempt_id=attempt_id
            )
            return

        self._goal_result_future = None
        self._goal_terminal = True
        self._cancel_result_status = status
        self.goal_handle = None

        # A result arriving after a terminal decision is accounted for, but
        # can neither revive the mission nor schedule recovery.
        if self.state in (TaskState.FAILED, TaskState.COMPLETED):
            self._clear_attempt(attempt_id)
            return

        if self.state in (TaskState.RECOVERY_CANCEL, TaskState.CANCELLING):
            self._maybe_finish_cancellation(attempt_id)
            return

        self._clear_attempt(attempt_id)
        if status == GoalStatus.STATUS_SUCCEEDED:
            self._finish_goal_success()
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().warn(f'[NAV] Goal {self.current_goal_label} aborted')
            self._handle_navigation_failure(
                f'Goal {self.current_goal_label} aborted'
            )
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warn(
                f'[NAV] Goal {self.current_goal_label} canceled unexpectedly'
            )
            self._handle_navigation_failure(
                f'Goal {self.current_goal_label} canceled unexpectedly'
            )
        else:
            self._handle_navigation_failure(
                f'Navigation returned unexpected action status {status}'
            )

    # ========================================================
    # 14. MAIN TIMER CALLBACK — TASK STATE MACHINE
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
            TaskState.RECOVERY_WAIT,
            TaskState.RECOVERY_CHECK,
            TaskState.RECOVERY_CANCEL,
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
                TaskState.FAILED,
                TaskState.CANCELLING
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
            # AMCL must not receive an initial pose before the supplied EKF has
            # started publishing odom -> base_link; that race generated the
            # observed future-extrapolation warnings.
            action_ready = self.client.wait_for_server(timeout_sec=0.0)
            odom_ready = self._odometry_tf_ready()
            if action_ready and odom_ready:

                self.get_logger().info(
                    '[STARTUP] NavigateToPose action server available; '
                    'odom -> base_link available'
                )

                # Start overall mission timer
                self._overall_deadline = (
                    now + self._param('overall_timeout_seconds')
                )

                # Initialize robot starting position
                self.get_logger().info('[AMCL] Publishing initial pose')
                self._publish_initial_pose()

                # Enter localization initialization state
                self._set_state(
                    TaskState.INITIALIZING_LOCALIZATION,
                    self._param('localization_timeout_seconds')
                )

            elif not action_ready:
                self.get_logger().info(
                    '[STARTUP] Waiting for NavigateToPose action server',
                    throttle_duration_sec=5.0
                )
            else:
                self.get_logger().info(
                    '[ODOM] Waiting for odom -> base_link transform',
                    throttle_duration_sec=5.0
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
                self.get_logger().info('[AMCL] Localization ready')
                self._set_state(
                    TaskState.WAITING_FOR_NAV2_LIFECYCLE,
                    self._param('nav2_activation_timeout_seconds')
                )
                return

            # Check localization timeout
            if now >= self._state_deadline:

                self._fail(
                    'AMCL did not converge within localization timeout'
                )
                return

            # Repeated initialpose messages reset AMCL and were the source of
            # repeated transform-future warnings in the physical run.
            if self._param('republish_initial_pose') and now - self._last_initial_pose >= 1.0:

                self._publish_initial_pose()

            # Continue waiting for localization
            self.state = TaskState.WAITING_FOR_LOCALIZATION

        # ----------------------------------------------------
        # F. STATE: WAITING FOR NAV2 LIFECYCLE ACTIVATION
        # ----------------------------------------------------

        elif self.state is TaskState.WAITING_FOR_NAV2_LIFECYCLE:

            if now >= self._state_deadline:
                self._fail('Nav2 lifecycle activation timeout')
                return

            if not self._nav2_lifecycle_ready(now):
                return

            self.get_logger().info('[NAV2] All required navigation nodes ACTIVE')
            self._send_next_goal()

        # ----------------------------------------------------
        # G. STATE: NAVIGATING
        # ----------------------------------------------------

        elif self.state is TaskState.NAVIGATING:

            if now >= self._state_deadline:
                reason = (
                    'Navigation goal acceptance timeout'
                    if self.goal_pending
                    else 'Navigation execution timeout'
                )
                self._handle_navigation_failure(
                    reason, attempt_id=self._active_attempt_id
                )

        # ----------------------------------------------------
        # H. STATES: RECOVERY WAIT / READINESS / CANCELLATION
        # ----------------------------------------------------

        elif self.state is TaskState.RECOVERY_WAIT:

            if now >= self._goal_retry_at:
                self._begin_recovery_check(now)

        elif self.state is TaskState.RECOVERY_CHECK:

            if self._recovery_ready(now):
                self.get_logger().info('[RECOVERY] Nav2 ready')
                self._goal_retry_at = None
                self._send_next_goal()
                return
            if now >= self._state_deadline:
                self._log_recovery_diagnostics(now, force=True)
                blockers = '; '.join(self._recovery_blockers) or 'unknown'
                self._fail(
                    f'Recovery readiness timeout; blockers: {blockers}'
                )

        elif self.state is TaskState.RECOVERY_CANCEL:

            if now >= self._state_deadline:
                self.get_logger().error(
                    '[RECOVERY] Cancellation timeout; old goal status is '
                    'ambiguous, so no replacement goal will be sent'
                )
                self._terminal_failure_reason = 'Recovery cancellation timeout'
                self._set_state(TaskState.FAILED)

        # ----------------------------------------------------
        # I. STATE: TERMINAL CANCELLING
        # ----------------------------------------------------

        elif self.state is TaskState.CANCELLING:

            # Check cancellation timeout
            if now >= self._state_deadline:

                self.get_logger().error(
                    '[TASK] Terminal cancellation timeout; command stop '
                    'remains active'
                )

                self._set_state(
                    TaskState.FAILED
                )

        # ----------------------------------------------------
        # J. STATE: DWELLING
        # ----------------------------------------------------

        elif self.state is TaskState.DWELLING:

            # Check whether robot has stopped
            if not self._robot_stopped():
                # Dwell means continuously stationary for the full period.
                self._dwell_deadline = None
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
                self._goal_attempts = 0
                self._navigation_retries = 0

                if self.goal_index < len(self.task['targets']):
                    self.get_logger().info(
                        f'[TASK] Continuing to '
                        f'{self.task["targets"][self.goal_index]}'
                    )

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

        # Any callback that happens to be queued during shutdown is terminal
        # and must never schedule recovery.
        node._shutdown_requested = True

        # Request robot stop before shutdown
        node.stop()

        # Destroy ROS node
        node.destroy_node()

        # Shut down ROS 2
        if rclpy.ok():
            rclpy.shutdown()
