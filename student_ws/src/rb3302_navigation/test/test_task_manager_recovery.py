"""Focused state-machine tests using mocked ROS action futures."""

import math
from types import SimpleNamespace

from action_msgs.msg import GoalStatus

from rb3302_navigation.task_manager import TaskManager, TaskState, WaypointState


class FakeLogger:
    def __init__(self):
        self.messages = {'debug': [], 'info': [], 'warn': [], 'error': []}

    def debug(self, message, **_kwargs): self.messages['debug'].append(message)
    def info(self, message, **_kwargs): self.messages['info'].append(message)
    def warn(self, message, **_kwargs): self.messages['warn'].append(message)
    def error(self, message, **_kwargs): self.messages['error'].append(message)


class FakeFuture:
    def __init__(self, value=None, *, done=True, error=None):
        self.value = value
        self._done = done
        self.error = error
        self.callbacks = []

    def done(self):
        return self._done

    def result(self):
        if self.error is not None:
            raise self.error
        return self.value

    def add_done_callback(self, callback):
        self.callbacks.append(callback)


class FakeGoalHandle:
    def __init__(self, accepted=True):
        self.accepted = accepted
        self.result_future = FakeFuture(done=False)
        self.cancel_future = FakeFuture(
            SimpleNamespace(goals_canceling=[object()]), done=False
        )
        self.cancel_calls = 0

    def get_result_async(self):
        return self.result_future

    def cancel_goal_async(self):
        self.cancel_calls += 1
        return self.cancel_future


class FakeActionClient:
    def __init__(self, available=True):
        self.available = available

    def wait_for_server(self, timeout_sec=0.0):
        del timeout_sec
        return self.available


class FakeTfBuffer:
    def __init__(self, stamps):
        self.stamps = stamps

    def lookup_transform(self, target, source, *_args, **_kwargs):
        value = self.stamps[(target, source)]
        if isinstance(value, Exception):
            raise value
        sec = int(value)
        nanosec = int(round((value - sec) * 1e9))
        return SimpleNamespace(
            header=SimpleNamespace(
                stamp=SimpleNamespace(sec=sec, nanosec=nanosec)
            )
        )


class FakeLifecycleClient:
    def __init__(self):
        self.calls = 0

    def service_is_ready(self):
        return True

    def call_async(self, _request):
        self.calls += 1
        return FakeFuture(done=False)


class FakeConverter:
    coordinates = {
        'K': (2.7, 1.5),
        'H': (20.0, 20.0),
        'J': (21.0, 20.0),
        'X': (22.0, 20.0),
    }

    def coordinate(self, label):
        return self.coordinates[label]

    def bearing_yaw(self, _bearing):
        return math.pi / 2


def make_manager():
    node = TaskManager.__new__(TaskManager)
    clock = [0.0]
    params = {
        'max_navigation_retries': 5,
        'retry_delay_seconds': 2.0,
        'cancellation_timeout_seconds': 5.0,
        'startup_goal_retries': 2,
        'startup_goal_retry_delay_seconds': 1.0,
        'goal_xy_tolerance': 0.15,
        'dwell_seconds': 3.0,
        'localization_xy_variance_limit': 0.25,
        'localization_yaw_variance_limit': 0.8,
        'recovery_tf_max_age_seconds': 2.0,
        'recovery_tf_future_tolerance_seconds': 1.5,
        'recovery_diagnostic_period_seconds': 2.0,
        'recovery_readiness_timeout_seconds': 30.0,
        'lifecycle_check_period_seconds': 0.5,
        'lifecycle_request_timeout_seconds': 2.0,
        'straight_speed': 0.20,
        'effective_angular_speed': 0.50,
        'significant_turn_threshold_degrees': 30.0,
        'turn_penalty_seconds': 2.0,
        'path_simplification_tolerance': 0.03,
        'navigation_safety_margin_seconds': 5.0,
        'mission_time_reserve_seconds': 10.0,
        'selection_retry_limit': 1,
        'path_estimation_timeout_seconds': 2.0,
        'failure_penalty_seconds': 5.0,
        'progress_window_seconds': 20.0,
        'progress_min_distance_meters': 0.10,
        'progress_min_odom_displacement_meters': 0.10,
        'progress_diagnostic_period_seconds': 5.0,
    }
    node._test_logger = FakeLogger()
    node.get_logger = lambda: node._test_logger
    node._now = lambda: clock[0]
    node._param = lambda name: params[name]
    node.stop = lambda: None
    node.state = TaskState.NAVIGATING
    node.task = {
        'start': {'square': 'K', 'bearing': 'N'},
        'targets': ['H', 'J', 'K', 'X'],
    }
    node.converter = FakeConverter()
    node.goal_index = 2
    node.current_goal_label = 'K'
    node.goal_target = (2.7, 1.5)
    node.goal_handle = None
    node.goal_pending = False
    node._goal_attempts = 1
    node._navigation_retries = 0
    node._goal_retry_at = None
    node._next_attempt_id = 1
    node._active_attempt_id = 1
    node._goal_send_future = None
    node._goal_result_future = None
    node._cancel_future = None
    node._goal_terminal = False
    node._cancel_for_retry = False
    node._cancel_reason = None
    node._cancel_result_status = None
    node._stale_goal_handles = {}
    node._recovery_reason = None
    node._terminal_failure_reason = None
    node._shutdown_requested = False
    node._overall_deadline = 180.0
    node._state_deadline = 120.0
    node._dwell_deadline = None
    node._last_amcl_pose = None
    node._last_odom = None
    node._initial_pose_stamp_ns = 0
    node._recovery_check_started_at = 0.0
    node._recovery_blockers = []
    node._recovery_readiness_details = ''
    node._last_recovery_diagnostic_log = float('-inf')
    node._lifecycle_clients = {}
    node._lifecycle_futures = {}
    node._lifecycle_states = {}
    node._lifecycle_query_started = {}
    node._lifecycle_state_query_started = {}
    node._last_lifecycle_refresh = float('-inf')
    node._last_lifecycle_wait_log = float('-inf')
    node.client = FakeActionClient()
    node._waypoint_status = {
        'H': WaypointState.PENDING,
        'J': WaypointState.PENDING,
        'K': WaypointState.ACTIVE,
        'X': WaypointState.PENDING,
    }
    node._waypoint_failures = {label: 0 for label in node.task['targets']}
    node._waypoint_scores = {label: 1.0 for label in node.task['targets']}
    node._completed_waypoints = []
    node._path_distance_cache = {}
    node._path_profile_cache = {}
    node._active_selection_cycle = None
    node._selection_goal_handles = {}
    node._selection_pending_edges = set()
    node._waypoint_started_at = 0.0
    node._progress_samples = []
    node._progress_monitor_started_at = None
    node._latest_nav_distance_remaining = None
    node._latest_feedback_position = None
    node._latest_nav_recoveries = 0
    node._last_progress_diagnostic_log = float('-inf')
    return node, clock


def result_with(status):
    return FakeFuture(SimpleNamespace(status=status))


def amcl_pose(stamp, covariance=(0.02, 0.03, 0.1)):
    values = [0.0] * 36
    values[0], values[7], values[35] = covariance
    sec = int(stamp)
    nanosec = int(round((stamp - sec) * 1e9))
    return SimpleNamespace(
        header=SimpleNamespace(
            frame_id='map',
            stamp=SimpleNamespace(sec=sec, nanosec=nanosec),
        ),
        pose=SimpleNamespace(covariance=values),
    )


def odom_pose(x, y):
    return SimpleNamespace(
        pose=SimpleNamespace(
            pose=SimpleNamespace(position=SimpleNamespace(x=x, y=y))
        )
    )


def navigation_feedback(distance, x, y=1.5, recoveries=0):
    return SimpleNamespace(
        feedback=SimpleNamespace(
            distance_remaining=distance,
            current_pose=SimpleNamespace(
                pose=SimpleNamespace(position=SimpleNamespace(x=x, y=y))
            ),
            number_of_recoveries=recoveries,
        )
    )


def configure_ready_recovery(node, clock, pose_stamp=0.0):
    clock[0] = 10.0
    node._last_amcl_pose = amcl_pose(pose_stamp)
    node._last_odom = object()
    node.tf_buffer = FakeTfBuffer({
        ('map', 'odom'): 10.5,       # Expected AMCL future dating.
        ('odom', 'base_link'): 9.9,
        ('map', 'base_link'): 9.9,
    })
    node._nav2_lifecycle_ready = lambda _now, fresh_since=None: True


def test_abort_schedules_same_waypoint_without_incrementing_index():
    node, _ = make_manager()
    node._goal_result_future = result_with(GoalStatus.STATUS_ABORTED)

    node._goal_result(node._goal_result_future, 1)

    assert node.state is TaskState.RECOVERY_WAIT
    assert node.goal_index == 2
    assert node.current_goal_label == 'K'
    assert node._goal_retry_at == 2.0
    assert node._navigation_retries == 0  # Counted only when resubmitted.


def test_rejection_uses_general_recovery_for_later_waypoint():
    node, _ = make_manager()
    node.goal_pending = True
    node._goal_send_future = FakeFuture(FakeGoalHandle(accepted=False))

    node._goal_response(node._goal_send_future, 1)

    assert node.state is TaskState.RECOVERY_WAIT
    assert node.goal_index == 2
    assert node._goal_retry_at == 2.0


def test_safe_progress_failure_waits_for_cancel_ack_and_terminal_result():
    node, _ = make_manager()
    handle = FakeGoalHandle()
    node.goal_handle = handle
    node._goal_result_future = handle.result_future

    node._handle_navigation_failure(
        'No meaningful navigation progress for 20.0s', attempt_id=1
    )

    assert node.state is TaskState.RECOVERY_CANCEL
    assert handle.cancel_calls == 1

    handle.cancel_future._done = True
    node._cancel_response(handle.cancel_future, 1)
    assert node.state is TaskState.RECOVERY_CANCEL

    handle.result_future.value = SimpleNamespace(
        status=GoalStatus.STATUS_CANCELED
    )
    handle.result_future._done = True
    node._goal_result(handle.result_future, 1)
    assert node.state is TaskState.RECOVERY_WAIT
    assert node.goal_index == 2


def test_continuous_progress_continues_beyond_thirty_five_seconds():
    node, clock = make_manager()
    node._reset_progress_monitor(0.0)
    failures = []
    node._handle_navigation_failure = (
        lambda reason, **_kwargs: failures.append(reason)
    )
    for timestamp, distance, x in (
        (0.0, 6.0, 0.0),
        (10.0, 5.0, 0.3),
        (20.0, 4.0, 0.6),
        (30.0, 3.0, 0.9),
        (45.0, 2.0, 1.2),
    ):
        clock[0] = timestamp
        node._last_odom = odom_pose(x, 0.0)
        node._navigation_feedback(
            navigation_feedback(distance, x), node._active_attempt_id
        )

    node.tick()

    assert failures == []
    assert node.state is TaskState.NAVIGATING


def test_temporary_stationary_turning_remains_inside_progress_grace():
    node, clock = make_manager()
    node._last_odom = odom_pose(0.0, 0.0)
    node._reset_progress_monitor(0.0)
    node._navigation_feedback(navigation_feedback(5.0, 0.0), 1)
    clock[0] = 15.0

    progressing, details = node._navigation_progress_status(clock[0])

    assert progressing is True
    assert 'observation grace' in details


def test_temporary_replanning_uses_odom_as_progress_evidence():
    node, clock = make_manager()
    node._last_odom = odom_pose(0.0, 0.0)
    node._reset_progress_monitor(0.0)
    node._navigation_feedback(navigation_feedback(5.0, 0.0), 1)
    clock[0] = 10.0
    node._navigation_feedback(
        navigation_feedback(7.0, 0.0, recoveries=1), 1
    )
    clock[0] = 21.0
    node._last_odom = odom_pose(0.25, 0.0)
    node._navigation_feedback(
        navigation_feedback(6.0, 0.0, recoveries=1), 1
    )

    progressing, details = node._navigation_progress_status(clock[0])

    assert progressing is True
    assert 'nav_reduction=-1.000m' in details
    assert 'odom_displacement=0.250m' in details


def test_sustained_lack_of_progress_triggers_policy_failure():
    node, clock = make_manager()
    node._last_odom = odom_pose(0.0, 0.0)
    node._reset_progress_monitor(0.0)
    node._navigation_feedback(navigation_feedback(5.0, 0.0), 1)
    failures = []
    node._handle_navigation_failure = (
        lambda reason, **_kwargs: failures.append(reason)
    )
    clock[0] = 21.0

    node.tick()

    assert failures == ['No meaningful navigation progress for 20.0s']


def test_stalled_goal_is_safely_canceled_before_replacement():
    node, clock = make_manager()
    handle = FakeGoalHandle()
    node.goal_handle = handle
    node._goal_result_future = handle.result_future
    node._last_odom = odom_pose(0.0, 0.0)
    node._reset_progress_monitor(0.0)
    node._navigation_feedback(navigation_feedback(5.0, 0.0), 1)
    clock[0] = 21.0

    node.tick()

    assert node.state is TaskState.RECOVERY_CANCEL
    assert handle.cancel_calls == 1
    assert node._active_attempt_id == 1


def test_late_acceptance_is_tracked_and_canceled():
    node, _ = make_manager()
    node.state = TaskState.RECOVERY_CANCEL
    node.goal_pending = True
    handle = FakeGoalHandle()
    response = FakeFuture(handle)
    node._goal_send_future = response

    node._goal_response(response, 1)

    assert node.goal_handle is handle
    assert node._goal_result_future is handle.result_future
    assert node._cancel_future is handle.cancel_future
    assert handle.cancel_calls == 1


def test_acceptance_timeout_does_not_submit_blind_replacement():
    node, _ = make_manager()
    node.goal_pending = True
    node._goal_send_future = FakeFuture(done=False)

    node._handle_navigation_failure(
        'Navigation goal acceptance timeout', attempt_id=1
    )

    assert node.state is TaskState.RECOVERY_CANCEL
    assert node._active_attempt_id == 1
    assert node._goal_retry_at is None


def test_stale_late_acceptance_cannot_replace_current_attempt():
    node, _ = make_manager()
    node._active_attempt_id = 2
    handle = FakeGoalHandle()

    node._goal_response(FakeFuture(handle), 1)

    assert node._active_attempt_id == 2
    assert node.goal_handle is None
    assert node._stale_goal_handles[1] is handle
    assert handle.cancel_calls == 1


def test_cancellation_timeout_fails_without_sending_replacement():
    node, clock = make_manager()
    node.state = TaskState.RECOVERY_CANCEL
    node._state_deadline = 1.0
    node._overall_deadline = None
    clock[0] = 2.0

    node.tick()

    assert node.state is TaskState.FAILED
    assert node._active_attempt_id == 1


def test_late_result_after_terminal_failure_cannot_revive_mission():
    node, _ = make_manager()
    node.state = TaskState.FAILED
    success = result_with(GoalStatus.STATUS_SUCCEEDED)
    node._goal_result_future = success

    node._goal_result(success, 1)

    assert node.state is TaskState.FAILED
    assert node._active_attempt_id is None


def test_success_advances_only_after_verified_dwell():
    node, clock = make_manager()
    node._navigation_retries = 1
    node._last_amcl_pose = SimpleNamespace(
        pose=SimpleNamespace(
            pose=SimpleNamespace(position=SimpleNamespace(x=2.7, y=1.5))
        )
    )
    success = result_with(GoalStatus.STATUS_SUCCEEDED)
    node._goal_result_future = success

    node._goal_result(success, 1)
    assert node.state is TaskState.DWELLING
    assert node.goal_index == 2

    node._robot_stopped = lambda: True
    selections = []
    node._begin_goal_selection = lambda reason: selections.append(reason)
    node._overall_deadline = None
    node.tick()
    assert node.goal_index == 2

    clock[0] = 4.0
    node.tick()
    assert node.goal_index == 1
    assert node._goal_attempts == 0
    assert node._navigation_retries == 0
    assert node._waypoint_status['K'] is WaypointState.COMPLETED
    assert selections == ['K completed']


def test_shutdown_failure_never_schedules_retry():
    node, _ = make_manager()
    node._shutdown_requested = True
    node._goal_terminal = True

    node._handle_navigation_failure('shutdown', attempt_id=1)

    assert node.state is TaskState.FAILED


def test_retry_scheduling_preserves_overall_deadline():
    node, _ = make_manager()
    original_deadline = node._overall_deadline
    node._goal_terminal = True

    node._handle_navigation_failure('aborted', attempt_id=1)

    assert node.state is TaskState.RECOVERY_WAIT
    assert node._overall_deadline == original_deadline


def test_retry_limit_is_five_additional_submissions():
    node, _ = make_manager()
    node._goal_attempts = 6
    node._navigation_retries = 5
    node._goal_terminal = True

    skipped = []
    node._skip_active_waypoint = lambda reason: skipped.append(reason)
    node._handle_navigation_failure('sixth attempt aborted', attempt_id=1)

    assert node.goal_index == 2
    assert skipped == [
        'sixth attempt aborted; navigation retry limit reached'
    ]


def test_stationary_old_amcl_pose_is_allowed_with_fresh_tf():
    node, clock = make_manager()
    configure_ready_recovery(node, clock, pose_stamp=0.0)

    assert node._recovery_ready(clock[0]) is True
    assert 'amcl_pose_age=10.000s' in node._recovery_readiness_details
    assert node._recovery_blockers == []


def test_missing_and_stale_tf_are_reported_individually():
    node, clock = make_manager()
    configure_ready_recovery(node, clock)
    node.tf_buffer = FakeTfBuffer({
        ('map', 'odom'): RuntimeError('no map transform'),
        ('odom', 'base_link'): 6.0,
        ('map', 'base_link'): 9.9,
    })

    assert node._recovery_ready(clock[0]) is False
    assert 'map->odom unavailable' in node._recovery_blockers
    assert any('odom->base_link stale' in item for item in node._recovery_blockers)


def test_excessive_amcl_covariance_reports_value_and_threshold():
    node, clock = make_manager()
    configure_ready_recovery(node, clock)
    node._last_amcl_pose = amcl_pose(0.0, covariance=(0.3, 0.03, 0.9))

    assert node._recovery_ready(clock[0]) is False
    assert 'AMCL covariance x=0.3 exceeds 0.25' in node._recovery_blockers
    assert 'AMCL covariance yaw=0.9 exceeds 0.8' in node._recovery_blockers


def test_unavailable_action_server_does_not_hide_other_diagnostics():
    node, clock = make_manager()
    configure_ready_recovery(node, clock)
    node.client.available = False

    assert node._recovery_ready(clock[0]) is False
    assert 'NavigateToPose action server unavailable' in node._recovery_blockers
    assert 'amcl_pose_age=10.000s' in node._recovery_readiness_details
    assert 'map->odom=available' in node._recovery_readiness_details


def test_recovery_discards_stuck_pre_recovery_lifecycle_request():
    node, clock = make_manager()
    name = '/planner_server'
    node._lifecycle_clients = {name: FakeLifecycleClient()}
    node._lifecycle_futures = {name: FakeFuture(done=False)}
    node._lifecycle_states = {name: 3}
    node._lifecycle_query_started = {name: -10.0}
    node._lifecycle_state_query_started = {name: -10.0}

    node._begin_recovery_check(clock[0])

    assert node._lifecycle_futures[name] is None
    assert node._lifecycle_states[name] is None


def test_stuck_recovery_lifecycle_request_is_reissued():
    node, clock = make_manager()
    name = '/planner_server'
    client = FakeLifecycleClient()
    node._lifecycle_clients = {name: client}
    node._lifecycle_futures = {name: FakeFuture(done=False)}
    node._lifecycle_states = {name: None}
    node._lifecycle_query_started = {name: 0.0}
    node._lifecycle_state_query_started = {name: float('-inf')}
    node._last_lifecycle_refresh = float('-inf')
    clock[0] = 3.0

    node._refresh_lifecycle_states(clock[0])

    assert client.calls == 1
    assert node._lifecycle_futures[name] is not None
    assert node._lifecycle_query_started[name] == 3.0


def test_recovery_timeout_logs_complete_blocker_list():
    node, clock = make_manager()
    configure_ready_recovery(node, clock)
    node.client.available = False
    node.state = TaskState.RECOVERY_CHECK
    node._state_deadline = 9.0
    node._overall_deadline = None

    node.tick()

    assert node.state is TaskState.FAILED
    warnings = '\n'.join(node._test_logger.messages['warn'])
    errors = '\n'.join(node._test_logger.messages['error'])
    assert 'NavigateToPose action server unavailable' in warnings
    assert 'Recovery readiness timeout; blockers:' in errors
    assert 'NavigateToPose action server unavailable' in errors

