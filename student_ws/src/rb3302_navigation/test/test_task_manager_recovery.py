"""Focused state-machine tests using mocked ROS action futures."""

from types import SimpleNamespace

from action_msgs.msg import GoalStatus

from rb3302_navigation.task_manager import TaskManager, TaskState


class FakeLogger:
    def debug(self, *_args, **_kwargs): pass
    def info(self, *_args, **_kwargs): pass
    def warn(self, *_args, **_kwargs): pass
    def error(self, *_args, **_kwargs): pass


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
    }
    node.get_logger = lambda: FakeLogger()
    node._now = lambda: clock[0]
    node._param = lambda name: params[name]
    node.stop = lambda: None
    node.state = TaskState.NAVIGATING
    node.task = {'targets': ['H', 'J', 'K', 'X']}
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
    return node, clock


def result_with(status):
    return FakeFuture(SimpleNamespace(status=status))


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


def test_execution_timeout_waits_for_cancel_ack_and_terminal_result():
    node, _ = make_manager()
    handle = FakeGoalHandle()
    node.goal_handle = handle
    node._goal_result_future = handle.result_future

    node._handle_navigation_failure('Navigation execution timeout', attempt_id=1)

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
    sent = []
    node._send_next_goal = lambda: sent.append(True)
    node._overall_deadline = None
    node.tick()
    assert node.goal_index == 2

    clock[0] = 4.0
    node.tick()
    assert node.goal_index == 3
    assert node._goal_attempts == 0
    assert node._navigation_retries == 0
    assert sent == [True]


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

    node._handle_navigation_failure('sixth attempt aborted', attempt_id=1)

    assert node.state is TaskState.FAILED
    assert node.goal_index == 2

