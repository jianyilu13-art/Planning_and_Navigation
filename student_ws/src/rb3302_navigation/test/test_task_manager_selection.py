"""Deterministic tests for time-aware waypoint selection policy."""

import itertools
import math
from types import SimpleNamespace

from rb3302_navigation.task_manager import TaskManager, TaskState, WaypointState


class Logger:
    def __init__(self):
        self.messages = []

    def debug(self, message, **_kwargs): self.messages.append(message)
    def info(self, message, **_kwargs): self.messages.append(message)
    def warn(self, message, **_kwargs): self.messages.append(message)
    def error(self, message, **_kwargs): self.messages.append(message)
    def fatal(self, message, **_kwargs): self.messages.append(message)


class Converter:
    points = {
        'H': (0.0, 0.0),
        'J': (1.0, 0.0),
        'K': (2.0, 0.0),
        'X': (3.0, 0.0),
    }

    def coordinate(self, label):
        return self.points[label]

    def bearing_yaw(self, bearing):
        return {'N': math.pi / 2, 'E': 0.0, 'S': -math.pi / 2, 'W': math.pi}[bearing]


class UnavailablePathClient:
    def wait_for_server(self, timeout_sec=0.0):
        del timeout_sec
        return False


def manager():
    node = TaskManager.__new__(TaskManager)
    clock = [0.0]
    params = {
        'overall_timeout_seconds': 180.0,
        'straight_speed': 0.20,
        'effective_angular_speed': 0.50,
        'significant_turn_threshold_degrees': 30.0,
        'turn_penalty_seconds': 2.0,
        'path_simplification_tolerance': 0.03,
        'dwell_seconds': 3.0,
        'navigation_safety_margin_seconds': 5.0,
        'mission_time_reserve_seconds': 10.0,
        'waypoint_time_budget_seconds': 35.0,
        'selection_retry_limit': 1,
        'path_estimation_timeout_seconds': 2.0,
        'failure_penalty_seconds': 5.0,
        'max_navigation_retries': 5,
        'retry_delay_seconds': 2.0,
    }
    node._now = lambda: clock[0]
    node._param = lambda name: params[name]
    node._logger = Logger()
    node.get_logger = lambda: node._logger
    node.stop = lambda: None
    node.get_clock = lambda: SimpleNamespace(
        now=lambda: SimpleNamespace(to_msg=lambda: SimpleNamespace())
    )
    node.task = {
        'start': {'square': 'H', 'bearing': 'N'},
        'targets': ['H', 'J', 'K', 'X'],
    }
    node.converter = Converter()
    node.state = TaskState.SELECTING_GOAL
    node.goal_index = 0
    node.current_goal_label = None
    node.goal_target = None
    node.goal_pending = False
    node.goal_handle = None
    node._active_attempt_id = None
    node._goal_send_future = None
    node._goal_result_future = None
    node._cancel_future = None
    node._goal_terminal = False
    node._shutdown_requested = False
    node._overall_deadline = 180.0
    node._state_deadline = None
    node._selection_cycle = 0
    node._active_selection_cycle = None
    node._selection_pending_edges = set()
    node._selection_distances = {}
    node._selection_profiles = {}
    node._selection_path_edges = set()
    node._selection_blocked_edges = set()
    node._selection_goal_handles = {}
    node._selection_deadline = None
    node._selection_reason = None
    node._path_distance_cache = {}
    node._path_profile_cache = {}
    node._selection_initial_heading = None
    node._last_amcl_pose = None
    node._waypoint_status = {
        label: WaypointState.PENDING for label in node.task['targets']
    }
    node._waypoint_failures = {label: 0 for label in node.task['targets']}
    node._waypoint_scores = {label: 1.0 for label in node.task['targets']}
    node._completed_waypoints = []
    node._waypoint_started_at = None
    node._waypoint_deadline = None
    node._goal_attempts = 0
    node._navigation_retries = 0
    node._goal_retry_at = None
    node.path_client = UnavailablePathClient()
    return node, clock


def distances(labels, default=1.0):
    result = {(None, label): default for label in labels}
    result.update({edge: default for edge in itertools.permutations(labels, 2)})
    return result


def test_all_four_reachable_selects_four_target_sequence():
    node, _ = manager()
    best = node._evaluate_candidate_sequences(
        node.task['targets'], distances(node.task['targets']), set(), 100.0
    )
    assert best is not None
    assert len(best[0]) == 4


def test_far_waypoint_is_excluded_by_per_waypoint_budget():
    node, _ = manager()
    values = distances(node.task['targets'])
    for edge in list(values):
        if 'X' in edge:
            values[edge] = 10.0
    best = node._evaluate_candidate_sequences(
        node.task['targets'], values, set(), 100.0
    )
    assert 'X' not in best[0]
    assert len(best[0]) == 3


def test_insufficient_time_selects_only_feasible_prefix():
    node, _ = manager()
    best = node._evaluate_candidate_sequences(
        node.task['targets'], distances(node.task['targets']), set(), 20.0
    )
    assert len(best[0]) == 1


def test_blocked_nearest_waypoint_selects_next_feasible_target():
    node, _ = manager()
    values = distances(node.task['targets'])
    values[(None, 'H')] = 0.1
    values[(None, 'J')] = 0.5
    best = node._evaluate_candidate_sequences(
        node.task['targets'], values, {(None, 'H')}, 20.0
    )
    assert best[0][0] == 'J'


def test_longer_first_leg_wins_when_it_enables_more_reward():
    node, _ = manager()
    labels = ['H', 'J', 'K']
    values = distances(labels, default=10.0)
    values[(None, 'H')] = 0.1
    values[(None, 'J')] = 1.0
    values[(None, 'K')] = 10.0
    values[('J', 'K')] = 1.0
    best = node._evaluate_candidate_sequences(labels, values, set(), 40.0)
    assert best[0] == ('J', 'K')
    assert best[2] == 2.0


def test_repeated_failure_uses_policy_skip_instead_of_all_five_retries():
    node, _ = manager()
    node.current_goal_label = 'H'
    node._waypoint_status['H'] = WaypointState.ACTIVE
    node._navigation_retries = 1
    node._waypoint_deadline = 35.0
    skipped = []
    node._skip_active_waypoint = lambda reason: skipped.append(reason)
    node._schedule_navigation_retry('controller failure')
    assert skipped
    assert 'selection-policy retry limit reached' in skipped[0]


def test_waypoint_budget_expiry_requests_skip_when_no_goal_is_active():
    node, clock = manager()
    node.state = TaskState.RECOVERY_WAIT
    node.current_goal_label = 'H'
    node._waypoint_status['H'] = WaypointState.ACTIVE
    node._waypoint_deadline = 5.0
    clock[0] = 6.0
    skipped = []
    node._skip_active_waypoint = lambda reason: skipped.append(reason)
    node.tick()
    assert skipped == ['waypoint time budget exhausted']


def test_skipped_waypoint_is_not_completed():
    node, _ = manager()
    node.current_goal_label = 'H'
    node._waypoint_status['H'] = WaypointState.ACTIVE
    node._begin_goal_selection = lambda _reason: None
    node._skip_active_waypoint('test skip')
    assert node._waypoint_status['H'] is WaypointState.SKIPPED
    assert 'H' not in node._completed_waypoints


def test_completed_waypoint_is_never_selected_again():
    node, _ = manager()
    node._waypoint_status['H'] = WaypointState.COMPLETED
    selected = []
    node._send_next_goal = lambda: selected.append(node.current_goal_label)
    node._begin_goal_selection('after H')
    assert selected
    assert selected[0] != 'H'


def test_no_feasible_target_ends_safely_without_completion_credit():
    node, _ = manager()
    node._selection_cycle = 1
    node._active_selection_cycle = 1
    node._selection_distances = distances(node.task['targets'], default=10.0)
    node._selection_blocked_edges = set()
    node._selection_path_edges = set()
    node._selection_reason = 'test'
    node._finalize_goal_selection(1)
    assert node.state is TaskState.COMPLETED
    assert not node._completed_waypoints
    assert all(
        status is WaypointState.SKIPPED
        for status in node._waypoint_status.values()
    )


def test_all_completed_targets_end_mission_without_new_goal():
    node, _ = manager()
    node._waypoint_status = {
        label: WaypointState.COMPLETED for label in node.task['targets']
    }
    node._completed_waypoints = list(node.task['targets'])
    node._begin_goal_selection('all done')
    assert node.state is TaskState.COMPLETED
    assert len(node._completed_waypoints) == 4


def test_deadline_during_navigation_is_terminal():
    node, clock = manager()
    node.state = TaskState.NAVIGATING
    node._overall_deadline = 5.0
    clock[0] = 6.0
    failures = []
    node._fail = lambda reason: failures.append(reason)
    node.tick()
    assert failures == ['Overall task limit expired']


def test_deadline_during_recovery_cancellation_is_terminal():
    node, clock = manager()
    node.state = TaskState.RECOVERY_CANCEL
    node._overall_deadline = 5.0
    clock[0] = 6.0
    failures = []
    node._fail = lambda reason: failures.append(reason)
    node.tick()
    assert failures == ['Overall task limit expired']


def test_higher_score_sequence_has_priority_over_shorter_time():
    node, _ = manager()
    node._waypoint_scores['X'] = 3.0
    values = distances(node.task['targets'])
    values[(None, 'X')] = 2.0
    best = node._evaluate_candidate_sequences(
        node.task['targets'], values, set(), 25.0
    )
    assert best[0][0] == 'X'
    assert math.isclose(best[2], 3.0)


def test_straight_path_has_no_turn_cost():
    node, _ = manager()
    profile = node._analyze_path([(0.0, 0.0), (0.5, 0.0), (1.0, 0.0)])
    breakdown = node._leg_time_breakdown(
        profile['distance'], 'H', profile, incoming_heading=0.0
    )
    assert math.isclose(profile['distance'], 1.0)
    assert profile['turn_count'] == 0
    assert math.isclose(breakdown['turning_time'], 0.0, abs_tol=1e-9)
    assert math.isclose(breakdown['penalty_time'], 0.0, abs_tol=1e-9)


def test_single_ninety_degree_turn_is_counted_once():
    node, _ = manager()
    profile = node._analyze_path([(0.0, 0.0), (1.0, 0.0), (1.0, 1.0)])
    assert profile['turn_count'] == 1
    assert math.isclose(profile['turn_angle'], math.pi / 2, rel_tol=1e-6)


def test_two_ninety_degree_turns_are_counted_separately():
    node, _ = manager()
    profile = node._analyze_path([
        (0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0)
    ])
    assert profile['turn_count'] == 2
    assert math.isclose(profile['turn_angle'], math.pi, rel_tol=1e-6)


def test_one_hundred_eighty_degree_reversal_is_one_turn():
    node, _ = manager()
    profile = node._analyze_path([(0.0, 0.0), (1.0, 0.0), (0.0, 0.0)])
    assert profile['turn_count'] == 1
    assert math.isclose(profile['turn_angle'], math.pi, rel_tol=1e-6)


def test_minor_path_fluctuations_do_not_create_turns():
    node, _ = manager()
    profile = node._analyze_path([
        (0.0, 0.0), (0.25, 0.01), (0.5, -0.01),
        (0.75, 0.01), (1.0, 0.0),
    ])
    assert profile['turn_count'] == 0
    assert math.isclose(profile['turn_angle'], 0.0, abs_tol=1e-9)


def test_smooth_curve_is_merged_into_one_significant_turn():
    node, _ = manager()
    points = [
        (math.sin(angle), 1.0 - math.cos(angle))
        for angle in [index * math.pi / 40 for index in range(21)]
    ]
    profile = node._analyze_path(points)
    assert profile['turn_count'] == 1
    assert math.radians(45.0) < profile['turn_angle'] < math.pi / 2


def test_missing_path_estimate_is_explicitly_uncertain():
    node, _ = manager()
    submitted = []
    node._send_next_goal = lambda: submitted.append(node.current_goal_label)
    node._begin_goal_selection('planner unavailable')
    assert submitted
    assert any('obstacle-unaware' in message for message in node._logger.messages)
    assert any('uncertain=true' in message for message in node._logger.messages)


def test_initial_robot_heading_misalignment_adds_turn_cost():
    node, _ = manager()
    profile = node._analyze_path([(0.0, 0.0), (1.0, 0.0)])
    breakdown = node._leg_time_breakdown(
        profile['distance'], 'H', profile, incoming_heading=math.pi / 2
    )
    assert breakdown['turn_count'] == 1
    assert math.isclose(breakdown['turn_angle'], math.pi / 2, rel_tol=1e-6)
    assert math.isclose(breakdown['turning_time'], math.pi, rel_tol=1e-6)
    assert math.isclose(breakdown['penalty_time'], 2.0, rel_tol=1e-6)
