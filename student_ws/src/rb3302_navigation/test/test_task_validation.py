import pytest
from rb3302_navigation.task_validation import validate_task


def test_repeated_targets_are_preserved():
    task = validate_task({'start': {'square': 'a', 'bearing': 'ne'}, 'targets': ['B', 'B', 'X', 'B']})
    assert task['targets'] == ['B', 'B', 'X', 'B']


@pytest.mark.parametrize('task', [
    {'start': {'square': 'Z', 'bearing': 'N'}, 'targets': ['A'] * 4},
    {'start': {'square': 'A', 'bearing': 'UP'}, 'targets': ['A'] * 4},
    {'start': {'square': 'A', 'bearing': 'N'}, 'targets': ['A'] * 3},
])
def test_invalid_tasks_rejected(task):
    with pytest.raises(ValueError): validate_task(task)
