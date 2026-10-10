"""YAML task parsing independent of ROS, for validation before motors move."""
import yaml
from .landmark_converter import validate_label, BEARINGS_DEG


def validate_task(data):
    if not isinstance(data, dict) or not isinstance(data.get('start'), dict):
        raise ValueError('task requires a start mapping')
    start = data['start']
    square = validate_label(start.get('square', ''))
    bearing = str(start.get('bearing', '')).upper()
    if bearing not in BEARINGS_DEG:
        raise ValueError('start.bearing must be one of N, NE, E, SE, S, SW, W, NW')
    targets = data.get('targets')
    if not isinstance(targets, list) or len(targets) != 4:
        raise ValueError('task.targets must contain exactly four labels')
    return {'start': {'square': square, 'bearing': bearing},
            'targets': [validate_label(target) for target in targets]}


def load_task(path):
    with open(path, encoding='utf-8') as stream:
        return validate_task(yaml.safe_load(stream))
