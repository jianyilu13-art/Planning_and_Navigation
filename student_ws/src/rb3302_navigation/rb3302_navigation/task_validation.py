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


def validate_landmark_config(data):
    """Reject placeholder or malformed calibration before any goal is sent."""
    if not isinstance(data, dict) or data.get('calibrated') is not True:
        raise ValueError('landmark calibration must explicitly set calibrated: true')
    grid = data.get('grid')
    if not isinstance(grid, dict):
        raise ValueError('landmark calibration requires a grid mapping')
    for key in ('origin_x', 'origin_y', 'spacing_m', 'rotation_deg'):
        try:
            grid[key] = float(grid[key])
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError(f'grid.{key} must be numeric') from exc
    if grid['spacing_m'] <= 0.0:
        raise ValueError('grid.spacing_m must be positive')
    overrides = data.get('overrides', {})
    if not isinstance(overrides, dict):
        raise ValueError('overrides must be a mapping')
    for label, point in overrides.items():
        validate_label(label)
        if not isinstance(point, dict):
            raise ValueError(f'override {label} must be a mapping')
        try:
            float(point['x']); float(point['y'])
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError(f'override {label} requires numeric x and y') from exc
    return data
