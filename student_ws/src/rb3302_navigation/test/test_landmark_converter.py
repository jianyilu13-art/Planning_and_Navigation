import math
from rb3302_navigation.landmark_converter import LandmarkConverter, grid_index, validate_label


def config(rotation=0):
    return {'grid': {'origin_x': 1.0, 'origin_y': 2.0, 'spacing_m': 0.6, 'rotation_deg': rotation}}


def test_labels_and_indices():
    assert grid_index('A') == (0, 0)
    assert grid_index('X') == (3, 5)
    try: validate_label('Y')
    except ValueError: pass
    else: assert False


def test_grid_and_rotation():
    assert LandmarkConverter(config()).coordinate('H') == (1.6, 1.4)
    x, y = LandmarkConverter(config(90)).coordinate('B')
    assert math.isclose(x, 1.0, abs_tol=1e-8) and math.isclose(y, 2.6, abs_tol=1e-8)


def test_bearing_applies_grid_rotation():
    assert math.isclose(LandmarkConverter(config()).bearing_yaw('N'), math.pi / 2)
    assert math.isclose(LandmarkConverter(config(90)).bearing_yaw('E'), math.pi / 2)
