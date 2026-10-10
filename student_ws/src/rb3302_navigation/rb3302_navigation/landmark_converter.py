"""Pure, testable conversion between board labels and map-frame poses."""
import math

LABELS = tuple(chr(ord('A') + i) for i in range(24))
BEARINGS_DEG = {'E': 0, 'NE': 45, 'N': 90, 'NW': 135,
                'W': 180, 'SW': 225, 'S': 270, 'SE': 315}


def validate_label(label):
    label = str(label).upper()
    if label not in LABELS:
        raise ValueError(f"landmark must be A-X, got {label!r}")
    return label


def grid_index(label):
    """Return zero-based (row, column), with A-F on the first row."""
    index = ord(validate_label(label)) - ord('A')
    return divmod(index, 6)


class LandmarkConverter:
    def __init__(self, config):
        grid = config.get('grid', config)
        self.origin_x = float(grid['origin_x'])
        self.origin_y = float(grid['origin_y'])
        self.spacing = float(grid.get('spacing_m', 0.6))
        self.rotation = math.radians(float(grid.get('rotation_deg', 0.0)))
        self.overrides = {str(k).upper(): v for k, v in config.get('overrides', {}).items()}

    def coordinate(self, label):
        label = validate_label(label)
        if label in self.overrides:
            point = self.overrides[label]
            return float(point['x']), float(point['y'])
        row, column = grid_index(label)
        # Board +x progresses A->F; board +y progresses bottom->top.
        lx, ly = column * self.spacing, -row * self.spacing
        c, s = math.cos(self.rotation), math.sin(self.rotation)
        return self.origin_x + c * lx - s * ly, self.origin_y + s * lx + c * ly

    def bearing_yaw(self, bearing):
        bearing = str(bearing).upper()
        if bearing not in BEARINGS_DEG:
            raise ValueError(f"bearing must be N, NE, E, SE, S, SW, W, or NW; got {bearing!r}")
        return math.atan2(math.sin(self.rotation + math.radians(BEARINGS_DEG[bearing])),
                          math.cos(self.rotation + math.radians(BEARINGS_DEG[bearing])))
