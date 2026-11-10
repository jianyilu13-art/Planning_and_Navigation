#!/usr/bin/env python3
"""Generate matching Gazebo walls, occupancy map, and labelled SVG plot."""
import argparse
import math
from pathlib import Path

import yaml


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('geometry'); parser.add_argument('sdf')
    parser.add_argument('pgm'); parser.add_argument('map_yaml')
    parser.add_argument('--plot', type=Path)
    parser.add_argument('--resolution', type=float, default=.05)
    args = parser.parse_args()
    spec = yaml.safe_load(Path(args.geometry).read_text())
    grid, wall = spec['grid'], spec['wall']
    spacing = float(grid['spacing']); ox = float(grid['origin_x']); oy = float(grid['origin_y'])
    rows, cols = int(grid['rows']), int(grid['cols'])
    thickness, height = float(wall['thickness']), float(wall['height'])
    wall_specs = spec['outer_segments'] + spec['internal_vertical'] + spec['internal_horizontal']
    walls = []
    for start, end in wall_specs:
        x1, y1 = ox + start[0] * spacing, oy + start[1] * spacing
        x2, y2 = ox + end[0] * spacing, oy + end[1] * spacing
        if not (math.isclose(x1, x2) or math.isclose(y1, y2)):
            raise ValueError(f'wall is not horizontal or vertical: {start}->{end}')
        horizontal = math.isclose(y1, y2)
        length = abs(x2 - x1) if horizontal else abs(y2 - y1)
        walls.append((horizontal, (x1 + x2) / 2, (y1 + y2) / 2, length))
    models = []
    for index, (horizontal, x, y, length) in enumerate(walls):
        sx, sy = (length, thickness) if horizontal else (thickness, length)
        models.append(f'<model name="wall_{index}"><static>true</static><pose>{x} {y} {height / 2} 0 0 0</pose><link name="wall"><collision name="collision"><geometry><box><size>{sx} {sy} {height}</size></box></geometry></collision><visual name="visual"><material><ambient>0.35 0.65 0.90 1</ambient><diffuse>0.35 0.65 0.90 1</diffuse><specular>0.05 0.05 0.05 1</specular></material><geometry><box><size>{sx} {sy} {height}</size></box></geometry></visual></link></model>')
    Path(args.sdf).write_text('<sdf version="1.9"><model name="course_walls"><static>true</static>' + ''.join(models) + '</model></sdf>')
    xmin, ymin = ox - spacing, oy - spacing
    width = math.ceil((cols * spacing + 2 * spacing) / args.resolution)
    map_height = math.ceil((rows * spacing + 2 * spacing) / args.resolution)
    data = bytearray([254]) * (width * map_height)
    def mark(x, y, sx, sy):
        """Rasterize an axis-aligned wall as one occupied cell across its width.

        Wall centre-lines lie on the grid vertices, so a one-cell raster cannot
        be centred on them exactly.  Use the cell immediately north/east of the
        line and include both segment endpoints; this keeps all junctions
        connected and represents every centre-line at its declared coordinate.
        """
        if sx <= args.resolution:  # Vertical wall.
            column = round((x - xmin) / args.resolution)
            first = round((y - sy / 2 - ymin) / args.resolution)
            last = round((y + sy / 2 - ymin) / args.resolution)
            for grid_y in range(first, last + 1):
                row = map_height - 1 - grid_y
                if 0 <= column < width and 0 <= row < map_height:
                    data[row * width + column] = 0
        else:  # Horizontal wall.
            grid_y = round((y - ymin) / args.resolution)
            row = map_height - 1 - grid_y
            first = round((x - sx / 2 - xmin) / args.resolution)
            last = round((x + sx / 2 - xmin) / args.resolution)
            for column in range(first, last + 1):
                if 0 <= column < width and 0 <= row < map_height:
                    data[row * width + column] = 0
    for horizontal, x, y, length in walls:
        mark(x, y, length if horizontal else thickness, thickness if horizontal else length)
    Path(args.pgm).write_bytes(f'P5\n{width} {map_height}\n255\n'.encode() + data)
    Path(args.map_yaml).write_text(f'image: {Path(args.pgm).name}\nmode: trinary\nresolution: {args.resolution}\norigin: [{xmin}, {ymin}, 0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n')
    if args.plot:
        scale, margin = 180, 45
        def point(x, y): return margin + x * scale, margin + (rows * spacing - y) * scale
        labels, lines = [], []
        for row in range(rows):
            for column in range(cols):
                label = chr(ord('A') + row * cols + column)
                x, y = point((column + .5) * spacing, (rows - row - .5) * spacing)
                labels.append(f'<text x="{x}" y="{y}" text-anchor="middle" dominant-baseline="central">{label}</text>')
        for horizontal, x, y, length in walls:
            x1, y1 = (x - length / 2, y) if horizontal else (x, y - length / 2)
            x2, y2 = (x + length / 2, y) if horizontal else (x, y + length / 2)
            p1, p2 = point(x1, y1), point(x2, y2)
            lines.append(f'<line x1="{p1[0]}" y1="{p1[1]}" x2="{p2[0]}" y2="{p2[1]}"/>')
        view_w, view_h = 2 * margin + cols * spacing * scale, 2 * margin + rows * spacing * scale
        args.plot.write_text(f'<svg xmlns="http://www.w3.org/2000/svg" width="{view_w}" height="{view_h}" viewBox="0 0 {view_w} {view_h}"><style>text{{font:16px sans-serif;fill:#444}}line{{stroke:#277da1;stroke-width:3}}</style>' + ''.join(lines + labels) + '</svg>')
    print(f'wrote {len(walls)} walls and {width}x{map_height} map')


if __name__ == '__main__':
    main()
