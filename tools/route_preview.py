#!/usr/bin/env python3
"""Offline slope-cost route experiment. This is NOT a physics or ROS simulation."""
import argparse
import heapq
import json
import math
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw


def inflate(cost, radius_cells):
    """Conservative disk dilation of blocked cells with blocked map boundaries."""
    bad = ~np.isfinite(cost)
    result = bad.copy()
    radius = math.ceil(radius_cells)
    for dy in range(-radius, radius+1):
        for dx in range(-radius, radius+1):
            if math.hypot(dx, dy) <= radius_cells:
                result |= np.roll(bad, (dy, dx), (0, 1))
    if radius:
        result[:radius] = result[-radius:] = True
        result[:, :radius] = result[:, -radius:] = True
    return np.where(result, np.inf, cost)


def astar(cost, start, goal):
    """8-connected A* with no diagonal corner cutting; costs must be >=1."""
    cost = np.asarray(cost, dtype=float)
    rows, cols = cost.shape
    for point in (start, goal):
        if not (0 <= point[0] < rows and 0 <= point[1] < cols) or not np.isfinite(cost[point]):
            raise ValueError('Start/goal is outside the map or blocked')
    if (cost[np.isfinite(cost)] < 1).any():
        raise ValueError('Traversal costs must be at least one')
    queue, parent, best = [(math.dist(start, goal), 0.0, start)], {}, {start: 0.0}
    while queue:
        _, g, p = heapq.heappop(queue)
        if g > best[p]:
            continue
        if p == goal:
            path = [p]
            while p != start:
                p = parent[p]
                path.append(p)
            return list(reversed(path))
        for dr, dc in [(-1,0),(1,0),(0,-1),(0,1),(-1,-1),(-1,1),(1,-1),(1,1)]:
            q = (p[0]+dr, p[1]+dc)
            if not (0 <= q[0] < rows and 0 <= q[1] < cols) or not np.isfinite(cost[q]):
                continue
            if dr and dc and (not np.isfinite(cost[p[0]+dr,p[1]]) or not np.isfinite(cost[p[0],p[1]+dc])):
                continue
            candidate = g + math.hypot(dr, dc) * (cost[p]+cost[q])/2
            if candidate < best.get(q, math.inf):
                parent[q], best[q] = p, candidate
                heapq.heappush(queue, (candidate+math.dist(q, goal), candidate, q))
    raise ValueError('No traversable path between selected cells')


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--terrain', type=Path, default=Path('data/processed/utah/terrain.npz'))
    p.add_argument('--start', type=int, nargs=2, required=True, metavar=('ROW','COL'))
    p.add_argument('--goal', type=int, nargs=2, required=True, metavar=('ROW','COL'))
    p.add_argument('--radius', type=float, default=0.6, help='Assumed rover footprint radius in meters')
    p.add_argument('--output', type=Path, default=Path('artifacts/route'))
    args = p.parse_args()
    data = np.load(args.terrain)
    resolution = float(data['resolution'])
    cost = inflate(data['cost'], args.radius/resolution)
    path = astar(cost, tuple(args.start), tuple(args.goal))
    origin, nrows = data['origin'], cost.shape[0]
    coords = [[float(origin[0]+(c+.5)*resolution), float(origin[1]+(nrows-r-.5)*resolution)] for r,c in path]
    report = {'kind': 'offline slope-cost planning experiment, not a driving simulation',
              'start_cell': args.start, 'goal_cell': args.goal, 'resolution_m': resolution,
              'path_length_m': sum(math.dist(a,b) for a,b in zip(coords,coords[1:])),
              'max_prior_slope_deg': float(max(data['slope'][r,c] for r,c in path)),
              'radius_assumption_m': args.radius, 'path_local_xy_m': coords}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.with_suffix('.json').write_text(json.dumps(report, indent=2)+'\n')
    slope = np.clip(data['slope']/30, 0, 1)
    rgb = np.stack([70+185*slope, 170-100*slope, 130-90*slope], axis=-1).astype('uint8')
    rgb[~np.isfinite(cost)] = [45, 40, 45]
    image = Image.fromarray(rgb)
    draw = ImageDraw.Draw(image)
    draw.line([(c,r) for r,c in path], fill=(50,220,255), width=3)
    for r,c in (path[0],path[-1]):
        draw.ellipse((c-7,r-7,c+7,r+7), fill='white', outline='black', width=2)
    image.save(args.output.with_suffix('.png'))
    print(json.dumps({k:v for k,v in report.items() if k!='path_local_xy_m'},indent=2))


if __name__ == '__main__':
    main()
