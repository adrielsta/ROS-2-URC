"""Validate local-map mission inputs before a rover is armed."""
import math


def validate_plan(data):
    if not isinstance(data, dict) or data.get('frame') != 'map':
        raise ValueError('Mission must explicitly use the local metric map frame')
    targets = data.get('targets')
    if not isinstance(targets, list) or not 1 <= len(targets) <= 100:
        raise ValueError('Mission requires 1–100 targets')
    result = []
    for target in targets:
        if not isinstance(target, dict):
            raise ValueError('Each target must be a mapping')
        vals = [float(target[k]) for k in ('x', 'y', 'yaw')]
        if not all(math.isfinite(v) for v in vals):
            raise ValueError('Coordinates must be finite')
        result.append(dict(zip(('x', 'y', 'yaw'), vals)))
    return result

