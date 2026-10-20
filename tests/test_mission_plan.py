import pytest
from urc_brain.mission_plan import validate_plan


def test_valid_metric_plan():
    assert validate_plan({'frame': 'map', 'targets': [{'x': 1, 'y': 2, 'yaw': 0}]})[0]['x'] == 1


@pytest.mark.parametrize('plan', [None, {}, {'frame': 'gps', 'targets': []},
    {'frame': 'map', 'targets': []}, {'frame': 'map', 'targets': [{'x': float('nan'), 'y': 0, 'yaw': 0}]}])
def test_invalid_plans_do_not_arm(plan):
    with pytest.raises(ValueError):
        validate_plan(plan)

