import numpy as np
import pytest
from route_preview import astar, inflate


def test_detour_through_only_open_gap():
    cost = np.ones((15,15))
    cost[:12,7] = np.inf
    path = astar(cost,(5,2),(5,12))
    assert max(r for r,c in path) >= 12
    assert all(np.isfinite(cost[p]) for p in path)


def test_diagonal_cannot_squeeze_between_obstacles():
    cost = np.array([[1,np.inf],[np.inf,1]])
    with pytest.raises(ValueError,match='No traversable'):
        astar(cost,(0,0),(1,1))


def test_slope_cost_prefers_longer_easier_route():
    cost = np.ones((10,10))
    cost[4:7,3:8] = 9
    path = astar(cost,(5,1),(5,9))
    assert any(r < 4 or r > 6 for r,c in path)


def test_footprint_closes_narrow_passage():
    cost = np.ones((10,10))
    cost[:,3] = np.inf
    cost[:,5] = np.inf
    result = inflate(cost,1)
    assert np.isinf(result[:,4]).all()
