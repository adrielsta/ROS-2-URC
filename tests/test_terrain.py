from argparse import Namespace
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from PIL import Image
import pytest
import rasterio
from rasterio.transform import from_origin
from terrain import traversability, export


def test_slope_uses_meters_and_degrees():
    z = np.tile(np.arange(20) * math.tan(math.radians(25)), (20, 1))
    slope, cost = traversability(z, 1, 20)
    assert slope[10, 10] == pytest.approx(25)
    assert not np.isfinite(cost).any()
    _, cost = traversability(z, 2, 20)
    assert np.isfinite(cost[10, 10])


def test_nodata_halo_and_map_edges_are_blocked():
    z = np.zeros((10, 10))
    z[4, 5] = np.nan
    _, cost = traversability(z, 1)
    assert np.isinf(cost[4, 5]) and np.isinf(cost[4, 4])
    assert np.isinf(cost[0]).all()
    assert cost[2, 2] == 1


def test_export_preserves_north_up_and_metric_scale(tmp_path):
    source = tmp_path/'dem.tif'
    # Highest elevation in NW, distinct E-W / N-S gradients catch flips.
    z = 100 + np.indices((65, 65))[0] * -0.1 + np.indices((65, 65))[1] * -0.01
    with rasterio.open(source, 'w', driver='GTiff', width=65, height=65, count=1,
                       dtype='float32', crs='EPSG:26912', transform=from_origin(519000, 4253000, 1, 1)) as dst:
        dst.write(z.astype('float32'), 1)
    out = tmp_path/'out'
    export(Namespace(dem=source, output=out, size=65, max_slope=20))
    manifest = json.loads((out/'manifest.json').read_text())
    assert manifest['output_spacing_m'] == 1
    h = np.asarray(Image.open(out/'height.png'))
    assert h[0, 0] == 65535 and h[-1, -1] == 0
    sdf = ET.parse(out/'terrain.sdf')
    assert sdf.find('.//heightmap/size').text.split()[:2] == ['64.0', '64.0']
    assert manifest['local_origin_projected_m'] == [519032.5, 4252967.5]


def test_geographic_dem_is_rejected(tmp_path):
    source = tmp_path/'bad.tif'
    with rasterio.open(source, 'w', driver='GTiff', width=33, height=33, count=1,
                       dtype='float32', crs='EPSG:4326', transform=from_origin(-110, 38, .001, .001)) as dst:
        dst.write(np.zeros((33, 33), dtype='float32'), 1)
    with pytest.raises(ValueError, match='projected meter'):
        export(Namespace(dem=source, output=tmp_path/'out', size=33, max_slope=20))

