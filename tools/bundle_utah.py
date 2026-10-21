#!/usr/bin/env python3
"""Bundle a small real-data practice patch; never change the source relief.

Requires the dated DEM export and NAIP GeoTIFF from tools/terrain.py.
The bundled assets let teammates run offline without repeating large downloads.
"""
import json
from pathlib import Path
import numpy as np
from PIL import Image
import rasterio
from rasterio.warp import reproject, Resampling
from rasterio.transform import from_bounds
from pyproj import Transformer
from terrain import sha256

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / 'src/urc_bringup/models/utah_site'


def main():
    original = ROOT / 'data/processed/utah'
    source = np.load(original / 'terrain.npz')
    provenance = json.loads((original / 'manifest.json').read_text())
    row, col, half = 350, 380, 128
    z = source['elevation'][row-half:row+half+1, col-half:col+half+1]
    slope = source['slope'][row-half:row+half+1, col-half:col+half+1]
    step = float(source['resolution'])
    grid_center = (source['elevation'].shape[0]-1)/2
    east = provenance['local_origin_projected_m'][0] + (col-grid_center)*step
    north = provenance['local_origin_projected_m'][1] + (grid_center-row)*step
    baseline, scale, spawn = float(z.min()), float(z.max()-z.min()), float(z[half,half])
    OUT.mkdir(parents=True, exist_ok=True)
    Image.fromarray(np.rint((z-baseline)/scale*65535).astype('uint16')).save(OUT/'height.png')
    # Millidegree precision keeps slope reconstruction deterministic.
    Image.fromarray(np.rint(slope*1000).clip(0,65535).astype('uint16')).save(OUT/'slope.png')
    world_size = 2*half*step
    rgb = np.zeros((3,1024,1024), dtype='uint8')
    with rasterio.open(ROOT/'data/raw/urc_naip.tif') as src:
        dest = from_bounds(east-world_size/2, north-world_size/2,
                           east+world_size/2, north+world_size/2, 1024,1024)
        for band in range(3):
            reproject(rasterio.band(src,band+1), rgb[band], src_transform=src.transform,
                      src_crs=src.crs, dst_transform=dest, dst_crs=provenance['source_crs'],
                      resampling=Resampling.bilinear)
    Image.fromarray(rgb.transpose(1,2,0)).save(OUT/'aerial.jpg',quality=90)
    Image.new('RGB',(4,4),(128,128,255)).save(OUT/'normal.png')
    convert = Transformer.from_crs(provenance['source_crs'],4326,always_xy=True)
    lon,lat = convert.transform(east,north)
    meta = dict(provenance)
    meta.update(grid_size=len(z),output_spacing_m=step,physical_size_m=world_size,
                baseline_elevation_m=baseline,height_scale_m=scale,
                spawn_elevation_m=spawn,local_origin_projected_m=[east,north],
                local_origin_wgs84=[lon,lat],source_crop_center_row_col=[row,col],
                imagery_acquisition_date='2024-07-06',
                imagery_source_sha256=sha256(ROOT/'data/raw/urc_naip.tif'),
                world_note='Real-data practice patch with ideal simulator pose. Not an official competition course.',
                vertical_datum_note='Retains source DEM elevations; no ellipsoidal GNSS altitude conversion.',
                geodesy_note='Map axes follow EPSG:26912 grid east/north. Gazebo GPS is illustrative and not used for navigation.')
    (OUT/'manifest.json').write_text(json.dumps(meta,indent=2)+'\n')
    print(f'Bundled {world_size:.2f}m square real Utah patch, {step}m samples.')


if __name__ == '__main__':
    main()
