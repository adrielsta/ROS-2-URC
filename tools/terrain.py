#!/usr/bin/env python3
"""Terrain acquisition and conversion with explicit provenance and metric frames.

Maps are prior knowledge, never substitutes for live obstacle/negative-obstacle sensing.
Requires requirements-dev.txt; no ROS dependency for data preparation.
"""
import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
import os
import ssl
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from PIL import Image
import requests
from requests.adapters import HTTPAdapter
from urllib3.util.retry import Retry
import yaml

AOI = [-110.786, 38.411, -110.768, 38.425]  # west, south, east, north; rule 1.e.xiii
TNM = 'https://tnmaccess.nationalmap.gov/api/v1/products'
NAIP = 'https://apps.geo.fpac.usda.gov/geo-imagery/rest/services/naip/conus_naip/ImageServer'
STAC = 'https://earth-search.aws.element84.com/v1/search'

# GDAL reads this environment setting when its HTTP driver is initialized.
# Preserve a caller-supplied bundle; otherwise use the Python/system trust store.
_system_ca = ssl.get_default_verify_paths().cafile
if _system_ca:
    os.environ.setdefault('GDAL_CURL_CA_BUNDLE', _system_ca)


def http():
    session = requests.Session()
    retry = Retry(total=3, backoff_factor=1, status_forcelist=[429, 500, 502, 503, 504])
    session.mount('https://', HTTPAdapter(max_retries=retry))
    return session


def request_json(url, params):
    response = http().get(url, params=params, timeout=60)
    response.raise_for_status()
    result = response.json()
    if 'error' in result:
        raise RuntimeError(f'Service error: {result["error"]}')
    return result


def catalog(args):
    """Keep acquisition dates separate from publication/index dates. Never invent coverage."""
    bounds = args.bbox
    if not (-180 <= bounds[0] < bounds[2] <= 180 and -90 <= bounds[1] < bounds[3] <= 90):
        raise ValueError('Invalid WGS84 bounding box')
    result = {'queried_at_utc': datetime.now(timezone.utc).isoformat(), 'bbox_wgs84': bounds,
              'aoi_source': 'URC 2027 rule 1.e.xiii; approximate envelope, NOT a legal geofence',
              'sources': {}, 'errors': {}}
    try:
        items, offset = [], 0
        while True:
            page = request_json(TNM, {'bbox': ','.join(map(str, bounds)),
                'datasets': 'Digital Elevation Model (DEM) 1 meter',
                'max': 100, 'offset': offset, 'outputFormat': 'JSON'})
            batch = page.get('items', [])
            items.extend(batch)
            if not batch or len(items) >= int(page.get('total', 0)):
                break
            offset += len(batch)
        result['sources']['usgs_3dep_1m'] = {
            'endpoint': TNM, 'items': items,
            'note': 'Check tile metadata for acquisition year and vertical datum. Publication is not capture date.'}
    except Exception as exc:
        result['errors']['usgs_3dep_1m'] = str(exc)
    try:
        page = request_json(NAIP + '/query', {
            'f': 'json', 'where': 'year_ts IS NOT NULL', 'geometry': ','.join(map(str, bounds)),
            'geometryType': 'esriGeometryEnvelope', 'inSR': 4326,
            'spatialRel': 'esriSpatialRelIntersects', 'outFields': '*', 'returnGeometry': 'false',
        })
        result['sources']['naip'] = {'endpoint': NAIP, 'catalog': {'features': [{'attributes': f['attributes']} for f in page.get('features', [])], 'exceededTransferLimit': page.get('exceededTransferLimit', False)},
            'note': 'Utah UGRC lists 2024 at 0.6m; verify tile AcquisitionDate, do not assume service-wide date.'}
    except Exception as exc:
        result['errors']['naip'] = str(exc)
    try:
        page = request_json(STAC, {'collections': 'sentinel-2-l2a',
            'bbox': ','.join(map(str, bounds)), 'limit': 10, 'sortby': '-properties.datetime'})
        result['sources']['sentinel2_recent'] = {'endpoint': STAC,
            'items': [{'id': f['id'], 'properties': f['properties'], 'links': f['links']}
                      for f in page.get('features', [])],
            'note': 'Ten recent scene candidates, not all imagery. 10m visible bands cannot resolve rover obstacles. Check clouds.'}
    except Exception as exc:
        result['errors']['sentinel2_recent'] = str(exc)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps({'catalog': str(args.output), 'sources': list(result['sources']), 'errors': result['errors']}, indent=2))
    if not result['sources']:
        raise RuntimeError('No catalog source was reachable')


def sha256(path):
    digest = hashlib.sha256()
    with open(path, 'rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def fetch_dem(args):
    """Crop selected USGS COG tile(s) with range reads; no full-state downloads."""
    import rasterio
    from rasterio.warp import reproject, transform_bounds, Resampling
    from rasterio.transform import from_origin
    from rasterio.windows import from_bounds
    cat = json.loads(args.catalog.read_text())
    items = cat['sources']['usgs_3dep_1m']['items']
    if not items:
        raise ValueError('No 1m DEM coverage. Supply a verified GeoTIFF; do not silently upsample a 10m DEM.')
    # Newest publication first; overlap does not erase valid cells already filled.
    items = sorted(items, key=lambda i: i.get('publicationDate', ''), reverse=True)
    crs = 'EPSG:26912'  # NAD83 / UTM zone 12N. Keep this distinct from WGS84 GPS.
    bounds = transform_bounds('EPSG:4326', crs, *cat['bbox_wgs84'], densify_pts=21)
    west, south, east, north = [math.floor(bounds[0]), math.floor(bounds[1]), math.ceil(bounds[2]), math.ceil(bounds[3])]
    transform = from_origin(west, north, 1.0, 1.0)
    shape = (north - south, east - west)
    if np.prod(shape) > 25_000_000:
        raise ValueError('AOI exceeds 25 million 1m cells; tile the request')
    dem = np.full(shape, np.nan, dtype=np.float32)
    used = []
    for item in items:
        if np.isfinite(dem).all():
            break
        url = item.get('downloadURL')
        if not url or not url.lower().endswith(('.tif', '.tiff')):
            continue
        print('Reading window:', item['title'], flush=True)
        # Respect the host trust store (including an enterprise proxy CA) without
        # disabling certificate verification. Rasterio wheels may ship a different CA set.
        ca = os.environ.get('GDAL_CURL_CA_BUNDLE') or ssl.get_default_verify_paths().cafile
        tls = {'GDAL_CURL_CA_BUNDLE': ca} if ca else {}
        with rasterio.Env(**tls, GDAL_DISABLE_READDIR_ON_OPEN='EMPTY_DIR',
                          CPL_VSIL_CURL_ALLOWED_EXTENSIONS='.tif,.tiff', GDAL_HTTP_TIMEOUT='60'):
            with rasterio.open(url) as src:
                crop_bounds = transform_bounds(crs, src.crs, west, south, east, north, densify_pts=21)
                overlap = (max(crop_bounds[0], src.bounds.left), max(crop_bounds[1], src.bounds.bottom),
                           min(crop_bounds[2], src.bounds.right), min(crop_bounds[3], src.bounds.top))
                if overlap[0] >= overlap[2] or overlap[1] >= overlap[3]:
                    continue
                window = from_bounds(*overlap, transform=src.transform).round_offsets().round_lengths()
                values = src.read(1, window=window, masked=True).astype(np.float32).filled(np.nan)
                reproj = np.full(shape, np.nan, dtype=np.float32)
                reproject(values, reproj, src_transform=src.window_transform(window), src_crs=src.crs,
                          dst_transform=transform, dst_crs=crs, src_nodata=np.nan, dst_nodata=np.nan,
                          resampling=Resampling.bilinear)
                fill = ~np.isfinite(dem) & np.isfinite(reproj)
                dem[fill] = reproj[fill]
                if fill.any():
                    used.append({'title': item['title'], 'download_url': url,
                                 'metadata_url': item.get('metaUrl'), 'publication_date': item.get('publicationDate'),
                                 'source_crs': str(src.crs), 'source_resolution': list(src.res),
                                 'source_bounds': list(src.bounds), 'source_tags': src.tags()})
    if not np.isfinite(dem).all():
        raise ValueError(f'Incomplete DEM coverage: {np.isfinite(dem).mean():.1%}; do not fabricate missing terrain')
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with rasterio.open(args.output, 'w', driver='GTiff', width=shape[1], height=shape[0], count=1,
                       dtype='float32', crs=crs, transform=transform, nodata=np.nan,
                       compress='deflate') as dst:
        dst.write(dem, 1)
    manifest = {'generated_at_utc': datetime.now(timezone.utc).isoformat(), 'sources': used,
                'crs': crs, 'resolution_m': 1.0, 'bbox_wgs84': cat['bbox_wgs84'],
                'sha256': sha256(args.output), 'vertical_units': 'm',
                'vertical_datum': 'USGS 3DEP NAVD88; verify linked source metadata before GNSS altitude fusion',
                'note': 'Derived clipped raster. The catalog may update independently of acquisition.'}
    args.output.with_suffix('.source.json').write_text(json.dumps(manifest, indent=2) + '\n')
    print(json.dumps({'file': str(args.output), 'shape': shape, 'min_m': float(dem.min()), 'max_m': float(dem.max()), 'sources': len(used)}, indent=2))


def fetch_imagery(args):
    """Retrieve a bounded NAIP RGB GeoTIFF, locked to dated primary source rasters."""
    import rasterio
    from rasterio.warp import transform_bounds
    if not math.isfinite(args.resolution) or args.resolution <= 0:
        raise ValueError('Imagery export resolution must be finite and positive')
    cat = json.loads(args.catalog.read_text())
    features = cat['sources']['naip']['catalog'].get('features', [])
    dated = [f['attributes'] for f in features if f['attributes'].get('QQDATE')]
    if not dated:
        raise ValueError('No dated primary imagery in the catalog')
    latest_year = max(f['year_ts'] for f in dated)
    selected = [f for f in dated if f['year_ts'] == latest_year]
    bounds = transform_bounds('EPSG:4326', 'EPSG:26912', *cat['bbox_wgs84'], densify_pts=21)
    width = min(4096, math.ceil((bounds[2]-bounds[0])/args.resolution))
    height = min(4096, math.ceil((bounds[3]-bounds[1])/args.resolution))
    export_params = {
        'f': 'image', 'bbox': ','.join(map(str, bounds)), 'bboxSR': 26912,
        'imageSR': 26912, 'size': f'{width},{height}', 'format': 'tiff',
        'pixelType': 'U8', 'bandIds': '0,1,2', 'interpolation': 'RSP_BilinearInterpolation',
        'adjustAspectRatio': 'false', 'noDataInterpretation': 'esriNoDataMatchAll',
        'mosaicRule': json.dumps({'mosaicMethod': 'esriMosaicLockRaster',
                                 'lockRasterIds': [f['OBJECTID'] for f in selected],
                                 'mosaicOperation': 'MT_FIRST'}),
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    download = http().get(NAIP+'/exportImage', params=export_params, timeout=120)
    download.raise_for_status()
    args.output.write_bytes(download.content)
    with rasterio.open(args.output) as image:
        if image.count < 3 or not image.crs or image.width < 10 or image.height < 10:
            raise ValueError('Imagery response is not a georeferenced RGB raster')
        # Preview remains separate from the exact GeoTIFF.
        rgb = image.read([1, 2, 3], out_shape=(3, 900, 900)).transpose(1, 2, 0)
        Image.fromarray(rgb).save(args.output.with_suffix('.preview.png'))
        info = {'source_endpoint': NAIP, 'source_tiles': selected,
                'retrieved_at_utc': datetime.now(timezone.utc).isoformat(),
                'crs': str(image.crs), 'resolution_m': list(image.res),
                'bounds': list(image.bounds), 'sha256': sha256(args.output),
                'note': 'NAIP aerial imagery. Resampled export, not new acquisition. Check actual coverage/nodata before use.'}
    args.output.with_suffix('.source.json').write_text(json.dumps(info, indent=2)+'\n')
    print(json.dumps(info, indent=2))


def fetch_satellite(args):
    """Clip the latest low-scene-cloud Sentinel-2 visual and scene-classification rasters."""
    import rasterio
    from rasterio.warp import transform_bounds
    from rasterio.windows import from_bounds
    cat = json.loads(args.catalog.read_text())
    candidates = cat['sources']['sentinel2_recent']['items']
    acceptable = [f for f in candidates if f['properties'].get('eo:cloud_cover', 100) <= args.max_cloud]
    if not acceptable:
        raise ValueError('No recent scene meets the cloud threshold; expand catalog range explicitly')
    acceptable.sort(key=lambda f: f['properties']['datetime'], reverse=True)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    ca = os.environ.get('GDAL_CURL_CA_BUNDLE') or ssl.get_default_verify_paths().cafile
    tls = {'GDAL_CURL_CA_BUNDLE': ca} if ca else {}
    rejections = []
    outputs = {}
    selected = None

    def clip(url):
        with rasterio.open(url) as src:
            bounds = transform_bounds('EPSG:4326', src.crs, *cat['bbox_wgs84'], densify_pts=21)
            window = from_bounds(*bounds, src.transform).round_offsets().round_lengths()
            values = src.read(window=window)
            profile = src.profile.copy()
            profile.update(width=values.shape[2], height=values.shape[1],
                           transform=src.window_transform(window), compress='deflate')
            return values, profile, list(src.res)

    with rasterio.Env(**tls, GDAL_DISABLE_READDIR_ON_OPEN='EMPTY_DIR',
                      CPL_VSIL_CURL_ALLOWED_EXTENSIONS='.tif,.tiff', GDAL_HTTP_TIMEOUT='60'):
        for candidate in acceptable:
            source_url = next(x['href'] for x in candidate['links'] if x['rel'] == 'self')
            item = request_json(source_url, {})
            scl, scl_profile, scl_res = clip(item['assets']['scl']['href'])
            invalid = np.isin(scl, [0, 1, 3, 8, 9, 10, 11])
            flagged = float(invalid.mean())
            counts = {str(k): int(v) for k, v in zip(*np.unique(scl, return_counts=True))}
            if flagged > args.max_aoi_flagged:
                rejections.append({'item': candidate['id'], 'aoi_flagged_fraction': flagged,
                                   'scl_class_counts': counts})
                continue
            selected = candidate
            visual, visual_profile, visual_res = clip(item['assets']['visual']['href'])
            for key, values, profile, res in [('visual', visual, visual_profile, visual_res),
                                              ('scl', scl, scl_profile, scl_res)]:
                target = args.output if key == 'visual' else args.output.with_name(args.output.stem+'_scl.tif')
                with rasterio.open(target, 'w', **profile) as dst:
                    dst.write(values)
                outputs[key] = {'url': item['assets'][key]['href'], 'sha256': sha256(target),
                                'native_pixel_size_m': res}
                if key == 'scl':
                    outputs[key].update(aoi_flagged_fraction=flagged, scl_class_counts=counts)
            break
    if selected is None:
        raise ValueError('No recent scene passes the AOI coverage/cloud check: ' + json.dumps(rejections))
    manifest = {'item': selected['id'], 'acquired_at_utc': selected['properties']['datetime'],
                'scene_cloud_percent': selected['properties'].get('eo:cloud_cover'),
                'retrieved_at_utc': datetime.now(timezone.utc).isoformat(), 'assets': outputs,
                'rejected_newer_candidates': rejections,
                'note': 'RGB at native 10m resolution for context/change review only; SCL flags require interpretation. No wheel-scale hazard claims.'}
    args.output.with_suffix('.source.json').write_text(json.dumps(manifest, indent=2)+'\n')
    print(json.dumps(manifest, indent=2))



def traversability(dem, resolution, max_slope=20.0):
    """Conservative slope prior. NoData and one-cell halo are blocked, never free."""
    dem = np.asarray(dem, dtype=float)
    if dem.ndim != 2 or min(dem.shape) < 3:
        raise ValueError('DEM must have at least 3x3 cells')
    if not math.isfinite(resolution) or resolution <= 0 or not 0 < max_slope < 90:
        raise ValueError('Invalid metric resolution or slope limit')
    valid = np.isfinite(dem)
    z = np.where(valid, dem, 0)
    dy, dx = np.gradient(z, resolution)
    slope = np.degrees(np.arctan(np.hypot(dx, dy)))
    bad = ~valid
    halo = bad.copy()
    for dr, dc in [(1, 0), (-1, 0), (0, 1), (0, -1)]:
        # Roll is safe here because all outer-edge cells are subsequently blocked.
        halo |= np.roll(bad, (dr, dc), (0, 1))
    halo[[0, -1], :] = True
    halo[:, [0, -1]] = True
    blocked = halo | (slope >= max_slope)
    cost = 1.0 + 8.0 * (slope / max_slope) ** 2
    cost[blocked] = np.inf
    return slope, cost


def export(args):
    """Create a Gazebo heightfield and a Nav2 slope prior from a metric DEM.

    Heightfield raster rows are north-to-south; Gazebo center is the local origin.
    Nav2's lower-left map corner includes half a pixel around heightfield samples.
    """
    import rasterio
    from rasterio.enums import Resampling
    from pyproj import Transformer
    args.output.mkdir(parents=True, exist_ok=True)
    with rasterio.open(args.dem) as src:
        if not src.crs or not src.crs.is_projected or src.crs.linear_units != 'metre':
            raise ValueError('DEM must use a projected meter CRS (not latitude/longitude or feet)')
        if src.transform.b != 0 or src.transform.d != 0 or src.transform.a <= 0 or src.transform.e >= 0:
            raise ValueError('DEM must be an unrotated north-up raster')
        # Square metric extent, 2^n+1 vertices, strictly inside source coverage.
        extent = min(src.bounds.right-src.bounds.left, src.bounds.top-src.bounds.bottom)
        cx, cy = (src.bounds.left+src.bounds.right)/2, (src.bounds.bottom+src.bounds.top)/2
        from rasterio.windows import from_bounds
        window = from_bounds(cx-extent/2, cy-extent/2, cx+extent/2, cy+extent/2, src.transform)
        size = args.size
        if size < 33 or size > 2049 or ((size-1) & (size-2)):
            raise ValueError('Heightfield size must be 2^n+1 in [33, 2049]')
        grid = src.read(1, window=window, out_shape=(size, size), resampling=Resampling.bilinear,
                        masked=True).astype(float).filled(np.nan)
        if not np.isfinite(grid).all():
            raise ValueError('NoData in terrain mesh; supply complete data, never fill unknown cliffs')
        step = extent / size  # GDAL pixel-center spacing after resampling
        physical = step * (size - 1)
        lon, lat = Transformer.from_crs(src.crs, 4326, always_xy=True).transform(cx, cy)
        source_crs, source_res = str(src.crs), list(src.res)
    baseline = float(grid.min())
    height = max(float(grid.max() - baseline), 0.01)
    Image.fromarray(np.round((grid-baseline)/height*65535).astype(np.uint16)).save(args.output/'height.png')
    slope, cost = traversability(grid, step, args.max_slope)
    # Scale map encodes graded slope costs; 0 is lethal black, 254 is free white.
    pixels = np.where(np.isfinite(cost), 254 - np.minimum(160, slope / args.max_slope * 160), 0).astype(np.uint8)
    Image.fromarray(pixels).save(args.output/'traversability.pgm')
    map_yaml = {'image': 'traversability.pgm', 'mode': 'scale', 'resolution': step,
                'origin': [-extent/2, -extent/2, 0.0], 'negate': 0,
                'occupied_thresh': 0.99, 'free_thresh': 0.01}
    (args.output/'map.yaml').write_text(yaml.safe_dump(map_yaml, sort_keys=False))
    np.savez_compressed(args.output/'terrain.npz', elevation=grid, slope=slope, cost=cost,
                        resolution=step, origin=np.array([-extent/2, -extent/2]))
    # Absolute file URI resolves when Gazebo is launched outside the output folder.
    # Re-run export after moving the folder to regenerate the URI.
    geom = f'<geometry><heightmap><uri>{(args.output/"height.png").resolve().as_uri()}</uri><size>{physical} {physical} {height}</size><pos>0 0 0</pos></heightmap></geometry>'
    sdf = f'''<sdf version="1.9"><world name="utah_terrain">
      <physics name="physics" type="ignored"><max_step_size>0.002</max_step_size><real_time_factor>1</real_time_factor></physics>
      <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
      <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
      <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
      <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors"><render_engine>ogre2</render_engine></plugin>
      <light name="sun" type="directional"><pose>0 0 100 0 0 0</pose><direction>-0.5 0.2 -0.8</direction></light>
      <spherical_coordinates><surface_model>EARTH_WGS84</surface_model><world_frame_orientation>ENU</world_frame_orientation><latitude_deg>{lat}</latitude_deg><longitude_deg>{lon}</longitude_deg><elevation>{baseline}</elevation><heading_deg>0</heading_deg></spherical_coordinates>
      <model name="terrain"><static>true</static><link name="link"><collision name="terrain">{geom}</collision><visual name="terrain">{geom}<material><diffuse>0.7 0.52 0.32 1</diffuse></material></visual></link></model>
    </world></sdf>'''
    root = ET.fromstring(sdf)
    ET.indent(root)
    ET.ElementTree(root).write(args.output/'terrain.sdf', encoding='unicode', xml_declaration=True)
    manifest = {'source_sha256': sha256(args.dem), 'source_crs': source_crs,
                'source_resolution_m': source_res, 'output_spacing_m': step,
                'resampling': 'bilinear; downsampling loses terrain detail, upsampling adds no detail',
                'local_origin_projected_m': [cx, cy], 'local_origin_wgs84': [lon, lat],
                'map_axes': 'projected-grid east, north, up; grid north is not exactly true north',
                'baseline_elevation_m': baseline, 'height_scale_m': height,
                'height_quantization_m': height/65535, 'grid_size': size,
                'max_slope_deg_assumption': args.max_slope,
                'driveable_fraction_prior_only': float(np.isfinite(cost).mean()),
                'warning': 'Slope-only prior. Unknown traction, rocks, erosion and negative obstacles. No real-rover certification.',
                'world_note': 'Terrain-only world. No rover is spawned; choose a safe start and configure a localized rover before navigation.'}
    (args.output/'manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
    print(json.dumps(manifest, indent=2))


def main():
    p = argparse.ArgumentParser(description=__doc__)
    sub = p.add_subparsers(dest='command', required=True)
    c = sub.add_parser('catalog')
    c.add_argument('--bbox', type=float, nargs=4, default=AOI, metavar=('W','S','E','N'))
    c.add_argument('--output', type=Path, default=Path('data/catalog.json'))
    c.set_defaults(func=catalog)
    f = sub.add_parser('fetch-dem')
    f.add_argument('--catalog', type=Path, default=Path('data/catalog.json'))
    f.add_argument('--output', type=Path, default=Path('data/raw/urc_dem.tif'))
    f.set_defaults(func=fetch_dem)
    i = sub.add_parser('fetch-imagery')
    i.add_argument('--catalog', type=Path, default=Path('data/catalog.json'))
    i.add_argument('--output', type=Path, default=Path('data/raw/urc_naip.tif'))
    i.add_argument('--resolution', type=float, default=0.6)
    i.set_defaults(func=fetch_imagery)
    s = sub.add_parser('fetch-satellite')
    s.add_argument('--catalog', type=Path, default=Path('data/catalog.json'))
    s.add_argument('--output', type=Path, default=Path('data/raw/urc_sentinel2.tif'))
    s.add_argument('--max-cloud', type=float, default=20.0)
    s.add_argument('--max-aoi-flagged', type=float, default=0.1)
    s.set_defaults(func=fetch_satellite)
    e = sub.add_parser('export')
    e.add_argument('--dem', type=Path, required=True)
    e.add_argument('--output', type=Path, default=Path('data/processed/utah'))
    e.add_argument('--size', type=int, default=1025)
    e.add_argument('--max-slope', type=float, default=20.0)
    e.set_defaults(func=export)
    args = p.parse_args()
    args.func(args)


if __name__ == '__main__':
    main()
