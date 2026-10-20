# Real terrain, imagery and data quality

## Sources and what each contributes

| Source | Use | Resolution / freshness boundary |
|---|---|---|
| [USGS 3DEP](https://www.usgs.gov/the-national-map-data-delivery/gis-data-download) | Bare-earth heightfield and slope prior | Query exact AOI; 1m tiles exist, but acquisition varies by project |
| [Utah UGRC lidar](https://gis.utah.gov/products/sgid/elevation/lidar/) | Cross-check project dates, vertical datum, finer native products | Not every statewide project covers the URC area |
| [Utah NAIP](https://gis.utah.gov/products/sgid/aerial-photography/naip/) / USDA service | Overhead visual context and optional texture | UGRC lists 2024, 0.6m pixels, ±4m horizontal accuracy for recent imagery; flight date is tile-specific |
| [Sentinel-2](https://dataspace.copernicus.eu/explore-data/data-collections/sentinel-data/sentinel-2) | Recent large-scale change / cloud-aware comparison | Visible/NIR bands are 10m, not suitable for detecting wheel-scale hazards |
| Organizer DEM | Supplement if released | Q&A says approval/timing not guaranteed |

NAIP is aerial photography, not satellite photography. A recent but coarse satellite scene is often less useful for rover geometry than an older high-resolution lidar DEM. Combining all images indiscriminately can introduce inconsistent dates, shadows, offsets and licensing. Select sources by capture date, usable resolution, cloud cover, accuracy and rights; record rejected/missing sources explicitly.

## Workflow

1. `terrain.py catalog` discovers candidates and saves service responses with query time. It queries 1m 3DEP and NAIP coverage plus up to ten recent Sentinel-2 candidates. It does **not** claim an exhaustive catalog of every public/commercial satellite image.
2. `fetch-dem` selects newer published tiles first, fills only missing coverage from older tiles, uses windowed range reads, and writes a cropped NAD83 UTM12 GeoTIFF plus source links and SHA-256. Publication-order selection is a first pass: review acquisition dates and seams before use. A changed publication date does not prove newer terrain measurement.
3. `export` resamples to a square `2^n+1` heightfield, creates an occupancy/slope prior, a NumPy raster bundle and a coordinate manifest. It rejects geographic-degree/feet rasters and NoData instead of inventing terrain. The square crop omits the edges of the original rectangle and the resolution may be coarser than the source.
4. Inspect the visual orientation, coordinate origin, seams, slope cutoffs and source age. The Gazebo terrain world is initially terrain-only. A safe rover start and correct geographic localization must be added before running route trials there.
5. Add procedural hazards that overhead data misses: small rocks, narrow trenches, soft sand, wheel-slip regions, occlusions and altered lighting. Use multiple scenarios, not only one visually convincing reconstruction.

Generated world files contain an absolute file URI for their height image. Re-run `export` after moving the directory. Raster rows remain north-to-south; heights are normalized with the physical height range retained in the manifest. Confirm the rendered Gazebo orientation against known controls before any real-world-coordinate trial.

## What satellite imagery cannot establish

- Centimeter-scale wheel hazards, holes under shadows or new erosion after acquisition.
- Friction, loose-soil bearing capacity, sand sinkage, wheel torque or center-of-gravity stability.
- Reliable 1m target arrival: map positional error may exceed the scoring radius.
- Radio multipath or actual link margin. A DEM line-of-sight model can inform tests, but measured antenna/system behavior is still needed.

Do not upscale 10m terrain and describe it as 1m terrain. Do not convert unknown pixels to free space. Do not assume DEM orthometric height equals GNSS ellipsoid height.
