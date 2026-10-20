#!/usr/bin/env python3
"""Render retrieved geospatial evidence; no generated/artificial landscape imagery."""
import json
from pathlib import Path
import numpy as np
from PIL import Image, ImageDraw, ImageFont
import rasterio
from rasterio.windows import from_bounds
from rasterio.enums import Resampling

ROOT = Path(__file__).resolve().parents[1]


def font(size):
    try:
        return ImageFont.truetype('DejaVuSans.ttf', size)
    except OSError:
        return ImageFont.load_default()


def main():
    data = np.load(ROOT/'data/processed/utah/terrain.npz')
    manifest = json.loads((ROOT/'data/processed/utah/manifest.json').read_text())
    route = json.loads((ROOT/'artifacts/utah-route.json').read_text())
    cx, cy = manifest['local_origin_projected_m']
    extent = data['elevation'].shape[0]*float(data['resolution'])
    panel = 600
    with rasterio.open(ROOT/'data/raw/urc_naip.tif') as src:
        window = from_bounds(cx-extent/2,cy-extent/2,cx+extent/2,cy+extent/2,src.transform)
        rgb = src.read([1,2,3],window=window,out_shape=(3,panel,panel),resampling=Resampling.bilinear)
        photo = Image.fromarray(rgb.transpose(1,2,0))
    slope = Image.open(ROOT/'artifacts/utah-route.png').convert('RGB').resize((panel,panel))
    canvas = Image.new('RGB',(1280,850),'#111b29')
    d = ImageDraw.Draw(canvas)
    d.text((30,20),'URC 2027 | Real Utah terrain evidence',font=font(30),fill='white')
    d.text((30,66),'Actual public data. Illustrative route endpoints; not a competition route or driving validation.',font=font(16),fill='#bacbdc')
    d.text((30,112),'NAIP aerial image | July 6, 2024',font=font(21),fill='white')
    d.text((650,112),'Slope prior + offline A* route',font=font(21),fill='white')
    canvas.paste(photo,(30,150)); canvas.paste(slope,(650,150))
    for ox in (30,650):
        d.rectangle((ox+10,165,ox+63,203),fill='#111b29')
        d.text((ox+20,170),'N ↑',font=font(20),fill='white')
        bar=round(500/extent*panel)
        d.rectangle((ox+15,709,ox+bar+25,742),fill='#111b29')
        d.line((ox+20,734,ox+20+bar,734),fill='white',width=3)
        d.text((ox+20,711),'500 m',font=font(14),fill='white')
    d.text((30,773),'60 cm imagery pixels; positional error can exceed the 1 m scoring radius.',font=font(14),fill='#bacbdc')
    d.text((650,773),'Cyan: route | dark: blocked prior | green → brown: rising slope',font=font(14),fill='#bacbdc')
    d.text((30,809),f'USGS source DEM: 1 m | working grid: {float(data["resolution"]):.2f} m | route: {route["path_length_m"]:.0f} m | slope cutoff assumption: 20°',font=font(17),fill='#79dae2')
    output=ROOT/'docs/evidence/utah-preview.png'
    output.parent.mkdir(parents=True,exist_ok=True)
    canvas.save(output)
    print(output)


if __name__ == '__main__':
    main()
