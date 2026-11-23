#!/usr/bin/env python3
"""Pack Templar Valley's four Poly Haven ground textures (1k JPGs from
fetch_assets.py) into the terrain layer images the demo loads:

  assets/generated/layer<N>_albedo.png  RGB albedo (sRGB), A = displacement (height blend)
  assets/generated/layer<N>_normal.png  RGB OpenGL normal
  assets/generated/layer<N>_orm.png     R = AO (cavity from displacement), G = roughness, B = 0

Slots follow Templar's ValleyGround: 0 grass, 1 rock, 2 forest floor, 3 dirt.
"""
import os
from PIL import Image, ImageChops, ImageFilter

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.join(HERE, "..", "assets", "fetched")
OUT = os.path.join(HERE, "..", "assets", "generated")
SLOTS = ["forrest_ground_01", "rock_face_03", "sparse_grass", "rocky_trail"]
SIZE = 1024

os.makedirs(OUT, exist_ok=True)
for i, ph in enumerate(SLOTS):
    load = lambda m, mode: Image.open(os.path.join(SRC, f"{ph}_{m}_1k.jpg")).convert(mode).resize((SIZE, SIZE), Image.LANCZOS)
    diff, nor, rough, disp = load("diff", "RGB"), load("nor_gl", "RGB"), load("rough", "L"), load("disp", "L")
    r, g, b = diff.split()
    Image.merge("RGBA", (r, g, b, disp)).save(os.path.join(OUT, f"layer{i}_albedo.png"))
    nor.save(os.path.join(OUT, f"layer{i}_normal.png"))
    blur = disp.filter(ImageFilter.GaussianBlur(8))
    cavity = ImageChops.subtract(blur, disp, scale=1.0, offset=0)
    ao = ImageChops.invert(cavity.point(lambda v: min(255, v * 3)))
    Image.merge("RGB", (ao, rough, Image.new("L", (SIZE, SIZE), 0))).save(os.path.join(OUT, f"layer{i}_orm.png"))
    print("layer", i, ph)
