#!/usr/bin/env python3
"""Split Templar's packed CC0 Poly Haven ground textures into engine PBR maps.

Input (Templar terrain pass, scripts/pack_terrain_textures.sh 512):
  N_<name>_albedo_height.png  RGB albedo (sRGB), A displacement
  N_<name>_normal_rough.png   RGB OpenGL normal, A roughness
Output (examples/templar3d/assets):
  ground_<name>_albedo.png  RGB albedo, A = 255
  ground_<name>_normal.png  RGB OpenGL normal (glTF convention), A = 255
  ground_<name>_orm.png     R = AO (from displacement cavity), G = roughness, B = metallic (0)
Usage: pack_ground.py TEMPLAR_TERRAIN_DIR OUT_DIR slot:name [slot:name ...]
"""
import sys
from PIL import Image, ImageFilter, ImageChops

src, out = sys.argv[1], sys.argv[2]
for spec in sys.argv[3:]:
    slot, name = spec.split(":")
    ah = Image.open(f"{src}/{slot}_{name}_albedo_height.png").convert("RGBA")
    nr = Image.open(f"{src}/{slot}_{name}_normal_rough.png").convert("RGBA")
    r, g, b, height = ah.split()
    Image.merge("RGB", (r, g, b)).save(f"{out}/ground_{name}_albedo.png", optimize=True)
    nx, ny, nz, rough = nr.split()
    Image.merge("RGB", (nx, ny, nz)).save(f"{out}/ground_{name}_normal.png", optimize=True)
    # cheap cavity AO: pixels below their blurred neighbourhood get darker
    blur = height.filter(ImageFilter.GaussianBlur(6))
    cavity = ImageChops.subtract(blur, height, scale=1.0, offset=0)
    ao = ImageChops.invert(cavity.point(lambda v: min(255, v * 3)))
    metal = Image.new("L", ah.size, 0)
    Image.merge("RGB", (ao, rough, metal)).save(f"{out}/ground_{name}_orm.png", optimize=True)
    print("packed", name)
