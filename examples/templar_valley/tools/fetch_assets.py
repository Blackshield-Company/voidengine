#!/usr/bin/env python3
"""Download the templar_valley demo's CC0 assets and check them against
assets.lock (sha256). Nothing here is committed to the repo except this
script and the lock; the files land in examples/templar_valley/assets/fetched/.

  python3 examples/templar_valley/tools/fetch_assets.py           # fetch + verify
  python3 examples/templar_valley/tools/fetch_assets.py --relock  # rewrite the lock

Every file is CC0 1.0. See ../ASSETS.md for authors and pages.
"""
import hashlib, os, sys, urllib.request

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
OUT = os.path.join(ROOT, "assets", "fetched")
LOCK = os.path.join(ROOT, "assets.lock")
MIRROR = "https://raw.githubusercontent.com/Papyszoo/CC0-Public-Domain-Models/main/packs/"
PH = "https://dl.polyhaven.org/file/ph-assets/Textures/jpg/1k/"

FILES = {}
for pack, name in [
    ("quaternius-ultimatemodularcharacters", "king"),
    ("quaternius-ultimateanimatedanimals", "horse"),
    ("quaternius-stylized-nature-megakit", "pine_1"),
    ("quaternius-stylized-nature-megakit", "pine_2"),
    ("quaternius-stylized-nature-megakit", "common_tree_1"),
    ("quaternius-stylized-nature-megakit", "common_tree_2"),
    ("quaternius-stylized-nature-megakit", "grass_common_short"),
    ("quaternius-stylized-nature-megakit", "grass_wispy_short"),
    ("quaternius-stylized-nature-megakit", "rock_medium_1"),
    ("quaternius-medievalvillage", "house_1"),
    ("quaternius-medievalvillage", "house_2"),
    ("quaternius-medievalvillage", "house_3"),
    ("quaternius-medievalvillage", "house_4"),
    ("quaternius-medievalvillage", "crate"),
    ("quaternius-medievalvillage", "barrel"),
    ("quaternius-medievalvillage", "hay"),
]:
    FILES[f"{name}.glb"] = f"{MIRROR}{pack}/models/{name}/{name}.glb"
# Templar Valley's four ground textures, at Poly Haven's 1k (Templar packs them at 512)
for ph in ["forrest_ground_01", "rock_face_03", "sparse_grass", "rocky_trail"]:
    for m in ["diff", "nor_gl", "rough", "disp"]:
        FILES[f"{ph}_{m}_1k.jpg"] = f"{PH}{ph}/{ph}_{m}_1k.jpg"

def sha(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()

def main():
    relock = "--relock" in sys.argv
    os.makedirs(OUT, exist_ok=True)
    lock = {}
    if os.path.exists(LOCK) and not relock:
        for line in open(LOCK):
            if line.strip() and not line.startswith("#"):
                h, n = line.split()
                lock[n] = h
    bad = 0
    out_lines = ["# sha256  file  (examples/templar_valley/tools/fetch_assets.py)"]
    for name, url in FILES.items():
        dst = os.path.join(OUT, name)
        if not os.path.exists(dst) or (name in lock and sha(dst) != lock[name]):
            req = urllib.request.Request(url, headers={"User-Agent": "voidengine-fetch"})
            with urllib.request.urlopen(req, timeout=120) as r, open(dst + ".part", "wb") as f:
                f.write(r.read())
            os.replace(dst + ".part", dst)
        h = sha(dst)
        if name in lock and lock[name] != h:
            print("SHA MISMATCH", name, h, "want", lock[name])
            bad += 1
        out_lines.append(f"{h}  {name}")
    if relock or not lock:
        open(LOCK, "w").write("\n".join(out_lines) + "\n")
        print("wrote", LOCK)
    print(f"{len(FILES)} files in {OUT}, {bad} mismatches")
    sys.exit(1 if bad else 0)

if __name__ == "__main__":
    main()
