#!/usr/bin/env python3
"""Prepare Redmi3DScanner's calibrated keyframe ZIP for COLMAP."""
import csv
import shutil
import sys
from pathlib import Path

if len(sys.argv) != 3:
    raise SystemExit("usage: prepare_colmap_dataset.py INPUT_DIR OUTPUT_DIR")
source, output = map(Path, sys.argv[1:])
manifest = source / "manifest.csv"
if not manifest.is_file():
    raise SystemExit("manifest.csv is missing")

with manifest.open(newline="", encoding="utf-8-sig") as stream:
    rows = list(csv.DictReader(stream))
if len(rows) < 8:
    raise SystemExit(f"Need at least 8 calibrated keyframes; got {len(rows)}")

required = {"image", "width", "height", "fx", "fy", "cx", "cy"}
if not required.issubset(rows[0]):
    raise SystemExit("manifest is missing camera calibration columns: " + ", ".join(sorted(required - set(rows[0]))))

first = rows[0]
width, height = int(first["width"]), int(first["height"])
fx, fy = float(first["fx"]), float(first["fy"])
cx, cy = float(first["cx"]), float(first["cy"])
if width <= 0 or height <= 0 or fx <= 0 or fy <= 0:
    raise SystemExit("Invalid image dimensions or focal length in manifest")

images = output / "images"
images.mkdir(parents=True, exist_ok=True)
copied = 0
for row in rows:
    # All frames are produced at the same normalized size. Reject mixed camera
    # calibration instead of silently feeding inconsistent intrinsics to COLMAP.
    if (int(row["width"]), int(row["height"])) != (width, height):
        raise SystemExit("Mixed image dimensions are not supported in one-camera mode")
    name = Path(row["image"]).name
    if name != row["image"] or not name.lower().endswith((".jpg", ".jpeg", ".png")):
        raise SystemExit(f"Unsafe or unsupported image path in manifest: {row['image']!r}")
    src = source / name
    if not src.is_file():
        raise SystemExit(f"Manifest image not found: {name}")
    shutil.copy2(src, images / name)
    copied += 1

output.mkdir(parents=True, exist_ok=True)
(output / "camera_params.txt").write_text(
    f"{fx:.8f},{fy:.8f},{cx:.8f},{cy:.8f}\n", encoding="ascii"
)
(output / "dataset-summary.txt").write_text(
    f"frames={copied}\nwidth={width}\nheight={height}\n"
    f"fx={fx}\nfy={fy}\ncx={cx}\ncy={cy}\n",
    encoding="utf-8",
)
print(f"Prepared {copied} images at {width}x{height}; PINHOLE intrinsics: {fx},{fy},{cx},{cy}")
