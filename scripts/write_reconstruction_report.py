#!/usr/bin/env python3
"""Write a concise status report into a COLMAP result bundle."""
import sys
from pathlib import Path

root = Path(sys.argv[1])
root.mkdir(parents=True, exist_ok=True)
files = sorted(p for p in root.rglob("*") if p.is_file() and p.name != "RECONSTRUCTION-REPORT.txt")
lines = ["Redmi3DScanner COLMAP reconstruction", "=" * 36, ""]
lines.append("Files produced:")
if files:
    for path in files:
        lines.append(f"- {path.relative_to(root)} ({path.stat().st_size} bytes)")
else:
    lines.append("- No reconstruction files were produced.")
lines += [
    "",
    "Interpretation:",
    "- sparse-model.ply is a sparse SfM point cloud, not a dense surface mesh.",
    "- Dense output is only available if the optional COLMAP MVS stage completed.",
    "- Camera poses and scale may be imperfect; monocular SfM has scale ambiguity.",
]
(root / "RECONSTRUCTION-REPORT.txt").write_text("\n".join(lines) + "\n", encoding="utf-8")
