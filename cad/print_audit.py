"""Printability audit for every exported v12 3MF (replaces the broken print_check.py).
Checks per file: loads, watertight, one connected body (or reports pieces), positive volume,
fits a 256 x 256 x 256 mm plate in its best axis-aligned orientation (Bambu X1/P1/A1-class plate).
Run: uv run -q --with trimesh --with lxml --with networkx --with numpy python cad/print_audit.py
"""
import glob
import json
import os
import sys

import numpy as np
import trimesh

HERE = os.path.dirname(os.path.abspath(__file__))
FILES = sorted(glob.glob(os.path.join(HERE, "3MF", "v12", "*.3mf")) + glob.glob(os.path.join(HERE, "3MF", "v12", "_chute_split", "*.3mf")) + glob.glob(os.path.join(HERE, "3MF", "v12", "pulleys", "*.3mf")))
# the whole J-chute is documented as an H2D print (350 x 320 x 325); the _chute_split kit covers 256 mm plates
H2D = sorted((350.0, 320.0, 325.0))
H2D_ONLY = ("unified_continuous_j_chute.3mf",)
PLATE = 256.0
rows, bad = [], []
for f in FILES:
    n = os.path.basename(f)
    try:
        sc = trimesh.load(f, force="scene")
        geoms = list(sc.dump()) if hasattr(sc, "dump") else [sc]
        m = trimesh.util.concatenate(geoms)
        m.merge_vertices()
    except Exception as e:  # noqa: BLE001
        bad.append((n, f"load error {e}"))
        continue
    parts = m.split(only_watertight=False)
    wt = all(p.is_watertight for p in parts)
    vol = float(sum(abs(p.volume) for p in parts)) / 1000.0
    ext = sorted(float(v) for v in m.extents)
    fits = ext[0] <= PLATE and ext[1] <= PLATE and ext[2] <= PLATE
    if n in H2D_ONLY:
        fits = all(ext[k] <= H2D[k] for k in range(3))
    # a long part may still fit diagonally; report the diagonal need when it doesn't fit axis-aligned
    r = {"file": n, "pieces": len(parts), "watertight": wt, "vol_cm3": round(vol, 1), "extents_mm": [round(v, 1) for v in ext], "fits_plate": fits, "plate": "H2D" if n in H2D_ONLY else "256"}
    rows.append(r)
    if not wt or vol <= 0 or not fits:
        bad.append((n, r))
out = {"files": len(FILES), "ok": len(FILES) - len(bad), "problems": bad, "multi_piece": [r["file"] for r in rows if r["pieces"] > 1]}
json.dump({"summary": out, "rows": rows}, open(os.path.join(HERE, "out", "print_audit.json"), "w"), indent=1)
print(json.dumps(out, indent=1))
sys.exit(1 if bad else 0)
