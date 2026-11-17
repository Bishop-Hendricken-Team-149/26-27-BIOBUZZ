"""Convert the J-chute print-split STLs (cad/export_chute_split.py) to 3MF and verify them.
Run from the scratch dir: uv run -q --with trimesh --with lxml --with networkx python <repo>/cad/build_chute_split_3mf.py
"""
import glob, os, trimesh
D = "C:/Users/danu0/Downloads/26-27-BIOBUZZ/cad/3MF/v12/_chute_split"
ok = True
for f in sorted(glob.glob(D + "/*.stl")):
    m = trimesh.load(f)
    m.merge_vertices()
    ext = sorted(m.extents, reverse=True)
    fits = ext[0] <= 250 and ext[1] <= 250 and ext[2] <= 250
    out = f[:-4] + ".3mf"
    m.export(out)
    back = trimesh.load(out, force="mesh")
    good = m.is_watertight and back.is_watertight and fits and abs(back.volume - m.volume) < 1.0
    ok &= good
    print(os.path.basename(out), [round(float(e), 1) for e in ext], "cm3", round(m.volume / 1000, 2), "watertight", m.is_watertight, "fits256", fits, "OK" if good else "FAIL")
print("ALL OK" if ok else "FAILURES")
