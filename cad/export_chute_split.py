"""Print-only split of unified_continuous_j_chute into 3 pieces that each fit a 256 mm Bambu bed.

The assembly keeps the single gate-verified chute body. Each piece contains its own frame
mounting station, so the pieces are located and held by the frame rather than by the seams:
  upper_wall      Z >= 150           rear collar (station 3) bolts to the upper rear crossbar face
  bend_lower      X <= 40, Z < 150   mid cross-channel saddle (station 2) + gate-shaft hanger (station 4)
  front_ramp L/R  X >= 40, split at Y = 0 so it prints flat; each half has its own front side hanger
                  (station 1) bolted to a rail inner wall
Seams are perpendicular to ball travel. Bond them flush (epoxy or VHB on the outside face).
Writes binary STL per piece; cad/build_chute_split_3mf.py converts them to 3MF.
Run through the bridge: python cad/fusion_cli.py run-file cad/export_chute_split.py --timeout 300
"""
import math, struct, os
import adsk.core as core, adsk.fusion as fusion

OUT = "C:/Users/danu0/Downloads/26-27-BIOBUZZ/cad/3MF/v12/_chute_split"
os.makedirs(OUT, exist_ok=True)
tb = fusion.TemporaryBRepManager.get()
INTER = fusion.BooleanTypes.IntersectionBooleanType


def find(name):
    hit = []
    def walk(o):
        for i in range(o.bRepBodies.count):
            b = o.bRepBodies.item(i)
            if b.name == name and b.isLightBulbOn:
                hit.append(b)
        for i in range(o.childOccurrences.count):
            walk(o.childOccurrences.item(i))
    for i in range(root.occurrences.count):
        walk(root.occurrences.item(i))
    return hit[0]


def box(x0, x1, y0, y1, z0, z1):
    return tb.createBox(core.OrientedBoundingBox3D.create(
        core.Point3D.create((x0 + x1) / 20, (y0 + y1) / 20, (z0 + z1) / 20),
        core.Vector3D.create(1, 0, 0), core.Vector3D.create(0, 1, 0), (x1 - x0) / 10, (y1 - y0) / 10, (z1 - z0) / 10))


chute = find("unified_continuous_j_chute")
# the occurrence transform of the chute's component is identity in this build (BaseFeature in a top-level comp)
pieces = {
    "j_chute_part1_upper_wall": box(-300, 300, -300, 300, 150, 500),
    "j_chute_part2_bend_lower": box(-300, 40, -300, 300, -50, 150),
    "j_chute_part3L_front_ramp_left": box(40, 300, 0, 300, -50, 150),     # left hanger (station 1L)
    "j_chute_part3R_front_ramp_right": box(40, 300, -300, 0, -50, 150),   # right hanger (station 1R)
}
out = {"chute_cm3": round(chute.volume, 3), "pieces": {}}
tot = 0.0
for nm, cutter in pieces.items():
    c = tb.copy(chute)
    tb.booleanOperation(c, cutter, INTER)
    tot += c.volume
    bb = c.boundingBox
    ext = sorted([(bb.maxPoint.x - bb.minPoint.x) * 10, (bb.maxPoint.y - bb.minPoint.y) * 10, (bb.maxPoint.z - bb.minPoint.z) * 10], reverse=True)
    mc = c.meshManager.createMeshCalculator()
    mc.setQuality(fusion.TriangleMeshQualityOptions.VeryHighQualityTriangleMesh)
    mesh = mc.calculate()
    co = mesh.nodeCoordinatesAsDouble
    idx = mesh.nodeIndices
    nt = len(idx) // 3
    path = f"{OUT}/{nm}.stl"
    with open(path, "wb") as f:
        f.write(b"HIVE-QUEEN v12.2 J-chute print split".ljust(80, b" "))
        f.write(struct.pack("<I", nt))
        for t in range(nt):
            p = [(co[3 * idx[3 * t + k]] * 10, co[3 * idx[3 * t + k] + 1] * 10, co[3 * idx[3 * t + k] + 2] * 10) for k in range(3)]
            ux, uy, uz = (p[1][j] - p[0][j] for j in range(3))
            vx, vy, vz = (p[2][j] - p[0][j] for j in range(3))
            n = (uy * vz - uz * vy, uz * vx - ux * vz, ux * vy - uy * vx)
            ln = math.sqrt(sum(a * a for a in n)) or 1.0
            f.write(struct.pack("<12fH", n[0] / ln, n[1] / ln, n[2] / ln, *p[0], *p[1], *p[2], 0))
    out["pieces"][nm] = {"lumps": c.lumps.count, "cm3": round(c.volume, 3), "extents_mm": [round(e, 1) for e in ext], "triangles": nt}
out["sum_cm3"] = round(tot, 3)
out["volume_conserved"] = abs(tot - chute.volume) < 1e-3
result = out
