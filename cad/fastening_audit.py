"""Fastening audit (v12.6): every custom part that isn't motion hardware must have at least one screw path, i.e. a
hole (r 1.4-3.0 mm: M3, M4 and goBILDA M4 clearance r 2.8) whose axis, just past the part, runs into a hole in ANOTHER body with material around it.
Contact alone never counts. Run through the Fusion bridge on the built document."""
import adsk.core as core, adsk.fusion as fusion, math
des = fusion.Design.cast(app.activeProduct)
root = des.rootComponent
IN_ = adsk.fusion.PointContainment.PointInsidePointContainment
P = lambda x, y, z: core.Point3D.create(x / 10.0, y / 10.0, z / 10.0)
# v12.6: classify by explicit category, never by a bare substring skip. (A skip list containing "wheel" and
# "bearing" silently hid the flywheel motor saddles and the intake bearing/stop plates, which had no holes.)
# Whole-word tokens only: substrings bite ("wing" is inside "swing_arm", "block" inside "blocker").
MOTION_HW = {"shaft", "jackshaft", "stub", "spacer", "collar", "coupler", "pin", "spring", "belt", "pulley", "axle", "bearing"}
STRUCT = {"plate", "saddle", "mount", "bracket", "cradle", "cage", "housing", "shroud", "wing", "shield", "chute",
          "deck", "rail", "block"}
MOVING = {"arm", "gate", "hood", "fingers", "wheel"}


def category(n):
    t = set(n.lower().split("_"))
    if n.startswith("staged_"):
        return "exempt: staged game piece"
    if n.startswith("robot_sign_") and "rail" not in t:
        return "exempt: sign plate slides into its bolted rails"
    if t & STRUCT and not (t & {"bearing"} and not t & {"stop"}):
        return "check"
    if t & MOTION_HW:
        return "exempt: shaft-borne motion hardware (held by its shaft/bore)"
    if t & MOVING:
        return "moving"
    return "check"


up = next(root.occurrences.item(i) for i in range(root.occurrences.count) if root.occurrences.item(i).component.name == "HIVE_QUEEN_v12_Remediation")
allb = []
for i in range(root.allOccurrences.count):
    o = root.allOccurrences.item(i)
    if not o.isLightBulbOn or not o.isVisible:
        continue
    for b in o.bRepBodies:
        if b.isLightBulbOn and b.isSolid:
            bb = b.boundingBox
            allb.append((o.fullPathName + "/" + b.name, b, (bb.minPoint.x * 10, bb.minPoint.y * 10, bb.minPoint.z * 10, bb.maxPoint.x * 10, bb.maxPoint.y * 10, bb.maxPoint.z * 10)))
for b in up.bRepBodies:
    if b.isLightBulbOn and b.isSolid:
        bb = b.boundingBox
        allb.append(("UP/" + b.name, b, (bb.minPoint.x * 10, bb.minPoint.y * 10, bb.minPoint.z * 10, bb.maxPoint.x * 10, bb.maxPoint.y * 10, bb.maxPoint.z * 10)))


def solid_other(pt, me):
    for key, b, bb in allb:
        if b is me or key == "UP/" + me.name:
            continue
        if bb[0] - .5 <= pt[0] <= bb[3] + .5 and bb[1] - .5 <= pt[1] <= bb[4] + .5 and bb[2] - .5 <= pt[2] <= bb[5] + .5:
            if b.pointContainment(P(*pt)) == IN_:
                return key
    return None


out, unfastened = {}, []
ONLY = globals().get('ONLY')
for b in up.bRepBodies:
    n = b.name
    cat = category(n)
    if not b.isLightBulbOn or cat.startswith("exempt") or (ONLY and n not in ONLY):
        continue
    holes = {}
    for f in b.faces:
        g = f.geometry
        if g.surfaceType != 1 or not (1.4 <= g.radius * 10 <= 3.0):
            continue
        a = g.axis
        a.normalize()
        fb = f.boundingBox
        lo = (fb.minPoint.x * 10, fb.minPoint.y * 10, fb.minPoint.z * 10)
        hi = (fb.maxPoint.x * 10, fb.maxPoint.y * 10, fb.maxPoint.z * 10)
        ax = (a.x, a.y, a.z)
        c = tuple((lo[i] + hi[i]) / 2 for i in range(3))
        L_ = abs(sum((hi[i] - lo[i]) * ax[i] for i in range(3)))
        holes.setdefault(tuple(round(c[i] - ax[i] * sum(c[j] * ax[j] for j in range(3)), 0) for i in range(3)), (c, ax, L_, g.radius * 10))
    paths, into = 0, set()
    for c, ax, L_, r in holes.values():
        perp = (ax[1], -ax[0], 0.0) if abs(ax[2]) < 0.9 else (1.0, 0.0, 0.0)
        pn = math.sqrt(sum(v * v for v in perp)) or 1.0
        perp = tuple(v / pn for v in perp)
        hit = None
        for sgn in (1, -1):
            for beyond in (1.0, 2.5, 4.0):
                e = tuple(c[i] + sgn * ax[i] * (L_ / 2 + beyond) for i in range(3))
                p2 = (ax[1] * perp[2] - ax[2] * perp[1], ax[2] * perp[0] - ax[0] * perp[2], ax[0] * perp[1] - ax[1] * perp[0])
                ks = [solid_other(tuple(e[i] + d[i] * (r + 1.0) for i in range(3)), b) for d in (perp, p2, tuple(-v for v in perp), tuple(-v for v in p2))]
                ks = [x for x in ks if x]
                k = ks[0] if len(ks) >= 2 else None      # material on at least 2 of 4 sides of the hole
                if k and not solid_other(e, b):
                    hit = k
                    break
            if hit:
                break
        if hit:
            paths += 1
            into.add(hit.split("/")[0][-40:] if not hit.startswith("UP/") else hit[3:])
    bores_ = sorted({round(f.geometry.radius * 10, 2) for f in b.faces if f.geometry.surfaceType == 1 and 3.9 < f.geometry.radius * 10 < 8.0})
    out[n] = {"holes": len(holes), "paths": paths, "into": sorted(into)[:4], "category": cat, "bores": bores_}
    if paths == 0 and not (cat == "moving" and bores_):
        unfastened.append(n)
bores = {}
for b in up.bRepBodies:
    if b.name in unfastened:
        bores[b.name] = sorted({round(f.geometry.radius * 10, 2) for f in b.faces if f.geometry.surfaceType == 1 and 3.0 < f.geometry.radius * 10 < 8.0})
result = {"pivoting_on_bores": sorted(k for k, v in out.items() if v["category"] == "moving" and v["paths"] == 0 and v["bores"]), "bores_of_unfastened": bores, "checked": len(out), "unfastened": unfastened, "detail": {k: v for k, v in out.items() if v["paths"] < 2}}
