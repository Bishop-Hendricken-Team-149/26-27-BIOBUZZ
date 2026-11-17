"""COTS mount audit (v12.6): is every mounted COTS device actually screwed to something?
For each device occurrence, collect its own fastener-size holes (r 1.4-2.6 mm, any axis). A hole counts as a
screw path when, just past either end of the hole along its axis, the axis is open (a hole) in some OTHER body
while a ring at r + 1.0 mm around it is solid in that body. Devices held otherwise (pods on REX pivot shafts,
motors through quad blocks) are reported with that note. Contact alone never counts.
Run: python cad/fusion_cli.py run-file cad/cots_mount_audit.py --timeout 590"""
import adsk.core as core, adsk.fusion as fusion, math

des = fusion.Design.cast(app.activeProduct)
root = des.rootComponent
IN_ = adsk.fusion.PointContainment.PointInsidePointContainment
P = lambda x, y, z: core.Point3D.create(x / 10.0, y / 10.0, z / 10.0)

DEVICES = ("official_rev_control_hub", "official_rev_expansion_hub", "servo_s1", "servo_s3", "servo_s6",
           "gobilda_nested_battery_mount", "official_gobilda_pinpoint", "rev_color_sensor", "rev_2m_distance",
           "rev_main_power_switch", "gobilda_servo_power_injector", "dfrobot_huskylens")

occs = [root.allOccurrences.item(i) for i in range(root.allOccurrences.count)]
targets = {}
for o in occs:
    if not o.isLightBulbOn:
        continue
    nm = o.name.lower()
    k = next((k for k in DEVICES if nm.startswith(k)), None)
    if not k:
        continue
    par = o.assemblyContext
    if par is not None and par.name.lower().startswith(k):
        continue
    targets[o.fullPathName] = o

# every visible solid body with its owning top occurrence path (proxies = world coordinates)
allb = []
for o in occs:
    if not o.isLightBulbOn:
        continue
    for b in o.bRepBodies:
        if b.isLightBulbOn and b.isSolid:
            bb = b.boundingBox
            allb.append((o.fullPathName, b, (bb.minPoint.x * 10, bb.minPoint.y * 10, bb.minPoint.z * 10,
                                             bb.maxPoint.x * 10, bb.maxPoint.y * 10, bb.maxPoint.z * 10)))


def solid_other(pt, own_prefix):
    for key, b, bb in allb:
        if key.startswith(own_prefix):
            continue
        if bb[0] - .3 <= pt[0] <= bb[3] + .3 and bb[1] - .3 <= pt[1] <= bb[4] + .3 and bb[2] - .3 <= pt[2] <= bb[5] + .3:
            if b.pointContainment(P(*pt)) == IN_:
                return key
    return None


res = {}
for path, o in targets.items():
    holes = {}
    sub, stack = [], [o]
    while stack:
        q = stack.pop()
        sub.append(q)
        stack.extend(q.childOccurrences.item(i) for i in range(q.childOccurrences.count))
    for q in sub:
        for b in q.bRepBodies:
            if not b.isLightBulbOn:
                continue
            for f in b.faces:
                g = f.geometry
                if g.surfaceType != 1 or not (1.4 <= g.radius * 10 <= 2.6):
                    continue
                a = g.axis
                a.normalize()
                ax = (a.x, a.y, a.z)
                fb = f.boundingBox
                lo = (fb.minPoint.x * 10, fb.minPoint.y * 10, fb.minPoint.z * 10)
                hi = (fb.maxPoint.x * 10, fb.maxPoint.y * 10, fb.maxPoint.z * 10)
                c = tuple((lo[i] + hi[i]) / 2 for i in range(3))
                L_ = abs(sum((hi[i] - lo[i]) * ax[i] for i in range(3)))
                key = tuple(round(c[i] - ax[i] * sum(c[j] * ax[j] for j in range(3)), 0) for i in range(3))
                if key not in holes or holes[key][2] < L_:
                    holes[key] = (c, ax, L_, g.radius * 10)
    paths, into = 0, set()
    for c, ax, L_, r in holes.values():
        perp = (ax[1], -ax[0], 0.0) if abs(ax[2]) < 0.9 else (1.0, 0.0, 0.0)
        pn = math.sqrt(sum(v * v for v in perp)) or 1.0
        perp = tuple(v / pn for v in perp)
        hit = None
        for sgn in (1, -1):
            for beyond in (0.6, 1.5, 3.0):
                e = tuple(c[i] + sgn * ax[i] * (L_ / 2 + beyond) for i in range(3))
                p2 = (ax[1] * perp[2] - ax[2] * perp[1], ax[2] * perp[0] - ax[0] * perp[2], ax[0] * perp[1] - ax[1] * perp[0])
                ks = [solid_other(tuple(e[i] + d[i] * (r + 1.0) for i in range(3)), path) for d in (perp, p2, tuple(-v for v in perp), tuple(-v for v in p2))]
                ks = [x for x in ks if x]
                k = ks[0] if len(ks) >= 2 else None      # material on at least 2 of 4 sides of the hole
                if k and not solid_other(e, path):
                    hit = k
                    break
            if hit:
                break
        if hit:
            paths += 1
            into.add(hit.split("+")[-1][-45:])
    res[path[-70:]] = {"holes": len(holes), "screw_paths": paths, "into": sorted(into)[:4]}
result = res
