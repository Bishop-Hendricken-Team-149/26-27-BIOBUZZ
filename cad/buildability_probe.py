"""Buildability / wiring probe for HIVE-QUEEN v12.2 (live Fusion doc).
- custom (printed/machined) bodies: count concave cylindrical faces with r 1.4-3.2 mm (fastener holes)
- electrical device positions (AABB centres) for cable-run estimates
- hub/battery clearance boxes (free space in front of connector faces / removal path)
Run: python cad/fusion_cli.py run-file cad/buildability_probe.py --timeout 400
"""
import math
import adsk.core as core, adsk.fusion as fusion

tb = fusion.TemporaryBRepManager.get()
INTER = fusion.BooleanTypes.IntersectionBooleanType
CYL = core.SurfaceTypes.CylinderSurfaceType

bodies = []
occs = []
def walk(o, path):
    if not o.isLightBulbOn:
        return
    p = path + "/" + o.name
    occs.append((p, o))
    for i in range(o.bRepBodies.count):
        b = o.bRepBodies.item(i)
        if b.isLightBulbOn and b.isSolid:
            bb = b.boundingBox
            bodies.append((p, b.name, b, [bb.minPoint.x * 10, bb.minPoint.y * 10, bb.minPoint.z * 10, bb.maxPoint.x * 10, bb.maxPoint.y * 10, bb.maxPoint.z * 10]))
    for i in range(o.childOccurrences.count):
        walk(o.childOccurrences.item(i), p)
for i in range(root.occurrences.count):
    walk(root.occurrences.item(i), "")

def is_custom(p):
    s = p.split("/")
    return len(s) > 1 and s[1].startswith("HIVE_QUEEN_v12")

NONPRINT = ("rex_shaft", "bearing", "belt", "pulley", "spring", "pin", "stub", "collar", "spacer", "miter", "staged_", "sensor", "sign_")
holes = {}
for p, n, b, bb in bodies:
    if not is_custom(p) or any(k in n for k in NONPRINT):
        continue
    cnt = 0
    for f in b.faces:
        if f.geometry.surfaceType == CYL:
            r = f.geometry.radius * 10
            if 1.4 <= r <= 3.2:
                # concave (hole) if the face normal at a point points toward the axis
                ev = f.evaluator
                ok, pt = ev.getPointAtParameter(core.Point2D.create(0.5 * (ev.parametricRange().minPoint.x + ev.parametricRange().maxPoint.x),
                                                                    0.5 * (ev.parametricRange().minPoint.y + ev.parametricRange().maxPoint.y)))
                ok2, nrm = ev.getNormalAtPoint(pt)
                ax = f.geometry.axis; org = f.geometry.origin
                v = core.Vector3D.create(pt.x - org.x, pt.y - org.y, pt.z - org.z)
                along = ax.copy(); along.normalize(); along.scaleBy(v.dotProduct(along))
                radial = v.copy(); radial.subtract(along)
                if radial.dotProduct(nrm) < 0:
                    cnt += 1
    holes[n] = cnt

dev_kw = ["control_hub", "expansion_hub", "battery", "power_switch", "servo_power_injector", "pinpoint", "odometry_pod",
          "huskylens", "5203_motor", "servo_s", "color_sensor", "ball_count_sensor", "webcam"]
devices = {}
for p, o in occs:
    nm = o.name.lower()
    for kw in dev_kw:
        if kw in nm and o.bRepBodies.count + o.childOccurrences.count > 0:
            bb = o.boundingBox
            devices[o.name] = [round(v, 1) for v in ((bb.minPoint.x + bb.maxPoint.x) * 5, (bb.minPoint.y + bb.maxPoint.y) * 5, (bb.minPoint.z + bb.maxPoint.z) * 5)] + \
                              [round(v, 1) for v in (bb.minPoint.x * 10, bb.minPoint.y * 10, bb.minPoint.z * 10, bb.maxPoint.x * 10, bb.maxPoint.y * 10, bb.maxPoint.z * 10)]
            break
for p, n, b, bb in bodies:   # custom sensors are bodies, not occurrences
    if is_custom(p) and ("sensor" in n or "switch" in n or "injector" in n):
        devices[n] = [round((bb[0] + bb[3]) / 2, 1), round((bb[1] + bb[4]) / 2, 1), round((bb[2] + bb[5]) / 2, 1)] + [round(v, 1) for v in bb]

def free_box(x0, x1, y0, y1, z0, z1, skip=()):
    c = tb.createBox(core.OrientedBoundingBox3D.create(core.Point3D.create((x0 + x1) / 20, (y0 + y1) / 20, (z0 + z1) / 20),
                     core.Vector3D.create(1, 0, 0), core.Vector3D.create(0, 1, 0), (x1 - x0) / 10, (y1 - y0) / 10, (z1 - z0) / 10))
    hits = []
    for p, n, b, bb in bodies:
        if any(k in (p + "/" + n).lower() for k in skip):
            continue
        if not (bb[0] < x1 and bb[3] > x0 and bb[1] < y1 and bb[4] > y0 and bb[2] < z1 and bb[5] > z0):
            continue
        d = tb.copy(b)
        try:
            tb.booleanOperation(d, tb.copy(c), INTER)
            v = d.volume * 1000
        except Exception:
            v = -1
        if v > 1.0 or v < 0:
            hits.append([(n or p.split("/")[-1])[:60], round(v, 1)])
    return hits

clear = {}
for key in ("official_rev_control_hub_31_1595", "official_rev_expansion_hub_31_1153", "official_gobilda_12v_battery_14_0014"):
    d = next((v for k, v in devices.items() if key in k), None)
    if not d:
        continue
    x0, y0, z0, x1, y1, z1 = d[3:]
    if "battery" in key:
        # removal: straight up from the battery to the top of the robot
        clear[key + "_lift_out_up"] = free_box(x0, x1, y0, y1, z1 + 0.5, 460, skip=("battery", "strap"))
        clear[key + "_slide_out_rear"] = free_box(x0 - 250, x0 - 0.5, y0, y1, z0, z1, skip=("battery", "strap"))
    else:
        # connector access: 30 mm of free space beyond each of the 4 vertical side faces
        clear[key + "_xmin"] = free_box(x0 - 30, x0 - 0.5, y0, y1, z0, z1, skip=(key,))
        clear[key + "_xmax"] = free_box(x1 + 0.5, x1 + 30, y0, y1, z0, z1, skip=(key,))
        clear[key + "_ymin"] = free_box(x0, x1, y0 - 30, y0 - 0.5, z0, z1, skip=(key,))
        clear[key + "_ymax"] = free_box(x0, x1, y1 + 0.5, y1 + 30, z0, z1, skip=(key,))
        clear[key + "_top"] = free_box(x0, x1, y0, y1, z1 + 0.5, z1 + 30, skip=(key,))

# lowest non-wheel, non-pod geometry (ground clearance)
low = sorted(((bb[2], n or p.split("/")[-1]) for p, n, b, bb in bodies
              if not any(k in (p + "/" + n).lower() for k in ("mecanum", "roller", "odometry_pod", "staged_", "wheel", "omni"))), key=lambda t: t[0])[:12]
result = {"printed_holes": holes, "devices": devices, "clearance_hits": clear, "lowest_parts": [[round(z, 2), n[:60]] for z, n in low]}
