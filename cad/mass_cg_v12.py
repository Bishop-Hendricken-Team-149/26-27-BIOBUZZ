"""Mass / CG / static-stability estimate for HIVE-QUEEN (v12.6) from the live Fusion model.
COTS electronics, motors and wheels use known catalogue masses spread over their bodies by volume.
Other COTS use aluminium (channel/plate) or steel (shafts, bearings, hardware) density on their solid
volume. Custom bodies use a density from their builder appearance tag (printed parts at ~55% of solid
for 40% infill + 3 walls).
Run: python cad/fusion_cli.py run-file cad/mass_cg_v12.py --timeout 300
"""
import math, json
import adsk.core as core, adsk.fusion as fusion

KNOWN_KG = [  # (path keyword, kg per occurrence)
    ("gobilda_5203_motor", 0.330), ("5203_motor", 0.330), ("mecanum", 0.270), ("battery_mount", 0.045), ("battery", 0.620),
    ("control_hub", 0.200), ("expansion_hub", 0.180), ("servo_s", 0.070), ("pinpoint", 0.020),
    ("odometry_pod", 0.050), ("huskylens", 0.050), ("gecko_wheel", 0.030), ("rhino", 0.110),
    ("counter_wheel", 0.080), ("color_sensor", 0.010), ("power_switch", 0.050), ("servo_power_injector", 0.040),
]
PRINT_EFF = 0.55
CUSTOM_DENSITY = {  # g/cm3 by appearance name (builder palette)
    "silver": 7.85, "bearing_steel": 7.85 * 0.55, "spring_steel": 7.85 * 0.25, "stop_alu": 2.70,
    "belt_black": 1.20, "rubber": 1.20, "brass": 8.50, "pulley_alu": 2.70 * 0.6,
    "compliant_teal": 1.20 * 0.5,
}
PRINTED = 1.25 * PRINT_EFF


def occ_mass_kw(path):
    p = path.lower()
    for kw, kg in KNOWN_KG:
        if kw in p:
            return kw, kg
    return None, None


bodies = []
def walk(o, path):
    if not o.isLightBulbOn:
        return
    p = path + "/" + o.name
    for i in range(o.bRepBodies.count):
        b = o.bRepBodies.item(i)
        if b.isLightBulbOn and b.isSolid:
            bodies.append((p, b))
    for i in range(o.childOccurrences.count):
        walk(o.childOccurrences.item(i), p)
for i in range(root.occurrences.count):
    walk(root.occurrences.item(i), "")

# group volumes of known-mass occurrences by their outermost matching occurrence path
groups = {}
rows = []
for p, b in bodies:
    if b.name.startswith("staged_"):
        continue
    pp = b.physicalProperties
    v = pp.volume  # cm3
    c = pp.centerOfMass
    cx, cy, cz = c.x * 10, c.y * 10, c.z * 10
    is_custom = p.split("/")[1].startswith("HIVE_QUEEN_v12") if len(p.split("/")) > 1 else False
    kw, kg = (None, None) if is_custom else occ_mass_kw(p)
    if kw:
        segs = p.split("/")
        key = "/".join(segs[: next(i for i, s in enumerate(segs) if kw in s.lower()) + 1])
        groups.setdefault(key, [kg, []])[1].append((v, cx, cy, cz))
        continue
    if is_custom:
        ap = (b.appearance.name if b.appearance else "").replace("Hermes_v12_", "")
        rho = CUSTOM_DENSITY.get(ap, PRINTED)
        cls = "custom_" + ("printed" if ap not in CUSTOM_DENSITY else ap)
    else:
        nm = (p + "/" + b.name).lower()
        steel = any(k in nm for k in ("shaft", "bearing", "screw", "bolt", "nut", "collar", "spacer", "gear", "axle", "hub_"))
        rho = 7.85 * (0.6 if "bearing" in nm else 1.0) if steel else 2.70
        cls = "cots_steel" if steel else "cots_alu"
    rows.append((cls, v * rho / 1000.0, cx, cy, cz))
for key, (kg, parts) in groups.items():
    vt = sum(v for v, *_ in parts) or 1.0
    for v, cx, cy, cz in parts:
        rows.append(("cots_known:" + key.split("/")[-1].split(":")[0], kg * v / vt, cx, cy, cz))

M = sum(r[1] for r in rows)
G = [sum(r[1] * r[k] for r in rows) / M for k in (2, 3, 4)]
by = {}
for r in rows:
    k = r[0] if not r[0].startswith("cots_known") else "cots_known"
    by[k] = by.get(k, 0) + r[1]

# wheel contact points: mecanum wheel occurrences' bbox centres projected to the floor
wheels = []
def wwalk(o):
    if not o.isLightBulbOn:
        return
    if "mecanum" in o.name.lower() and o.bRepBodies.count + o.childOccurrences.count > 0:
        bb = o.boundingBox
        wheels.append(((bb.minPoint.x + bb.maxPoint.x) * 5, (bb.minPoint.y + bb.maxPoint.y) * 5, bb.minPoint.z * 10))
        return
    for i in range(o.childOccurrences.count):
        wwalk(o.childOccurrences.item(i))
for i in range(root.occurrences.count):
    wwalk(root.occurrences.item(i))
xs = sorted(w[0] for w in wheels); ys = sorted(w[1] for w in wheels)
x_front, x_rear = max(xs), min(xs); y_left, y_right = max(ys), min(ys)
h = G[2]
tip = {
    "forward_deg": math.degrees(math.atan2(x_front - G[0], h)),
    "backward_deg": math.degrees(math.atan2(G[0] - x_rear, h)),
    "left_deg": math.degrees(math.atan2(y_left - G[1], h)),
    "right_deg": math.degrees(math.atan2(G[1] - y_right, h)),
}
# max braking/accel before tipping (static): a/g = lever/h
a_tip_g = min((x_front - G[0]) / h, (G[0] - x_rear) / h)
front_load = (G[0] - x_rear) / (x_front - x_rear)
result = {
    "mass_kg": round(M, 2), "mass_lb": round(M * 2.2046, 1), "cg_mm": [round(v, 1) for v in G],
    "by_class_kg": {k: round(v, 2) for k, v in sorted(by.items(), key=lambda t: -t[1])},
    "n_wheels": len(wheels), "wheel_contacts_mm": [[round(a, 1) for a in w] for w in wheels],
    "wheelbase_mm": round(x_front - x_rear, 1), "track_mm": round(y_left - y_right, 1),
    "static_tip_angles_deg": {k: round(v, 1) for k, v in tip.items()},
    "accel_to_tip_g": round(a_tip_g, 2), "front_axle_load_fraction": round(front_load, 3),
    "left_right_offset_mm": round(G[1] - (y_left + y_right) / 2, 1),
}
