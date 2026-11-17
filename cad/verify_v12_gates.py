"""
HIVE-QUEEN v12 acceptance gates (docs/04 section 5). Runs inside Fusion 360 on the final document:
  python cad/fusion_cli.py run-file cad/verify_v12_gates.py --timeout 900

Writes cad/out/v12_gate_report.json and returns the same dict.
"""
import json
import math
import time
import adsk.core as core
import adsk.fusion as fusion

t0 = time.time()
L = {"__name__": "v12_design"}
exec(open("C:/Users/danu0/Downloads/26-27-BIOBUZZ/cad/v12_design.py", encoding="utf-8").read(), L)
tb = fusion.TemporaryBRepManager.get()
INTER = fusion.BooleanTypes.IntersectionBooleanType
EPS = 0.01   # mm^3 numerical noise floor

up_occ = None
for i in range(root.occurrences.count):
    o = root.occurrences.item(i)
    if o.isLightBulbOn and o.component.name == "HIVE_QUEEN_v12_Remediation":
        up_occ = o
up_comp = up_occ.component


def P(x, y, z):
    return core.Point3D.create(x / 10.0, y / 10.0, z / 10.0)


# ---------------------------------------------------------------- collect bodies
bodies = []   # (path, name, body, aabb_mm)


def collect(occ, path=""):
    cur = f"{path}/{occ.name}" if path else occ.name
    if not occ.isLightBulbOn:
        return
    for i in range(occ.bRepBodies.count):
        b = occ.bRepBodies.item(i)
        if b.isLightBulbOn and b.isSolid:
            bb = b.boundingBox
            bodies.append((cur, b.name, b, (bb.minPoint.x * 10, bb.minPoint.y * 10, bb.minPoint.z * 10, bb.maxPoint.x * 10, bb.maxPoint.y * 10, bb.maxPoint.z * 10)))
    for i in range(occ.childOccurrences.count):
        collect(occ.childOccurrences.item(i), cur)


for i in range(root.occurrences.count):
    collect(root.occurrences.item(i))


def group(path, bname):
    parts = path.split("/")
    if parts[0] == up_occ.name:
        return bname
    if parts[0] == "robot_base_core:1" and len(parts) >= 3:
        return parts[2]
    if parts[0] in ("turret_launcher_assembly:1", "stinger_flower_capper_arm:1") and len(parts) >= 2:
        return parts[1]
    return parts[0]


def bb_hit(a, b, tol=0.01):
    return a[0] < b[3] - tol and a[3] > b[0] + tol and a[1] < b[4] - tol and a[4] > b[1] + tol and a[2] < b[5] - tol and a[5] > b[2] + tol


def ivol(b1, b2):
    c1 = tb.copy(b1)
    tb.booleanOperation(c1, tb.copy(b2), INTER)
    return c1.volume * 1000.0


# ---------------------------------------------------------------- Gate 1: unfiltered overlaps
groups = [group(p, n) for p, n, _, _ in bodies]
ov = {}
for i in range(len(bodies)):
    for j in range(i + 1, len(bodies)):
        if groups[i] == groups[j] or not bb_hit(bodies[i][3], bodies[j][3]):
            continue
        try:
            v = ivol(bodies[i][2], bodies[j][2])
        except Exception as e:
            v = -1.0
        if v > EPS or v < 0:
            k = f"{groups[i]} || {groups[j]}"
            ov[k] = round(ov.get(k, 0.0) + v, 3)
gate1 = sorted(ov.items(), key=lambda kv: -kv[1])

# ---------------------------------------------------------------- Gate 2: lumps / volume
gate2 = [(b.name, b.lumps.count, round(b.volume * 1000, 1)) for b in up_comp.bRepBodies if b.lumps.count != 1 or b.volume * 1000 < 250.0]


# ---------------------------------------------------------------- Gate 3/4: envelopes
def env(items):
    mn = [min(a[k] for a in items) for k in range(3)]
    mx = [max(a[k] for a in items) for k in range(3, 6)]
    return [round(v, 2) for v in mn], [round(v, 2) for v in mx], [round(mx[k] - mn[k], 2) for k in range(3)]


stowed = [bb for p, n, b, bb in bodies if not n.startswith("staged_")]
s_mn, s_mx, s_sz = env(stowed)
gate3 = all(s_sz[k] <= L["R102_MAX"][k] for k in range(3))

PV = L["INTAKE_PIVOT"]
d_int = L["ang"](PV, L["INTAKE_SHAFT_HIGH"]) - L["ang"](PV, L["INTAKE_SHAFT_LOW"])
INTAKE_MOVERS = ("floating_intake_swing_arm", "intake_floating_front_rex_shaft", "floating_spoked_compliant_wheel", "intake_arm_front_bearing",
                 "intake_stage2_floating_pulley", "intake_stage2_htd5m")
expanded = []
for p, n, b, bb in bodies:
    if n.startswith("staged_"):
        continue
    rot = None
    if n.startswith(INTAKE_MOVERS) or "star_roller_sleeve" in p:
        rot = (PV, d_int)
    if rot is None:
        expanded.append(bb)
        continue
    c = tb.copy(b)
    m = core.Matrix3D.create()
    m.setToRotation(math.radians(rot[1]), core.Vector3D.create(0, -1, 0), P(rot[0][0], 0.0, rot[0][1]))
    tb.transform(c, m)
    cb = c.boundingBox
    expanded.append((cb.minPoint.x * 10, cb.minPoint.y * 10, cb.minPoint.z * 10, cb.maxPoint.x * 10, cb.maxPoint.y * 10, cb.maxPoint.z * 10))
e_mn, e_mx, e_sz = env(expanded)
gate4 = all(e_sz[k] < L["R105_MAX"][k] for k in range(3))

# ---------------------------------------------------------------- Gate 5: Nectar swept spheres
R = L["NECTAR_R"]
FW = L["FRONT_WHEEL"]
x_nip = FW[0] - L["FRONT_WHEEL_D"] / 2 - R - 0.5
stations = {
    "1_intake_mouth": (215.0, R),
    "2_floor_wedge": L["ramp_ball_center"](178.0, R),
    "3_lower_ramp": L["ramp_ball_center"](120.0, R),
    "4_center_crossbar": L["ramp_ball_center"](-24.0, R),
    "5_j_elbow": L["bend_ball_center"](215.0, R),
    "6_deck_throat": (L["WALL_X"] + R + 0.02, 219.0),
    "7_open_gate": L["bend_ball_center"](L["GATE_A"], R),
    "8_expanded_nip": (x_nip, 324.0),
    "9_exit_past_hood_hinge_z382": (x_nip, 382.0),
    "10_exit_past_hood_hinge_z415": (x_nip, 415.0),
}
MOVING = ("staged_", "floating_", "intake_floating_", "intake_arm_front_bearing", "intake_stage2_floating", "intake_stage2_htd5m",
          "gecko", "magazine_b1", "magazine_b3", "indexer_stopper_gate", "shooter_counter", "shooter_coaxial_swing_arm",
          "shooter_swing_arm", "shooter_right_counter", "shooter_right_htd5m", "shooter_spring", "shooter_return",
          "variable_pitch_exit_hood_full")
MOVING_COTS = ("gobilda_gecko", "star_roller", "gobilda_rhino", "gobilda_steel_flywheel", "gobilda_counter_wheel",
               "gobilda_flywheel_plate", "gobilda_1313_flywheel_hub")   # v12.6: real flywheel plates on 1313 hubs
HH = L["HOOD_HINGE"]
hinge_zone = tb.createCylinderOrCone(P(HH[0], -70, HH[1]), 1.2, P(HH[0], 70, HH[1]), 1.2)
gate5 = {}
for sname, c in stations.items():
    sph = tb.createSphere(P(c[0], 0.0, c[1]), R / 10.0)
    sbb = (c[0] - R, -R, c[1] - R, c[0] + R, R, c[1] + R)
    hits = []
    for p, n, b, bb in bodies:
        if n.startswith(MOVING) and n != "variable_pitch_exit_hood_full":
            continue
        if any(k in p for k in MOVING_COTS):
            continue
        if not bb_hit(sbb, bb):
            continue
        if n == "variable_pitch_exit_hood_full":
            hz = tb.copy(b)
            tb.booleanOperation(hz, tb.copy(hinge_zone), INTER)
            v = ivol(sph, hz) if hz.volume > 0 else 0.0
        else:
            v = ivol(sph, b)
        if v > EPS:
            hits.append([group(p, n), round(v, 2)])
    gate5[sname] = {"center_xz": [round(c[0], 2), round(c[1], 2)], "hits": hits}
gate5_pass = all(not v["hits"] for v in gate5.values())

# ---------------------------------------------------------------- Gate 6: floor contact
# v12.3: TIGHT z-min (oriented bounding box). Fusion's body.boundingBox is NOT tight on curved B-reps:
# a pod's omni-wheel side plate reported Z -0.24 while its real lowest point was ~9 mm, so the old gate
# "passed" pods whose tyres were 6 mm in the air. For pods only the TYRES count as floor contact.
import adsk.core as _core
_mm = app.measureManager
_VX, _VY = _core.Vector3D.create(1, 0, 0), _core.Vector3D.create(0, 1, 0)


def _tight_zmin(body):
    ob = _mm.getOrientedBoundingBox(body, _VY, _VX)
    return (ob.centerPoint.z - ob.height / 2.0) * 10.0


zmin = {}
for p, n, b, bb in bodies:
    for key in ("gobilda_mecanum_96mm", "official_gobilda_4bar_odometry_pod"):
        if key in p:
            if key.startswith("official_gobilda_4bar") and "/Tire" not in p:
                continue
            seg = [s for s in p.split("/") if s.startswith(key)][0]
            zmin[seg] = round(min(zmin.get(seg, 1e9), _tight_zmin(b)), 3)
    if n.startswith("floating_spoked_compliant_wheel"):
        zmin[n] = round(bb[2], 3)
pods = sorted(k for k in zmin if "odometry_pod" in k)
gate6 = all(-0.5 <= v <= 0.2 for k, v in zmin.items() if "mecanum" in k or "odometry" in k) and \
    all(v <= 56.0 for k, v in zmin.items() if k.startswith("floating_spoked")) and len(pods) == 2

# ---------------------------------------------------------------- Gate 7: keyholes / bearings
SLOTS = {
    "turret_side_plate_left_clean": [(L["M6_SHAFT"], L["COUNTER_ARM_R"], L["COUNTER_A_REST"], L["COUNTER_A_MAX"])],
    "turret_side_plate_right_clean": [(L["M6_SHAFT"], L["COUNTER_ARM_R"], L["COUNTER_A_REST"], L["COUNTER_A_MAX"])],
    "turret_outboard_double_shear_plate_right": [(L["M6_SHAFT"], L["COUNTER_ARM_R"], L["COUNTER_A_REST"], L["COUNTER_A_MAX"])],
    "sprung_magazine_shroud_left": [(L["J2"], L["G2_R"], L["G2_A_LIFT"] - 2.0, L["G2_A_REST"])],
    "sprung_magazine_shroud_right": [(L["J2"], L["G2_R"], L["G2_A_LIFT"] - 2.0, L["G2_A_REST"]), (L["M8_SHAFT"], L["G1_R"], L["G1_A_REST"], L["G1_A_LIFT"] + 2.0)],
    "magazine_outboard_double_shear_plate_right": [],
}


def arc_dist(p, c, Rr, a0, a1):
    a = math.degrees(math.atan2(p[1] - c[1], p[0] - c[0]))
    a = min(max(a, a0), a1) if a0 <= a <= a1 or True else a
    best = 1e9
    for aa in (min(max(math.degrees(math.atan2(p[1] - c[1], p[0] - c[0])), a0), a1), a0, a1):
        q = L["polar"](c, Rr, aa)
        best = min(best, math.hypot(p[0] - q[0], p[1] - q[1]))
    return best


keyholes = []
for bn, slots in SLOTS.items():
    b = up_comp.bRepBodies.itemByName(bn)
    if b is None:
        keyholes.append([bn, "missing"])
        continue
    for f in b.faces:
        g = f.geometry
        if isinstance(g, core.Cylinder) and 4.7 < g.radius * 10 < 20.0:
            pc = (g.origin.x * 10, g.origin.z * 10)
            for c, Rr, a0, a1 in slots:
                if math.hypot(pc[0] - c[0], pc[1] - c[1]) < 0.5:
                    continue
                if arc_dist(pc, c, Rr, a0, a1) < g.radius * 10 + L["COUNTER_SLOT_R"] - 0.01:
                    keyholes.append([bn, round(g.radius * 10, 2), [round(v, 2) for v in pc]])
ARM_BEARINGS = {
    "intake_left": ["intake_arm_pivot_bearing_left", "intake_arm_front_bearing_left"],
    "intake_right": ["intake_arm_pivot_bearing_right", "intake_arm_front_bearing_right"],
    "gecko1": ["gecko1_arm_pivot_bearing_on_m8_shaft", "gecko1_arm_shaft_bearing", "gecko1_arm_shaft_bearing_outer"],
    "gecko2_left": ["gecko2_arm_pivot_bearing_left", "gecko2_arm_shaft_bearing_left"],
    "gecko2_right": ["gecko2_arm_pivot_bearing_right", "gecko2_arm_shaft_bearing_right"],
    "shooter_left": ["shooter_swing_arm_pivot_bearing_left", "shooter_swing_arm_counter_bearing_left"],
    "shooter_right": ["shooter_swing_arm_pivot_bearing_right", "shooter_swing_arm_counter_bearing_right"],
}
missing_bearings = [n for v in ARM_BEARINGS.values() for n in v if up_comp.bRepBodies.itemByName(n) is None]
gate7 = not keyholes and not missing_bearings

# ---------------------------------------------------------------- Gate 8: topspin
gate8 = L["V_REAR"] > L["V_FRONT"] and 6.8 <= L["V_EXIT"] <= 8.5

# ---------------------------------------------------------------- Gate 9: mechanism sweeps (v12.1)
# Every moving mechanism is rotated through its full travel in steps; the moving set may not hit
# anything else (springs that deform and staged balls excluded).
def _in(p, n, names=(), prefixes=(), paths=()):
    return n in names or n.startswith(prefixes) or any(k in p for k in paths)


MECHS = [
    ("intake_float_low_to_high", L["INTAKE_PIVOT"], d_int,
     lambda p, n: n.startswith(INTAKE_MOVERS) or n.startswith("intake_arm_pivot_bearing"), ()),
    ("gecko1_rest_to_lift", L["M8_SHAFT"], L["G1_A_LIFT"] - L["G1_A_REST"],
     lambda p, n: (n.startswith("gecko1_") and n not in ("gecko1_return_extension_spring", "gecko1_spring_fixed_pin"))
     or n.startswith(("magazine_b1_gecko1_pulley_", "magazine_b1_htd")) or "gecko_wheel_72mm_1:1" in p,
     ("gecko1_return_extension_spring",)),
    ("gecko2_rest_to_lift", L["J2"], L["G2_A_LIFT"] - L["G2_A_REST"],
     lambda p, n: (n.startswith("gecko2_") and n != "gecko2_torsion_return_spring")
     or n.startswith(("magazine_b3_gecko2_pulley_", "magazine_b3_htd")) or "gecko_wheel_72mm_1:2" in p,
     ("gecko2_torsion_return_spring",)),
    ("counter_wheel_rest_to_max", L["M6_SHAFT"], L["COUNTER_A_MAX"] - L["COUNTER_A_REST"],
     lambda p, n: n.startswith(("shooter_coaxial_swing_arm_", "shooter_counter_", "shooter_swing_arm_", "shooter_spring_arm_pin_"))
     or n.startswith(("shooter_right_counter_pulley_", "shooter_right_htd")) or "gobilda_counter_wheel" in p,
     ("shooter_return_extension_spring_left", "shooter_return_extension_spring_right")),
    ("indexer_gate_retract", L["GATE_PIVOT"], L["GATE_RETRACT_DEG"],
     lambda p, n: n in ("indexer_stopper_gate_3prong_fingers", "indexer_gate_rex_shaft") or "gobilda_servo_rex_coupler_s3" in p or ("servo_s3_magazine_stopper" in p and "H25T Spline" in p), ()),   # v12.6: gate, REX shaft and coupler turn together
    ("g407_blocker_stowed_to_deployed", L["BLOCKER_PIVOT"], L["BLOCKER_A_DEPLOYED"] - L["BLOCKER_A_STOWED"],
     lambda p, n: n in ("g407_front_lip_blocker_gate", "g407_blocker_pivot_stub_left_servo") or "gobilda_servo_rex_coupler_s1" in p or ("servo_s1_g407_lip_gate" in p and "H25T Spline" in p), ()),
]
gate9 = {}
for mname, piv, total, pred, excl in MECHS:
    movers = [(p, n, b) for p, n, b, bb in bodies if pred(p, n)]
    others = [(p, n, b, bb) for p, n, b, bb in bodies if not pred(p, n) and not n.startswith("staged_") and n not in excl]
    hits = {}
    for k in range(1, 9):
        ang = total * k / 8.0
        m = core.Matrix3D.create()
        m.setToRotation(math.radians(ang), core.Vector3D.create(0, -1, 0), P(piv[0], 0.0, piv[1]))
        for mp, mn, mb in movers:
            c = tb.copy(mb)
            tb.transform(c, m)
            cb = c.boundingBox
            cbb = (cb.minPoint.x * 10, cb.minPoint.y * 10, cb.minPoint.z * 10, cb.maxPoint.x * 10, cb.maxPoint.y * 10, cb.maxPoint.z * 10)
            for p, n, b, bb in others:
                if not bb_hit(cbb, bb):
                    continue
                d = tb.copy(c)
                try:
                    tb.booleanOperation(d, tb.copy(b), INTER)
                    v = d.volume * 1000.0
                except Exception:
                    v = -1.0
                if v > EPS or v < 0:
                    key = f"{group(mp, mn)} -> {group(p, n)}"
                    hits[key] = max(hits.get(key, 0.0), round(v, 2))
    gate9[mname] = {"pivot": [round(piv[0], 2), round(piv[1], 2)], "travel_deg": round(total, 2), "n_moving": len(movers), "hits": hits}
gate9_pass = all(not v["hits"] for v in gate9.values()) and all(v["n_moving"] > 0 for v in gate9.values())

# ---------------------------------------------------------------- Gate 10: support connectivity (v12.1)
# Every custom body must connect to the COTS chassis through a chain of contacts <= 0.15 mm
# (exact measureMinimumDistance between custom bodies; AABB proximity against COTS NURBS).
mmgr = app.measureManager
cust = [(p, n, b, bb) for p, n, b, bb in bodies if p.split("/")[0] == up_occ.name and not n.startswith("staged_")]
cots_bbs = [bb for p, n, b, bb in bodies if p.split("/")[0] != up_occ.name]
TOL = 0.15
grounded = set()
for i, (p, n, b, bb) in enumerate(cust):
    if any(bb_hit(bb, cb, tol=-TOL) for cb in cots_bbs):
        grounded.add(i)
adj = {i: set() for i in range(len(cust))}
for i in range(len(cust)):
    for j in range(i + 1, len(cust)):
        if not bb_hit(cust[i][3], cust[j][3], tol=-TOL):
            continue
        try:
            dmm = mmgr.measureMinimumDistance(cust[i][2], cust[j][2]).value * 10.0
        except Exception:
            dmm = 0.0
        if dmm <= TOL:
            adj[i].add(j)
            adj[j].add(i)
seen, stack = set(grounded), list(grounded)
while stack:
    i = stack.pop()
    for j in adj[i]:
        if j not in seen:
            seen.add(j)
            stack.append(j)
unsupported = sorted(cust[i][1] for i in range(len(cust)) if i not in seen)
gate10_pass = not unsupported

# ---------------------------------------------------------------- Gate 11: device screw paths (v12.5)
# Gate 10 accepts mere contact, which passed plain placeholder boxes that nothing held. Here every real
# device must have screw paths: for each of its mounting holes, just past one hole end there must be a hole
# in a custom part (axis point empty) with material around it (ring at r + 1.5 mm solid).
DEVICE_SPECS = {   # occurrence prefix: (hole radius range mm, min hole length mm, holes required)
    "rev_2m_distance_sensor_ball_count_1": ((1.6, 1.9), 2.0, 2),
    "rev_2m_distance_sensor_ball_count_2": ((1.6, 1.9), 2.0, 2),
    "rev_color_sensor_v3_throat": ((1.6, 1.9), 2.0, 2),
    "rev_color_sensor_v3_lip": ((1.6, 1.9), 2.0, 2),
    "rev_main_power_switch_31_1387": ((1.8, 1.9), 2.0, 3),
    "gobilda_servo_power_injector_3125": ((1.95, 2.05), 7.0, 4),
    "dfrobot_huskylens_sen0305": ((1.5, 1.7), 0.5, 2),
}
IN_ = fusion.PointContainment.PointInsidePointContainment
cust_b = [(n, b, bb) for p, n, b, bb in bodies if p.split("/")[0] == up_occ.name and not n.startswith("staged_")]


def solid_at(pt, tolbb=0.5):
    for n, b, bb in cust_b:
        if bb[0] - tolbb <= pt[0] <= bb[3] + tolbb and bb[1] - tolbb <= pt[1] <= bb[4] + tolbb and bb[2] - tolbb <= pt[2] <= bb[5] + tolbb:
            if b.pointContainment(P(*pt)) == IN_:
                return n
    return None


gate11 = {}
for pref, ((r0, r1), wmin, need) in DEVICE_SPECS.items():
    occs = [root.occurrences.item(i) for i in range(root.occurrences.count) if root.occurrences.item(i).name.startswith(pref)]
    if not occs:
        gate11[pref] = {"found": False, "fastened": 0, "need": need}
        continue
    holes = {}
    stack = [occs[0]]
    while stack:
        q = stack.pop()
        for bi in range(q.bRepBodies.count):
            for f in q.bRepBodies.item(bi).faces:
                g = f.geometry
                if g.surfaceType != 1 or not (r0 <= g.radius * 10 <= r1):
                    continue
                fb = f.boundingBox
                a = g.axis
                a.normalize()
                lo = (fb.minPoint.x * 10, fb.minPoint.y * 10, fb.minPoint.z * 10)
                hi = (fb.maxPoint.x * 10, fb.maxPoint.y * 10, fb.maxPoint.z * 10)
                ax = (a.x, a.y, a.z)
                length = abs(sum((hi[i] - lo[i]) * ax[i] for i in range(3)))
                if length < wmin:
                    continue
                c = tuple((lo[i] + hi[i]) / 2 for i in range(3))
                key = tuple(round(c[i] - ax[i] * sum(c[j] * ax[j] for j in range(3)), 0) for i in range(3))   # axis line id
                holes.setdefault(key, (c, ax, length, g.radius * 10))
        stack.extend(q.childOccurrences.item(k) for k in range(q.childOccurrences.count))
    fastened, into = 0, set()
    for c, ax, length, r in holes.values():
        perp = (ax[1], -ax[0], 0.0) if abs(ax[2]) < 0.9 else (1.0, 0.0, 0.0)
        pn = math.sqrt(sum(v * v for v in perp)) or 1.0
        perp = tuple(v / pn for v in perp)
        ok = None
        for sgn in (1, -1):
            for beyond in (1.5, 3.0):
                e = tuple(c[i] + sgn * ax[i] * (length / 2 + beyond) for i in range(3))
                ring = tuple(e[i] + perp[i] * (r + 1.6) for i in range(3))
                n_ring = solid_at(ring)
                if n_ring and not solid_at(e):
                    ok = n_ring
                    break
            if ok:
                break
        if ok:
            fastened += 1
            into.add(ok)
    gate11[pref] = {"found": True, "holes": len(holes), "fastened": fastened, "need": need, "into": sorted(into)}
# v12.6: custom-to-custom screw joints. A hole in part A counts when, past A's hole end, the axis is still open
# (a matching hole) and there is part-B material right around it -- contact alone never counts.
JOINT_SPECS = {("dfrobot_huskylens_mount_block", "turret_outboard_double_shear_plate_right"): 2}
cust_by = {n: (b, bb) for n, b, bb in cust_b}
gate11_joints = {}
for (na, nb), need in JOINT_SPECS.items():
    ba = cust_by.get(na)
    holes = {}
    if ba:
        for f in ba[0].faces:
            g = f.geometry
            if g.surfaceType != 1 or not (1.4 <= g.radius * 10 <= 2.0):
                continue
            a = g.axis
            a.normalize()
            fb = f.boundingBox
            lo = (fb.minPoint.x * 10, fb.minPoint.y * 10, fb.minPoint.z * 10)
            hi = (fb.maxPoint.x * 10, fb.maxPoint.y * 10, fb.maxPoint.z * 10)
            ax = (a.x, a.y, a.z)
            c = tuple((lo[i] + hi[i]) / 2 for i in range(3))
            length = abs(sum((hi[i] - lo[i]) * ax[i] for i in range(3)))
            holes.setdefault(tuple(round(c[i] - ax[i] * sum(c[j] * ax[j] for j in range(3)), 0) for i in range(3)), (c, ax, length, g.radius * 10))
    ok_n = 0
    for c, ax, length, r in holes.values():
        perp = (ax[1], -ax[0], 0.0) if abs(ax[2]) < 0.9 else (1.0, 0.0, 0.0)
        pn = math.sqrt(sum(v * v for v in perp)) or 1.0
        perp = tuple(v / pn for v in perp)
        hit = False
        for sgn in (1, -1):
            e = tuple(c[i] + sgn * ax[i] * (length / 2 + 2.5) for i in range(3))
            ring = tuple(e[i] + perp[i] * (r + 1.2) for i in range(3))
            bb_ = cust_by.get(nb)
            if bb_ and bb_[0].pointContainment(P(*ring)) == IN_ and bb_[0].pointContainment(P(*e)) != IN_:
                hit = True
        ok_n += hit
    gate11_joints[f"{na} -> {nb}"] = {"holes_in_a": len(holes), "screwed_into_b": ok_n, "need": need}
gate11_pass = all(v["fastened"] >= v["need"] for v in gate11.values()) and all(v["screwed_into_b"] >= v["need"] for v in gate11_joints.values())

# ---------------------------------------------------------------- extras
belts = L["belt_ledger"]()
belt_ok = all(abs(v["err_mm"]) < 0.25 for v in belts.values())
motors = sorted({s for p, n, b, bb in bodies for s in p.split("/") if s.startswith("gobilda_5203_motor")})
servos = sorted({s for p, n, b, bb in bodies for s in p.split("/") if s.startswith("servo_")})
rigid = [root.rigidGroups.item(i).name for i in range(root.rigidGroups.count)]

report = {
    "n_bodies": len(bodies),
    "gate1_unfiltered_brep_overlaps": gate1, "gate1_pass": not gate1,
    "gate2_multi_lumps_or_tiny": gate2, "gate2_pass": not gate2,
    "gate3_r102_stowed": {"min": s_mn, "max": s_mx, "size": s_sz, "limit": L["R102_MAX"]}, "gate3_pass": gate3,
    "gate4_r105_expanded": {"min": e_mn, "max": e_mx, "size": e_sz, "limit": L["R105_MAX"]}, "gate4_pass": gate4,
    "gate5_nectar_92mm_swept": gate5, "gate5_pass": gate5_pass,
    "gate6_floor_contact_zmin": zmin, "gate6_pods": pods, "gate6_pass": gate6,
    "gate7_keyholes": keyholes, "gate7_missing_arm_bearings": missing_bearings, "gate7_pass": gate7,
    "gate8_topspin": {"v_front": round(L["V_FRONT"], 3), "v_rear": round(L["V_REAR"], 3), "v_exit": round(L["V_EXIT"], 3)}, "gate8_pass": gate8,
    "gate9_mechanism_sweeps": gate9, "gate9_pass": gate9_pass,
    "gate10_unsupported_custom_bodies": unsupported, "gate10_pass": gate10_pass,
    "gate11_device_screw_paths": gate11, "gate11_custom_joints": gate11_joints, "gate11_pass": gate11_pass,
    "last_ball_handoff": {"g2_top_contact_z": round(L["G2_TOP_CONTACT_Z"], 1), "counter_grab_z": round(L["COUNTER_GRAB_Z"], 1),
                          "unpowered_gap_mm": round(L["UNPOWERED_GAP_MM"], 1), "coast_mm_at_M8_rpm": round(L["LAST_BALL_COAST_MM"], 1), "m8_rpm": L["M8_RPM"]},
    "belts": belts, "belts_pass": belt_ok, "motors": motors, "servos_visible": servos, "rigid_groups": rigid,
    "secs": round(time.time() - t0, 1),
}
report["all_quantitative_pass"] = all(report[k] for k in ("gate1_pass", "gate2_pass", "gate3_pass", "gate4_pass", "gate5_pass", "gate6_pass", "gate7_pass", "gate8_pass", "gate9_pass", "gate10_pass", "gate11_pass", "belts_pass")) and L["LAST_BALL_COAST_MM"] > 2.0 * L["UNPOWERED_GAP_MM"]
open("C:/Users/danu0/Downloads/26-27-BIOBUZZ/cad/out/v12_gate_report.json", "w").write(json.dumps(report, indent=1))
result = {k: v for k, v in report.items() if k.endswith("_pass") or k in ("secs", "n_bodies", "all_quantitative_pass")}
