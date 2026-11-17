"""Transfer-drill fastener holes into the HIVE-QUEEN v12 custom (printed/CNC) structural parts.

Run AFTER cad/run_full_robot_upgrade.py and BEFORE cad/verify_v12_gates.py:
    python cad/fusion_cli.py run-file cad/drill_fastener_holes.py --timeout 900

1. Custom part on a COTS part (goBILDA channel/plate, hub, servo, pod...): every COTS hole
   (concave cylinder r 1.4-2.45 mm, axis-aligned) whose end face is covered by a structural
   custom part is transferred into that part.
     M4 pattern hole (r >= 1.9): r 2.8 x 8 mm, an M4 heat-set insert hole (Ø5.6). It becomes a
       through hole on parts thinner than 8 mm, so use an M4 bolt + nylock nut there.
     M3 hole: r 2.0 x 6 mm (M3 heat-set insert).
   At most 4 holes per (custom, COTS) pair, spread out.
2. Custom part on custom part (flat contact patch >= 8 mm): up to 3 M3 joints per pair.
   The thinner part gets an r 1.7 clearance hole, the thicker an r 2.0 x 8 mm M3 insert hole.
3. Safety: no drill may touch the swept NECTAR/POLLEN path (+1.5 mm), and device bodies
   (sensors, injector, switch, HuskyLens) are never drilled. Every drilled body must stay one lump.
"""
import math, json, time
import adsk, adsk.core as core, adsk.fusion as fusion

L = {"__name__": "v12_design"}
exec(open("C:/Users/danu0/Downloads/26-27-BIOBUZZ/cad/v12_design.py", encoding="utf-8").read(), L)
tb = fusion.TemporaryBRepManager.get()
INTER = fusion.BooleanTypes.IntersectionBooleanType
DIFF = fusion.BooleanTypes.DifferenceBooleanType
UNION = fusion.BooleanTypes.UnionBooleanType
CYL = core.SurfaceTypes.CylinderSurfaceType
P = lambda x, y, z: core.Point3D.create(x / 10.0, y / 10.0, z / 10.0)

STRUCT_KW = ("plate", "bracket", "saddle", "cradle", "mount", "shroud", "wing", "shield", "cage", "tray", "strap",
             "hanger", "deck", "chute", "tower", "upright", "stop_plate", "guard_cage", "double_shear", "housing")
NEVER = ("sensor", "injector", "switch_panel", "switch_guard_red", "huskylens_pcb", "huskylens_front", "staged_", "sign_",
         "rex_shaft", "bearing", "belt", "pulley", "spring", "pin", "stub", "collar", "spacer", "miter", "wheel", "swing_arm",
         "gate_3prong", "blocker_gate", "hood_full", "coupler", "jackshaft")


# v12.6: NEVER is a substring list, and "wheel"/"bearing" in it silently excluded the flywheel motor saddles and the
# intake bearing/stop plates (no holes at all). These are structural no matter what words their names contain.
ALWAYS = ("turret_flywheel_motor_saddle", "intake_upright_bearing_and_stop_plate", "turret_side_plate_foot_bracket")
# v12.6: CNC aluminium parts never get heat-set insert holes: COTS transfers become clearance holes (bolt + nut or
# a tapped mate), and in printed<->aluminium joints the aluminium side is the clearance side.
ALU_PARTS = ("turret_side_plate_left_clean", "turret_side_plate_right_clean", "turret_outboard_double_shear_plate_left",
             "turret_outboard_double_shear_plate_right", "cnc_turret_mounting_deck_plate_clean")


def is_struct(name):
    n = name.lower()
    if any(n.startswith(k) for k in ALWAYS):
        return True
    return any(k in n for k in STRUCT_KW) and not any(k in n for k in NEVER)


# ---------------------------------------------------------------- collect
up_occ = [root.occurrences.item(i) for i in range(root.occurrences.count) if root.occurrences.item(i).component.name.startswith("HIVE_QUEEN_v12")][0]
up_comp = up_occ.component

cots = []
def walk(o):
    if not o.isLightBulbOn or o == up_occ:
        return
    for i in range(o.bRepBodies.count):
        b = o.bRepBodies.item(i)
        if b.isLightBulbOn and b.isSolid:
            bb = b.boundingBox
            cots.append((o.fullPathName, b, (bb.minPoint.x * 10, bb.minPoint.y * 10, bb.minPoint.z * 10, bb.maxPoint.x * 10, bb.maxPoint.y * 10, bb.maxPoint.z * 10)))
    for i in range(o.childOccurrences.count):
        walk(o.childOccurrences.item(i))
for i in range(root.occurrences.count):
    walk(root.occurrences.item(i))


def bbox(b):
    q = b.boundingBox
    return (q.minPoint.x * 10, q.minPoint.y * 10, q.minPoint.z * 10, q.maxPoint.x * 10, q.maxPoint.y * 10, q.maxPoint.z * 10)


def near(a, b, tol):
    return all(a[k] < b[k + 3] + tol and b[k] < a[k + 3] + tol for k in range(3))


def cyl(p0, p1, r):
    return tb.createCylinderOrCone(P(*p0), r / 10.0, P(*p1), r / 10.0)


def vol(a, b):
    d = tb.copy(a)
    try:
        tb.booleanOperation(d, tb.copy(b), INTER)
        return d.volume * 1000.0
    except Exception:
        return -1.0


# swept ball keep-out (NECTAR envelope + 1.5 mm): analytic distance from the drill segment to the
# ball-centre path (path lies in the Y = 0 plane), much cheaper than a sphere-union boolean.
PATH = [(p[0], 0.0, p[1]) for p in L["path_points"](L["NECTAR_R"], step=4.0)]
KEEP_R = L["NECTAR_R"] + 1.5


def seg_pt_dist(a, b, p):
    ab = [b[i] - a[i] for i in range(3)]
    ap = [p[i] - a[i] for i in range(3)]
    L2 = sum(x * x for x in ab) or 1e-9
    t = max(0.0, min(1.0, sum(ab[i] * ap[i] for i in range(3)) / L2))
    return math.dist([a[i] + t * ab[i] for i in range(3)], p)


def hits_keepout(p0, p1, r):
    if any(seg_pt_dist(p0, p1, q) < KEEP_R + r for q in PATH):
        return True
    # sideways (Y-axis) holes through the corridor walls must never break into the 102 mm ball corridor
    # (|Y| < 51) near the path: a bolt head there would catch balls. Floor holes are covered by the check above.
    if abs(p1[1] - p0[1]) < 0.5 * math.dist(p0, p1):
        return False
    for t in (0.0, 0.25, 0.5, 0.75, 1.0):
        x = [p0[i] + t * (p1[i] - p0[i]) for i in range(3)]
        if abs(x[1]) - r < 52.0 and any(math.hypot(x[0] - q[0], x[2] - q[2]) < KEEP_R + 15.0 for q in PATH):
            return True
    return False


def inside(body, xyz):
    return body.pointContainment(P(*xyz)) == adsk.fusion.PointContainment.PointInsidePointContainment

T0 = time.time()
out = {"holes": {}, "skipped_keepout": 0, "skipped_lumps": 0}
# Everything below works on TEMPORARY copies, outside any edit session (COTS proxies go stale inside
# a base-feature edit, and fresh STEP imports are direct-modelling designs).
cust = {}
for i in range(up_comp.bRepBodies.count):
    b = up_comp.bRepBodies.item(i)
    if b.isLightBulbOn and b.isSolid and is_struct(b.name):
        cust[b.name] = [tb.copy(b), bbox(b), [], b]   # temp copy, bbox, drills, real body


def add_drill(name, p0, p1, r, key):
    if hits_keepout(p0, p1, r):
        out["skipped_keepout"] += 1
        return False
    cust[name][2].append((cyl(p0, p1, r), key, ((p0[0] + p1[0]) / 2, (p0[1] + p1[1]) / 2, (p0[2] + p1[2]) / 2), r))
    return True


def spread(points, k):
    """farthest-point selection of up to k points (each item: (xyz, payload))."""
    if len(points) <= k:
        return points
    sel = [points[0]]
    while len(sel) < k:
        best = max(points, key=lambda q: min(math.dist(q[0], s[0]) for s in sel))
        sel.append(best)
    return sel


# ---------------------------------------------------------------- 1. COTS hole transfer
AX = [(1, 0, 0), (0, 1, 0), (0, 0, 1)]
for cname, cb, cbb in cots:
    if "injector" in cname.lower():
        continue      # v12.5: its M4 base screws thread into explicit tapped holes in the aluminium deck
    targets = [(n, v) for n, v in cust.items() if near(cbb, v[1], 0.3)]
    if not targets:
        continue
    for f in cb.faces:
        g = f.geometry
        if g.surfaceType != CYL:
            continue
        r = g.radius * 10.0
        if not (1.4 <= r <= 2.45):
            continue
        a = g.axis
        a.normalize()
        k = next((i for i in range(3) if abs((a.x, a.y, a.z)[i]) > 0.999), None)
        if k is None:
            continue
        fb = f.boundingBox
        lo = (fb.minPoint.x * 10, fb.minPoint.y * 10, fb.minPoint.z * 10)
        hi = (fb.maxPoint.x * 10, fb.maxPoint.y * 10, fb.maxPoint.z * 10)
        c = [(lo[i] + hi[i]) / 2 for i in range(3)]
        w = hi[k] - lo[k]
        if w < 0.5 or w > 30:
            continue
        # concave? (hole): a point on the face lies farther from the axis than... use face area vs r: holes in COTS are concave by
        # convention when the body's material surrounds the axis; test a probe ring just outside the radius
        ev = f.evaluator
        pr = ev.parametricRange()
        okp, pt = ev.getPointAtParameter(core.Point2D.create(0.5 * (pr.minPoint.x + pr.maxPoint.x), 0.5 * (pr.minPoint.y + pr.maxPoint.y)))
        okn, nrm = ev.getNormalAtPoint(pt)
        org = g.origin
        vv_ = core.Vector3D.create(pt.x - org.x, pt.y - org.y, pt.z - org.z)
        al = a.copy(); al.scaleBy(vv_.dotProduct(a))
        radial = vv_.copy(); radial.subtract(al)
        if radial.dotProduct(nrm) >= 0:    # normal points away from the axis -> boss/pin, not a hole
            continue
        big = r >= 1.9 and "servo" not in cname.lower()   # servo ear holes take M3 screws -> M3 inserts
        dr, depth = (2.8, 8.0) if big else (2.0, 6.0)
        for sgn in (1, -1):
            end = [c[i] + sgn * (w / 2) * AX[k][i] for i in range(3)]
            for n, v in targets:
                if "quad_block" in n and "5203" in cname:
                    continue      # motor face screws pass through: the block has explicit M4 clearance holes
                if not (v[1][0] - 3 <= end[0] <= v[1][3] + 3 and v[1][1] - 3 <= end[1] <= v[1][4] + 3 and v[1][2] - 3 <= end[2] <= v[1][5] + 3):
                    continue
                # contact: points just beyond the hole end (ring r 3.2 + centre) must be inside the custom part
                o_ = [i for i in range(3) if i != k]
                ring = []
                for ang_ in (0, 90, 180, 270):
                    q = [end[i] + sgn * 0.4 * AX[k][i] for i in range(3)]
                    q[o_[0]] += 3.2 * math.cos(math.radians(ang_))
                    q[o_[1]] += 3.2 * math.sin(math.radians(ang_))
                    ring.append(q)
                if sum(inside(v[0], q) for q in ring) < 3:
                    continue
                p0 = [end[i] - sgn * 0.01 * AX[k][i] for i in range(3)]
                p1 = [end[i] + sgn * depth * AX[k][i] for i in range(3)]
                v[2].append(("cand", cname, tuple(p0), tuple(p1), ((2.25 if big else 1.7) if n in ALU_PARTS else dr)))

# choose <= 4 per (custom, cots) pair
for n, v in cust.items():
    cands = [x for x in v[2] if x[0] == "cand"]
    v[2] = [x for x in v[2] if x[0] != "cand"]
    bypair = {}
    for x in cands:
        bypair.setdefault(x[1], []).append(x)
    for cname, xs in bypair.items():
        uniq = []
        for x in xs:
            mid = tuple((x[2][i] + x[3][i]) / 2 for i in range(3))
            if all(math.dist(mid, u[0]) > 6.0 for u in uniq):
                uniq.append((mid, x))
        for mid, x in spread(uniq, 4):
            add_drill(n, x[2], x[3], x[4], "cots:" + cname)

# ---------------------------------------------------------------- 2. custom <-> custom joints
# Face-based: an axis-aligned planar face of A and an opposite-facing coplanar face of B (within
# 0.15 mm) form a contact patch; sample it on a 6 mm grid with containment checks on both sides.
PLANE = core.SurfaceTypes.PlaneSurfaceType


def flat_faces(body):
    res = []
    for f in body.faces:
        if f.geometry.surfaceType != PLANE:
            continue
        ok_, nrm = f.evaluator.getNormalAtPoint(f.pointOnFace)
        v = (nrm.x, nrm.y, nrm.z)
        k = next((i for i in range(3) if abs(v[i]) > 0.999), None)
        if k is None:
            continue
        q = f.boundingBox
        lo = (q.minPoint.x * 10, q.minPoint.y * 10, q.minPoint.z * 10)
        hi = (q.maxPoint.x * 10, q.maxPoint.y * 10, q.maxPoint.z * 10)
        res.append((k, 1 if v[k] > 0 else -1, lo[k], lo, hi))
    return res


FF = {n: flat_faces(v[0]) for n, v in cust.items()}
names = list(cust)
for i in range(len(names)):
    for j in range(i + 1, len(names)):
        A, B = cust[names[i]], cust[names[j]]
        a, b = A[1], B[1]
        if not near(a, b, 0.2):
            continue
        pts = []
        for (ka, sa, pa, loa, hia) in FF[names[i]]:
            for (kb, sb, pb, lob, hib) in FF[names[j]]:
                if ka != kb or sa != -sb or abs(pa - pb) > 0.15:
                    continue
                kn = ka
                t = [m for m in range(3) if m != kn]
                lo2 = [max(loa[m], lob[m]) for m in t]
                hi2 = [min(hia[m], hib[m]) for m in t]
                if hi2[0] - lo2[0] < 8 or hi2[1] - lo2[1] < 8:
                    continue
                plane = 0.5 * (pa + pb)
                side_a = -sa                      # A's material is on the side opposite its outward normal
                u = lo2[0] + 4
                while u <= hi2[0] - 4 + 1e-6:
                    vv = lo2[1] + 4
                    while vv <= hi2[1] - 4 + 1e-6:
                        q = [0, 0, 0]
                        q[kn], q[t[0]], q[t[1]] = plane, u, vv
                        ok_all = True
                        for body, sd in ((A[0], side_a), (B[0], -side_a)):
                            base = [q[m] + sd * 0.8 * AX[kn][m] for m in range(3)]
                            probes = [base] + [[base[m] + dd * AX[ax_][m] for m in range(3)] for ax_ in t for dd in (3.0, -3.0)]
                            if not all(inside(body, pp) for pp in probes):
                                ok_all = False
                                break
                        if ok_all:
                            pts.append((tuple(q), side_a, kn))
                        vv += 2      # fine grid: narrow real patches (e.g. 8 mm flange vs a cut-out deck) must be sampled
                    u += 2
        if not pts:
            continue
        for q, side_a, kn in [(p_[0], p_[1][0], p_[1][1]) for p_ in spread([(pp[0], (pp[1], pp[2])) for pp in pts], 3)]:
            ta, tb_ = a[kn + 3] - a[kn], b[kn + 3] - b[kn]
            ra, rb = (1.7, 2.0) if ta <= tb_ else (2.0, 1.7)
            if names[i] in ALU_PARTS or names[j] in ALU_PARTS:     # aluminium side = clearance, printed side = insert
                ra = 1.7 if names[i] in ALU_PARTS else 2.0
                rb = 1.7 if names[j] in ALU_PARTS else 2.0
            pa1 = [q[m] + side_a * 8.0 * AX[kn][m] for m in range(3)]
            pb1 = [q[m] - side_a * 8.0 * AX[kn][m] for m in range(3)]
            q0a = [q[m] - side_a * 0.01 * AX[kn][m] for m in range(3)]
            q0b = [q[m] + side_a * 0.01 * AX[kn][m] for m in range(3)]
            # both halves must clear the keep-out or neither is drilled
            if hits_keepout(q0a, pa1, ra) or hits_keepout(q0b, pb1, rb):
                out["skipped_keepout"] += 1
                continue
            da, db_ = cyl(q0a, pa1, ra), cyl(q0b, pb1, rb)
            # v12.6: the key carries the clearance-side grip (wall the screw passes, <= the 8 mm hole) for the BOM
            grip = round(min(ta if ra < rb else tb_, 8.0), 1)
            A[2].append((da, "joint:" + names[j] + "|" + str(grip), q, ra))
            B[2].append((db_, "joint:" + names[i] + "|" + str(grip), q, rb))

# ---------------------------------------------------------------- 3. apply
total = 0
INTER = fusion.BooleanTypes.IntersectionBooleanType


def _mkey(n, key, mid):
    """Joint holes come in mated pairs (same contact point q): key both halves alike."""
    other = key.split(":", 1)[1].split("|")[0]
    return (tuple(sorted((n, other))), tuple(round(c, 1) for c in mid))


def _crosses(d, tools):
    """v12.6: a new hole may not intersect a hole already kept in the same body (two inserts in one place)."""
    for t in tools:
        x = tb.copy(d)
        try:
            tb.booleanOperation(x, t, INTER)
        except Exception:
            continue
        if x.volume > 1e-6:
            return True
    return False


def _tangent(body, d, rr):
    """v12.6: a hole whose wall is tangent (within 0.15 mm) to a planar face of its own part leaves a zero-thickness
    edge that tears the printed mesh (seen on the side-plate foot brackets)."""
    q = d.boundingBox
    lo = (q.minPoint.x * 10, q.minPoint.y * 10, q.minPoint.z * 10)
    hi = (q.maxPoint.x * 10, q.maxPoint.y * 10, q.maxPoint.z * 10)
    k = max(range(3), key=lambda i: hi[i] - lo[i])            # hole axis (longest extent)
    c = [(lo[i] + hi[i]) / 2 for i in range(3)]
    for f in body.faces:
        if f.geometry.surfaceType != PLANE:
            continue
        ok_, nrm = f.evaluator.getNormalAtPoint(f.pointOnFace)
        v = (nrm.x, nrm.y, nrm.z)
        j = next((i for i in range(3) if abs(v[i]) > 0.999), None)
        if j is None or j == k:
            continue
        p = [f.pointOnFace.x * 10, f.pointOnFace.y * 10, f.pointOnFace.z * 10][j]
        if abs(abs(p - c[j]) - rr) < 0.15:
            fb = f.boundingBox
            flo = (fb.minPoint.x * 10, fb.minPoint.y * 10, fb.minPoint.z * 10)
            fhi = (fb.maxPoint.x * 10, fb.maxPoint.y * 10, fb.maxPoint.z * 10)
            o = [i for i in range(3) if i != j]
            if all(flo[i] < hi[i] - 0.05 and fhi[i] > lo[i] + 0.05 for i in o):
                return True
    return False


# pass 1 finds joint holes that cross another hole or graze a face; pass 2 drops them AND their mates (no orphans)
REJECT, REJECT_NEW = set(), set()
for PASS in (1, 2):
  if PASS == 2:
    REJECT = set(REJECT_NEW)
    out["skipped_cross_or_tangent"] = 0
    out["skipped_lumps"] = 0
    out["rejected_joint_pairs"] = len(REJECT)
  total = 0
  new_bodies = []
  for n, v in cust.items():
    drills = v[2]
    if not drills:
        out["holes"][n] = 0
        continue
    nb = tb.copy(v[0])
    kept = 0
    tools = []
    for d, key, mid, rr in drills:
        if key.startswith("joint:") and _mkey(n, key, mid) in REJECT:
            continue
        if _crosses(d, tools) or _tangent(nb, d, rr):
            out["skipped_cross_or_tangent"] = out.get("skipped_cross_or_tangent", 0) + 1
            if key.startswith("joint:"):
                REJECT_NEW.add(_mkey(n, key, mid))
            continue
        trial = tb.copy(nb)
        try:
            tb.booleanOperation(trial, d, DIFF)
        except Exception:
            continue
        if trial.lumps.count != 1 or trial.volume >= nb.volume - 1e-9:
            out["skipped_lumps"] += 1 if trial.lumps.count != 1 else 0
            continue
        nb = trial
        tools.append(d)
        kept += 1
        if PASS == 2:
            out.setdefault("by_radius", {}).setdefault(str(rr), 0)
            out["by_radius"][str(rr)] += 1
            out.setdefault("detail", []).append([n, key, round(rr, 2)])   # v12.6: per-hole record for the BOM fastener count
    if kept:
        new_bodies.append((n, nb))
    out["holes"][n] = kept
    total += kept

# write back: edit the builder's base feature in place (all geometry was computed above on temporary
# copies, so no COTS proxy is touched inside the edit). Never delete bodies: in a direct design that can
# take the whole base feature with it.
if new_bodies:
    bf = up_comp.features.baseFeatures.item(0)
    bf.startEdit()
    byname = {bf.bodies.item(i).name: bf.bodies.item(i) for i in range(bf.bodies.count)}
    for n, nb in new_bodies:
        if n in byname:
            bf.updateBody(byname[n], nb)
        else:
            out.setdefault("not_in_base_feature", []).append(n)
    bf.finishEdit()
out["total_holes"] = total
out["secs"] = round(time.time() - T0, 1)
out["bodies_drilled"] = sum(1 for k in out["holes"].values() if k)
out["structural_bodies"] = len(cust)
out["undrilled"] = sorted(k for k, v in out["holes"].items() if not v)
result = out
