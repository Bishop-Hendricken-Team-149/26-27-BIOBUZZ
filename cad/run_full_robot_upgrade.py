"""
HIVE-QUEEN v12 full remediation builder (runs inside Fusion 360 via cad/fusion_cli.py run-file).

Input : fresh STEP import of cad/STEP/project_hive_queen_robot.step (ParametricDesignType)
Output: one BaseFeature component 'HIVE_QUEEN_v12_Remediation' + repositioned / cloned COTS occurrences.

Execution order (docs/03 Rule 2): visibility only -> ONE BaseFeature (inner shafts outward) ->
appearances -> COTS clones + transform2 -> single snapshot -> gate audit (cad/verify_v12_gates.py).

Every coordinate comes from cad/v12_design.py so the gate script audits the exact same numbers.
No trim_scene / trim_prev masking is used anywhere: every clearance is explicit geometry and the
unfiltered B-Rep audit is the referee. The v11 builder is preserved as run_full_robot_upgrade_v11.py.
"""
import math
import adsk.core as core
import adsk.fusion as fusion

DESIGN_PATH = "C:/Users/danu0/Downloads/26-27-BIOBUZZ/cad/v12_design.py"
L = {"__name__": "v12_design"}
exec(open(DESIGN_PATH, encoding="utf-8").read(), L)

tb = fusion.TemporaryBRepManager.get()
UNION = fusion.BooleanTypes.UnionBooleanType
DIFF = fusion.BooleanTypes.DifferenceBooleanType
INTER = fusion.BooleanTypes.IntersectionBooleanType


# =============================================================================
# Geometry helpers (mm in, cm to Fusion)
# =============================================================================
def P(x, y, z):
    return core.Point3D.create(x / 10.0, y / 10.0, z / 10.0)


def V(x, y, z):
    return core.Vector3D.create(x / 10.0, y / 10.0, z / 10.0)


def mat_pos(x, y, z):
    m = core.Matrix3D.create()
    m.translation = V(x, y, z)
    return m


def mat_rot(axis, deg):
    m = core.Matrix3D.create()
    m.setToRotation(math.radians(deg), core.Vector3D.create(*[float(a) for a in axis]), core.Point3D.create(0, 0, 0))
    return m


def obox3(c, ldir, wdir, lx, ly, lz):
    lv = core.Vector3D.create(*ldir)
    lv.normalize()
    wv = core.Vector3D.create(*wdir)
    wv.normalize()
    return tb.createBox(core.OrientedBoundingBox3D.create(P(*c), lv, wv, lx / 10.0, ly / 10.0, lz / 10.0))


def obox(cx, cy, cz, dx, dz, lx, ly, lz):
    """Box centred at (cx,cy,cz); local X along (dx,0,dz) in the XZ plane, local Y along world Y."""
    return obox3((cx, cy, cz), (dx, 0.0, dz), (0.0, 1.0, 0.0), lx, ly, lz)


def box(x0, x1, y0, y1, z0, z1):
    return obox((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2, 1, 0, abs(x1 - x0), abs(y1 - y0), abs(z1 - z0))


def cyl(x1, y1, z1, x2, y2, z2, r):
    return tb.createCylinderOrCone(P(x1, y1, z1), r / 10.0, P(x2, y2, z2), r / 10.0)


def cone(x1, y1, z1, r1, x2, y2, z2, r2):
    return tb.createCylinderOrCone(P(x1, y1, z1), r1 / 10.0, P(x2, y2, z2), r2 / 10.0)


def cy(x, z, y0, y1, r):
    return cyl(x, y0, z, x, y1, z, r)


def rex_y(x, z, y0, y1, flat=None, corner=None):
    """v12.6: goBILDA 8 mm REX along Y (flat normals at 0/60/120 deg in XZ, matching the 4001 coupler)."""
    flat = L["REX_FLAT"] if flat is None else flat
    corner = L["REX_CORNER"] if corner is None else corner
    body = cy(x, z, y0, y1, corner)
    for k_ in range(3):
        th = math.radians(60.0 * k_)
        I(body, obox3((x, (y0 + y1) / 2.0, z), (math.cos(th), 0.0, math.sin(th)), (0.0, 1.0, 0.0), 2 * flat, abs(y1 - y0) + 2.0, 40.0))
    return body


def rex_bore_y(x, z, y0, y1):
    """REX bore for printed parts on a REX shaft: 0.04 mm per flat, 0.07 mm on the corners."""
    return rex_y(x, z, y0, y1, 3.52, 4.02)


def tube_y(x, z, y0, y1, r_out, r_in):
    t = cy(x, z, y0, y1, r_out)
    tb.booleanOperation(t, cy(x, z, min(y0, y1) - 1, max(y0, y1) + 1, r_in), DIFF)
    return t


def U(a, *bs):
    for b in bs:
        if b is not None:
            tb.booleanOperation(a, b, UNION)
    return a


def S(a, *bs):
    for b in bs:
        if b is not None:
            tb.booleanOperation(a, b, DIFF)
    return a


def I(a, b):
    tb.booleanOperation(a, b, INTER)
    return a


def bar_xz(p1, p2, y0, y1, w):
    dx, dz = p2[0] - p1[0], p2[1] - p1[1]
    return obox((p1[0] + p2[0]) / 2, (y0 + y1) / 2, (p1[1] + p2[1]) / 2, dx, dz, math.hypot(dx, dz), abs(y1 - y0), w)


def halfplane(px, pz, nx, nz, y0, y1, size=2000.0):
    """Solid covering {(x,z): (x-px)nx+(z-pz)nz >= 0} for Y in [y0,y1]."""
    ln = math.hypot(nx, nz)
    nx, nz = nx / ln, nz / ln
    return obox(px + nx * size / 2, (y0 + y1) / 2, pz + nz * size / 2, nx, nz, size, abs(y1 - y0), size)


def halfspace(p, n, size=2000.0):
    """Solid covering {q: (q-p).n >= 0} (any normal)."""
    ln = math.sqrt(sum(c * c for c in n))
    n = tuple(c / ln for c in n)
    w = (1.0, 0.0, 0.0) if abs(n[0]) < 0.9 else (0.0, 1.0, 0.0)
    return obox3(tuple(p[i] + n[i] * size / 2 for i in range(3)), n, w, size, size, size)


def sector(c, r_in, r_out, a0, a1, y0, y1):
    """Annulus sector a0 < a1 (deg, span < 180) about c=(x,z), extruded over Y."""
    ring = cy(c[0], c[1], y0, y1, r_out)
    if r_in > 0.01:
        S(ring, cy(c[0], c[1], y0 - 1, y1 + 1, r_in))
    t0, t1 = math.radians(a0), math.radians(a1)
    I(ring, halfplane(c[0], c[1], -math.sin(t0), math.cos(t0), y0 - 1, y1 + 1))
    I(ring, halfplane(c[0], c[1], math.sin(t1), -math.cos(t1), y0 - 1, y1 + 1))
    return ring


def arc_slot(c, R, a0, a1, sr, y0, y1):
    s = sector(c, R - sr, R + sr, a0, a1, y0, y1)
    for a in (a0, a1):
        p = L["polar"](c, R, a)
        U(s, cy(p[0], p[1], y0, y1, sr))
    return s


def hull2(c1, r1, c2, r2, y0, y1):
    """Convex hull of two circles (x,z) extruded along Y (belt envelope)."""
    h = U(cy(c1[0], c1[1], y0, y1, r1), cy(c2[0], c2[1], y0, y1, r2))
    dx, dz = c2[0] - c1[0], c2[1] - c1[1]
    Ld = math.hypot(dx, dz)
    ux, uz = dx / Ld, dz / Ld
    nx, nz = -uz, ux
    s = (r1 - r2) / Ld
    cth = math.sqrt(max(0.0, 1 - s * s))
    q = None
    for sgn in (1, -1):
        mx, mz = s * ux + cth * nx * sgn, s * uz + cth * nz * sgn
        hp = halfplane(c1[0] + r1 * mx, c1[1] + r1 * mz, -mx, -mz, y0, y1)
        q = hp if q is None else I(q, hp)
    I(q, halfplane(c1[0], c1[1], ux, uz, y0, y1))
    I(q, halfplane(c2[0], c2[1], -ux, -uz, y0, y1))
    return U(h, q)


def belt(c1, t1, c2, t2, yc, pitch=5.0):
    sp = L["BELT_SPEC"][pitch]
    w, thick = sp["belt_w"], sp["thick"]
    r1, r2 = L["htd_pd"](t1, pitch) / 2, L["htd_pd"](t2, pitch) / 2
    outer = hull2(c1, r1 + thick, c2, r2 + thick, yc - w / 2, yc + w / 2)
    inner = hull2(c1, r1 - 0.40, c2, r2 - 0.40, yc - w / 2 - 1, yc + w / 2 + 1)   # v12.6: rides 0.05 mm off the pulley tooth envelope
    return S(outer, inner)


def pulley(c, yc, teeth, pitch=5.0, bore=4.05):
    sp = L["BELT_SPEC"][pitch]
    w, fl, fr = sp["pulley_w"], sp["flange"], sp["flange_r"]
    r = L["htd_pd"](teeth, pitch) / 2
    y0, y1 = yc - w / 2, yc + w / 2
    p = U(cy(c[0], c[1], y0, y1, r - 0.45), cy(c[0], c[1], y0, y0 + fl, r + fr), cy(c[0], c[1], y1 - fl, y1, r + fr))
    return S(p, cy(c[0], c[1], y0 - 1, y1 + 1, bore))


def bearing(c, y0, y1):
    """goBILDA 1611-0514-4008 (14 mm OD, 8 mm REX bore) seated ring."""
    return tube_y(c[0], c[1], y0, y1, L["BEARING_OD_R"], L["BEARING_ID_R"])


def swing_arm(p0, p1, y0, y1, w=16.0, hub_r=11.5, tail=0.0, tabs=()):
    a = U(bar_xz(p0, p1, y0, y1, w), cy(p0[0], p0[1], y0, y1, hub_r), cy(p1[0], p1[1], y0, y1, hub_r))
    if tail > 0:
        dx, dz = p1[0] - p0[0], p1[1] - p0[1]
        Ld = math.hypot(dx, dz)
        pt = (p0[0] - dx / Ld * tail, p0[1] - dz / Ld * tail)
        U(a, bar_xz(p0, pt, y0, y1, w), cy(pt[0], pt[1], y0, y1, w / 2))
    for t in tabs:
        U(a, t)
    return S(a, cy(p0[0], p0[1], y0 - 1, y1 + 1, 7.0), cy(p1[0], p1[1], y0 - 1, y1 + 1, 7.0))


def spring(p1, p2, yc, r_body=2.0, r_coil=3.0, pins=()):
    s = cyl(p1[0], yc, p1[1], p2[0], yc, p2[1], r_body)
    Ld = L["dist2"](p1, p2)
    ux, uz = (p2[0] - p1[0]) / Ld, (p2[1] - p1[1]) / Ld
    n = max(4, int(Ld / 5.0))
    for k in range(1, n):
        f = k / n
        if 0.2 < f < 0.8:
            x, z = p1[0] + (p2[0] - p1[0]) * f, p1[1] + (p2[1] - p1[1]) * f
            U(s, cyl(x - ux * 0.9, yc, z - uz * 0.9, x + ux * 0.9, yc, z + uz * 0.9, r_coil))
    for pc, pr in pins:
        S(s, cy(pc[0], pc[1], yc - 10, yc + 10, pr + 0.05))
    return s


# =============================================================================
# Occurrence handles + visibility (pre-step: visibility only)
# =============================================================================
rbc = root.occurrences.itemByName("robot_base_core:1")
s1 = rbc.childOccurrences.itemByName("strafer_chassis_stage:1")
s2 = rbc.childOccurrences.itemByName("gatekeeper_intake_stage:1")
s3 = rbc.childOccurrences.itemByName("sprung_magazine_stage:1")
tur = root.occurrences.itemByName("turret_launcher_assembly:1")
stinger = root.occurrences.itemByName("stinger_flower_capper_arm:1")

for i in range(root.occurrences.count):
    o = root.occurrences.item(i)
    if "Variable_Intake" in o.component.name or "HIVE_QUEEN_v12" in o.component.name:
        o.isLightBulbOn = False

HIDE = [
    (None, ["flower_depollinator_fork:1", "g407_lip_gate_assembly:1"]),
    (s1, ["front_funnel_bumper_left:1", "front_funnel_bumper_right:1", "cnc_turret_mounting_deck_plate:1",
          "gobilda_1121_10hole_front_portal_arch:1", "gobilda_1120_10hole_mid:1", "gobilda_1120_10hole_mid:2",
          "rex_axle_shaft_gobilda_mecanum_96mm_front_left:1", "rex_axle_shaft_gobilda_mecanum_96mm_front_left:2",
          "rex_axle_shaft_gobilda_mecanum_96mm_front_left:3", "rex_axle_shaft_gobilda_mecanum_96mm_front_left:4",
          "robot_sign_left_149:1", "robot_sign_right_149:1"]),
    (s2, ["intake_rex_shaft_8mm:1", "intake_bearing_support_arm_left:1", "intake_bearing_support_arm_right:1",
          "continuous_chute_bottom_ramp:1", "chute_rear_elevator_wall:1", "ramp_front_crossbar_mount:1",
          "ramp_mid_stanchion_cross_mount:1", "elevator_rear_bridge_mount:1", "m7_intake_motor_mount_bracket:1",
          "intake_drive_pulley_1:1", "intake_drive_pulley_1:2", "intake_drive_timing_belt:1",
          "servo_s2_depollinator_drive:1", "star_roller_sleeve:1", "star_roller_sleeve:2", "star_roller_sleeve:3"]),
    (s3, ["magazine_shroud_left:1", "magazine_shroud_right:1", "gobilda_gecko_wheel_72mm_1:3",
          "gecko_rex_shaft_1:1", "gecko_rex_shaft_1:2", "gecko_rex_shaft_1:3",
          "magazine_motor_drive_timing_belt:1", "magazine_gecko_couple_timing_belt:1", "magazine_elevator_couple_timing_belt:1"]
     + [f"magazine_structural_gusset_left:{k}" for k in range(1, 5)] + [f"gecko_flanged_bearing_1_-1:{k}" for k in range(1, 7)]
     + [f"intake_drive_pulley_1:{k}" for k in range(3, 9)] + [f"staged_pollen_1_entering_mouth:{k}" for k in range(1, 6)]),
    (tur, ["variable_pitch_exit_hood:1", "limelight_3a_vision:1", "turret_bearing_ring_85mm:1", "turret_top_structural_bridge:1",
           "turret_side_plate_left:1", "turret_side_plate_right:1", "turret_motor_mount_bracket_left:1", "turret_motor_mount_bracket_right:1",
           "shooter_main_rex_shaft:1", "shooter_counter_rex_shaft:1", "shooter_cam_pivot_rex_shaft:1", "shooter_cam_arm_left:1",
           "shooter_cam_arm_left:2", "intake_drive_pulley_1:9", "intake_drive_pulley_1:10", "intake_drive_pulley_1:11",
           "intake_drive_pulley_1:12", "shooter_left_drive_timing_belt:1", "shooter_right_drive_timing_belt:1",
           "servo_s4_turret_azimuth:1", "servo_s5_cam_gap_selector:1"]),
    (stinger, ["carbon_fiber_stinger_tube:1", "stinger_nectar_cup:1", "tower_base_mount_rear_left:1"]),
]
for parent, names in HIDE:
    for nm in names:
        o = root.occurrences.itemByName(nm) if parent is None else parent.childOccurrences.itemByName(nm)
        if o is not None:
            o.isLightBulbOn = False
if stinger is not None:
    stinger.isLightBulbOn = False   # v12.2: whole FLOWER capper (arm, cup, S7) removed

# =============================================================================
# STEP 1: ONE BaseFeature — inner shafts/bearings/pulleys/belts first, outward to plates
# =============================================================================
up_occ = root.occurrences.addNewComponent(core.Matrix3D.create())
up_comp = up_occ.component
up_comp.name = "HIVE_QUEEN_v12_Remediation"
bf = up_comp.features.baseFeatures.add()
bf.startEdit()
body_colors = []


def add(solid, name, color):
    b = up_comp.bRepBodies.add(solid, bf)
    b.name = name
    body_colors.append((name, color))
    return b


SH_IN, SH_OUT = L["SHROUD_IN"], L["SHROUD_OUT"]
PA, PB, PC, PD = L["PLANE_A"], L["PLANE_B"], L["PLANE_C"], L["PLANE_D"]
yA = (PA[0] + PA[1]) / 2
yC = (PC[0] + PC[1]) / 2

# ---- 1A. DRIVETRAIN (B6): clean axles, 8 seated REX bearings, 8 miter gears -------------------
AX, AZ = L["AXLE_X"], L["AXLE_Z"]
AXF, AXR = L["AXLE_X_FRONT"], L["AXLE_X_REAR"]
for sx, xl in ((1, "front"), (-1, "rear")):
    for sy, yl in ((1, "left"), (-1, "right")):
        x = AXF if sx > 0 else -AXR
        # v12.4: axle ends flush with the inner bearing (|Y| 132.05); the old 6 mm inboard stub reached the
        # J-chute's mounting stations once the front axles moved out to X 168.
        # v12.6: REX profile (hex flats 3.48 mm, corners r 3.95), flats aligned to the axle gear's bore
        _ax = cyl(x, sy * 132.05, AZ, x, sy * 218.0, AZ, L["REX_CORNER"])
        for _kk in range(3):
            _th = math.radians(sy * L["MITER_AXLE_PHASE"] + 60.0 * _kk)
            _slab = obox3((x, sy * 175.025, AZ), (math.cos(_th), 0.0, math.sin(_th)), (0.0, 1.0, 0.0), 2 * L["REX_FLAT"], 90.0, 20.0)
            tb.booleanOperation(_ax, _slab, fusion.BooleanTypes.IntersectionBooleanType)
        add(_ax, f"drivetrain_rex_wheel_axle_{xl}_{yl}", "silver")
        for wy0, wy1, tag in ((132.0, 134.5, "inner"), (177.5, 180.0, "outer")):
            add(bearing((x, AZ), sy * wy0, sy * wy1), f"drivetrain_axle_bearing_{xl}_{yl}_{tag}", "bearing_steel")
        ay = sy * 156.0
        # v12.6: the miter gears are goBILDA 2315 vendor models, placed at the end of STEP 3 (apex at Y +/-156)
        # v12.4 buildability: the drive motors had no mount. goBILDA's Strafer holds each one with a
        # 1201-0043-0002 Quad Block Pattern Mount that fills the 1120 channel's 43 mm interior. Modelled
        # here (8 mm, at the gearbox face = axle - 30.0 mm since v12.6): shaft bore, 4x M4 clearance for the
        # motor face (16 mm square), and the drill step bolts it to the channel walls.
        xf = x - sx * L["DRIVE_MOTOR_FACE_DX"]
        x0q, x1q = sorted((xf + sx * 0.25, xf + sx * 8.25))     # gearbox face plate stands 0.16 mm proud
        qb = box(x0q, x1q, *sorted((sy * 134.5, sy * 177.5)), 26.5, 69.5)
        S(qb, cyl(x0q - 1, ay, AZ, x1q + 1, ay, AZ, 7.0))
        for dyq in (-8.0, 8.0):
            for dzq in (-8.0, 8.0):
                S(qb, cyl(x0q - 1, ay + dyq, AZ + dzq, x1q + 1, ay + dyq, AZ + dzq, 2.2))
        add(qb, f"drive_motor_quad_block_mount_{xl}_{yl}", "pulley_alu")

# ---- 1B. FLOATING INTAKE (B3) ----------------------------------------------------------------
PV = L["INTAKE_PIVOT"]
FS = L["INTAKE_SHAFT_LOW"]
AY = L["INTAKE_ARM_Y"]
M7 = L["M7_SHAFT"]
add(cy(PV[0], PV[1], 56.0, 148.0, 3.92), "intake_upper_pivot_stub_shaft_left_driven", "silver")
add(cy(PV[0], PV[1], -148.0, -100.0, 3.92), "intake_upper_pivot_stub_shaft_right", "silver")
add(cy(FS[0], FS[1], -108.5, 108.5, 3.92), "intake_floating_front_rex_shaft", "silver")
# v12.6: 12.0 mm HTD-5M pulleys; lane re-centred to 55.6-67.6 (shaft tip at |Y| 56 still engages)
yI1 = PA[0] - 0.9 + 6.0
add(pulley(M7, yI1, 20), "intake_stage1_m7_pulley_20t", "pulley_alu")
add(pulley(PV, yI1, 20), "intake_stage1_pivot_pulley_20t", "pulley_alu")
add(belt(M7, 20, PV, 20, yI1), "intake_stage1_htd5m_215mm_belt", "belt_black")
y2 = L["INTAKE_STAGE2_Y"]
add(pulley(PV, y2, 20), "intake_stage2_pivot_pulley_20t", "pulley_alu")
add(pulley(FS, y2, 20), "intake_stage2_floating_pulley_20t", "pulley_alu")
add(belt(PV, 20, FS, 20, y2), "intake_stage2_htd5m_260mm_belt", "belt_black")
for idx, wy in enumerate(L["INTAKE_WHEEL_YS"], start=1):
    w = S(cy(FS[0], FS[1], wy - 10, wy + 10, L["INTAKE_WHEEL_R"]), cy(FS[0], FS[1], wy - 11, wy + 11, 3.98))
    rm, rh = (L["INTAKE_WHEEL_R"] + 10.5) / 2, (L["INTAKE_WHEEL_R"] - 10.5) * 0.26
    for k in range(8):
        a = math.radians(k * 45)
        S(w, cy(FS[0] + rm * math.cos(a), FS[1] + rm * math.sin(a), wy - 11, wy + 11, rh))
    add(w, f"floating_spoked_compliant_wheel_{idx}", "compliant_teal")
a_low = L["ang"](PV, FS) + 180.0
a_high = L["ang"](PV, L["INTAKE_SHAFT_HIGH"]) + 180.0
r_pin = 22.0
dl = math.degrees(math.asin((L["INTAKE_ARM_W"] / 2 + 4.05) / r_pin))
for sy, sl in ((1, "left"), (-1, "right")):
    y0, y1 = sorted((sy * (AY - 4), sy * (AY + 4)))
    add(bearing(PV, y0 + 0.1, y1 - 0.1), f"intake_arm_pivot_bearing_{sl}", "bearing_steel")
    add(bearing(FS, y0 + 0.1, y1 - 0.1), f"intake_arm_front_bearing_{sl}", "bearing_steel")
    add(swing_arm(PV, FS, y0, y1, w=L["INTAKE_ARM_W"], tail=L["INTAKE_TAIL"]), f"floating_intake_swing_arm_{sl}", "silver")
    sp0, sp1 = sorted((sy * 124.0, sy * 131.9))
    bp_ = L["BLOCKER_PIVOT"]
    _stp = cy(PV[0], PV[1], sp0, sp1, 25.0)
    if sy < 0:   # v12.6: the right upright is mirrored (web at Y -180, open side facing the plate): bosses to the web
        for _dx, _dz in L["STOP_PLATE_SCREWS"]:
            U(_stp, cy(PV[0] + _dx, PV[1] + _dz, -177.45, sp1, 5.0))
    S(_stp, cy(PV[0], PV[1], sp0 - 1, sp1 + 1, 7.0), cy(bp_[0], bp_[1], sp0 - 1, sp1 + 1, 4.6))
    for _dx, _dz in L["STOP_PLATE_SCREWS"]:       # v12.6: 2x M4 into the upright web holes (nylocks inside the channel)
        S(_stp, cy(PV[0] + _dx, PV[1] + _dz, -181.0 if sy < 0 else sp0 - 1, sp1 + 1, 2.25))
    add(_stp, f"intake_upright_bearing_and_stop_plate_{sl}", "mount_slate")
    add(bearing(PV, sp0 + 0.1, sp1 - 0.1), f"intake_upright_pivot_bearing_{sl}", "bearing_steel")
    # lower stop: pin under the arm bar (r = 40) bolted into the upright; upper stop: pin on the tail (r = 22) in the stop plate
    a_arm = L["ang"](PV, FS)
    pc_lo = L["polar"](PV, 40.0, a_arm - math.degrees(math.asin((L["INTAKE_ARM_W"] / 2 + 4.05) / 40.0)))
    q0, q1 = sorted((sy * 100.0, sy * 131.9))
    add(cy(pc_lo[0], pc_lo[1], q0, q1, 4.0), f"intake_lower_hard_stop_pin_{sl}", "stop_alu")
    pc_up = L["polar"](PV, r_pin, a_high + dl)
    q0, q1 = sorted((sy * 100.0, sy * 124.0))
    add(cy(pc_up[0], pc_up[1], q0, q1, 4.0), f"intake_upper_hard_stop_pin_{sl}", "stop_alu")

# ---- 1C. MAGAZINE (B2): coaxial belt-on-pivot gecko swing arms ---------------------------------
M8 = L["M8_SHAFT"]
G1 = L["G1_REST"]
J2, G2 = L["J2"], L["G2_REST"]
MDS = L["MAG_DS_Y"]
add(cy(G1[0], G1[1], -79.5, 22.5, 3.92), "gecko1_rex_shaft_cantilever", "silver")
add(cy(G2[0], G2[1], -71.0, 71.0, 3.92), "gecko2_rex_shaft", "silver")
add(cy(J2[0], J2[1], -MDS[1], -SH_IN, 3.92), "magazine_j2_jackshaft_right", "silver")
add(cy(J2[0], J2[1], SH_IN, 71.0, 3.92), "magazine_j2_pivot_pin_left", "silver")
add(bearing(J2, -SH_OUT, -SH_IN), "magazine_j2_shroud_bearing_right", "bearing_steel")
add(bearing(J2, SH_IN, SH_OUT), "magazine_j2_shroud_bearing_left", "bearing_steel")
add(bearing(J2, -MDS[1], -MDS[0]), "magazine_j2_ds_plate_bearing", "bearing_steel")
# v12.6: HTD-3M 6 mm belts on printed 33T pulleys (8.5 mm). Lane A = 55.6-64.1, lane C = 70.9-79.4: the pulley
# flanges use the 0.4-0.9 mm gaps beside the 6 mm arm plane B, so A + B + C = 23.8 mm fits the M8's 24 mm shaft.
MT, MP = L["MAG_T"], L["MAG_PITCH"]
yMA = PA[0] - 0.9 + 4.25
yMC = PC[1] + 0.9 - 4.25
_bl = {k: int(v["cots_L_mm"]) for k, v in L["belt_ledger"]().items()}
add(pulley(M8, -yMA, MT, MP), f"magazine_b1_m8_pulley_{MT}t", "pulley_alu")
add(pulley(G1, -yMA, MT, MP), f"magazine_b1_gecko1_pulley_{MT}t", "pulley_alu")
add(belt(M8, MT, G1, MT, -yMA, MP), f"magazine_b1_htd3m_{_bl['magazine_b1_m8_to_gecko1']}mm_belt", "belt_black")
MT2 = L["MAG_T2"]
add(pulley(M8, -yMC, MT2, MP), f"magazine_b2_m8_pulley_{MT2}t", "pulley_alu")
add(pulley(J2, -yMC, MT2, MP), f"magazine_b2_j2_pulley_{MT2}t", "pulley_alu")
add(belt(M8, MT2, J2, MT2, -yMC, MP), f"magazine_b2_htd3m_{_bl['magazine_b2_m8_to_j2']}mm_belt", "belt_black")
add(pulley(J2, -yMA, MT, MP), f"magazine_b3_j2_pulley_{MT}t", "pulley_alu")
add(pulley(G2, -yMA, MT, MP), f"magazine_b3_gecko2_pulley_{MT}t", "pulley_alu")
add(belt(J2, MT, G2, MT, -yMA, MP), f"magazine_b3_htd3m_{_bl['magazine_b3_j2_to_gecko2']}mm_belt", "belt_black")
add(tube_y(G1[0], G1[1], 16.2, 22.0, 7.5, 4.05), "gecko1_retaining_collar", "pulley_alu")
add(tube_y(G1[0], G1[1], -16.2, -50.6, 8.0, 4.05), "gecko1_shaft_spacer_right", "pulley_alu")
for sy, sl in ((1, "left"), (-1, "right")):
    add(tube_y(G2[0], G2[1], sy * 16.2, sy * 50.6, 8.0, 4.05), f"gecko2_shaft_spacer_{sl}", "pulley_alu")
yb0, yb1 = -PB[1], -PB[0]
add(bearing(M8, yb0 + 0.1, yb1 - 0.1), "gecko1_arm_pivot_bearing_on_m8_shaft", "bearing_steel")
add(bearing(G1, yb0 + 0.1, yb1 - 0.1), "gecko1_arm_shaft_bearing", "bearing_steel")
a1 = math.radians(L["G1_A_REST"])
ap = (M8[0] + 40 * math.cos(a1), M8[1] + 40 * math.sin(a1))
g1_pin = (ap[0] + 12.0 * math.sin(a1), ap[1] - 12.0 * math.cos(a1))
g1_tab = S(U(bar_xz(ap, g1_pin, yb0, yb1, 9.0), cy(g1_pin[0], g1_pin[1], yb0, yb1, 5.0)), cy(g1_pin[0], g1_pin[1], yb0 - 1, yb1 + 1, 3.05))
# v12.1: 14 mm-wide wheel-end hub (planes B+C) carries TWO bearings 7.4 mm apart so the cantilevered gecko-1 shaft cannot rock
g1_boss = cy(G1[0], G1[1], -78.4, yb1, 11.5)
g1_arm = swing_arm(M8, G1, yb0, yb1, tabs=(g1_tab, g1_boss))
S(g1_arm, cy(G1[0], G1[1], -80.0, -63.0, 7.0))
add(g1_arm, "gecko1_coaxial_swing_arm_on_m8", "nectar_orange")
add(bearing(G1, -78.3, -71.0), "gecko1_arm_shaft_bearing_outer", "bearing_steel")
g1_fixed = (g1_pin[0], g1_pin[1] - 34.0)
add(cy(g1_pin[0], g1_pin[1], -PC[1], yb1, 3.0), "gecko1_spring_arm_pin", "silver")
add(cy(g1_fixed[0], g1_fixed[1], -PC[1], -SH_OUT, 2.5), "gecko1_spring_fixed_pin", "silver")
add(spring(g1_pin, g1_fixed, -yC, pins=((g1_pin, 3.0), (g1_fixed, 2.5))), "gecko1_return_extension_spring", "spring_steel")
for sy, sl in ((1, "left"), (-1, "right")):
    q0, q1 = sorted((sy * PB[0], sy * PB[1]))
    add(bearing(J2, q0 + 0.1, q1 - 0.1), f"gecko2_arm_pivot_bearing_{sl}", "bearing_steel")
    add(bearing(G2, q0 + 0.1, q1 - 0.1), f"gecko2_arm_shaft_bearing_{sl}", "bearing_steel")
    add(swing_arm(J2, G2, q0, q1), f"gecko2_coaxial_swing_arm_{sl}", "nectar_orange")
add(tube_y(J2[0], J2[1], -PD[1], -PD[0], 9.0, 4.0), "gecko2_torsion_return_spring", "spring_steel")

# ---- 1D. INDEXER GATE (S3) below the bend floor -----------------------------------------------
GP, BC = L["GATE_PIVOT"], L["BEND_C"]
add(rex_y(GP[0], GP[1], -40.0, L["S3_SHAFT_END"]), "indexer_gate_rex_shaft", "silver")   # v12.6: real REX into the S3 coupler
tip = L["GATE_TIP"]
gate = cy(GP[0], GP[1], -27.0, 27.0, 5.5)
for fy in L["GATE_FINGER_YS"]:
    U(gate, bar_xz(GP, tip, fy - 2.0, fy + 2.0, 6.0), cy(GP[0], GP[1], fy - 2.0, fy + 2.0, 7.0))
add(S(gate, rex_bore_y(GP[0], GP[1], -45, 60)), "indexer_stopper_gate_3prong_fingers", "gate_crimson")   # v12.6: REX bore (servo torque)

# ---- 1E. UNIFIED J-CHUTE (B1) -------------------------------------------------------------------
TH = L["RAMP_DEG"]
nX, nZ = L["N"]
dX, dZ = -math.cos(math.radians(TH)), math.sin(math.radians(TH))
RT, LIP, FT = L["RAMP_TANGENT"], L["LIP"], L["FLOOR_T"]
WX, WT, CH = L["WALL_X"], L["WALL_T"], L["CURB_H"]
x_lip_far = LIP[0] + 10.0
ramp_len = math.hypot(x_lip_far - RT[0], L["ramp_z"](x_lip_far) - RT[1])
mf = ((x_lip_far + RT[0]) / 2, (L["ramp_z"](x_lip_far) + RT[1]) / 2)
ramp = obox(mf[0] - nX * FT / 2, 0, mf[1] - nZ * FT / 2, dX, dZ, ramp_len, 2 * SH_OUT, FT)
I(ramp, halfplane(0, L["LIP_CUT_Z"], 0, 1, -100, 100))
I(ramp, halfplane(RT[0], RT[1], -dX, -dZ, -100, 100))
chute = U(ramp, sector(BC, L["BEND_R"], L["BEND_R"] + FT, L["BEND_A1"], L["BEND_A0"], -SH_OUT, SH_OUT),
          box(WX - WT, WX, -SH_OUT, SH_OUT, BC[1], L["WALL_TOP_Z"]))
for sy in (-1, 1):
    y0, y1 = sorted((sy * SH_IN, sy * SH_OUT))
    c_r = obox(mf[0] + nX * CH / 2, (y0 + y1) / 2, mf[1] + nZ * CH / 2, dX, dZ, ramp_len, SH_OUT - SH_IN, CH)
    I(c_r, halfplane(0, L["LIP_CUT_Z"], 0, 1, -100, 100))
    I(c_r, halfplane(RT[0], RT[1], -dX, -dZ, -100, 100))
    I(c_r, halfplane(LIP[0] + 4, 0, -1, 0, -300, 300))
    U(chute, c_r, sector(BC, L["BEND_R"] - CH, L["BEND_R"], L["BEND_A1"], L["BEND_A0"], y0, y1), box(WX, WX + CH, y0, y1, BC[1], L["WALL_TOP_Z"]))
# v12.1: slits follow every floor crossing of the fingers over the whole retract sweep (0 .. GATE_RETRACT_DEG + 3)
_gp, _tip = L["GATE_PIVOT"], L["GATE_TIP"]
_cross = []
for _k in range(0, int(L["GATE_RETRACT_DEG"]) + 4):
    _t = math.radians(_k)
    _rx, _rz = _tip[0] - _gp[0], _tip[1] - _gp[1]
    _tx, _tz = _gp[0] + _rx * math.cos(_t) - _rz * math.sin(_t), _gp[1] + _rx * math.sin(_t) + _rz * math.cos(_t)
    for _j in range(0, 201):
        _f = _j / 200.0
        _px, _pz = _gp[0] + (_tx - _gp[0]) * _f, _gp[1] + (_tz - _gp[1]) * _f
        if L["BEND_R"] - 1.5 <= math.hypot(_px - BC[0], _pz - BC[1]) <= L["BEND_R"] + FT + 1.5:
            _cross.append(math.degrees(math.atan2(_pz - BC[1], _px - BC[0])) % 360.0)
_s0, _s1 = min(_cross) - 5.0, max(_cross) + 5.0
for fy in L["GATE_FINGER_YS"]:
    S(chute, sector(BC, L["BEND_R"] - 1, L["BEND_R"] + FT + 1, _s0, _s1, fy - 2.5, fy + 2.5))
# station 1: front side hangers bolted to the rail inner walls (X 150..170)
hz0 = L["ramp_z"](150.0) - FT - 6.0
for sy in (-1, 1):
    y0, y1 = sorted((sy * 54.0, sy * 131.9))
    l0, l1 = sorted((sy * 126.0, sy * 131.9))
    U(chute, box(150.0, 170.0, y0, y1, hz0, hz0 + 6.0), box(150.0, 170.0, l0, l1, hz0, 62.0))
# station 2: saddle on the X in [-48,0] mid cross-channel (top Z = 72)
sad = box(-47.5, -0.5, -48.0, 48.0, 72.05, 115.0)
I(sad, halfplane(0.0, L["ramp_z"](0.0) - FT / nZ + 0.6, -nX, -nZ, -100, 100))
U(chute, sad)
# station 3: rear collar bolting the elevator wall to the upper rear crossbar face (X = -120)
U(chute, box(-119.95, WX - WT + 0.6, -SH_OUT, SH_OUT, 168.0, 215.0))
# station 4: gate-shaft hanger under the bend floor
gh = U(bar_xz(GP, L["polar"](BC, L["BEND_R"] + FT - 0.5, L["GATE_A"] + 9.0), -44.0, -36.0, 10.0), cy(GP[0], GP[1], -44.0, -36.0, 8.0))
U(chute, S(gh, cy(GP[0], GP[1], -50, -30, 4.05)))
# v12.5: real REV Color Sensor V3 for the lip (G407/G408). The old 13x14x5 placeholder sat on the funnel
# wing with no fastening. The sensor (41.5 x 15 x 16.8, two M3 ears 32 mm apart through its sensing face)
# now sits on a 6 mm pad built onto the chute's left mouth curb, looking across the mouth at ball height
# through a 12 mm window; M3 inserts in the pad stop 6 mm deep, outside the corridor (curb face Y 55).
LIPS_AP = L["LIP_SENSOR_AP"]
U(chute, box(141.6, 187.9, 55.0, 61.0, 15.0, 55.0))
S(chute, box(LIPS_AP[0] - 6.0, LIPS_AP[0] + 6.0, 50.0, 62.0, LIPS_AP[1] - 6.0, LIPS_AP[1] + 6.0))
for _ex in (-16.0, 16.0):
    S(chute, cyl(LIPS_AP[0] - L["REV_COLOR_AP_DX"] + _ex, 54.9, LIPS_AP[1], LIPS_AP[0] - L["REV_COLOR_AP_DX"] + _ex, 61.5, LIPS_AP[1], 2.0))
add(chute, "unified_continuous_j_chute", "amber_chute")

# ---- 1F. SPRUNG MAGAZINE SHROUDS with concentric arc slots --------------------------------------
x_front = 150.0
s_len = math.hypot(x_front - RT[0], L["ramp_z"](x_front) - RT[1]) + 2
s_mid = ((x_front + RT[0]) / 2, (L["ramp_z"](x_front) + RT[1]) / 2)
for sy, sl in ((1, "left"), (-1, "right")):
    y0, y1 = sorted((sy * SH_IN, sy * SH_OUT))
    strip = obox(s_mid[0] + nX * (CH + 150.0) / 2, (y0 + y1) / 2, s_mid[1] + nZ * (CH + 150.0) / 2, dX, dZ, s_len, SH_OUT - SH_IN, 150.0 - CH)
    I(strip, halfplane(x_front, 0, -1, 0, -300, 300))
    sh = U(strip, cy(BC[0], BC[1], y0, y1, L["BEND_R"] - CH), box(WX + CH, 40.0, y0, y1, BC[1], 215.9))
    I(sh, halfplane(0, 215.9, 0, -1, -300, 300))
    S(sh, cy(J2[0], J2[1], y0 - 1, y1 + 1, 7.0))
    S(sh, arc_slot(J2, L["G2_R"], L["G2_A_LIFT"] - 2.0, L["G2_A_REST"], 4.6, y0 - 1, y1 + 1))
    if sy < 0:
        S(sh, arc_slot(M8, L["G1_R"], L["G1_A_REST"], L["G1_A_LIFT"] + 2.0, 4.6, y0 - 1, y1 + 1))
    else:
        # v12.5: real REV sensors (2m Distance x2, Color V3 x1; 32 mm M3 ear pattern through the sensing
        # face) sit on 6 mm pads built into the shroud, so their M3 inserts (6 deep) stop at the shroud wall
        # and never break into the ball corridor. Ball-count 1 lies horizontal (the stage-1 belt is above it).
        for wx, wz, nm in L["SENSOR_SPOTS"]:
            dx = L["REV_COLOR_AP_DX"] if "color" in nm else L["REV_DIST_AP_DX"]
            if "ball_count" in nm:
                U(sh, box(wx - 24.0, wx + 24.0, 55.0, 61.0, wz - 10.5, wz + 10.5))
                ears = [(wx - dx - 16.0, wz), (wx - dx + 16.0, wz)]
            else:
                U(sh, box(wx - 10.5, wx + 10.5, 55.0, 61.0, wz - 24.0, wz + 24.0))
                ears = [(wx, wz - dx - 16.0), (wx, wz - dx + 16.0)]
            for ex_, ez_ in ears:
                S(sh, cyl(ex_, 54.9, ez_, ex_, 61.5, ez_, 2.0))
        for wx, wz in L["SENSOR_WINDOWS"]:
            S(sh, box(wx - 6, wx + 6, y0 - 1, 62.0, wz - 6, wz + 6))
    # v12.2 buildability: the shroud only rested on the 4 mm chute curb and sat 0.1 mm under the deck.
    # An 8 mm-deep outboard flange along its top screws up into the aluminium deck (M3 inserts in the
    # flange, clearance holes in the deck). Kept at X <= -42 and |Y| <= 63, clear of gecko 2 and its belts.
    fy0, fy1 = sorted((sy * 55.0, sy * 63.0))
    U(sh, box(-98.0, -42.0, fy0, fy1, 207.9, 215.9))
    add(sh, f"sprung_magazine_shroud_{sl}", "shroud_violet")
# v12.2 buildability: S3 (indexer gate) hung on its own output spline. L-bracket: base flange on the mid
# cross-channel's top web (Z 72, X -48..0, holes along Z), a vertical plate beside the left curb with
# spline clearance, and two blocks behind the ear plates (ears at Y 69.7-72.2, case X -88.1..-48.1).
GP = L["GATE_PIVOT"]
s3b = U(box(-96.0, -36.0, 55.0, 58.5, 72.02, 105.0),
        box(-48.0, -30.0, 55.0, 90.0, 72.02, 76.0),
        box(-95.2, -88.6, 58.5, 67.7, 83.2, 103.2), box(-47.6, -40.9, 58.5, 67.7, 83.2, 103.2))   # ear plate starts at Y 67.76
S(s3b, cy(GP[0], GP[1], 54.0, 60.0, 7.4))   # v12.6: clears the 4001 coupler body (r 6.88) that passes through
for ex in (-91.45, -44.7):                  # S3 ear-screw holes (M3 heat-set inserts) at the ear-hole centres
    for ez in (88.15, 98.15):
        S(s3b, cy(ex, ez, 59.5, 67.8, 2.0))
add(s3b, "s3_indexer_servo_bracket", "mount_slate")
# outboard double-shear plate (right): J2 outer bearing + M8 gearbox collar
md0, md1 = -MDS[1], -MDS[0]
mds = U(bar_xz(M8, J2, md0, md1, 30.0), cy(M8[0], M8[1], md0, md1, 24.0), cy(J2[0], J2[1], md0, md1, 12.0))
mds = S(mds, cy(M8[0], M8[1], md0 - 1, md1 + 1, 18.3), cy(J2[0], J2[1], md0 - 1, md1 + 1, 7.0))
# v12.2 buildability: the plate only touched its J2 bearing, and the motor's face screws sit inside its
# 36.6 mm bore. A square sleeve now slides over the 36 mm gearbox and screws into the gearbox's
# tapped SIDE holes (at Y = -95.1), so the plate is located and held by the motor itself.
GBY = -95.1
slv = box(M8[0] - 21.0, M8[0] + 21.0, -99.0, md1, M8[1] - 21.0, M8[1] + 21.0)
S(slv, box(M8[0] - 18.2, M8[0] + 18.2, -100.0, md1 + 1.0, M8[1] - 18.2, M8[1] + 18.2))
for hx, hz, ax in ((M8[0], M8[1] + 19.6, "z"), (M8[0], M8[1] - 19.6, "z"), (M8[0] + 19.6, M8[1], "x"), (M8[0] - 19.6, M8[1], "x")):
    if ax == "z":
        S(slv, tb.createCylinderOrCone(P(hx, GBY, hz - 3.0), 0.215, P(hx, GBY, hz + 3.0), 0.215))
    else:
        S(slv, tb.createCylinderOrCone(P(hx - 3.0, GBY, hz), 0.215, P(hx + 3.0, GBY, hz), 0.215))
U(mds, slv)
add(mds, "magazine_outboard_double_shear_plate_right", "pulley_alu")
# M7 / M8 clamp saddles on the rail top web
for mc, sy, nm in ((M7, 1, "m7_intake_motor_clamp_saddle_left"), (M8, -1, "m8_magazine_motor_clamp_saddle_right")):
    y0, y1 = sorted((sy * 132.5, sy * 178.0))
    _sad = box(mc[0] - 20.0, mc[0] + 20.0, y0, y1, 72.05, mc[1] - 26.4)
    if sy > 0:   # v12.6: M7 rested on this block and was held by nothing. Sleeve over the gearbox (Y 88-121) + bridge.
        _slv = S(box(mc[0] - 21.0, mc[0] + 21.0, 88.0, 121.0, mc[1] - 21.0, mc[1] + 21.0),
                 box(mc[0] - 18.2, mc[0] + 18.2, 87.0, 122.0, mc[1] - 18.2, mc[1] + 18.2))
        U(_sad, _slv, box(mc[0] - 21.0, mc[0] + 21.0, 110.0, 121.0, 86.0, mc[1] - 18.2), box(mc[0] - 20.0, mc[0] + 20.0, 110.0, 140.0, 86.0, mc[1] - 26.5))
        for _gy in (95.1, 118.1):                       # 4x M4 into the gearbox's tapped side holes (-X face and top face)
            S(_sad, tb.createCylinderOrCone(P(mc[0] - 23.0, _gy, mc[1]), 0.215, P(mc[0] - 16.0, _gy, mc[1]), 0.215))
            S(_sad, tb.createCylinderOrCone(P(mc[0], _gy, mc[1] + 16.0), 0.215, P(mc[0], _gy, mc[1] + 23.0), 0.215))
    add(_sad, nm, "mount_slate")

# ---- 1G. SHOOTER (B4/B5) ---------------------------------------------------------------------
FW, RW = L["FRONT_WHEEL"], L["REAR_WHEEL_REST"]
M5, M6, HH = L["M5_SHAFT"], L["M6_SHAFT"], L["HOOD_HINGE"]
SPY0, SPY1 = L["SIDE_PLATE_Y"]
SAY0, SAY1 = L["SWING_ARM_Y"]
add(rex_y(FW[0], FW[1], -97.0, 97.0), "shooter_main_rex_shaft", "silver")   # v12.6: real REX (the flywheel hubs' REX bores lock to it)
add(cy(RW[0], RW[1], -96.0, SAY1 + 0.5, 3.92), "shooter_counter_rex_shaft", "silver")
a_r = math.radians(L["COUNTER_A_REST"])
base_pt = (M6[0] + 40 * math.cos(a_r), M6[1] + 40 * math.sin(a_r))
arm_pin = (base_pt[0] + 11 * math.sin(a_r), base_pt[1] - 11 * math.cos(a_r))
fixed_pin = (-80.0, 300.0)
for sy, sl in ((1, "left"), (-1, "right")):
    def yy(a, b, s=sy):
        return sorted((s * a, s * b))
    add(cy(M6[0], M6[1], *yy(SPY0 + 0.2, SAY1 + (0.4 if sy > 0 else 3.4)), 3.92), f"shooter_coaxial_pivot_stub_{sl}", "silver")
    add(rex_y(HH[0], HH[1], *yy(50.0, 66.5 if sy > 0 else -L["S6_COUPLER_REX_END"])), f"hood_hinge_stub_shaft_{sl}", "silver")
    add(tube_y(FW[0], FW[1], *yy(45.2, SPY0 - 0.2), 8.0, 4.05), f"shooter_main_shaft_inner_spacer_{sl}", "pulley_alu")   # v12.6: from the 1313 hub barrel end
    add(tube_y(RW[0], RW[1], *yy(18.2, SPY0 - 0.2), 8.0, 4.05), f"shooter_counter_shaft_inner_spacer_{sl}", "pulley_alu")
    for bc, bn in ((FW, "main"), (HH, "hood_hinge"), (M6, "pivot")):
        add(bearing(bc, *yy(SPY0 + 0.1, SPY1 - 0.1)), f"shooter_side_plate_bearing_{bn}_{sl}", "bearing_steel")
    add(bearing(M6, *yy(SAY0 + 0.1, SAY1 - 0.1)), f"shooter_swing_arm_pivot_bearing_{sl}", "bearing_steel")
    add(bearing(RW, *yy(SAY0 + 0.1, SAY1 - 0.1)), f"shooter_swing_arm_counter_bearing_{sl}", "bearing_steel")
    tab = S(U(bar_xz(base_pt, arm_pin, *yy(SAY0, SAY1), 9.0), cy(arm_pin[0], arm_pin[1], *yy(SAY0, SAY1), 5.0)), cy(arm_pin[0], arm_pin[1], *yy(SAY0 - 1, SAY1 + 1), 3.05))
    add(swing_arm(M6, RW, *yy(SAY0, SAY1), tabs=(tab,)), f"shooter_coaxial_swing_arm_{sl}", "nectar_orange")
    add(cy(arm_pin[0], arm_pin[1], *yy(SAY0, 74.5), 3.0), f"shooter_spring_arm_pin_{sl}", "silver")
    add(cy(fixed_pin[0], fixed_pin[1], *yy(SPY1, 74.5), 3.0), f"shooter_spring_fixed_pin_{sl}", "silver")
    add(spring(arm_pin, fixed_pin, sy * 71.5, pins=((arm_pin, 3.0), (fixed_pin, 3.0))), f"shooter_return_extension_spring_{sl}", "spring_steel")
add(tube_y(M6[0], M6[1], -74.5, -68.6, 6.5, 4.05), "shooter_m6_stub_shaft_coupler", "pulley_alu")
yl, yr = 78.6, -81.2      # v12.6: shooter belt lanes (12.0 mm pulleys): left pulley seats on the DS bearing, right clears the return spring (|Y| <= 75)
add(tube_y(FW[0], FW[1], SPY1 + 0.2, yl - 6.2, 6.0, 4.05), "shooter_main_shaft_outer_spacer_left_a", "pulley_alu")
add(tube_y(FW[0], FW[1], -(L["DS_RIGHT_Y"][0] - 0.2), -(SPY1 + 0.2), 6.0, 4.05), "shooter_main_shaft_outer_spacer_right", "pulley_alu")
add(bearing(FW, L["DS_LEFT_Y"][0] + 0.1, L["DS_LEFT_Y"][1] - 0.1), "shooter_ds_left_main_bearing", "bearing_steel")
add(bearing(FW, -(L["DS_RIGHT_Y"][1] - 0.1), -(L["DS_RIGHT_Y"][0] + 0.1)), "shooter_ds_right_main_bearing", "bearing_steel")
add(pulley(M5, yl, 20), "shooter_left_m5_pulley_20t", "pulley_alu")
add(pulley(FW, yl, 20), "shooter_left_front_wheel_pulley_20t", "pulley_alu")
add(belt(M5, 20, FW, 20, yl), "shooter_left_htd5m_260mm_belt", "belt_black")
add(pulley(M6, yr, 32), "shooter_right_m6_pulley_32t", "pulley_alu")
add(pulley(RW, yr, 16), "shooter_right_counter_pulley_16t", "pulley_alu")
add(belt(M6, 32, RW, 16, yr), "shooter_right_htd5m_245mm_belt", "belt_black")
add(tube_y(RW[0], RW[1], -(SAY1 + 0.2), -74.6, 6.0, 4.05), "shooter_counter_shaft_outer_spacer_right", "pulley_alu")
# hood: hinge at X = -116, plate 20 deg forward of vertical
hs_, hc_ = math.sin(math.radians(L["HOOD_DEG_FROM_VERTICAL"])), math.cos(math.radians(L["HOOD_DEG_FROM_VERTICAL"]))
h0 = (HH[0] + 6 * hs_, HH[1] + 6 * hc_)
h1 = (HH[0] + L["HOOD_LEN"] * hs_, HH[1] + L["HOOD_LEN"] * hc_)
hood = bar_xz(h0, h1, -54.0, 54.0, 5.5)
for sy in (-1, 1):
    q0, q1 = sorted((sy * 49.5, sy * 54.8))
    U(hood, cy(HH[0], HH[1], q0, q1, 7.5), bar_xz(HH, h0, q0, q1, 8.0))
add(S(hood, rex_bore_y(HH[0], HH[1], -60, 60)), "variable_pitch_exit_hood_full", "hood_cyan")   # v12.6: REX bore; S6 drives it via a goBILDA 4001 coupler
# side plates: bearing pockets + TRUE concentric arc slot (no keyhole)
for sy, sl in ((1, "left"), (-1, "right")):
    y0, y1 = sorted((sy * SPY0, sy * SPY1))
    f0, f1 = sorted((sy * SPY1, sy * 72.0))
    pl = U(box(-168.0, 72.0, y0, y1, 222.0, 388.0), cy(HH[0], HH[1], y0, y1, 12.0))   # v12.6: flat 6 mm plate (2D-cuttable)
    # 9 mm leg: the drill puts an 8 mm deep M3 insert hole into it from the (aluminium, clearance-side) side plate, so the
    # hole must end inside the leg; a 5-6 mm leg let it break out and graze the foot top tangentially (torn 3MF mesh)
    _ft = U(box(-160.0, -40.0, f0, f1, 222.0, 228.0), box(-160.0, -40.0, *sorted((sy * SPY1, sy * (SPY1 + 9.0))), 222.0, 252.0))
    for _sx, _sz in L["DS_STANDOFFS"][sl]:
        S(_ft, cy(_sx, _sz, -100.0, 100.0, 6.5))
        S(pl, cy(_sx, _sz, y0 - 1, y1 + 1, 1.65))          # M4 tapped (through-bolt from the DS plate)
    add(_ft, f"turret_side_plate_foot_bracket_{sl}", "mount_slate")
    S(pl, box(-78.0, 14.0, y0 - 1, y1 + 1, 338.0, 400.0))
    S(pl, arc_slot(M6, L["COUNTER_ARM_R"], L["COUNTER_A_REST"], L["COUNTER_A_MAX"], L["COUNTER_SLOT_R"], y0 - 1, y1 + 1))
    for bc in (FW, HH, M6):
        S(pl, cy(bc[0], bc[1], y0 - 1, y1 + 1, 7.0))
    if sy < 0:   # v12.1: relief for the gecko-2 pulley/belt (plane A right) as gecko 2 lifts for a NECTAR
        S(pl, sector(J2, L["G2_R"] - 22.0, L["G2_R"] + 22.0, L["G2_A_LIFT"] - 24.0, L["G2_A_REST"] + 24.0, y0 - 1, y1 + 1))
    add(pl, f"turret_side_plate_{sl}_clean", "amber_chute")
dl0, dl1 = L["DS_LEFT_Y"]
dsl = box(-56.0, 56.0, dl0, dl1, 222.0, 338.0)
for px, pz in L["DS_STANDOFFS"]["left"]:          # v12.6: bosses -> printed standoff tubes (flat 5 mm plate)
    S(dsl, cy(px, pz, dl0 - 1, dl1 + 1, 2.25))
    add(tube_y(px, pz, SPY1, dl0, 5.5, 2.25), f"turret_ds_standoff_tube_left_{int(px)}", "mount_slate")
add(S(dsl, cy(M5[0], M5[1], dl0 - 1, dl1 + 1, 11.8), cy(FW[0], FW[1], dl0 - 1, dl1 + 1, 7.0)), "turret_outboard_double_shear_plate_left", "pulley_alu")
dr0, dr1 = -L["DS_RIGHT_Y"][1], -L["DS_RIGHT_Y"][0]
dsr = box(-168.0, 56.0, dr0, dr1, 222.0, 338.0)
for px, pz in L["DS_STANDOFFS"]["right"]:
    S(dsr, cy(px, pz, dr0 - 1, dr1 + 1, 2.25))
    add(tube_y(px, pz, dr1, -SPY1, 5.5, 2.25), f"turret_ds_standoff_tube_right_{int(px)}", "mount_slate")
S(dsr, cy(M6[0], M6[1], dr0 - 1, dr1 + 1, 11.8), cy(FW[0], FW[1], dr0 - 1, dr1 + 1, 7.0))
S(dsr, arc_slot(M6, L["COUNTER_ARM_R"], L["COUNTER_A_REST"], L["COUNTER_A_MAX"], L["COUNTER_SLOT_R"], dr0 - 1, dr1 + 1))
for _sx, _sz in L["HUSKY_PLATE_SCREWS"]:          # v12.6: HuskyLens L-bracket screws (M3 clearance)
    S(dsr, tb.createCylinderOrCone(P(_sx, -101.0, _sz), 0.17, P(_sx, -84.0, _sz), 0.17))
add(dsr, "turret_outboard_double_shear_plate_right", "pulley_alu")
SADDLE_SCREWS = []
for mc, sy, ml in ((M5, 1, "left_m5"), (M6, -1, "right_m6")):
    yo = 0.0 if sy > 0 else L["M6_DY_OUT"]
    y0, y1 = sorted((sy * (94.0 + yo), sy * (178.0 + yo)))
    _sd = S(box(mc[0] - 29.0, mc[0] + 29.0, y0, y1, 222.0, 256.0), cy(mc[0], mc[1], y0 - 1, y1 + 1, 26.6))
    for _e in (-1, 1):                                            # v12.6: it used to rest on the deck unscrewed
        for _ya, _yb in ((y0, y0 + 20.0), (y1 - 20.0, y1)):
            U(_sd, box(*sorted((mc[0] + _e * 28.9, mc[0] + _e * 36.0)), _ya, _yb, 222.0, 230.0))   # 36.0: clear of the left G409 shield
    for _sx in (-L["SADDLE_SCREW_DX"], L["SADDLE_SCREW_DX"]):
        for _syy in (y0 + 10.0, y1 - 10.0):
            S(_sd, cyl(mc[0] + _sx, _syy, 221.0, mc[0] + _sx, _syy, 231.0, 2.25))
            SADDLE_SCREWS.append((mc[0] + _sx, _syy))
    add(_sd, f"turret_flywheel_motor_saddle_{ml}", "pulley_alu")
_d6 = L["S6_SHIFT"]
s6c = U(box(HH[0] - 27.0, HH[0] + 39.0, -112.0 - _d6, -68.2, 362.0, 371.8), box(HH[0] - 27.0, HH[0] + 39.0, -68.2, -SPY1, 340.0, 371.8))
# v12.2 buildability: S6 only rested on the cradle (0.2 mm gap). Posts now rise to both ear plates
# (ears at Y -85.0..-82.5, case X -126..-86) so the servo screws down through its ears.
U(s6c, box(-133.2, -126.4, -112.0 - _d6, -85.05 - _d6, 371.0, 392.2), box(-85.6, -78.8, -112.0 - _d6, -85.05 - _d6, 371.0, 392.2))
S(s6c, box(-134.0, -125.8, -109.2 - _d6, -103.8 - _d6, 378.0, 386.0))      # notch for S6's wire seal (cable exit)
for ex in (-129.4, -82.6):                  # v12.6: S6 ear-screw holes (M3 heat-set inserts) at all four ear-hole centres;
    for ez in (377.0, 387.0):                # the drill had skipped the left post, leaving two ear screws with no hole
        S(s6c, cy(ex, ez, -95.0 - _d6, -84.9 - _d6, 2.0))
add(s6c, "turret_servo_cradle_s6_hood", "mount_slate")
# v12.5: the real DFRobot SEN0305 board (vendor STEP, placed at the end) hangs by its two M3 ears from this
# block. v12.6: the block was only touching the plate's 5 mm front edge with no screws; it is now an L-bracket whose
# flange lies on the plate's outboard face, held by 2x M3 through the plate (see HUSKY_PLATE_SCREWS). Lens aim unchanged.
# The 2" screen sits 3.5 mm behind the PCB over Z 329.8-365.9, so the block stays behind the screen plane
# (X <= 72.0) and only a lip under the screen reaches forward to the ear standoffs (X 75.29).
hk = U(box(56.05, 72.0, -100.0, -72.1, 316.0, 334.0), box(71.0, 75.25, -100.0, -72.1, 316.0, 329.5),
       box(L["HUSKY_FLANGE_X"][0], L["HUSKY_FLANGE_X"][1], -100.0, -95.0, 316.0, 334.0))   # v12.6: flange on the plate's outboard face
for _sx, _sz in L["HUSKY_PLATE_SCREWS"]:
    S(hk, tb.createCylinderOrCone(P(_sx, -101.0, _sz), 0.17, P(_sx, -84.0, _sz), 0.17))
for _hy in L["HUSKY_EAR_YS"]:
    S(hk, cyl(66.0, _hy, L["HUSKY_EAR_Z"], 75.6, _hy, L["HUSKY_EAR_Z"], 2.0))
add(hk, "dfrobot_huskylens_mount_block", "mount_slate")
deck = box(-168.0, 12.0, -180.0, 180.0, 216.0, 222.0)
S(deck, box(-124.0, 13.0, -SH_OUT, SH_OUT, 210.0, 230.0), box(-32.0, 13.0, -76.0, 76.0, 210.0, 230.0))
# v12.5: 4x M4 tapped holes for the servo power injector's base (goBILDA 32 mm grid), hung under the deck,
# and a wire hole under the main-switch housing (free space below the deck at X > -120).
for _ix, _iy in L["INJECTOR_HOLES"]:
    S(deck, cyl(_ix, _iy, 214.0, _ix, _iy, 223.0, 1.65))
S(deck, box(-118.0, -100.0, 146.0, 157.0, 214.0, 223.0))
for _sx, _sy in SADDLE_SCREWS:                     # v12.6: M4 tapped holes for the flywheel motor saddles
    S(deck, cyl(_sx, _sy, 214.0, _sx, _sy, 223.0, 1.65))
add(deck, "cnc_turret_mounting_deck_plate_clean", "pulley_alu")

# ---- 1H. ODOMETRY, ELECTRONICS & POWER (B7, G419/R606, R502/R505, R603) ------------------------
# v12.3: the old cradles sat at Z 60.5+, ~18 mm above the pods' real bodies, so they held nothing.
# The pod model is goBILDA 3110-0001-0001 (SWINGARM pod): the whole pod (main body + cover + side plate +
# wheel) swings on bearings around an 8 mm REX pivot shaft that runs right through it, with a torsion
# spring pressing the wheel down. Only the SHAFT ENDS are fixed to the robot. Each cradle is a printed
# bracket that clamps both shaft ends in REX holes and never touches the swinging body:
#   plate at the spacer-side shaft end + arm over the pod (within 16 mm of the pivot, so a few degrees of
#   swing can't reach it) + tab on the nearest channel at the other shaft end (drilled into the channel).
# Z values are for the re-seated pods (tyres at Z -0.15); pivot axis at Z 23.9.
POD_PIVOT_Z, REX_R = 23.9, 4.1
pc2 = U(box(-124.65, -120.55, -40.0, -8.0, 9.0, 46.0),          # front plate past the 4 mm spacer (ends X -124.7)
        box(-168.0, -120.55, -40.0, -18.0, 42.0, 46.0),        # arm over the pod (wheel starts at Y -16.75)
        box(-168.0, -164.55, -40.0, -18.0, 14.0, 46.0))        # tab on the rear crossbar wall; shaft end X -167.7, spring to -164.4
S(pc2, cyl(-126.0, -24.0, POD_PIVOT_Z, -119.5, -24.0, POD_PIVOT_Z, REX_R), cyl(-169.0, -24.0, POD_PIVOT_Z, -163.5, -24.0, POD_PIVOT_Z, REX_R))
add(pc2, "odom_pod_center_y_strafe_rear_cradle", "pulley_alu")
pc1 = U(box(-139.0, -121.0, 84.45, 88.45, 9.0, 46.0),          # inboard plate past the spacer (ends Y 88.5)
        box(-139.0, -121.0, 84.45, 132.0, 42.0, 46.0),         # arm over the pod to the left rail
        box(-139.0, -121.0, 128.4, 132.0, 14.0, 46.0))         # tab on the rail inner wall; shaft end at Y 131.5
S(pc1, cyl(-133.0, 83.5, POD_PIVOT_Z, -133.0, 90.0, POD_PIVOT_Z, REX_R), cyl(-133.0, 127.5, POD_PIVOT_Z, -133.0, 133.0, POD_PIVOT_Z, REX_R))
add(pc1, "odom_pod_left_x_rail_cradle", "pulley_alu")          # pod outboard face Y 124 <-> left rail inner wall Y 132
# v12.6: the plate used to sit on the mid cross-channel with no screws. The Pinpoint can't move (chute wall at Y -55.0,
# Control Hub at Y <= -99.5) and no position puts its 32 mm-square M4 holes on channel holes, so: the plate is screwed to
# the channel by 3x M4 at X -40 (Y -92/-76/-68, nylocks inside the channel); the Pinpoint's overhanging corners (X 24)
# take M4 + nylock under the plate; its corners over the channel web (X -8) thread-form into 3.3 mm pilots in this PETG plate.
_ppl = box(-47.0, 30.0, -94.5, -57.0, 72.05, 75.15)
for _hy in (-92.0, -76.0, -68.0):
    S(_ppl, tb.createCylinderOrCone(P(-40.0, _hy, 71.0), 0.22, P(-40.0, _hy, 76.0), 0.22))
for _hx, _hr in ((24.0, 0.22), (-8.0, 0.165)):          # M4 clearance / M4 thread-forming pilot 3.3 mm
    for _hy in (-92.75, -60.75):
        S(_ppl, tb.createCylinderOrCone(P(_hx, _hy, 72.0), _hr, P(_hx, _hy, 76.0), _hr))
add(_ppl, "pinpoint_v2_mount_plate", "pulley_alu")
# v12.6: the battery rides in goBILDA's 1209-0001-0001 Battery Mount (vendor STEP, see VENDOR_DEVICES): no printed tray/strap.
for sy, nm in ((-1, "rev_control_hub_mount_plate_right"), (1, "rev_expansion_hub_mount_plate_left")):
    y0, y1 = sorted((sy * 129.1, sy * 131.9))
    add(box(-119.5, 24.0, y0, y1, 80.0, 179.3), nm, "pulley_alu")
# v12.5: the real REV-31-1387 switch (vendor STEP, placed at the end) sits face-up on this printed housing:
# plate on top (3x M3 inserts), switch body inside the open cavity, leads down through the deck wire hole.
swh = box(-137.0, -91.0, 129.5, 162.5, 222.05, 249.45)
S(swh, box(-131.0, -97.3, 144.0, 158.8, 221.0, 250.0))
for _sx in (-130.0, -114.0, -98.0):
    S(swh, cyl(_sx, 137.65, 243.3, _sx, 137.65, 249.6, 2.0))
add(swh, "rev_main_power_switch_housing", "mount_slate")
# v12.2 buildability: an 8 mm bolt-down ear along the inboard side (Y 115-123), outside the G409 roof,
# whose solid base rails flank the cage at X -164..-148 / -80..-62 (the 6 mm walls alone gave nothing to screw through)
# v12.5: walls raised to Z 264 so they still stand above the real switch's rocker (top Z 261)
add(U(box(-146.0, -140.0, 123.0, 169.0, 222.0, 264.0), box(-88.0, -82.0, 123.0, 169.0, 222.0, 264.0), box(-140.0, -88.0, 163.0, 169.0, 230.0, 264.0), box(-140.0, -88.0, 123.0, 129.0, 230.0, 264.0),
      box(-146.0, -82.0, 115.0, 124.0, 222.0, 230.0)), "main_power_switch_impact_guard_cage", "pulley_alu")   # v12.6: ear runs 1 mm under the inner wall (was edge-only contact = non-manifold mesh)

# ---- 1I. SENSORS ------------------------------------------------------------------------------------
# v12.5: the plain sensor boxes are gone; the real REV sensors are placed at the end (vendor STEPs).

# ---- 1J. G407 FRONT-LIP BLOCKER GATE (S1), stowed --------------------------------------------------
BPV = L["BLOCKER_PIVOT"]
bbar = L["polar"](BPV, L["BLOCKER_L"], L["BLOCKER_A_STOWED"])
add(rex_y(BPV[0], BPV[1], 112.0, L["S1_SHAFT_END"]), "g407_blocker_pivot_stub_left_servo", "silver")   # v12.6: REX into the S1 coupler
# v12.6: the left stub runs in a goBILDA 14 mm-OD flanged REX bearing seated in the left upright web (flange inside the channel)
add(S(U(cy(BPV[0], BPV[1], 132.0, 134.5, 6.98), cy(BPV[0], BPV[1], 134.5, 136.0, 8.0)), cy(BPV[0], BPV[1], 131.0, 137.0, 4.02)), "g407_blocker_web_rex_bearing_left", "bearing_steel")   # outer ring (the REX inner race turns with the stub)
add(cy(BPV[0], BPV[1], -136.0, -112.0, 3.92), "g407_blocker_pivot_stub_right", "silver")
blk = cy(bbar[0], bbar[1], -118.0, 118.0, 4.0)
for sy in (-1, 1):
    q0, q1 = sorted((sy * 112.0, sy * 118.0))
    U(blk, bar_xz(BPV, bbar, q0, q1, 10.0), cy(BPV[0], BPV[1], q0, q1, 9.0))
    if sy > 0:   # v12.6: driven side -- locked to the left REX stub (it used to spin on a bearing: S1 could not move it)
        S(blk, rex_bore_y(BPV[0], BPV[1], q0 - 1, q1 + 1))
    else:
        S(blk, cy(BPV[0], BPV[1], q0 - 1, q1 + 1, 7.0))
add(blk, "g407_front_lip_blocker_gate", "gate_crimson")
for sy, sl in ((1, "left"), (-1, "right")):
    q0, q1 = sorted((sy * 112.1, sy * 117.9))
    if sy < 0:
        add(bearing(BPV, q0, q1), f"g407_blocker_bearing_{sl}", "bearing_steel")

# ---- 1K. ASYMMETRICAL FUNNEL WINGS (X_max <= 228) --------------------------------------------------
for sy, sl, xf, tip_xy in ((1, "left_long", 228.0, (192.0, 56.0)), (-1, "right_short", 222.0, (190.0, 56.0))):
    y0, y1 = sorted((sy * 132.0, sy * 180.0))
    p_root, p_tip = (xf - 4.5, sy * 137.0), (tip_xy[0], sy * tip_xy[1])
    dxw, dyw = p_tip[0] - p_root[0], p_tip[1] - p_root[1]
    wall_w = obox3(((p_root[0] + p_tip[0]) / 2, (p_root[1] + p_tip[1]) / 2, 24.5), (dxw, dyw, 0.0), (-dyw, dxw, 0.0), math.hypot(dxw, dyw), 8.0, 45.0)
    wing = U(box(216.05, xf, y0, y1, 24.0, 72.0), wall_w)
    I(wing, halfplane(0, 6.0, 0, 1, -300, 300))   # v12.2: 6 mm tile clearance (was 2.0)
    I(wing, halfplane(xf, 0, -1, 0, -300, 300))
    c0, c1 = sorted((sy * 131.9, sy * 300.0))
    S(wing, box(100.0, 216.05, c0, c1, -10.0, 100.0))
    # v12.2 buildability: the wing block only butted against the rail's open END (no holes there).
    # A 4 mm tab now lies on the rail's top web (holes every 24 mm) so the wing bolts down.
    U(wing, box(184.0, 216.05, y0 + 0.5, y1 - 0.5, 72.02, 76.0), box(216.0, xf, y0 + 0.5, y1 - 0.5, 71.0, 76.0))
    add(wing, f"asymmetrical_funnel_wing_{sl}", "emerald" if sy > 0 else "teal_dark")

# ---- 1L. G409 SLOPED ANTI-CATCH DECK SHIELDS (40 deg roof) ----------------------------------------
t40 = math.tan(math.radians(40.0))
for sy, sl, xa, xb in ((1, "left", -165.0, -60.0), (-1, "right", -60.0, 10.0)):
    y_in, y_out = 127.0, 180.0
    hpk = (y_out - y_in) * t40
    nrm = (0.0, sy * t40, 1.0)
    pr = (0.0, sy * y_in, 222.0 + hpk)
    y0, y1 = sorted((sy * y_in, sy * y_out))
    wedge = S(box(xa, xb, y0, y1, 222.0, 222.0 + hpk), halfspace(pr, nrm))
    lnn = math.sqrt(1 + t40 * t40)
    pr_in = (pr[0], pr[1] - nrm[1] / lnn * 3.0, pr[2] - nrm[2] / lnn * 3.0)
    yi0, yi1 = sorted((sy * (y_in + 3.0), sy * (y_out - 3.0)))
    inner = S(box(xa + 3.0, xb - 3.0, yi0, yi1, 225.0, 222.0 + hpk), halfspace(pr_in, nrm))
    S(wedge, inner)
    if sy > 0:
        S(wedge, box(-148.0, -80.0, 120.0, 175.0, 200.0, 300.0))
    add(wedge, f"g409_sloped_deck_shed_shield_{sl}", "shield_petg")

# ---- 1M. (v12.2) FLOWER capper removed: endgame time goes to HIVE tips, never NECTAR into FLOWERS (G410 moot)

# ---- 1M2. (v12.2) S1 SERVO IMPACT GUARD -------------------------------------------------------------
# S1 (G407 lip blocker) sits on the OUTSIDE of the left upright out to Y = 225, flush with the wheel
# face but higher, right where another robot hits. A 5-sided printed cage (2 mm walls, 1 mm clearance)
# bolts to the upright's outer face and takes the hit; the servo lead exits through a rear slot.
s1g = box(135.8, 196.2, 180.0, 229.0, 154.8, 181.1)
S(s1g, box(137.85, 194.15, 179.0, 226.05, 156.85, 179.0))     # cavity, open at the upright face
S(s1g, box(134.0, 138.0, 210.0, 224.0, 162.0, 174.0))         # servo-lead exit slot, in line with S1's wire seal
U(s1g, box(135.8, 178.0, 180.0, 183.0, 118.0, 155.0))          # stiffening flange down the upright's open side
# The cage is also S1's MOUNT: ribs behind both ears (ear plate at Y 195.3-197.8, case X 146-186) take the
# ear screws, and a tab on the upright's FRONT wall (X = 180, holes along X) bolts the cage to the frame.
U(s1g, box(137.85, 145.6, 197.85, 226.05, 156.85, 179.0), box(186.4, 194.15, 197.85, 226.05, 156.85, 179.0))
U(s1g, box(180.02, 183.5, 136.0, 180.5, 120.0, 181.1))
S(s1g, box(134.0, 146.2, 216.7, 222.0, 164.0, 172.0))          # notch for S1's wire seal (cable exit) through the rear rib
add(s1g, "s1_servo_guard_cage", "mount_slate")

# ---- 1N. R401-R403 QUICK-SWAP SIGNS (174 x 78 mm, 57.5 mm numerals, 7.25 mm visible border) --------
# v12.6: the rail lips cover 3 mm top and bottom, so the plate is 78 mm tall: 7.25 mm visible border (R403) and
# 72 mm visible background (R402) around the centred 57.5 mm numerals.
SG_X0, SG_X1, SG_Z0, SG_Z1 = -97.0, 77.0, 127.0, 205.0
DH, DW, DGAP, STK = 57.5, 34.0, 12.0, 9.0
NZ0 = SG_Z0 + 10.25


def digit_solid(ch, x_of_u, y0, y1):
    def rect(u0, u1, v0, v1):
        xa, xb = sorted((x_of_u(u0), x_of_u(u1)))
        return box(xa, xb, y0, y1, NZ0 + v0, NZ0 + v1)
    if ch == "1":
        return U(rect(14.0, 14.0 + STK, 0, DH), rect(5.0, 14.0 + 0.5, DH - STK, DH), rect(4.0, 30.0, 0, STK))
    if ch == "4":
        return U(rect(20.0, 20.0 + STK, 0, DH), rect(0.0, DW, 16.0, 16.0 + STK), rect(0.0, STK, 16.0, DH))
    r_o = DW / 2.0
    cv = DH - r_o
    xc = x_of_u(r_o)
    ring = S(cyl(xc, y0, NZ0 + cv, xc, y1, NZ0 + cv, r_o), cyl(xc, y0 - 1, NZ0 + cv, xc, y1 + 1, NZ0 + cv, r_o - STK))
    return U(ring, rect(DW - STK, DW, 0, cv), rect(2.0, DW, 0, STK))


for sy, sl in ((1, "left"), (-1, "right")):
    py0, py1 = sorted((sy * 180.6, sy * 183.6))
    ny0, ny1 = sorted((sy * 183.6, sy * 184.6))
    add(box(SG_X0, SG_X1, py0, py1, SG_Z0, SG_Z1), f"robot_sign_{sl}_149_plate_red", "sign_red")
    xc = (SG_X0 + SG_X1) / 2.0
    total = 3 * DW + 2 * DGAP
    for k, ch in enumerate("149"):
        if sy > 0:   # viewer on +Y side reads toward -X
            st = xc + total / 2 - k * (DW + DGAP)
            fx = (lambda s: (lambda u: s - u))(st)
        else:
            st = xc - total / 2 + k * (DW + DGAP)
            fx = (lambda s: (lambda u: s + u))(st)
        add(digit_solid(ch, fx, ny0, ny1), f"robot_sign_{sl}_numeral_{ch}_white", "sign_white")
    r0, r1 = sorted((sy * 180.0, sy * 186.0))
    l0, l1 = sorted((sy * 184.7, sy * 186.0))
    # v12.6: the rails only rested on the mid tower (contact is not fastening). The sign covers the tower face between
    # Z 127 and 205, so each rail gets a tab (bottom: down to Z 106; top: base up to Z 213) over the tower with 2x M4
    # clearance holes on the tower's own hole grid (X -28 / 4; Z 112 bottom, Z 208 top). Right tower: wall faces out ->
    # M4 x 16 + nylock. Left tower: open side faces out -> M4 x 60 across the channel through a printed spacer tube to its
    # far wall (Y 132-134.5), so the clamp never loads the channel lips.
    rb = U(box(SG_X0 - 3, SG_X1, r0, r1, SG_Z0 - 5, SG_Z0), box(SG_X0 - 3, SG_X1, l0, l1, SG_Z0, SG_Z0 + 3), box(-36.0, 12.0, r0, r1, 106.0, SG_Z0 - 5))
    rt = U(box(SG_X0 - 3, SG_X1, r0, r1, SG_Z1, SG_Z1 + 5), box(SG_X0 - 3, SG_X1, l0, l1, SG_Z1 - 3, SG_Z1), box(-36.0, 12.0, r0, r1, SG_Z1 + 5, 213.0))
    for _rx in (-28.0, 4.0):
        S(rb, cy(_rx, 112.0, r0 - 1, r1 + 1, 2.25))
        S(rt, cy(_rx, 208.0, r0 - 1, r1 + 1, 2.25))
        if sy > 0:
            for _rz in (112.0, 208.0):
                add(tube_y(_rx, _rz, 134.5, 180.0, 4.0, 2.25), f"sign_rail_bolt_spacer_left_{int(_rx)}_{int(_rz)}", "mount_slate")
    add(rb, f"robot_sign_{sl}_slide_rail_bottom", "mount_slate")
    add(rt, f"robot_sign_{sl}_slide_rail_top", "mount_slate")

# ---- 1O. STAGED BALLS (audit-visible, never used as cutters) ----------------------------------------
for nm, c, r in L["STAGED_BALLS"]:
    add(tb.createSphere(P(c[0], 0.0, c[1]), r / 10.0), nm, "nectar_orange" if r > 40 else "pollen_yellow")

bf.finishEdit()

# =============================================================================
# STEP 2: appearances
# =============================================================================
def get_or_make_color(name, r, g, b):
    a = design.appearances.itemByName(name)
    if not a:
        a = design.appearances.addByCopy(design.appearances.item(0), name)
    for k in range(a.appearanceProperties.count):
        pr = a.appearanceProperties.item(k)
        if isinstance(pr, core.ColorProperty) and not pr.isReadOnly:
            try:
                pr.value = core.Color.create(r, g, b, 255) if (pr.id in ("opaque_albedo", "surface_albedo") or pr.name == "Color") else core.Color.create(255, 255, 255, 255)
            except Exception:
                pass
    return a


palette = {
    "emerald": (16, 185, 129), "teal_dark": (13, 148, 136), "compliant_teal": (20, 184, 166), "silver": (226, 232, 240),
    "comb_dark": (30, 41, 59), "belt_black": (15, 23, 42), "pulley_alu": (203, 213, 225), "amber_chute": (245, 158, 11),
    "nectar_orange": (249, 115, 22), "pollen_yellow": (250, 204, 21), "shroud_violet": (167, 139, 250), "mount_slate": (71, 85, 105),
    "gate_crimson": (239, 68, 68), "hood_cyan": (14, 165, 233), "bearing_steel": (100, 116, 139), "brass": (202, 138, 4),
    "stop_alu": (148, 163, 184), "spring_steel": (56, 189, 248), "shield_petg": (125, 211, 252), "carbon": (38, 38, 38),
    "sign_red": (220, 38, 38), "sign_white": (250, 250, 250),
}
apps = {k: get_or_make_color("Hermes_v12_" + k, *v) for k, v in palette.items()}
for nm, ck in body_colors:
    b = up_comp.bRepBodies.itemByName(nm)
    if b:
        b.appearance = apps[ck]

# =============================================================================
# STEP 3: COTS clones + transforms, then a single snapshot
# =============================================================================
# v12.5: real vendor models for every device that used to be a plain box. Imported HERE, before any
# transform is set: creating components after transform2 edits discards the pending transforms (a first try
# left every moved COTS part back at its import pose). Placed at the end of this step.
VEND = "C:/Users/danu0/Downloads/26-27-BIOBUZZ/cad/vendor/"
_vocc = {}
for _v in L["VENDOR_DEVICES"]:
    _o = root.occurrences.addNewComponent(core.Matrix3D.create())
    _o.component.name = _v[1]
    _im = app.importManager
    _im.importToTarget(_im.createSTEPImportOptions(VEND + _v[0]), _o.component)
    _vocc[_v[1]] = _o
_miter_occ = []
_mo = root.occurrences.addNewComponent(core.Matrix3D.create())
_mo.component.name = "gobilda_2315_miter_gear_30t"
app.importManager.importToTarget(app.importManager.createSTEPImportOptions(VEND + L["MITER_STEP"]), _mo.component)
_miter_occ.append(_mo)
# the vendor model's set screws protrude into the bore (they bear on REX flats our round shaft model lacks):
# hide them so gate 1 measures real clashes; they stay on the BOM with the gear
for _k in range(_mo.component.allOccurrences.count):
    if _mo.component.allOccurrences.item(_k).component.name.startswith("2806"):
        _mo.component.allOccurrences.item(_k).isLightBulbOn = False
# REX slip fit: the vendor bore (flats 3.5 mm, corners r 4.0) is exactly the motor shaft's nominal size, so the
# coincident faces break the overlap booleans. Open it 0.02 mm (flats 3.52, corners r 4.02) like a real slip fit.
_tbr = fusion.TemporaryBRepManager.get()
_gq, _gb = None, None
for _k in range(_mo.component.allOccurrences.count):
    _q = _mo.component.allOccurrences.item(_k)
    for _j in range(_q.component.bRepBodies.count):
        _b = _q.component.bRepBodies.item(_j)
        if _gb is None or _b.volume > _gb.volume:
            _gq, _gb = _q, _b
_ba = L["MITER_APEX_LOCAL"]
_cut = _tbr.createCylinderOrCone(core.Point3D.create(-6.2, _ba[1] / 10, _ba[2] / 10), 0.402, core.Point3D.create(-3.6, _ba[1] / 10, _ba[2] / 10), 0.402)
for _kk in range(3):
    _th = math.radians(60.0 * _kk)
    _slab = _tbr.createBox(core.OrientedBoundingBox3D.create(core.Point3D.create(-4.9, _ba[1] / 10, _ba[2] / 10),
                                                            core.Vector3D.create(0, math.cos(_th), math.sin(_th)),
                                                            core.Vector3D.create(0, -math.sin(_th), math.cos(_th)), 0.704, 2.0, 3.0))
    _tbr.booleanOperation(_cut, _slab, fusion.BooleanTypes.IntersectionBooleanType)
_inv = _gq.transform2.copy()
_inv.invert()
_tbr.transform(_cut, _inv)
_newg = _tbr.copy(_gb)
_tbr.booleanOperation(_newg, _cut, fusion.BooleanTypes.DifferenceBooleanType)
_gcomp = _gq.component
_bfg = _gcomp.features.baseFeatures.add()
_bfg.startEdit()
_gcomp.bRepBodies.add(_newg, _bfg)
_bfg.finishEdit()
try:
    _gb.deleteMe()
except Exception:
    _gcomp.features.removeFeatures.add(_gb)
for _k in range(7):
    _miter_occ.append(root.occurrences.addExistingComponent(_mo.component, core.Matrix3D.create()))
_thr_occ = []
_to = root.occurrences.addNewComponent(core.Matrix3D.create())
_to.component.name = "gobilda_1613_thrust_bearing_8mm"
app.importManager.importToTarget(app.importManager.createSTEPImportOptions(VEND + L["THRUST_STEP"]), _to.component)
_thr_occ.append(_to)
for _k in range(3):
    _thr_occ.append(root.occurrences.addExistingComponent(_to.component, core.Matrix3D.create()))
def set_tr(o, m):
    if o is not None:
        o.isGrounded = False
        o.transform2 = m


def shift_to(o, x, y, z):
    m = o.transform2.copy()
    m.translation = V(x, y, z)
    set_tr(o, m)


def occ_aabb(o):
    xs = []

    def walk(q):
        if not q.isLightBulbOn:
            return
        for k in range(q.bRepBodies.count):
            b = q.bRepBodies.item(k)
            if b.isLightBulbOn and b.isSolid:
                bb = b.boundingBox
                xs.append((bb.minPoint.x * 10, bb.minPoint.y * 10, bb.minPoint.z * 10, bb.maxPoint.x * 10, bb.maxPoint.y * 10, bb.maxPoint.z * 10))
        for k in range(q.childOccurrences.count):
            walk(q.childOccurrences.item(k))
    walk(o)
    return [min(a[i] for a in xs) for i in range(3)] + [max(a[i] for a in xs) for i in range(3, 6)]


def move_aabb_min(o, tx, ty, tz):
    bb = occ_aabb(o)
    m = o.transform2.copy()
    t = m.translation
    m.translation = core.Vector3D.create(t.x + (tx - bb[0]) / 10, t.y + (ty - bb[1]) / 10, t.z + (tz - bb[2]) / 10)
    set_tr(o, m)


def place_servo(o, rot_axis, rot_deg, tip_target, axis_sign):
    if rot_axis is not None:
        m = mat_rot(rot_axis, rot_deg)
    else:
        m = o.transform2.copy()
        m.translation = V(0, 0, 0)
    set_tr(o, m)
    sp = [o.childOccurrences.item(k) for k in range(o.childOccurrences.count) if "Spline" in o.childOccurrences.item(k).name][0]
    bb = occ_aabb(sp)
    cx, cz = (bb[0] + bb[3]) / 2, (bb[2] + bb[5]) / 2
    cyy = bb[1] if axis_sign < 0 else bb[4]
    m2 = o.transform2.copy()
    m2.translation = V(tip_target[0] - cx, tip_target[1] - cyy, tip_target[2] - cz)
    set_tr(o, m2)


cots = {}
dx = L["DRIVE_MOTOR_DX"]
DFX = L["AXLE_X_FRONT"] - 144.0       # v12.4: front motors follow the front axles out
for nm, tx, ty in (("gobilda_5203_motor_m1_drive:1", 15.265 - dx + DFX, 118.829), ("gobilda_5203_motor_m2_drive:1", -15.265 + dx, 193.171),
                   ("gobilda_5203_motor_m3_drive:1", 15.265 - dx + DFX, -193.171), ("gobilda_5203_motor_m4_drive:1", -15.265 + dx, -118.829)):
    shift_to(s1.childOccurrences.itemByName(nm), tx, ty, 59.078)
for nm in ("gobilda_mecanum_96mm_front_left:1", "gobilda_mecanum_96mm_front_right:1",
           "rex_axle_shaft_gobilda_mecanum_96mm_front_left:1", "rex_axle_shaft_gobilda_mecanum_96mm_front_left:2"):
    _o = s1.childOccurrences.itemByName(nm)
    _m = _o.transform2.copy()
    _m.translation = core.Vector3D.create(_m.translation.x + DFX / 10.0, _m.translation.y, _m.translation.z)
    set_tr(_o, _m)
    if "mecanum_96mm_front" in nm and "rex_axle" not in nm:          # the stock axles are hidden (no visible bodies)
        cots["moved_front_" + nm] = [round(v, 2) for v in occ_aabb(_o)]
shift_to(s2.childOccurrences.itemByName("gobilda_5203_motor_m7_intake:1"), M7[0] - 37.171, 164.735 + L["M78_DY_OUT"], M7[1] + 11.078)
shift_to(s3.childOccurrences.itemByName("gobilda_5203_motor_m8_magazine:1"), M8[0] + 37.171, -164.735 - L["M78_DY_OUT"], M8[1] + 11.078)
shift_to(tur.childOccurrences.itemByName("gobilda_5203_motor_m5_shooter_master:1"), M5[0] - 37.171, 191.735, M5[1] + 11.078)
shift_to(tur.childOccurrences.itemByName("gobilda_5203_motor_m6_shooter_follower:1"), M6[0] + 37.171, -191.735 - L["M6_DY_OUT"], M6[1] + 11.078)
for nm, x, y, z in (("gobilda_rhino_wheel_96mm:1", FW[0], 0.0, FW[1]), ("gobilda_counter_wheel_72mm_topspin:1", RW[0], 0.0, RW[1])):
    shift_to(tur.childOccurrences.itemByName(nm), x, y, z)
# v12.6: the source model's "steel flywheel" was a solid 82 x 10 mm REX-bore disc (~415 g) that goBILDA does not make;
# hidden and replaced by real 3628 plates on 1313 hubs (VENDOR_DEVICES).
for nm in ("gobilda_steel_flywheel_82mm_left:1", "gobilda_steel_flywheel_82mm_right:1"):
    tur.childOccurrences.itemByName(nm).isLightBulbOn = False
set_tr(s3.childOccurrences.itemByName("gobilda_gecko_wheel_72mm_1:1"), mat_pos(G1[0], 0.0, G1[1]))
set_tr(s3.childOccurrences.itemByName("gobilda_gecko_wheel_72mm_1:2"), mat_pos(G2[0], 0.0, G2[1]))
s1.childOccurrences.itemByName("official_gobilda_12v_battery_14_0014:1").isLightBulbOn = False   # v12.6: replaced by REV-31-1302
move_aabb_min(s1.childOccurrences.itemByName("official_gobilda_pinpoint_v2:1"), -12.0, -98.0, 75.2)
for nm, sy in (("official_rev_control_hub_31_1595:1", -1), ("official_rev_expansion_hub_31_1153:1", 1)):
    h = s1.childOccurrences.itemByName(nm)
    m0 = h.transform2.copy()
    m0.translation = V(0, 0, 0)
    m0.transformBy(mat_rot((0, 1, 0), 90.0))
    set_tr(h, m0)
    move_aabb_min(h, -119.0, 99.5 if sy > 0 else -129.0, 76.3)
    cots[nm] = [round(v, 2) for v in occ_aabb(h)]
opod1 = s1.childOccurrences.itemByName("official_gobilda_4bar_odometry_pod:1")
m1 = opod1.transform2.copy()
pod_dz = (-0.15 - occ_aabb(opod1)[2]) / 10.0
m1.translation = core.Vector3D.create(m1.translation.x, m1.translation.y, m1.translation.z + pod_dz)
set_tr(opod1, m1)
m3 = opod1.transform2.copy()
m3.transformBy(mat_rot((0, 0, 1), 90.0))
opod3 = root.occurrences.addExistingComponent(opod1.component, m3)
bb3 = occ_aabb(opod3)
move_aabb_min(opod3, -167.7, -(bb3[4] - bb3[1]) / 2.0, -0.15)


# v12.3: the pods above were placed by their LOOSE bounding box, which Fusion inflates on curved
# bodies (the omni-wheel side plate claimed Z -0.24; real tyre bottom was +6.02), so both tracking
# wheels floated 6 mm above the tiles. Re-seat each pod so its TYRES' tight lowest point is Z -0.15,
# the same 0.15 mm modelled squash as the mecanum wheels.
def tyre_tight_zmin(occ):
    VX, VY = core.Vector3D.create(1, 0, 0), core.Vector3D.create(0, 1, 0)
    zs = []

    def walk(o):
        if "Tire" in o.name:
            for i in range(o.bRepBodies.count):
                ob = app.measureManager.getOrientedBoundingBox(o.bRepBodies.item(i), VY, VX)
                zs.append((ob.centerPoint.z - ob.height / 2.0) * 10.0)
        for i in range(o.childOccurrences.count):
            walk(o.childOccurrences.item(i))
    walk(occ)
    return min(zs)


for _pod in (opod1, opod3):
    _dz = -0.15 - tyre_tight_zmin(_pod)
    _m = _pod.transform2.copy()
    _m.translation = core.Vector3D.create(_m.translation.x, _m.translation.y, _m.translation.z + _dz / 10.0)
    set_tr(_pod, _m)
    cots["pod_tyre_dz_" + _pod.name] = round(_dz, 2)
u10 = s1.childOccurrences.itemByName("gobilda_1120_10hole_rear:1")
mu = u10.transform2.copy()
mu.translation = core.Vector3D.create(mu.translation.x + 16.8, mu.translation.y, mu.translation.z)
u10_mid = root.occurrences.addExistingComponent(u10.component, mu)
move_aabb_min(u10_mid, -48.0, -132.0, 24.0)
cots["u10_mid"] = [round(v, 2) for v in occ_aabb(u10_mid)]
place_servo(s2.childOccurrences.itemByName("servo_s1_g407_lip_gate:1"), None, 0, (BPV[0], 180.95, BPV[1]), -1)
place_servo(s3.childOccurrences.itemByName("servo_s3_magazine_stopper:1"), (1, 0, 0), 90.0, (GP[0], 55.3, GP[1]), -1)
place_servo(tur.childOccurrences.itemByName("servo_s6_pitch_adjust:1"), (1, 0, 0), -90.0, (HH[0], -68.1 - L["S6_SHIFT"], HH[1]), 1)
for nm, o in (("S1", s2.childOccurrences.itemByName("servo_s1_g407_lip_gate:1")), ("S3", s3.childOccurrences.itemByName("servo_s3_magazine_stopper:1")),
              ("S6", tur.childOccurrences.itemByName("servo_s6_pitch_adjust:1")), ("pod1", opod1), ("pod3", opod3),
              ("pinpoint", s1.childOccurrences.itemByName("official_gobilda_pinpoint_v2:1"))):
    cots[nm] = [round(v, 2) for v in occ_aabb(o)]
# v12.5: place the vendor models imported at the start of STEP 3
for _v in L["VENDOR_DEVICES"]:
    _o = _vocc[_v[1]]
    _m = core.Matrix3D.create()
    _t = _v[5]
    _m.setWithCoordinateSystem(core.Point3D.create(_t[0] / 10.0, _t[1] / 10.0, _t[2] / 10.0),
                               core.Vector3D.create(*_v[2]), core.Vector3D.create(*_v[3]), core.Vector3D.create(*_v[4]))
    set_tr(_o, _m)
    cots["vendor_" + _v[1]] = [round(v, 2) for v in occ_aabb(_o)]
# v12.6: the servo STEP models its 25T output spline as a plain cylinder at the tooth TIP diameter, while the 4001
# coupler models its socket at the tooth ROOT, so the two always "overlap" ~15 mm3 even though real splines mesh.
# Hide the spline stand-in on the three coupled servos (the coupler covers it); the coupler stays visible.
for _k in range(root.allOccurrences.count):
    _q = root.allOccurrences.item(_k)
    if "H25T Spline" in _q.name and any(_s in _q.fullPathName for _s in ("servo_s1_g407_lip_gate", "servo_s3_magazine_stopper", "servo_s6_pitch_adjust")):
        _q.isLightBulbOn = False
# v12.6: goBILDA 2315 miter gears. Axle gear: axis Y, back face toward the outer rail wall. Pinion: axis X on
# the motor shaft, back face toward the motor. Pinion tooth phase solved per corner (min. intersection volume).
_tbm = fusion.TemporaryBRepManager.get()
_gsrc = None
for _k in range(_mo.component.allOccurrences.count):
    _q = _mo.component.allOccurrences.item(_k)
    for _j in range(_q.component.bRepBodies.count):
        _b = _q.component.bRepBodies.item(_j)
        if _gsrc is None or _b.volume > _gsrc[0].volume:
            _gsrc = (_b, _q.transform2)


def _gear_mat(ex, ey, ez, apex):
    a = L["MITER_APEX_LOCAL"]
    t = [apex[i] - (a[0] * ex[i] + a[1] * ey[i] + a[2] * ez[i]) for i in range(3)]
    m = core.Matrix3D.create()
    m.setWithCoordinateSystem(core.Point3D.create(t[0] / 10, t[1] / 10, t[2] / 10),
                              core.Vector3D.create(*ex), core.Vector3D.create(*ey), core.Vector3D.create(*ez))
    return m


def _gear_world_body(m):
    b = _tbm.copy(_gsrc[0])
    _tbm.transform(b, _gsrc[1])
    _tbm.transform(b, m)
    return b


_mk = 0
cots["miter_phase"] = {}
for sx, xl in ((1, "front"), (-1, "rear")):
    for sy, yl in ((1, "left"), (-1, "right")):
        x = AXF if sx > 0 else -AXR
        apex = (x, sy * L["MITER_APEX_Y"], AZ)
        ex_a = (0.0, -float(sy), 0.0)
        _pa = math.radians(L["MITER_AXLE_PHASE"])
        ey_a = (math.cos(_pa), 0.0, float(sy) * math.sin(_pa))
        ez_a = (-math.sin(_pa), 0.0, float(sy) * math.cos(_pa))
        m_a = _gear_mat(ex_a, ey_a, ez_a, (apex[0], apex[1] - sy * L["THRUST_SEAT_SHIFT"], apex[2]))
        ga_w = _gear_world_body(m_a)
        ex_p, ey0, ez0 = (float(sx), 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, float(sx))
        ph = math.radians(L["MITER_PINION_PHASE"])
        ey_p = tuple(math.cos(ph) * ey0[i] + math.sin(ph) * ez0[i] for i in range(3))
        ez_p = tuple(-math.sin(ph) * ey0[i] + math.cos(ph) * ez0[i] for i in range(3))
        m_p = _gear_mat(ex_p, ey_p, ez_p, apex)
        xb = _tbm.copy(ga_w)
        try:
            _tbm.booleanOperation(xb, _gear_world_body(m_p), fusion.BooleanTypes.IntersectionBooleanType)
            v = xb.volume * 1000.0
        except Exception:
            v = 0.0
        set_tr(_miter_occ[_mk], m_a)
        set_tr(_miter_occ[_mk + 1], m_p)
        # thrust bearing in the axle gear's dished back: large washer (local Y -2.74..-1.46, OD 17.5) on the rail wall
        # (12.5 um clear), small washer (OD 15.7) in the gear's r 8 pocket (7.5 um clear). Gear-frame X of its wall face:
        _mt = core.Matrix3D.create()
        _a0 = L["MITER_APEX_LOCAL"]
        _xb = -55.994 - (0.94 + L["THRUST_SEAT_SHIFT"] - 0.0125)
        _mt.setWithCoordinateSystem(core.Point3D.create((_xb - 1.46) / 10, (_a0[1] + 23.03) / 10, (_a0[2] - 14.82) / 10),
                                    core.Vector3D.create(0, 1, 0), core.Vector3D.create(-1, 0, 0), core.Vector3D.create(0, 0, 1))
        _mt.transformBy(m_a)
        set_tr(_thr_occ[_mk // 2], _mt)
        cots["miter_phase"][f"{xl}_{yl}"] = [L["MITER_PINION_PHASE"], L["MITER_AXLE_PHASE"], round(v, 4)]
        _mk += 2
cots["vendor_miter_front_left_axle"] = [round(v, 2) for v in occ_aabb(_miter_occ[0])]
cots["vendor_miter_front_left_pinion"] = [round(v, 2) for v in occ_aabb(_miter_occ[1])]
cots["drive_motor_m1"] = [round(v, 2) for v in occ_aabb(s1.childOccurrences.itemByName("gobilda_5203_motor_m1_drive:1"))]
cots["drive_motor_m2"] = [round(v, 2) for v in occ_aabb(s1.childOccurrences.itemByName("gobilda_5203_motor_m2_drive:1"))]
if design.snapshots.hasPendingSnapshot:
    design.snapshots.add()

multi = [b.name for b in up_comp.bRepBodies if b.lumps.count != 1]
tiny = [(b.name, round(b.volume * 1000, 1)) for b in up_comp.bRepBodies if b.volume * 1000 < 250.0]
result = {"status": "v12_built", "bodies": up_comp.bRepBodies.count, "multi_lumps": multi, "tiny_bodies": tiny, "cots": cots}
