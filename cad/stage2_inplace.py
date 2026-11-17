"""
Fast in-place Stage 2 Builder for Autodesk Fusion 360 (via HermesFusionBridge):
Builds the FTC Team 149 BIOBUZZ 'Project HIVE-QUEEN' Stage 2 'Gatekeeper' Front Intake,
Pre-Control G407/G408 Lip Gate, Floor-Level G418.B De-Pollinator Fork, 96mm-wide Continuous Ball Chute,
and Team 149 Number Signs directly onto the Stage 1 Strafer Chassis in <1s.
"""
import math
import adsk.core as core
import adsk.fusion as fusion


def mat_pos_mm(dx_mm: float, dy_mm: float, dz_mm: float) -> core.Matrix3D:
    m = core.Matrix3D.create()
    m.translation = core.Vector3D.create(dx_mm / 10.0, dy_mm / 10.0, dz_mm / 10.0)
    return m


def mat_rot_deg(axis_xyz: tuple[float, float, float], angle_deg: float) -> core.Matrix3D:
    m = core.Matrix3D.create()
    vec = core.Vector3D.create(float(axis_xyz[0]), float(axis_xyz[1]), float(axis_xyz[2]))
    m.setToRotation(math.radians(angle_deg), vec, core.Point3D.create(0, 0, 0))
    return m


def mat_mul(*matrices: core.Matrix3D) -> core.Matrix3D:
    """Returns M_0 * M_1 * ... * M_n (applies rightmost transform first to a column vector)."""
    out = core.Matrix3D.create()
    for m in reversed(matrices):
        out.transformBy(m)
    return out


STAGE1_OCCS = STATE.setdefault("stage1_occs", {})
STAGE2_OCCS = STATE.setdefault("stage2_occs", {})

saved_s1_transforms = {
    k: o.transform2.copy()
    for k, o in STAGE1_OCCS.items()
    if k != "drivetrain_hardware" and o and o.isValid
}

# IMPORTANT: Delete/recreate stage2_hardware FIRST before setting transform2 on other occurrences,
# because deleteMe() in ParametricDesignType rolls the timeline to the last snapshot!
if "stage2_hardware" in STAGE2_OCCS and STAGE2_OCCS["stage2_hardware"].isValid:
    STAGE2_OCCS["stage2_hardware"].deleteMe()


def place_s2_part(lib_key: str, world_mat: core.Matrix3D, label: str) -> fusion.Occurrence:
    existing = STAGE2_OCCS.get(label)
    if existing is not None and existing.isValid:
        existing.isGrounded = False
        try:
            existing.isGroundToParent = False
        except Exception:
            pass
        existing.transform2 = world_mat
        existing.isLightBulbOn = True
        return existing
    comp = STATE["lib"][lib_key]
    occ = root.occurrences.addExistingComponent(comp, world_mat)
    occ.isGrounded = False
    try:
        occ.isGroundToParent = False
    except Exception:
        pass
    occ.transform2 = world_mat
    occ.isLightBulbOn = True
    STAGE2_OCCS[label] = occ
    return occ


QB_CENTERED = mat_pos_mm(0.0, -6.0, 0.0)
QB_FACE_PLUS_X = mat_mul(mat_rot_deg((0, 0, 1), -90.0), QB_CENTERED)
QB_FACE_MINUS_X = mat_mul(mat_rot_deg((0, 0, 1), 90.0), QB_CENTERED)
MOT_FACE_PLUS_X = mat_mul(mat_rot_deg((0, 0, 1), -90.0), mat_pos_mm(37.171, -101.735, 11.078))
STAR_ALONG_Y = mat_rot_deg((1, 0, 0), 90.0)
SERVO_S1_IN_UPRIGHT = mat_mul(mat_rot_deg((0, 0, 1), -90.0), mat_rot_deg((0, 1, 0), 90.0))

# ==============================================================================
# NATIVE B-REP STAGE 2 HARDWARE & CONTINUOUS 96mm-WIDE BALL CHUTE COMPONENT
# (Created FIRST before setting transform2 on STEP occurrences!)
# ==============================================================================
s2_hw_occ = root.occurrences.addNewComponent(core.Matrix3D.create())
s2_hw_comp = s2_hw_occ.component
s2_hw_comp.name = "Stage2_Intake_And_Chute_Hardware"
STAGE2_OCCS["stage2_hardware"] = s2_hw_occ

tb = fusion.TemporaryBRepManager.get()
base_feat = s2_hw_comp.features.baseFeatures.add()
base_feat.startEdit()

# A) Black Box FTC-Style Floating/Pivoting Compliant-Wheel Intake Architecture:
#    1. Short Left-Pocket M7 Bevel Countershaft at (X = +192.0, Z = 96.0, Y in [+106.0, +135.0 mm]) — does NOT cross the central ball path!
#    2. Twin Coaxial Pivot Stub Shafts at (X = +192.0, Z = 120.0, |Y| in [98.0, 135.0 mm]) in Hole 2 of Front Uprights (leaving the entire
#       central 196mm ball channel 100% open with NO fixed cross-shaft!).
#    3. Left & Right Pivoting Floating Intake Side Arms (|Y| = 102.5 mm) hanging forward/downward from (192.0, 120.0) to (206.0, 104.0)
#       carrying the Floating Front Roller Shaft (3 Star Sleeves + 4 Compliant Grip Wheel Clusters), belt-driven at constant center distance!
intake_axle = tb.createCylinderOrCone(
    core.Point3D.create(19.2, 10.6, 9.6),
    0.4,
    core.Point3D.create(19.2, 13.5, 9.6),
    0.4,
)
b_iax = s2_hw_comp.bRepBodies.add(intake_axle, base_feat)
b_iax.name = "m7_bevel_stub_rex_axle_8mm"

# Twin Coaxial Pivot Stub Shafts at (X = +192.0, Z = 120.0, |Y| in [98.0, 135.0 mm] — Hole 2 of Front Uprights!)
piv_stubs = []
for side, sname in ((1.0, "left"), (-1.0, "right")):
    p_stub = tb.createCylinderOrCone(
        core.Point3D.create(19.2, side * 9.8, 12.0),
        0.4,
        core.Point3D.create(19.2, side * 13.5, 12.0),
        0.4,
    )
    piv_stubs.append(tb.copy(p_stub))
    s2_hw_comp.bRepBodies.add(p_stub, base_feat).name = f"intake_floating_pivot_rex_axle_{sname}"

# Floating Front Roller Shaft at (X = +203.0, Z = 104.0, Y in [-105, +120 mm]) carried by Left & Right Floating Intake Arms!
float_axle = tb.createCylinderOrCone(
    core.Point3D.create(20.3, -10.5, 10.4),
    0.4,
    core.Point3D.create(20.3, 12.0, 10.4),
    0.4,
)
s2_hw_comp.bRepBodies.add(float_axle, base_feat).name = "floating_intake_roller_rex_axle_8mm"

# 4 Compliant Grip Wheel Clusters (R = 23.0 mm) flanking the 3 intake roller clusters on the Floating Shaft (203.0, 104.0):
for cy_mm, clbl in ((-88.0, "R_outer"), (-31.0, "R_inner"), (31.0, "L_inner"), (88.0, "L_outer")):
    cw_tire = tb.createCylinderOrCone(
        core.Point3D.create(20.3, (cy_mm - 3.5) / 10.0, 10.4),
        2.3,
        core.Point3D.create(20.3, (cy_mm + 3.5) / 10.0, 10.4),
        2.3,
    )
    tb.booleanOperation(cw_tire, tb.copy(float_axle), fusion.BooleanTypes.DifferenceBooleanType)
    s2_hw_comp.bRepBodies.add(cw_tire, base_feat).name = f"floating_compliant_wheel_chute_ramp_{clbl}"

# Left & Right Pivoting Floating Intake Side Arms (at Y = ±100.5 mm) from (192.0, 120.0) to (203.0, 104.0):
for side, sname in ((1.0, "left"), (-1.0, "right")):
    arm_box = tb.createBox(
        core.OrientedBoundingBox3D.create(
            core.Point3D.create(0.0, 0.0, 0.0),
            core.Vector3D.create(1.0, 0.0, 0.0),
            core.Vector3D.create(0.0, 1.0, 0.0),
            3.2,
            0.4,
            1.2,
        )
    )
    tb.transform(arm_box, mat_mul(mat_pos_mm(197.5, side * 100.5, 112.0), mat_rot_deg((0, 1, 0), 55.5)))
    for ps in piv_stubs:
        tb.booleanOperation(arm_box, tb.copy(ps), fusion.BooleanTypes.DifferenceBooleanType)
    tb.booleanOperation(arm_box, tb.copy(float_axle), fusion.BooleanTypes.DifferenceBooleanType)
    s2_hw_comp.bRepBodies.add(arm_box, base_feat).name = f"floating_intake_arm_rex_axle_{sname}"

# Constant-Length HTD-5M Belt Drive on Left Side:
# (1) M7 stub shaft (192.0, 96.0) -> Upper Pivot Shaft (192.0, 120.0) at Y = +124.0 mm
# (2) Upper Pivot Shaft (192.0, 120.0) -> Floating Front Roller Shaft (203.0, 104.0) at Y = +117.5 mm
for (x1, z1, x2, z2, by_mm, bname) in (
    (192.0, 96.0, 192.0, 120.0, 124.0, "intake_pivot_drive_mount"),
    (192.0, 120.0, 203.0, 104.0, 117.5, "floating_arm_belt_mount"),
):
    cx, cz = (x1 + x2) / 20.0, (z1 + z2) / 20.0
    dist_cm = math.hypot(x2 - x1, z2 - z1) / 10.0
    ang_deg = -math.degrees(math.atan2(z2 - z1, x2 - x1))
    b_box = tb.createBox(
        core.OrientedBoundingBox3D.create(
            core.Point3D.create(0.0, 0.0, 0.0),
            core.Vector3D.create(1.0, 0.0, 0.0),
            core.Vector3D.create(0.0, 1.0, 0.0),
            dist_cm + 1.2,
            0.4,
            1.2,
        )
    )
    tb.transform(b_box, mat_mul(mat_pos_mm(cx * 10.0, by_mm, cz * 10.0), mat_rot_deg((0, 1, 0), ang_deg)))
    tb.booleanOperation(b_box, tb.copy(intake_axle), fusion.BooleanTypes.DifferenceBooleanType)
    for ps in piv_stubs:
        tb.booleanOperation(b_box, tb.copy(ps), fusion.BooleanTypes.DifferenceBooleanType)
    tb.booleanOperation(b_box, tb.copy(float_axle), fusion.BooleanTypes.DifferenceBooleanType)
    s2_hw_comp.bRepBodies.add(b_box, base_feat).name = bname

# B) Flanged Bearings on inner walls (|Y| = 132 mm) of Left & Right Front Uprights at Z = 120.0 mm (Hole 2!) + Left Z = 96.0 mm (Hole 1!)
for side in (1.0, -1.0):
    fp1 = core.Point3D.create(19.2, (side * 129.5) / 10.0, 12.0)
    fp2 = core.Point3D.create(19.2, (side * 131.9) / 10.0, 12.0)
    flange = tb.createCylinderOrCone(fp1, 0.85, fp2, 0.85)
    for ps in piv_stubs:
        tb.booleanOperation(flange, tb.copy(ps), fusion.BooleanTypes.DifferenceBooleanType)
    b_brg = s2_hw_comp.bRepBodies.add(flange, base_feat)
    b_brg.name = f"intake_pivot_flanged_bearing_{int(side * 132)}"

fp1_m7 = core.Point3D.create(19.2, 12.95, 9.6)
fp2_m7 = core.Point3D.create(19.2, 13.19, 9.6)
flange_m7 = tb.createCylinderOrCone(fp1_m7, 0.85, fp2_m7, 0.85)
tb.booleanOperation(flange_m7, tb.copy(intake_axle), fusion.BooleanTypes.DifferenceBooleanType)
s2_hw_comp.bRepBodies.add(flange_m7, base_feat).name = "intake_m7_flanged_bearing_132"

# C) 90° Brass Miter Bevel Gear Pair + CNC Aluminum Inboard Motor Subframe bolting M7 & qb_m7_intake_mount
#    SOLIDLY to left_rail_17h (Y=+132, Z=72)!
m7_ext = tb.createCylinderOrCone(
    core.Point3D.create(14.98, 10.3, 9.6),
    0.4,
    core.Point3D.create(18.55, 10.3, 9.6),
    0.4,
)
s2_hw_comp.bRepBodies.add(m7_ext, base_feat).name = "m7_intake_drive_rex_axle"

gp1 = core.Point3D.create(18.0, 10.3, 9.6)
gp2 = core.Point3D.create(18.52, 10.3, 9.6)
g_m7 = tb.createCylinderOrCone(gp1, 1.0, gp2, 0.55)
tb.booleanOperation(g_m7, tb.copy(m7_ext), fusion.BooleanTypes.DifferenceBooleanType)
b_gm7 = s2_hw_comp.bRepBodies.add(g_m7, base_feat)
b_gm7.name = "miter_gear_m7_driver"

ap1 = core.Point3D.create(19.2, 10.95, 9.6)
ap2 = core.Point3D.create(19.2, 11.47, 9.6)
g_ishaft = tb.createCylinderOrCone(ap1, 0.55, ap2, 1.0)
tb.booleanOperation(g_ishaft, tb.copy(intake_axle), fusion.BooleanTypes.DifferenceBooleanType)
b_gishaft = s2_hw_comp.bRepBodies.add(g_ishaft, base_feat)
b_gishaft.name = "miter_gear_intake_driven"

# Heavy-Duty CNC 6061 Aluminum Motor Subframe + Dual Upper Clamping Rings for M7 (Left) AND M8 (Right) at Z = 96.0 mm:
for side, mname in ((1.0, "m7_left"), (-1.0, "m8_right")):
    sub_shelf = tb.createBox(
        core.OrientedBoundingBox3D.create(
            core.Point3D.create(8.5, side * 10.4, 6.7),
            core.Vector3D.create(1.0, 0.0, 0.0),
            core.Vector3D.create(0.0, 1.0, 0.0),
            11.0,
            5.56,
            0.8,
        )
    )
    sub_wall = tb.createBox(
        core.OrientedBoundingBox3D.create(
            core.Point3D.create(8.5, side * 12.89, 8.2),
            core.Vector3D.create(1.0, 0.0, 0.0),
            core.Vector3D.create(0.0, 1.0, 0.0),
            11.0,
            0.58,
            3.6,
        )
    )
    tb.booleanOperation(sub_shelf, sub_wall, fusion.BooleanTypes.UnionBooleanType)

    # Dual Upper Clamping Rings + Full-Width Lateral Mounting Flanges at X = +55.0 mm and X = +100.0 mm
    for cx_cm in (5.5, 10.0):
        ring_out = tb.createCylinderOrCone(
            core.Point3D.create(cx_cm - 0.6, side * 10.3, 9.6),
            3.15,
            core.Point3D.create(cx_cm + 0.6, side * 10.3, 9.6),
            3.15,
        )
        lat_flange = tb.createBox(
            core.OrientedBoundingBox3D.create(
                core.Point3D.create(cx_cm, side * 10.4, 9.6),
                core.Vector3D.create(1.0, 0.0, 0.0),
                core.Vector3D.create(0.0, 1.0, 0.0),
                1.2,
                5.56,
                1.2,
            )
        )
        tb.booleanOperation(ring_out, lat_flange, fusion.BooleanTypes.UnionBooleanType)
        tb.booleanOperation(sub_shelf, ring_out, fusion.BooleanTypes.UnionBooleanType)

    mot_bore = tb.createCylinderOrCone(
        core.Point3D.create(1.0, side * 10.3, 9.6),
        2.66,
        core.Point3D.create(15.0, side * 10.3, 9.6),
        2.66,
    )
    tb.booleanOperation(sub_shelf, mot_bore, fusion.BooleanTypes.DifferenceBooleanType)
    if side < 0:
        s2_top_cut = tb.createBox(
            core.OrientedBoundingBox3D.create(
                core.Point3D.create(8.7, -9.0, 6.8),
                core.Vector3D.create(1.0, 0.0, 0.0),
                core.Vector3D.create(0.0, 1.0, 0.0),
                1.8,
                3.8,
                1.6,
            )
        )
        tb.booleanOperation(sub_shelf, s2_top_cut, fusion.BooleanTypes.DifferenceBooleanType)

    b_sub = s2_hw_comp.bRepBodies.add(sub_shelf, base_feat)
    b_sub.name = f"inboard_motor_subframe_rex_axle_{mname}"

# S2 Servo Mounting Bracket bolting Servo S2 (X=+96, Y=-96, Z=50) solidly to the inner wall of right_rail_17h (Y=-132, Z=48)
# with a boolean-subtracted pocket for Midcase, Botcase, Wire Seal & Screws!
s2_brkt = tb.createBox(
    core.OrientedBoundingBox3D.create(
        core.Point3D.create(9.6, -10.9, 3.7),
        core.Vector3D.create(1.0, 0.0, 0.0),
        core.Vector3D.create(0.0, 1.0, 0.0),
        5.6,
        4.58,
        2.58,
    )
)
s2_pocket = tb.createBox(
    core.OrientedBoundingBox3D.create(
        core.Point3D.create(9.3, -9.6, 3.7),
        core.Vector3D.create(1.0, 0.0, 0.0),
        core.Vector3D.create(0.0, 1.0, 0.0),
        4.6,
        2.1,
        3.0,
    )
)
tb.booleanOperation(s2_brkt, s2_pocket, fusion.BooleanTypes.DifferenceBooleanType)
s2_hw_comp.bRepBodies.add(s2_brkt, base_feat).name = "servo_s2_rail_mount_bracket"

# D) Continuous 96mm-Inner-Width Ball Chute Entry Ramp (+18.0° from X=+172, Z=36 to X=-27.72, Z=100.89)
#    with low 5mm side curbs so the balls and tangent contact are 100% visible in side cutaway views!
ramp_len_cm = 21.0
ramp_w_cm = 10.4  # 104mm outer width, 96mm clear inner width (clears 91.95mm Nectar + 71.12mm Pollen!)
floor_box = tb.createBox(
    core.OrientedBoundingBox3D.create(
        core.Point3D.create(0.0, 0.0, 0.0),
        core.Vector3D.create(1.0, 0.0, 0.0),
        core.Vector3D.create(0.0, 1.0, 0.0),
        ramp_len_cm,
        ramp_w_cm,
        0.4,
    )
)
for side in (1.0, -1.0):
    wall_box = tb.createBox(
        core.OrientedBoundingBox3D.create(
            core.Point3D.create(0.0, side * 5.0, 0.45),
            core.Vector3D.create(1.0, 0.0, 0.0),
            core.Vector3D.create(0.0, 1.0, 0.0),
            ramp_len_cm,
            0.4,
            0.5,
        )
    )
    tb.booleanOperation(floor_box, wall_box, fusion.BooleanTypes.UnionBooleanType)

ramp_mat = mat_mul(mat_pos_mm(71.52, 0.0, 66.54), mat_rot_deg((0, 1, 0), 18.0))
tb.transform(floor_box, ramp_mat)
b_ramp = s2_hw_comp.bRepBodies.add(floor_box, base_feat)
b_ramp.name = "continuous_ball_chute_ramp"

# E) Flush Brushed-Aluminum Side-Wing Gussets bolting the Ball Chute Ramp directly to the Left & Right Side Rails (X = +160 mm, Z = 40 mm, clear of X=144 wheel bearings!)
for side in (1.0, -1.0):
    wing = tb.createBox(
        core.OrientedBoundingBox3D.create(
            core.Point3D.create(16.0, side * 9.19, 4.0),
            core.Vector3D.create(1.0, 0.0, 0.0),
            core.Vector3D.create(0.0, 1.0, 0.0),
            1.4,
            7.98,
            0.6,
        )
    )
    tb.booleanOperation(wing, tb.copy(floor_box), fusion.BooleanTypes.DifferenceBooleanType)
    b_wing = s2_hw_comp.bRepBodies.add(wing, base_feat)
    b_wing.name = f"chute_side_rail_wing_rex_axle_{int(side)}"

    # Vertical 7mm Bright-Steel Guide Rods connecting g407_lip_gate_bridge ears (X = +190, Y = ±120, Z = 168.05) up to Z = 191.95 (flush with Z=192 front arch!)
    g1 = core.Point3D.create(19.0, side * 12.0, 16.805)
    g2 = core.Point3D.create(19.0, side * 12.0, 19.195)
    g_rod = tb.createCylinderOrCone(g1, 0.35, g2, 0.35)
    b_grod = s2_hw_comp.bRepBodies.add(g_rod, base_feat)
    b_grod.name = f"lip_gate_rex_axle_guide_rod_{int(side)}"

# S1 Servo Horn + Linkage Bar connecting Servo S1 spline (X = +192, Y = +130.8, Z = 168) directly to g407_lip_gate_bridge left ear (Y = +126.05)!
s1_link = tb.createCylinderOrCone(
    core.Point3D.create(19.2, 12.605, 16.4),
    0.30,
    core.Point3D.create(19.2, 13.08, 16.4),
    0.30,
)
b_s1l = s2_hw_comp.bRepBodies.add(s1_link, base_feat)
b_s1l.name = "servo_s1_gate_pushrod_rex_axle"

# S2 Servo Horn + Pushrod connecting Servo S2 H25T Spline top (X = +86, Y = -96.0, Z = 69.5) via an outboard clevis arm to flower_depollinator_fork (X = +114, Y = -41.05, Z = 25.0)
s2_horn = tb.createCylinderOrCone(
    core.Point3D.create(8.6, -9.6, 6.95),
    0.22,
    core.Point3D.create(8.6, -7.4, 6.95),
    0.22,
)
s2_link = tb.createCylinderOrCone(
    core.Point3D.create(8.6, -7.4, 6.95),
    0.22,
    core.Point3D.create(11.4, -4.105, 2.5),
    0.22,
)
tb.booleanOperation(s2_link, s2_horn, fusion.BooleanTypes.UnionBooleanType)
b_s2l = s2_hw_comp.bRepBodies.add(s2_link, base_feat)
b_s2l.name = "servo_s2_depol_pushrod_rex_axle"

# F) 2x REV Color Sensor V3 Modules inside the G407 Front-Lip Gate sensor pods (X = +216.0, Y = ±55.0, Z = 152.0)
for side, slbl in ((1.0, "left"), (-1.0, "right")):
    pcb = tb.createBox(
        core.OrientedBoundingBox3D.create(
            core.Point3D.create(21.6, side * 5.5, 15.2),
            core.Vector3D.create(1.0, 0.0, 0.0),
            core.Vector3D.create(0.0, 1.0, 0.0),
            1.4,
            1.8,
            0.8,
        )
    )
    b_pcb = s2_hw_comp.bRepBodies.add(pcb, base_feat)
    b_pcb.name = f"rev_color_sensor_v3_{slbl}"

base_feat.finishEdit()

for s1_k, s1_m in saved_s1_transforms.items():
    s1_o = STAGE1_OCCS.get(s1_k)
    if s1_o and s1_o.isValid:
        s1_o.isGrounded = False
        s1_o.transform2 = s1_m
        s1_o.isLightBulbOn = True

# Re-apply ALL Stage 2 STEP occurrence transforms AFTER addNewComponent/baseFeatures so ParametricDesignType doesn't roll back pending transforms!
place_s2_part(
    "quad_block",
    mat_mul(mat_pos_mm(210.0, 156.0, 48.0), QB_FACE_PLUS_X),
    "qb_front_bumper_left",
)
place_s2_part(
    "quad_block",
    mat_mul(mat_pos_mm(210.0, -156.0, 48.0), QB_FACE_PLUS_X),
    "qb_front_bumper_right",
)
place_s2_part("bumper_l", mat_pos_mm(216.0, 156.0, 48.0), "front_funnel_bumper_left")
place_s2_part("bumper_r", mat_pos_mm(216.0, -156.0, 48.0), "front_funnel_bumper_right")
for idx, ry in enumerate((-62.0, 0.0, 62.0), start=1):
    place_s2_part(
        "star_sleeve",
        mat_mul(mat_pos_mm(203.0, ry, 104.0), mat_rot_deg((0, 1, 0), idx * 20.0), STAR_ALONG_Y),
        f"star_roller_sleeve_{idx}",
    )
place_s2_part(
    "quad_block",
    mat_mul(mat_pos_mm(132.0, 103.0, 96.0), QB_FACE_MINUS_X),
    "qb_m7_intake_mount",
)
place_s2_part(
    "motor_5203",
    mat_mul(mat_pos_mm(126.0, 103.0, 96.0), MOT_FACE_PLUS_X),
    "motor_5203_m7_intake",
)
place_s2_part(
    "lip_gate",
    mat_pos_mm(202.0, 0.0, 148.0),
    "g407_lip_gate_bridge",
)
# Nest Servo S1 FLUSH inside front_upright_4h_left open cavity (Y = +148.0 mm, Z = 158.0 mm -> H25T spline at Z = 168.0 mm Hole 4!)
place_s2_part(
    "servo_2000",
    mat_mul(mat_pos_mm(192.0, 148.0, 158.0), SERVO_S1_IN_UPRIGHT),
    "servo_s1_lip_gate",
)
place_s2_part(
    "depol_fork",
    mat_pos_mm(148.0, 0.0, 20.0),
    "flower_depollinator_fork",
)
place_s2_part(
    "servo_2000",
    mat_pos_mm(96.0, -96.0, 50.0),
    "servo_s2_depollinator",
)
place_s2_part(
    "sign_149",
    mat_mul(mat_pos_mm(84.0, 181.65, 140.0), mat_rot_deg((0, 0, 1), 180.0)),
    "robot_sign_left_149",
)
place_s2_part(
    "sign_149",
    mat_pos_mm(84.0, -181.65, 140.0),
    "robot_sign_right_149",
)

# ==============================================================================
# 7. APPLY HIGH-CONTRAST FUSION 360 APPEARANCES TO STAGE 2 PARTS
# ==============================================================================
def get_or_make_color(name: str, r: int, g: int, b: int):
    existing_app = design.appearances.itemByName(name)
    if existing_app:
        return existing_app
    base_app = design.appearances.item(0)
    new_app = design.appearances.addByCopy(base_app, name)
    for p_i in range(new_app.appearanceProperties.count):
        prop = new_app.appearanceProperties.item(p_i)
        if isinstance(prop, core.ColorProperty) and prop.isReadOnly is False:
            try:
                prop.value = core.Color.create(r, g, b, 255)
            except Exception:
                pass
    return new_app


app_emerald = get_or_make_color("Hermes_Emerald_Green", 16, 185, 129)
app_teal = get_or_make_color("Hermes_StarRoller_Teal", 52, 211, 153)
app_rose = get_or_make_color("Hermes_LipGate_Rose", 251, 113, 133)
app_amber = get_or_make_color("Hermes_Chute_Amber", 251, 191, 36)
app_red = get_or_make_color("Hermes_Sign149_Red", 220, 38, 38)
app_white = get_or_make_color("Hermes_Sign149_White", 248, 250, 252)
app_blue = get_or_make_color("Hermes_Sensor_Blue", 37, 99, 235)
app_brass = get_or_make_color("Hermes_Miter_Brass", 218, 165, 32)
app_steel = get_or_make_color("Hermes_Bright_Steel", 226, 232, 240)
app_bearing = get_or_make_color("Hermes_Dark_Bearing", 51, 65, 85)


def color_component(comp: fusion.Component, app_color: core.Appearance):
    for b_i in range(comp.bRepBodies.count):
        body = comp.bRepBodies.item(b_i)
        body.appearance = app_color
        for f_i in range(body.faces.count):
            body.faces.item(f_i).appearance = app_color


color_component(STATE["lib"]["bumper_l"], app_emerald)
color_component(STATE["lib"]["bumper_r"], app_emerald)
color_component(STATE["lib"]["star_sleeve"], app_teal)
color_component(STATE["lib"]["lip_gate"], app_rose)
color_component(STATE["lib"]["depol_fork"], app_emerald)

# Color Team 149 sign background red and raised numerals white
sign_comp = STATE["lib"]["sign_149"]
for b_i in range(sign_comp.bRepBodies.count):
    b_s = sign_comp.bRepBodies.item(b_i)
    b_s.appearance = app_red if b_i == 0 else app_white
for o_i in range(sign_comp.occurrences.count):
    ch_s = sign_comp.occurrences.item(o_i)
    color_component(ch_s.component, app_red if o_i == 0 else app_white)

# Color Stage 2 Hardware bodies
for b_i in range(s2_hw_comp.bRepBodies.count):
    body = s2_hw_comp.bRepBodies.item(b_i)
    if "miter_gear" in body.name:
        body.appearance = app_brass
    elif "rex_axle" in body.name:
        body.appearance = app_steel
    elif "flanged_bearing" in body.name or "mount" in body.name:
        body.appearance = app_bearing
    elif "chute_ramp" in body.name:
        body.appearance = app_amber
    elif "color_sensor" in body.name:
        body.appearance = app_blue

if design.snapshots.hasPendingSnapshot:
    design.snapshots.add()

root.isJointsFolderLightBulbOn = False

result = {
    "stage1_count": len(STAGE1_OCCS),
    "stage2_count": len(STAGE2_OCCS),
}
