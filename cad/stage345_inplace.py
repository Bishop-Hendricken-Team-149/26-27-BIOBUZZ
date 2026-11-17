"""
Fast in-place Stage 3, 4 & 5 Builder for Autodesk Fusion 360 (via HermesFusionBridge):
Completes the full FTC Team 149 BIOBUZZ 'Project HIVE-QUEEN' Robot Assembly in Fusion 360:
  - Stage 3: Single-File Sprung Magazine Shrouds + 4 Staged Balls + 2x 72mm Gecko Feeder Wheels + Lengthwise M8 Motor + S3 Stopper
  - Stage 4: 270° Rear-Centered Turret + CNC Deck Plate + Slewing Ring + M5/M6 6000 RPM Dual-Wheel Topspin Launcher
             + Bi-Positional Cam Gap Arms (S5) + Hinged Pitch Hood (S6) + Limelight 3A Smart Camera
  - Stage 5: 140g Carbon-Fiber 'Stinger' Endgame Flower Capper Arm (S7 + Quad Block Pivot + Nectar Drop Cup)
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
STAGE345_OCCS = STATE.setdefault("stage345_occs", {})

saved_s1_transforms = {
    k: o.transform2.copy()
    for k, o in STAGE1_OCCS.items()
    if k != "drivetrain_hardware" and o and o.isValid
}

# IMPORTANT: Delete/recreate custom B-Rep hardware components FIRST before setting transform2 on STEP occurrences,
# because deleteMe() / addNewComponent() in ParametricDesignType rolls back uncommitted pending transforms!
for hw_key in ("stage3_hardware", "stage4_turret_hardware", "stage5_stinger_hardware", "magazine_shroud_left", "magazine_shroud_right", "qb_stinger_pivot_mount", "servo_s7_stinger_lift", "stinger_nectar_drop_cup"):
    if hw_key in STAGE345_OCCS and STAGE345_OCCS[hw_key] and STAGE345_OCCS[hw_key].isValid:
        STAGE345_OCCS[hw_key].deleteMe()
        del STAGE345_OCCS[hw_key]


def place_s345_part(lib_key: str, world_mat: core.Matrix3D, label: str) -> fusion.Occurrence:
    existing = STAGE345_OCCS.get(label)
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
    STAGE345_OCCS[label] = occ
    return occ


tb = fusion.TemporaryBRepManager.get()


def make_gobilda_wheel_brep(
    cx_mm: float,
    cy_mm: float,
    cz_mm: float,
    outer_r_mm: float,
    width_mm: float,
    hub_r_mm: float,
) -> tuple[core.BRepBody, core.BRepBody]:
    """Creates a detailed goBILDA Gecko / Rhino compliant wheel (tire + recessed hub with 8x pattern holes) with axis along Y."""
    y1 = (cy_mm - width_mm / 2.0) / 10.0
    y2 = (cy_mm + width_mm / 2.0) / 10.0
    p1 = core.Point3D.create(cx_mm / 10.0, y1, cz_mm / 10.0)
    p2 = core.Point3D.create(cx_mm / 10.0, y2, cz_mm / 10.0)
    tire = tb.createCylinderOrCone(p1, outer_r_mm / 10.0, p2, outer_r_mm / 10.0)
    inner_cut = tb.createCylinderOrCone(
        core.Point3D.create(cx_mm / 10.0, y1 - 0.1, cz_mm / 10.0),
        hub_r_mm / 10.0,
        core.Point3D.create(cx_mm / 10.0, y2 + 0.1, cz_mm / 10.0),
        hub_r_mm / 10.0,
    )
    tb.booleanOperation(tire, inner_cut, fusion.BooleanTypes.DifferenceBooleanType)

    # 12 compliant outer tread slots
    slot_r = (outer_r_mm + hub_r_mm) / 2.0
    hole_rad = (outer_r_mm - hub_r_mm) * 0.28 / 10.0
    for i in range(12):
        ang = math.radians(i * 30.0)
        sx = (cx_mm + slot_r * math.cos(ang)) / 10.0
        sz = (cz_mm + slot_r * math.sin(ang)) / 10.0
        slot_cyl = tb.createCylinderOrCone(
            core.Point3D.create(sx, y1 - 0.1, sz),
            hole_rad,
            core.Point3D.create(sx, y2 + 0.1, sz),
            hole_rad,
        )
        tb.booleanOperation(tire, slot_cyl, fusion.BooleanTypes.DifferenceBooleanType)

    # Aluminum hub core with 8mm bore (exact line-to-line mate at hub_r_mm so 0.0 mm3 tire-hub overlap!)
    hy1 = (cy_mm - width_mm * 0.35) / 10.0
    hy2 = (cy_mm + width_mm * 0.35) / 10.0
    hub = tb.createCylinderOrCone(
        core.Point3D.create(cx_mm / 10.0, hy1, cz_mm / 10.0),
        (hub_r_mm - 0.05) / 10.0,
        core.Point3D.create(cx_mm / 10.0, hy2, cz_mm / 10.0),
        (hub_r_mm - 0.05) / 10.0,
    )
    bore_cut = tb.createCylinderOrCone(
        core.Point3D.create(cx_mm / 10.0, hy1 - 0.1, cz_mm / 10.0),
        0.41,
        core.Point3D.create(cx_mm / 10.0, hy2 + 0.1, cz_mm / 10.0),
        0.41,
    )
    tb.booleanOperation(hub, bore_cut, fusion.BooleanTypes.DifferenceBooleanType)
    return tire, hub


def make_pulley_and_belt_brep(
    x1_mm: float,
    z1_mm: float,
    x2_mm: float,
    z2_mm: float,
    y_mm: float,
    r_pulley_mm: float,
    width_mm: float,
) -> tuple[core.BRepBody, core.BRepBody, core.BRepBody]:
    """Creates two flanged HTD-5M aluminum timing pulleys at (x1, y, z1) & (x2, y, z2) + closed rubber timing belt loop."""
    y_cm = y_mm / 10.0
    w_cm = width_mm / 10.0
    rp_cm = r_pulley_mm / 10.0

    pulleys = []
    for px_mm, pz_mm in ((x1_mm, z1_mm), (x2_mm, z2_mm)):
        px, pz = px_mm / 10.0, pz_mm / 10.0
        p_body = tb.createCylinderOrCone(
            core.Point3D.create(px, y_cm - w_cm / 2.0, pz),
            rp_cm,
            core.Point3D.create(px, y_cm + w_cm / 2.0, pz),
            rp_cm,
        )
        fl1 = tb.createCylinderOrCone(
            core.Point3D.create(px, y_cm - w_cm / 2.0, pz),
            rp_cm + 0.22,
            core.Point3D.create(px, y_cm - w_cm / 2.0 + 0.15, pz),
            rp_cm + 0.22,
        )
        fl2 = tb.createCylinderOrCone(
            core.Point3D.create(px, y_cm + w_cm / 2.0 - 0.15, pz),
            rp_cm + 0.22,
            core.Point3D.create(px, y_cm + w_cm / 2.0, pz),
            rp_cm + 0.22,
        )
        tb.booleanOperation(p_body, fl1, fusion.BooleanTypes.UnionBooleanType)
        tb.booleanOperation(p_body, fl2, fusion.BooleanTypes.UnionBooleanType)
        bore = tb.createCylinderOrCone(
            core.Point3D.create(px, y_cm - w_cm, pz),
            0.41,
            core.Point3D.create(px, y_cm + w_cm, pz),
            0.41,
        )
        tb.booleanOperation(p_body, bore, fusion.BooleanTypes.DifferenceBooleanType)
        pulleys.append(p_body)

    dx_cm = (x2_mm - x1_mm) / 10.0
    dz_cm = (z2_mm - z1_mm) / 10.0
    dist_cm = math.hypot(dx_cm, dz_cm)
    ux, uz = dx_cm / dist_cm, dz_cm / dist_cm
    cx_cm = (x1_mm + x2_mm) / 20.0
    cz_cm = (z1_mm + z2_mm) / 20.0

    r_out = rp_cm + 0.16
    r_in = rp_cm + 0.01
    bw = w_cm - 0.34

    c1_out = tb.createCylinderOrCone(
        core.Point3D.create(x1_mm / 10.0, y_cm - bw / 2.0, z1_mm / 10.0),
        r_out,
        core.Point3D.create(x1_mm / 10.0, y_cm + bw / 2.0, z1_mm / 10.0),
        r_out,
    )
    c2_out = tb.createCylinderOrCone(
        core.Point3D.create(x2_mm / 10.0, y_cm - bw / 2.0, z2_mm / 10.0),
        r_out,
        core.Point3D.create(x2_mm / 10.0, y_cm + bw / 2.0, z2_mm / 10.0),
        r_out,
    )
    span_out = tb.createBox(
        core.OrientedBoundingBox3D.create(
            core.Point3D.create(cx_cm, y_cm, cz_cm),
            core.Vector3D.create(ux, 0.0, uz),
            core.Vector3D.create(0.0, 1.0, 0.0),
            dist_cm,
            bw,
            2.0 * r_out,
        )
    )
    tb.booleanOperation(c1_out, c2_out, fusion.BooleanTypes.UnionBooleanType)
    tb.booleanOperation(c1_out, span_out, fusion.BooleanTypes.UnionBooleanType)

    c1_in = tb.createCylinderOrCone(
        core.Point3D.create(x1_mm / 10.0, y_cm - bw, z1_mm / 10.0),
        r_in,
        core.Point3D.create(x1_mm / 10.0, y_cm + bw, z1_mm / 10.0),
        r_in,
    )
    c2_in = tb.createCylinderOrCone(
        core.Point3D.create(x2_mm / 10.0, y_cm - bw, z2_mm / 10.0),
        r_in,
        core.Point3D.create(x2_mm / 10.0, y_cm + bw, z2_mm / 10.0),
        r_in,
    )
    span_in = tb.createBox(
        core.OrientedBoundingBox3D.create(
            core.Point3D.create(cx_cm, y_cm, cz_cm),
            core.Vector3D.create(ux, 0.0, uz),
            core.Vector3D.create(0.0, 1.0, 0.0),
            dist_cm,
            2.0 * bw,
            2.0 * r_in,
        )
    )
    tb.booleanOperation(c1_in, c2_in, fusion.BooleanTypes.UnionBooleanType)
    tb.booleanOperation(c1_in, span_in, fusion.BooleanTypes.UnionBooleanType)
    tb.booleanOperation(c1_out, c1_in, fusion.BooleanTypes.DifferenceBooleanType)
    return pulleys[0], pulleys[1], c1_out


# ==============================================================================
# BUILD STAGE 3 B-REP HARDWARE COMPONENT:
# - 4x Staged BIOBUZZ Balls (3x 71.1mm Yellow Pollen + 1x 71.1mm/91.9mm staged along exact +21° ramp tangent!)
# - 2x 72mm goBILDA Gecko Compliant Feeder Wheels + 8mm REX Shafts + 4x Flanged Bearings + HTD-5M Belt Loop
# - 2x Brass 90° Miter Bevel Gears inside front_upright_4h_right (X=+192, Y=-156, Z=96) + Feeder Drive Shaft
# - 4x Aluminum Structural L-Gussets bolting Left & Right Magazine Shrouds to Mid Tower Stanchions (X=0, Y=±132)
# - Steep Rear Elevator Trough rising from (X=-68, Z=122) up through the Turret Feed Throat (X=-84, Z=230)!
# ==============================================================================
s3_occ = root.occurrences.addNewComponent(core.Matrix3D.create())
s3_comp = s3_occ.component
s3_comp.name = "Stage3_Sprung_Magazine_Hardware"
STAGE345_OCCS["stage3_hardware"] = s3_occ

s3_feat = s3_comp.features.baseFeatures.add()
s3_feat.startEdit()

# 1) 4x Staged BIOBUZZ Balls along exact +21° ramp normal & vertical throat (100% tangent, ZERO clipping with star rollers, Gecko wheels, or flywheels!):
#    - Ball 1 (s=+60mm on ramp): (X=123.65, Y=0.0, Z=90.03)
#    - Ball 2 (s=-16mm on ramp): (X=52.70, Y=0.0, Z=117.26)
#    - Ball 3 (s=-92mm at rear elevator curve): (X=-24.0, Y=0.0, Z=136.0)
#    - Ball 4 (rising vertically at X=-96.0mm through Turret Throat into the 54.3mm Nip Channel between Main 96mm Wheel and Counter Wheel!):
#      (X=-96.0, Y=0.0, Z=232.0) -> dist to Main Wheel (-22,282,R=48) is 89.3mm > 83.56mm; dist to Throat Kicker (-32,194,R=26) is 74.4mm > 61.56mm!
# 1) 4x Staged BIOBUZZ Balls (3 held stationary in the Magazine behind the Servo S3 Stopper Gate + 1 in the Shooter Nip!):
#    - Ball 1 (bottom of magazine): (X=123.65, Y=0.0, Z=90.03) — held between gecko_feeder_1 (136.3, 156.4, R=30) & +21° ramp
#    - Ball 2 (middle of magazine): (X=52.70, Y=0.0, Z=117.26) — held between gecko_feeder_2 (69.1, 182.2, R=30) & +21° ramp
#    - Ball 3 (top of magazine, held by Servo S3 Stopper Gate before firing!): (X=-16.00, Y=0.0, Z=143.65)
#    - Ball 4 (fired through Gecko 3 -> Main 96mm Wheel (-10,284,R=48) & Counter 48mm Wheel (-122,360,R=24) 63.4mm Nip!):
#      (X=-75.90, Y=0.0, Z=327.60)
# 1) 4-Stage Spring-Loaded Dual-Ball Feeder Chain (handles BOTH 71.12mm Pollen AND 91.95mm Nectar balls!):
#    - Nominal tire-to-ramp gap is 64.0 mm (giving +7.1 mm active compliant squeeze on 71.12 mm Pollen balls!).
#    - Each Gecko shaft rides in a 25 mm angled spring-loaded guide slot in the Purple Shrouds so a 91.95 mm Nectar ball
#      lifts the shaft +21 mm to an 85.0 mm tire gap (99.0 mm solid aluminum hub clearance -> ZERO hub jam!).
#    - Center-to-center roller handoff spans:
#        Intake (192,96) -> Gecko 1 (141.8,143.0): 68.7 mm (< 71.12 mm!)
#        Gecko 1 -> Gecko 2 (85.8,164.5):          60.0 mm (< 71.12 mm!)
#        Gecko 2 -> Gecko 3 (31.0,183.4):          58.0 mm (< 71.12 mm!)
#        Gecko 3 -> Gecko 4 Kicker (-14.0,206.0):  50.3 mm (< 71.12 mm!)
#        Gecko 4 Kicker -> Main 96mm (-6.0,286.5): 80.9 mm (with 50mm simultaneous 2-wheel bite zone against the sprung -14° rear baffle!)
gecko_coords = [
    # (gx_cur, gz_cur, gx_nom, gz_nom, gr, ghr, glbl)
    # Note: gecko_feeder_1 is shown deflected upward in its spring slot above staged_nectar_ball_1_orange (91.95mm)!
    (134.00, 175.50, 126.50, 154.90, 25.0, 12.5, "gecko_feeder_1"),
    (82.00, 174.50, 82.00, 174.50, 24.0, 12.0, "gecko_feeder_2"),
    (32.50, 184.80, 32.50, 184.80, 23.0, 11.5, "gecko_feeder_3"),
    (-11.00, 200.50, -11.00, 200.50, 20.0, 10.5, "gecko_feeder_4_kicker"),
]

# 1b) Precision CNC/3D-Printed Purple Magazine Side Shroud Plates (|Y| in [50.0, 55.0 mm], tilted +21° at X=+52.0, Z=154.0)
#     with 25 mm angled spring-travel guide slots at every Gecko feeder stage!
s21_sh, c21_sh = math.sin(math.radians(21.0)), math.cos(math.radians(21.0))
for side, sname in ((1.0, "left"), (-1.0, "right")):
    shroud_box = tb.createBox(
        core.OrientedBoundingBox3D.create(
            core.Point3D.create(0.0, 0.0, 0.0),
            core.Vector3D.create(1.0, 0.0, 0.0),
            core.Vector3D.create(0.0, 1.0, 0.0),
            18.8,
            0.5,
            9.2,
        )
    )
    tb.transform(shroud_box, mat_mul(mat_pos_mm(50.0, side * 52.5, 154.0), mat_rot_deg((0, 1, 0), 21.0)))
    for gx, gz, gx_nom, gz_nom, _, _, _ in gecko_coords:
        # Cut angled 25mm spring-travel guide slot from nominal (64mm Pollen gap) up to +22mm (86mm Nectar gap)
        gx_top = gx_nom + 22.0 * s21_sh
        gz_top = gz_nom + 22.0 * c21_sh
        for sx_p, sz_p in ((gx_nom, gz_nom), (gx, gz), (gx_top, gz_top)):
            sh_hole = tb.createCylinderOrCone(
                core.Point3D.create(sx_p / 10.0, -8.0, sz_p / 10.0),
                0.44,
                core.Point3D.create(sx_p / 10.0, 8.0, sz_p / 10.0),
                0.44,
            )
            tb.booleanOperation(shroud_box, sh_hole, fusion.BooleanTypes.DifferenceBooleanType)
        slot_mid_x = (gx_nom + gx_top) / 20.0
        slot_mid_z = (gz_nom + gz_top) / 20.0
        slot_box = tb.createBox(
            core.OrientedBoundingBox3D.create(
                core.Point3D.create(slot_mid_x, side * 5.25, slot_mid_z),
                core.Vector3D.create(s21_sh, 0.0, c21_sh),
                core.Vector3D.create(0.0, 1.0, 0.0),
                2.2,
                1.2,
                0.88,
            )
        )
        tb.booleanOperation(shroud_box, slot_box, fusion.BooleanTypes.DifferenceBooleanType)
    s3_hole = tb.createCylinderOrCone(
        core.Point3D.create(-1.6, -8.0, 16.8),
        0.38,
        core.Point3D.create(-1.6, 8.0, 16.8),
        0.38,
    )
    tb.booleanOperation(shroud_box, s3_hole, fusion.BooleanTypes.DifferenceBooleanType)
    b_sh = s3_comp.bRepBodies.add(shroud_box, s3_feat)
    b_sh.name = f"magazine_shroud_purple_{sname}"

# 2) 4x Sprung Magazine Gecko Feeder & Throat Kicker Shafts + Sliding Spring-Loaded Bearing Blocks:
gecko_tire_copies = []
for gx, gz, gx_nom, gz_nom, gr, ghr, glbl in gecko_coords:
    for gy, gside in ((-20.0, "R"), (20.0, "L")):
        tire, hub = make_gobilda_wheel_brep(gx, gy, gz, gr, 24.0, ghr)
        gecko_tire_copies.append(tb.copy(tire))
        b_t = s3_comp.bRepBodies.add(tire, s3_feat)
        b_t.name = f"{glbl}_{gside}_tire_green"
        b_h = s3_comp.bRepBodies.add(hub, s3_feat)
        b_h.name = f"{glbl}_{gside}_hub_silver"

    # 8mm REX Shaft spanning through Left & Right Magazine Shrouds out to the Right Pulley Plane (Y in [-72, +58 mm])
    ax = tb.createCylinderOrCone(
        core.Point3D.create(gx / 10.0, -7.2, gz / 10.0),
        0.4,
        core.Point3D.create(gx / 10.0, 5.8, gz / 10.0),
        0.4,
    )
    b_ax = s3_comp.bRepBodies.add(ax, s3_feat)
    b_ax.name = f"{glbl}_rex_axle"

    # 2x Sliding Spring-Loaded Bearing Blocks + Return Spring Posts on outer faces of Left & Right Magazine Shrouds (|Y| in [55.05, 57.5 mm])
    for side in (1.0, -1.0):
        brg = tb.createCylinderOrCone(
            core.Point3D.create(gx / 10.0, side * 5.505, gz / 10.0),
            0.85,
            core.Point3D.create(gx / 10.0, side * 5.75, gz / 10.0),
            0.85,
        )
        tb.booleanOperation(brg, tb.copy(ax), fusion.BooleanTypes.DifferenceBooleanType)
        b_brg = s3_comp.bRepBodies.add(brg, s3_feat)
        b_brg.name = f"{glbl}_flanged_bearing_{int(side)}"

# Servo S3 Indexing Stopper Gate Bar + Pivot Shaft at (X = -16.0, Z = 168.0, Y in [-55.0, +98.0 mm]):
s3_gate_shaft = tb.createCylinderOrCone(
    core.Point3D.create(-1.6, -5.5, 16.8),
    0.35,
    core.Point3D.create(-1.6, 9.8, 16.8),
    0.35,
)
s3_comp.bRepBodies.add(s3_gate_shaft, s3_feat).name = "s3_stopper_gate_rex_axle"
s3_gate_bar = tb.createBox(
    core.OrientedBoundingBox3D.create(
        core.Point3D.create(-1.6, 0.0, 16.2),
        core.Vector3D.create(1.0, 0.0, 0.0),
        core.Vector3D.create(0.0, 1.0, 0.0),
        0.6,
        7.6,
        0.8,
    )
)
tb.booleanOperation(s3_gate_bar, tb.copy(s3_gate_shaft), fusion.BooleanTypes.DifferenceBooleanType)
s3_comp.bRepBodies.add(s3_gate_bar, s3_feat).name = "s3_stopper_gate_counter_tire_rose"

# Continuous 4-Stage HTD-5M Timing Belt Chain alternating at Y = -68.5 mm and Y = -61.0 mm (1.5mm clear flange gap everywhere!):
pm8_g, p1b_g, belt_m8 = make_pulley_and_belt_brep(144.0, 96.0, 134.00, 175.50, -68.5, 10.0, 6.0)
s3_comp.bRepBodies.add(pm8_g, s3_feat).name = "gecko_pulley_m8_drive"
s3_comp.bRepBodies.add(p1b_g, s3_feat).name = "gecko_pulley_1b"
s3_comp.bRepBodies.add(belt_m8, s3_feat).name = "gecko_timing_belt_m8"

p1_g, p2_g, belt_g12 = make_pulley_and_belt_brep(134.00, 175.50, 82.00, 174.50, -61.0, 10.5, 6.0)
s3_comp.bRepBodies.add(p1_g, s3_feat).name = "gecko_pulley_1"
s3_comp.bRepBodies.add(p2_g, s3_feat).name = "gecko_pulley_2"
s3_comp.bRepBodies.add(belt_g12, s3_feat).name = "gecko_timing_belt_12"

p2b_g, p3_g, belt_g23 = make_pulley_and_belt_brep(82.00, 174.50, 32.50, 184.80, -68.5, 10.5, 6.0)
s3_comp.bRepBodies.add(p2b_g, s3_feat).name = "gecko_pulley_2b"
s3_comp.bRepBodies.add(p3_g, s3_feat).name = "gecko_pulley_3"
s3_comp.bRepBodies.add(belt_g23, s3_feat).name = "gecko_timing_belt_23"

p3b_g, p4_g, belt_g34 = make_pulley_and_belt_brep(32.50, 184.80, -11.00, 200.50, -61.0, 10.0, 6.0)
s3_comp.bRepBodies.add(p3b_g, s3_feat).name = "gecko_pulley_3b"
s3_comp.bRepBodies.add(p4_g, s3_feat).name = "gecko_pulley_4"
s3_comp.bRepBodies.add(belt_g34, s3_feat).name = "gecko_timing_belt_34"

# M8 Right-Angle Bevel Gear & Cross-Shaft connecting Motor M8 (X=118..141.6, Y=-103, Z=96) to gecko_pulley_m8_drive (X=144.0, Y=-68.5, Z=96)!
m8_g1 = tb.createCylinderOrCone(
    core.Point3D.create(13.3, -10.3, 9.6),
    1.0,
    core.Point3D.create(13.78, -10.3, 9.6),
    0.55,
)
m8_bore = tb.createCylinderOrCone(
    core.Point3D.create(13.0, -10.3, 9.6),
    0.61,
    core.Point3D.create(14.5, -10.3, 9.6),
    0.61,
)
tb.booleanOperation(m8_g1, m8_bore, fusion.BooleanTypes.DifferenceBooleanType)
s3_comp.bRepBodies.add(m8_g1, s3_feat).name = "miter_gear_m8_driver"
m8_cshaft = tb.createCylinderOrCone(
    core.Point3D.create(14.4, -9.65, 9.6),
    0.4,
    core.Point3D.create(14.4, -6.5, 9.6),
    0.4,
)
m8_g2 = tb.createCylinderOrCone(
    core.Point3D.create(14.4, -9.65, 9.6),
    0.55,
    core.Point3D.create(14.4, -9.15, 9.6),
    1.0,
)
tb.booleanOperation(m8_g2, tb.copy(m8_cshaft), fusion.BooleanTypes.DifferenceBooleanType)
s3_comp.bRepBodies.add(m8_g2, s3_feat).name = "miter_gear_m8_driven"
s3_comp.bRepBodies.add(m8_cshaft, s3_feat).name = "m8_countershaft_rex_axle"

# 3) 4x Bolted Aluminum Shroud Mounting Gussets at Z = 136 & 182 mm (with right-side belt clearance window!)
for side in (1.0, -1.0):
    for gz in (136.0, 182.0):
        y_center = 9.35 if side > 0 else -10.25
        y_span = 7.68 if side > 0 else 5.88
        gus = tb.createBox(
            core.OrientedBoundingBox3D.create(
                core.Point3D.create(0.0, y_center, gz / 10.0),
                core.Vector3D.create(1.0, 0.0, 0.0),
                core.Vector3D.create(0.0, 1.0, 0.0),
                2.0,
                y_span,
                0.6,
            )
        )
        b_gus = s3_comp.bRepBodies.add(gus, s3_feat)
        b_gus.name = f"mag_shroud_gusset_mount_{int(side)}_{int(gz)}"

# 4) True Cylindrical Quarter-Pipe Golden J-Chute Curve (R_in = 122.5 mm, R_out = 126.5 mm centered at X_c = +10.13 mm, Z_c = 217.40 mm)
#    Tangentially continuing the +18.0° entry ramp from (-27.72, 100.89) smoothly through 90° vertical at (-112.37, 215.80)
#    to redirect horizontal intake momentum into pure vertical upward force into the Shooter Flywheel!
j_cyl_out = tb.createCylinderOrCone(
    core.Point3D.create(1.013, -5.2, 21.74),
    12.65,
    core.Point3D.create(1.013, 5.2, 21.74),
    12.65,
)
j_cyl_in = tb.createCylinderOrCone(
    core.Point3D.create(1.013, -5.3, 21.74),
    12.25,
    core.Point3D.create(1.013, 5.3, 21.74),
    12.25,
)
tb.booleanOperation(j_cyl_out, j_cyl_in, fusion.BooleanTypes.DifferenceBooleanType)

# Add low 5mm side curbs at |Y| in [48.0, 52.0 mm] along the cylindrical quarter-pipe
for side in (1.0, -1.0):
    curb_out = tb.createCylinderOrCone(
        core.Point3D.create(1.013, side * 4.8, 21.74),
        12.26,
        core.Point3D.create(1.013, side * 5.2, 21.74),
        12.26,
    )
    curb_in = tb.createCylinderOrCone(
        core.Point3D.create(1.013, side * 4.75, 21.74),
        11.80,
        core.Point3D.create(1.013, side * 5.25, 21.74),
        11.80,
    )
    tb.booleanOperation(curb_out, curb_in, fusion.BooleanTypes.DifferenceBooleanType)
    tb.booleanOperation(j_cyl_out, curb_out, fusion.BooleanTypes.UnionBooleanType)

# Intersect the cylindrical shell with the lower-rear quadrant (X in [-130.0, -26.0 mm], Z in [95.0, 215.8 mm])
# so it butt-joins the +18.0° entry ramp at (-27.72, 100.89) with ZERO gap!
quad_z_box = tb.createBox(
    core.OrientedBoundingBox3D.create(
        core.Point3D.create(-7.8, 0.0, 15.54),
        core.Vector3D.create(1.0, 0.0, 0.0),
        core.Vector3D.create(0.0, 1.0, 0.0),
        10.4,
        11.0,
        12.08,
    )
)
tb.booleanOperation(j_cyl_out, quad_z_box, fusion.BooleanTypes.IntersectionBooleanType)

# Add Through-Bore Golden Chute Neck (Y in [-35.0, +35.0 mm], Z in [214.0, 239.0 mm]) passing cleanly inside the R_in=58mm Turret Slewing Ring
# so the Golden J-Chute is 100% continuous from the front intake (Z=36) all the way into the Upper Turret Throat (Z=286)!
bore_neck = tb.createBox(
    core.OrientedBoundingBox3D.create(
        core.Point3D.create(0.0, 0.0, 0.0),
        core.Vector3D.create(1.0, 0.0, 0.0),
        core.Vector3D.create(0.0, 1.0, 0.0),
        0.4,
        6.6,
        2.8,
    )
)
tb.transform(bore_neck, mat_mul(mat_pos_mm(-120.2, 0.0, 226.5), mat_rot_deg((0, 1, 0), 24.0)))
tb.booleanOperation(j_cyl_out, bore_neck, fusion.BooleanTypes.UnionBooleanType)

ramp_cut_box = tb.createBox(
    core.OrientedBoundingBox3D.create(
        core.Point3D.create(0.0, 0.0, 0.0),
        core.Vector3D.create(1.0, 0.0, 0.0),
        core.Vector3D.create(0.0, 1.0, 0.0),
        21.0,
        10.5,
        0.4,
    )
)
for side in (1.0, -1.0):
    w_cut = tb.createBox(
        core.OrientedBoundingBox3D.create(
            core.Point3D.create(0.0, side * 5.0, 0.45),
            core.Vector3D.create(1.0, 0.0, 0.0),
            core.Vector3D.create(0.0, 1.0, 0.0),
            21.0,
            0.42,
            0.52,
        )
    )
    tb.booleanOperation(ramp_cut_box, w_cut, fusion.BooleanTypes.UnionBooleanType)
tb.transform(ramp_cut_box, mat_mul(mat_pos_mm(71.52, 0.0, 66.54), mat_rot_deg((0, 1, 0), 18.0)))
tb.booleanOperation(j_cyl_out, tb.copy(ramp_cut_box), fusion.BooleanTypes.DifferenceBooleanType)

# Structural Rear Spring-Guide Backing Frame at X = -117.5 mm (clearing S4 at X = -119.8 mm!) supporting the curved J-Chute:
elev_vert = tb.createBox(
    core.OrientedBoundingBox3D.create(
        core.Point3D.create(-11.75, 0.0, 17.5),
        core.Vector3D.create(1.0, 0.0, 0.0),
        core.Vector3D.create(0.0, 1.0, 0.0),
        0.3,
        9.6,
        7.6,
    )
)
for side in (1.0, -1.0):
    for sz_cm, sx_end in ((14.8, -9.8), (18.8, -11.1)):
        sp_rod = tb.createCylinderOrCone(
            core.Point3D.create(-11.75, side * 4.4, sz_cm),
            0.35,
            core.Point3D.create(sx_end, side * 4.4, sz_cm),
            0.35,
        )
        tb.booleanOperation(elev_vert, sp_rod, fusion.BooleanTypes.UnionBooleanType)

tb.booleanOperation(elev_vert, tb.copy(j_cyl_out), fusion.BooleanTypes.DifferenceBooleanType)
s3_comp.bRepBodies.add(j_cyl_out, s3_feat).name = "rear_elevator_chute_curve_gold"
s3_comp.bRepBodies.add(elev_vert, s3_feat).name = "rear_elevator_scoop_mount"

# 5) 4x Staged Uncut BIOBUZZ Balls (1x 91.95mm Orange Nectar Ball + 3x 71.12mm Yellow Pollen Balls)
#    Pure uncut spheres with +1.5mm to +2.9mm 2D/3D tangent clearance to BOTH the Gold J-Chute and the Green Gecko / Shooter Wheels!
ball_centers_mm = [
    (107.73, 0.0, 106.80, 45.975, "staged_nectar_ball_1_orange"),
    (17.02, 0.0, 125.32, 35.56, "staged_pollen_ball_2"),
    (-59.86, 0.0, 168.39, 35.56, "staged_pollen_ball_3"),
    (-89.00, 0.0, 268.00, 35.56, "staged_pollen_ball_4"),
]
for bx, by, bz, br, blbl in ball_centers_mm:
    sp = tb.createSphere(core.Point3D.create(bx / 10.0, by / 10.0, bz / 10.0), br / 10.0)
    b_ball = s3_comp.bRepBodies.add(sp, s3_feat)
    b_ball.name = blbl

s3_feat.finishEdit()

# ==============================================================================
# BUILD STAGE 4 B-REP TURRET & LAUNCHER HARDWARE COMPONENT:
# - 6mm CNC 6061 Aluminum Turret Mounting Deck Plate (X in [-192, +18], Y in [-180, +180], Z in [216, 222])
#   with R=58mm Central Feed Throat Bore at (X = -84.0, Y = 0.0)!
# - 270° Turret Slewing Bearing Ring (R_out=72mm, R_in=58mm, Z in [222, 234]) + S4 Brass Azimuth Pinion Gear!
# - Left & Right CNC Aluminum Turret Side Plates (Y = ±64.0 mm, X in [-148, -20], Z in [234, 354])
#   + Top Structural Bridge (X = -132, Y in [-64, +64], Z in [346, 354])!
# - Left & Right Outboard Motor Stand Plates at Y = ±80.0 mm holding Motors M5 & M6 (with HTD-5M Belts at Y = ±73.0 mm
#   running cleanly between the Side Plates |Y|=64 and Motor Stands |Y|=80 — ZERO bracket or plate clipping!)
# - 96mm Main Rhino Wheel + 2x 82mm Steel Flywheels at (X = -38.0, Z = 308.0) on 8mm REX Shaft + Flanged Bearings
# - 72mm Topspin Counter-Wheel at (X = -126.0, Z = 320.0) on 8mm REX Shaft + Flanged Bearings + Cam Pivot Shaft
# - Hinged Variable-Pitch Exit Guide Hood (Z = 358.0) + Detailed Limelight 3A Smart Camera (with dual lenses & LED bar)!
# ==============================================================================
s4_occ = root.occurrences.addNewComponent(core.Matrix3D.create())
s4_comp = s4_occ.component
s4_comp.name = "Stage4_Turret_Launcher_Hardware"
STAGE345_OCCS["stage4_turret_hardware"] = s4_occ

s4_feat = s4_comp.features.baseFeatures.add()
s4_feat.startEdit()

tx_mm = -84.0
tz_mm = 222.0

# 1) 6mm CNC 6061 Aluminum Turret Mounting Deck Plate (X in [-192, +14], Y in [-180, +180], Z in [216, 222])
#    with R=58mm Central Feed Throat Bore + Forward Magazine Throat Cutout (X in [-84, +18], Y in [-68, +68])!
deck_box = tb.createBox(
    core.OrientedBoundingBox3D.create(
        core.Point3D.create(-8.9, 0.0, 21.9),
        core.Vector3D.create(1.0, 0.0, 0.0),
        core.Vector3D.create(0.0, 1.0, 0.0),
        20.6,
        36.0,
        0.6,
    )
)
deck_bore = tb.createCylinderOrCone(
    core.Point3D.create(tx_mm / 10.0, 0.0, 21.4),
    5.8,
    core.Point3D.create(tx_mm / 10.0, 0.0, 22.4),
    5.8,
)
deck_slot = tb.createBox(
    core.OrientedBoundingBox3D.create(
        core.Point3D.create(-3.3, 0.0, 21.9),
        core.Vector3D.create(1.0, 0.0, 0.0),
        core.Vector3D.create(0.0, 1.0, 0.0),
        10.6,
        13.6,
        1.2,
    )
)
s4_pinion_bore = tb.createCylinderOrCone(
    core.Point3D.create(-13.0, -7.1, 21.4),
    0.5,
    core.Point3D.create(-13.0, -7.1, 22.4),
    0.5,
)
tb.booleanOperation(deck_box, deck_bore, fusion.BooleanTypes.DifferenceBooleanType)
tb.booleanOperation(deck_box, deck_slot, fusion.BooleanTypes.DifferenceBooleanType)
tb.booleanOperation(deck_box, s4_pinion_bore, fusion.BooleanTypes.DifferenceBooleanType)
b_deck = s4_comp.bRepBodies.add(deck_box, s4_feat)
b_deck.name = "cnc_turret_mounting_deck_plate"

# 8x M4 Socket Head Cap Screw Fastener Bosses bolting the Deck Plate into the Rear (X=-168) and Mid (X=0) Tower Bridges!
for bx_mm in (-168.0, 0.0):
    for by_mm in (-168.0, -144.0, 144.0, 168.0):
        bolt = tb.createCylinderOrCone(
            core.Point3D.create(bx_mm / 10.0, by_mm / 10.0, 22.205),
            0.38,
            core.Point3D.create(bx_mm / 10.0, by_mm / 10.0, 22.55),
            0.38,
        )
        s4_comp.bRepBodies.add(bolt, s4_feat).name = f"deck_m4_bolt_rex_axle_{int(bx_mm)}_{int(by_mm)}"

# 2) Turret Slewing Bearing Ring (R_out=72mm, R_in=58mm, Z in [222.05, 230.95]) + Rotating Turret Base Flange Plate (R_out=78mm, Z in [231, 236])
slew_ring = tb.createCylinderOrCone(
    core.Point3D.create(tx_mm / 10.0, 0.0, 22.205),
    7.2,
    core.Point3D.create(tx_mm / 10.0, 0.0, 23.095),
    7.2,
)
slew_bore = tb.createCylinderOrCone(
    core.Point3D.create(tx_mm / 10.0, 0.0, 22.0),
    5.8,
    core.Point3D.create(tx_mm / 10.0, 0.0, 23.3),
    5.8,
)
tb.booleanOperation(slew_ring, slew_bore, fusion.BooleanTypes.DifferenceBooleanType)
s4_comp.bRepBodies.add(slew_ring, s4_feat).name = "turret_slewing_bearing_ring"

rot_base = tb.createCylinderOrCone(
    core.Point3D.create(tx_mm / 10.0, 0.0, 23.1),
    7.8,
    core.Point3D.create(tx_mm / 10.0, 0.0, 23.6),
    7.8,
)
rot_bore = tb.createCylinderOrCone(
    core.Point3D.create(tx_mm / 10.0, 0.0, 22.9),
    5.4,
    core.Point3D.create(tx_mm / 10.0, 0.0, 23.8),
    5.4,
)
tb.booleanOperation(rot_base, rot_bore, fusion.BooleanTypes.DifferenceBooleanType)
s4_comp.bRepBodies.add(rot_base, s4_feat).name = "turret_rotating_base_deck_plate"

# Brass Azimuth Pinion Gear at (X = -130.0, Y = -71.0, Z in [222.2, 230.8]) driven coaxially from below by Servo S4 spline!
s4_pinion = tb.createCylinderOrCone(
    core.Point3D.create(-13.0, -7.1, 22.05),
    0.4,
    core.Point3D.create(-13.0, -7.1, 23.08),
    0.4,
)
s4_gear_head = tb.createCylinderOrCone(
    core.Point3D.create(-13.0, -7.1, 22.22),
    1.15,
    core.Point3D.create(-13.0, -7.1, 23.08),
    1.15,
)
tb.booleanOperation(s4_pinion, s4_gear_head, fusion.BooleanTypes.UnionBooleanType)
s4_comp.bRepBodies.add(s4_pinion, s4_feat).name = "miter_gear_turret_azimuth_pinion"

main_fw_x, main_fw_z = -6.0, 286.5
cnt_fw_x, cnt_fw_z = -134.0, 316.0
cam_piv_x, cam_piv_z = -38.0, 316.0
hood_piv_x, hood_piv_z = -102.0, 352.0

# 3) Left & Right CNC Aluminum Turret Side Plates (|Y| = 62.0 mm, X in [-156, +2], Z in [236.05, 368]) seated flush on Rotating Base!
for side, sname in ((1.0, "left"), (-1.0, "right")):
    s_plate = tb.createBox(
        core.OrientedBoundingBox3D.create(
            core.Point3D.create(-7.7, side * 6.2, 30.2025),
            core.Vector3D.create(1.0, 0.0, 0.0),
            core.Vector3D.create(0.0, 1.0, 0.0),
            15.8,
            0.6,
            13.195,
        )
    )
    for hx_mm, hz_mm, hr_cm in (
        (main_fw_x, main_fw_z, 0.42),
        (cnt_fw_x, cnt_fw_z, 0.42),
        (cam_piv_x, cam_piv_z, 0.42),
        (hood_piv_x, hood_piv_z, 0.37),
        (tx_mm, 260.0, 0.65),
    ):
        p_hole = tb.createCylinderOrCone(
            core.Point3D.create(hx_mm / 10.0, side * 5.0, hz_mm / 10.0),
            hr_cm,
            core.Point3D.create(hx_mm / 10.0, side * 7.5, hz_mm / 10.0),
            hr_cm,
        )
        tb.booleanOperation(s_plate, p_hole, fusion.BooleanTypes.DifferenceBooleanType)
    s4_comp.bRepBodies.add(s_plate, s4_feat).name = f"turret_side_plate_{sname}"

    # Heavy-Duty Cantilevered Rotating Motor Cradle + Twin Side Gussets + Outer R=31.5mm Clamping Collar Ring
    m_stand = tb.createBox(
        core.OrientedBoundingBox3D.create(
            core.Point3D.create(tx_mm / 10.0, side * 7.75, 25.6),
            core.Vector3D.create(1.0, 0.0, 0.0),
            core.Vector3D.create(0.0, 1.0, 0.0),
            5.4,
            0.38,
            4.0,
        )
    )
    m_cradle = tb.createBox(
        core.OrientedBoundingBox3D.create(
            core.Point3D.create(tx_mm / 10.0, side * 11.555, 23.35),
            core.Vector3D.create(1.0, 0.0, 0.0),
            core.Vector3D.create(0.0, 1.0, 0.0),
            5.6,
            10.09,
            0.5,
        )
    )
    for gx_cm in (-10.8, -6.0):
        m_gus = tb.createBox(
            core.OrientedBoundingBox3D.create(
                core.Point3D.create(gx_cm, side * 7.03, 24.0),
                core.Vector3D.create(1.0, 0.0, 0.0),
                core.Vector3D.create(0.0, 1.0, 0.0),
                0.6,
                1.04,
                0.8,
            )
        )
        tb.booleanOperation(m_stand, m_gus, fusion.BooleanTypes.UnionBooleanType)
    m_strap_out = tb.createCylinderOrCone(
        core.Point3D.create(tx_mm / 10.0, side * 11.2, 26.0),
        3.15,
        core.Point3D.create(tx_mm / 10.0, side * 12.6, 26.0),
        3.15,
    )
    m_post = tb.createBox(
        core.OrientedBoundingBox3D.create(
            core.Point3D.create(tx_mm / 10.0, side * 15.6, 23.6),
            core.Vector3D.create(1.0, 0.0, 0.0),
            core.Vector3D.create(0.0, 1.0, 0.0),
            4.8,
            1.4,
            0.8,
        )
    )
    tb.booleanOperation(m_stand, m_cradle, fusion.BooleanTypes.UnionBooleanType)
    tb.booleanOperation(m_stand, m_strap_out, fusion.BooleanTypes.UnionBooleanType)
    tb.booleanOperation(m_stand, m_post, fusion.BooleanTypes.UnionBooleanType)
    # Subtract Rotating Base cylinder (R=78.1mm), Motor Can cylinder (R=26.5mm), and Motor Boss bore (R=15.0mm)
    rot_cut = tb.createCylinderOrCone(
        core.Point3D.create(tx_mm / 10.0, 0.0, 22.0),
        7.81,
        core.Point3D.create(tx_mm / 10.0, 0.0, 24.0),
        7.81,
    )
    mot_can_cut = tb.createCylinderOrCone(
        core.Point3D.create(tx_mm / 10.0, side * 7.9, 26.0),
        2.65,
        core.Point3D.create(tx_mm / 10.0, side * 20.0, 26.0),
        2.65,
    )
    mot_boss_cut = tb.createCylinderOrCone(
        core.Point3D.create(tx_mm / 10.0, side * 7.0, 26.0),
        1.5,
        core.Point3D.create(tx_mm / 10.0, side * 8.5, 26.0),
        1.5,
    )
    tb.booleanOperation(m_stand, rot_cut, fusion.BooleanTypes.DifferenceBooleanType)
    tb.booleanOperation(m_stand, mot_can_cut, fusion.BooleanTypes.DifferenceBooleanType)
    tb.booleanOperation(m_stand, mot_boss_cut, fusion.BooleanTypes.DifferenceBooleanType)
    s4_comp.bRepBodies.add(m_stand, s4_feat).name = f"turret_motor_mount_stand_{sname}"

    # Coaxial U-Shaped Servo Mount Brackets on Left Side Plate (S5 at X=-28, Z=316) and Right Side Plate (S6 at X=-92, Z=352)
    sv_x_cm = -2.8 if side > 0 else -9.2
    sv_z_cm = 31.6 if side > 0 else 35.2
    sv_brkt = tb.createBox(
        core.OrientedBoundingBox3D.create(
            core.Point3D.create(sv_x_cm, side * 7.18, sv_z_cm - 1.3),
            core.Vector3D.create(1.0, 0.0, 0.0),
            core.Vector3D.create(0.0, 1.0, 0.0),
            5.45,
            1.35,
            0.5,
        )
    )
    for ex_cm in (sv_x_cm - 2.42, sv_x_cm + 2.42):
        sv_ear = tb.createBox(
            core.OrientedBoundingBox3D.create(
                core.Point3D.create(ex_cm, side * 7.18, sv_z_cm - 0.1),
                core.Vector3D.create(1.0, 0.0, 0.0),
                core.Vector3D.create(0.0, 1.0, 0.0),
                0.62,
                1.35,
                2.0,
            )
        )
        tb.booleanOperation(sv_brkt, sv_ear, fusion.BooleanTypes.UnionBooleanType)
    s4_comp.bRepBodies.add(sv_brkt, s4_feat).name = f"turret_servo_standoff_mount_{sname}"

# S4 Azimuth Servo Under-Deck Mounting Bracket bolting S4 (X=-130, Y=-61, Z=203) SOLIDLY to underside of cnc_turret_mounting_deck_plate (Z=216)!
s4_brkt = tb.createBox(
    core.OrientedBoundingBox3D.create(
        core.Point3D.create(-13.0, -6.1, 21.45),
        core.Vector3D.create(1.0, 0.0, 0.0),
        core.Vector3D.create(0.0, 1.0, 0.0),
        2.0,
        5.5,
        0.28,
    )
)
for ey_cm in (-8.45, -3.75):
    s4_ear = tb.createBox(
        core.OrientedBoundingBox3D.create(
            core.Point3D.create(-13.0, ey_cm, 21.18),
            core.Vector3D.create(1.0, 0.0, 0.0),
            core.Vector3D.create(0.0, 1.0, 0.0),
            2.0,
            0.65,
            0.82,
        )
    )
    tb.booleanOperation(s4_brkt, s4_ear, fusion.BooleanTypes.UnionBooleanType)
s4_mid_cut = tb.createBox(
    core.OrientedBoundingBox3D.create(
        core.Point3D.create(-13.0, -6.1, 20.9),
        core.Vector3D.create(1.0, 0.0, 0.0),
        core.Vector3D.create(0.0, 1.0, 0.0),
        2.2,
        4.2,
        1.6,
    )
)
tb.booleanOperation(s4_brkt, s4_mid_cut, fusion.BooleanTypes.DifferenceBooleanType)
s4_comp.bRepBodies.add(s4_brkt, s4_feat).name = "servo_s4_deck_mount_stand"

# Turret Top Structural Bridge (X = -140.0, Y in [-58.95, +58.95], Z in [358, 366])
t_bridge = tb.createBox(
    core.OrientedBoundingBox3D.create(
        core.Point3D.create(-14.0, 0.0, 36.2),
        core.Vector3D.create(1.0, 0.0, 0.0),
        core.Vector3D.create(0.0, 1.0, 0.0),
        2.8,
        11.79,
        0.8,
    )
)
s4_comp.bRepBodies.add(t_bridge, s4_feat).name = "turret_top_bridge"

# 4) Main 96mm Rhino Wheel (R=48mm) + 2x 82mm Steel Flywheels (R=41mm) at (X = -6.0 mm, Z = 286.5 mm):
rhino_96_tire, rhino_96_hub = make_gobilda_wheel_brep(main_fw_x, 0.0, main_fw_z, 48.0, 34.0, 24.0)
s4_comp.bRepBodies.add(rhino_96_tire, s4_feat).name = "rhino_96_main_tire_amber"
s4_comp.bRepBodies.add(rhino_96_hub, s4_feat).name = "rhino_96_main_hub_silver"

# Inclined (+8.0°) Upper Turret Gold Throat Guide inside Turret (continuing from X=-125, Z=238 up to X=-131, Z=286 mm):
# Tangent (+1.5mm clear) to staged_pollen_ball_4 (-89.0, 268.0, R=35.56 -> X_min=-124.56) and leading straight into Counter-Wheel (-134, 316, R=24)!
ut_backplate = tb.createBox(
    core.OrientedBoundingBox3D.create(
        core.Point3D.create(0.0, 0.0, 0.0),
        core.Vector3D.create(1.0, 0.0, 0.0),
        core.Vector3D.create(0.0, 1.0, 0.0),
        0.4,
        8.2,
        5.0,
    )
)
tb.transform(ut_backplate, mat_mul(mat_pos_mm(-129.0, 0.0, 262.0), mat_rot_deg((0, 1, 0), 8.0)))
ut_anchor = tb.createBox(
    core.OrientedBoundingBox3D.create(
        core.Point3D.create(-12.8, 0.0, 23.85),
        core.Vector3D.create(1.0, 0.0, 0.0),
        core.Vector3D.create(0.0, 1.0, 0.0),
        0.8,
        8.2,
        0.45,
    )
)
tb.booleanOperation(ut_backplate, ut_anchor, fusion.BooleanTypes.UnionBooleanType)
s4_comp.bRepBodies.add(ut_backplate, s4_feat).name = "turret_upper_throat_elevator_chute"

for side, sname in ((1.0, "left"), (-1.0, "right")):
    fw_disc = tb.createCylinderOrCone(
        core.Point3D.create(main_fw_x / 10.0, side * 2.1, main_fw_z / 10.0),
        4.1,
        core.Point3D.create(main_fw_x / 10.0, side * 3.1, main_fw_z / 10.0),
        4.1,
    )
    fw_bore = tb.createCylinderOrCone(
        core.Point3D.create(main_fw_x / 10.0, side * 2.0, main_fw_z / 10.0),
        0.41,
        core.Point3D.create(main_fw_x / 10.0, side * 3.2, main_fw_z / 10.0),
        0.41,
    )
    tb.booleanOperation(fw_disc, fw_bore, fusion.BooleanTypes.DifferenceBooleanType)
    s4_comp.bRepBodies.add(fw_disc, s4_feat).name = f"steel_flywheel_82mm_{sname}_rex_axle"

# Main Flywheel 8mm REX Shaft (Y in [-74, +74 mm])
main_shaft = tb.createCylinderOrCone(
    core.Point3D.create(main_fw_x / 10.0, -7.4, main_fw_z / 10.0),
    0.4,
    core.Point3D.create(main_fw_x / 10.0, 7.4, main_fw_z / 10.0),
    0.4,
)
s4_comp.bRepBodies.add(main_shaft, s4_feat).name = "shooter_main_rex_axle"

# 5) Upper-Rear 48mm Topspin Counter-Wheel (R=24mm) at (X = -134.0 mm, Z = 316.0 mm):
rhino_72_tire, rhino_72_hub = make_gobilda_wheel_brep(cnt_fw_x, 0.0, cnt_fw_z, 24.0, 34.0, 13.0)
s4_comp.bRepBodies.add(rhino_72_tire, s4_feat).name = "rhino_72_counter_tire_rose"
s4_comp.bRepBodies.add(rhino_72_hub, s4_feat).name = "rhino_72_counter_hub_silver"

cnt_shaft = tb.createCylinderOrCone(
    core.Point3D.create(cnt_fw_x / 10.0, -7.4, cnt_fw_z / 10.0),
    0.4,
    core.Point3D.create(cnt_fw_x / 10.0, 7.4, cnt_fw_z / 10.0),
    0.4,
)
s4_comp.bRepBodies.add(cnt_shaft, s4_feat).name = "shooter_counter_rex_axle"

# Outboard Shoulder-Bolt Stub Axles for Cam Arms (X=-38, Z=316) & Hood Hinge (X=-102, Z=352)
for side, sname in ((1.0, "left"), (-1.0, "right")):
    cam_stub = tb.createCylinderOrCone(
        core.Point3D.create(cam_piv_x / 10.0, side * 4.3, cam_piv_z / 10.0),
        0.4,
        core.Point3D.create(cam_piv_x / 10.0, side * 6.55, cam_piv_z / 10.0),
        0.4,
    )
    s4_comp.bRepBodies.add(cam_stub, s4_feat).name = f"shooter_cam_pivot_stub_rex_axle_{sname}"
    hood_stub = tb.createCylinderOrCone(
        core.Point3D.create(hood_piv_x / 10.0, side * 5.2, hood_piv_z / 10.0),
        0.35,
        core.Point3D.create(hood_piv_x / 10.0, side * 6.55, hood_piv_z / 10.0),
        0.35,
    )
    s4_comp.bRepBodies.add(hood_stub, s4_feat).name = f"hood_hinge_stub_rex_axle_{sname}"

# Flanged Bearings for Main & Counter Shafts on Left & Right Side Plates (|Y| in [65.05, 67.5 mm])
for sx, sz, slbl in ((main_fw_x, main_fw_z, "main"), (cnt_fw_x, cnt_fw_z, "cnt")):
    for side in (1.0, -1.0):
        brg = tb.createCylinderOrCone(
            core.Point3D.create(sx / 10.0, side * 6.505, sz / 10.0),
            0.85,
            core.Point3D.create(sx / 10.0, side * 6.75, sz / 10.0),
            0.85,
        )
        brg_bore = tb.createCylinderOrCone(
            core.Point3D.create(sx / 10.0, side * 6.4, sz / 10.0),
            0.41,
            core.Point3D.create(sx / 10.0, side * 6.9, sz / 10.0),
            0.41,
        )
        tb.booleanOperation(brg, brg_bore, fusion.BooleanTypes.DifferenceBooleanType)
        s4_comp.bRepBodies.add(brg, s4_feat).name = f"shooter_flanged_bearing_{slbl}_{int(side)}"

# 6) Left & Right HTD-5M Timing Pulleys & Belts at Y = +71.5 mm (M5 -> Main Flywheel) and Y = -71.5 mm (M6 -> Counter Flywheel)
p1_l, p2_l, belt_l = make_pulley_and_belt_brep(tx_mm, 260.0, main_fw_x, main_fw_z, 71.5, 11.5, 7.0)
s4_comp.bRepBodies.add(p1_l, s4_feat).name = "shooter_left_pulley_1"
s4_comp.bRepBodies.add(p2_l, s4_feat).name = "shooter_left_pulley_2"
s4_comp.bRepBodies.add(belt_l, s4_feat).name = "shooter_left_timing_belt"

p1_r, p2_r, belt_r = make_pulley_and_belt_brep(tx_mm, 260.0, cnt_fw_x, cnt_fw_z, -71.5, 11.5, 7.0)
s4_comp.bRepBodies.add(p1_r, s4_feat).name = "shooter_right_pulley_1"
s4_comp.bRepBodies.add(p2_r, s4_feat).name = "shooter_right_pulley_2"
s4_comp.bRepBodies.add(belt_r, s4_feat).name = "shooter_right_timing_belt"

# 7) Full-Width Hinged Variable-Pitch Deflector Hood at (X = -86.0, Y = 0.0, Z = 360.0, spanning Y in [-58.9, +58.9 mm]):
hood_box = tb.createBox(
    core.OrientedBoundingBox3D.create(
        core.Point3D.create(0.0, 0.0, 0.0),
        core.Vector3D.create(1.0, 0.0, 0.0),
        core.Vector3D.create(0.0, 1.0, 0.0),
        5.2,
        11.78,
        0.5,
    )
)
for side in (1.0, -1.0):
    hear = tb.createBox(
        core.OrientedBoundingBox3D.create(
            core.Point3D.create(-1.6, side * 5.59, -0.6),
            core.Vector3D.create(1.0, 0.0, 0.0),
            core.Vector3D.create(0.0, 1.0, 0.0),
            1.6,
            0.6,
            1.4,
        )
    )
    tb.booleanOperation(hood_box, hear, fusion.BooleanTypes.UnionBooleanType)
tb.transform(hood_box, mat_mul(mat_pos_mm(-86.0, 0.0, 360.0), mat_rot_deg((0, 1, 0), 28.0)))
hood_pin_cut = tb.createCylinderOrCone(
    core.Point3D.create(hood_piv_x / 10.0, -7.0, hood_piv_z / 10.0),
    0.37,
    core.Point3D.create(hood_piv_x / 10.0, 7.0, hood_piv_z / 10.0),
    0.37,
)
tb.booleanOperation(hood_box, hood_pin_cut, fusion.BooleanTypes.DifferenceBooleanType)
s4_comp.bRepBodies.add(hood_box, s4_feat).name = "variable_pitch_exit_hood_cyan"

# 8) Detailed Limelight 3A Smart Camera mounted on CNC Camera Bracket centered on Turret Top Bridge (X = -140.0, Y = 0.0, Z = 366.05..400 mm)
ll_body = tb.createBox(
    core.OrientedBoundingBox3D.create(
        core.Point3D.create(-14.0, 0.0, 38.3),
        core.Vector3D.create(1.0, 0.0, 0.0),
        core.Vector3D.create(0.0, 1.0, 0.0),
        2.6,
        7.4,
        3.4,
    )
)
ll_flange = tb.createBox(
    core.OrientedBoundingBox3D.create(
        core.Point3D.create(-14.0, 0.0, 36.855),
        core.Vector3D.create(1.0, 0.0, 0.0),
        core.Vector3D.create(0.0, 1.0, 0.0),
        3.2,
        8.6,
        0.5,
    )
)
tb.booleanOperation(ll_body, ll_flange, fusion.BooleanTypes.UnionBooleanType)
s4_comp.bRepBodies.add(ll_body, s4_feat).name = "limelight_3a_housing_green"

# Dual optical camera & LED illumination lenses on BOTH faces of Limelight 3A so target tracking is unmistakable
for ly in (-1.8, 1.8):
    for lx1, lx2 in ((-15.3, -15.8), (-12.7, -12.2)):
        ll_lens = tb.createCylinderOrCone(
            core.Point3D.create(lx1, ly, 38.5),
            0.75,
            core.Point3D.create(lx2, ly, 38.5),
            0.75,
        )
        s4_comp.bRepBodies.add(ll_lens, s4_feat).name = f"limelight_3a_lens_bearing_{int(ly * 10)}_{int(lx1 * 10)}"

s4_feat.finishEdit()

# ==============================================================================
# NUDGE 4 MECANUM WHEELS INWARD BY 1.5 MM (|Y| = 203.5 MM) AND TRIM AXLE TIPS SO OVERALL WIDTH IS 454.21 MM <= 457.20 MM (100% R102 LEGAL!)
# ==============================================================================
for s1_k, s1_m in saved_s1_transforms.items():
    s1_o = STAGE1_OCCS.get(s1_k)
    if s1_o and s1_o.isValid:
        s1_o.isGrounded = False
        s1_o.transform2 = s1_m
        s1_o.isLightBulbOn = True

MEC_S1_CENTERED = mat_mul(mat_rot_deg((1, 0, 0), 90.0), mat_pos_mm(0.0, 0.0, 7.515))
for wname, wx, wy, wyaw in (
    ("wheel_front_left", 144.0, 203.5, 0.0),
    ("wheel_front_right", 144.0, -203.5, 180.0),
    ("wheel_rear_left", -144.0, 203.5, 0.0),
    ("wheel_rear_right", -144.0, -203.5, 180.0),
):
    w_occ = STAGE1_OCCS.get(wname)
    if w_occ and w_occ.isValid:
        w_occ.isGrounded = False
        try:
            w_occ.isGroundToParent = False
        except Exception:
            pass
        w_occ.transform2 = mat_mul(mat_pos_mm(wx, wy, 48.0), mat_rot_deg((0, 0, 1), wyaw), MEC_S1_CENTERED)

# ==============================================================================
# PLACE ALL STAGE 3, 4 & 5 VENDOR & CUSTOM STEP OCCURRENCES (AFTER B-REP CREATION!)
# ==============================================================================
QB_CENTERED = mat_pos_mm(0.0, -6.0, 0.0)
QB_FACE_MINUS_X = mat_mul(mat_rot_deg((0, 0, 1), 90.0), QB_CENTERED)
MOT_FACE_PLUS_X = mat_mul(mat_rot_deg((0, 0, 1), -90.0), mat_pos_mm(37.171, -101.735, 11.078))
# Motor with faceplate at (0,0,0), output shaft along -Y [0..-23.57mm] and body along +Y [0..+116.56mm]:
MOT_SHAFT_MINUS_Y = mat_mul(mat_rot_deg((0, 0, 1), 180.0), mat_pos_mm(37.171, -101.735, 11.078))
# Motor with faceplate at (0,0,0), output shaft along +Y [0..+23.57mm] and body along -Y [-116.56..0mm]:
MOT_SHAFT_PLUS_Y = mat_pos_mm(37.171, -101.735, 11.078)

# 1. Stage 3 Left & Right Magazine Shrouds are now precision B-Rep plates (magazine_shroud_purple_left/right) at |Y| in [50, 55 mm] with zero pulley collision!

# 2. Stage 3 Magazine Motor M8 (5203-2402-0019) + Quad Block Mount in the Right Inboard Corridor (Y = -103.0, Z = 96.0 — zero chassis or shroud collision!)
place_s345_part(
    "quad_block",
    mat_mul(mat_pos_mm(124.0, -103.0, 96.0), QB_FACE_MINUS_X),
    "qb_m8_magazine_mount",
)
place_s345_part(
    "motor_5203",
    mat_mul(mat_pos_mm(118.0, -103.0, 96.0), MOT_FACE_PLUS_X),
    "motor_5203_m8_magazine",
)

# 3. Stage 3 Magazine Top Stopper Servo S3 (nested FLUSH inside Left Mid Vertical 5H Stanchion open cavity at X = 0.0, Y = +148.0, Z = 158.0 -> spline at Z = 168.0!)
SERVO_IN_LEFT_UPRIGHT = mat_mul(mat_rot_deg((0, 0, 1), -90.0), mat_rot_deg((0, 1, 0), 90.0))
place_s345_part(
    "servo_2000",
    mat_mul(mat_pos_mm(0.0, 148.0, 158.0), SERVO_IN_LEFT_UPRIGHT),
    "servo_s3_magazine_stopper",
)

# 4. Stage 4 Coupled Shooter Motors M5 (Left, Y = +79.5) and M6 (Right, Y = -79.5) seated on Under-Motor Support Cradles at X = -84.0, Z = 260.0!
place_s345_part(
    "motor_5203",
    mat_mul(mat_pos_mm(tx_mm, 79.5, 260.0), MOT_SHAFT_MINUS_Y),
    "motor_5203_m5_shooter_left",
)
place_s345_part(
    "motor_5203",
    mat_mul(mat_pos_mm(tx_mm, -79.5, 260.0), MOT_SHAFT_PLUS_Y),
    "motor_5203_m6_shooter_right",
)

# 5. Stage 4 Bi-Positional Cam & Counter-Wheel Arms (cam_arm) at Y = ±50.0 mm, X = -86.0 mm, Z = 316.0 mm (coaxial with X=-134 and X=-38 shafts!)
place_s345_part(
    "cam_arm",
    mat_pos_mm(-86.0, 50.0, 316.0),
    "shooter_cam_arm_left",
)
place_s345_part(
    "cam_arm",
    mat_pos_mm(-86.0, -50.0, 316.0),
    "shooter_cam_arm_right",
)

# 6. Stage 4 Turret Servos (100% coaxial H25T Splines with zero plate/gear interference!):
place_s345_part(
    "servo_2000",
    mat_mul(mat_pos_mm(-130.0, -61.0, 203.0), mat_rot_deg((0, 0, 1), 90.0)),
    "servo_s4_turret_azimuth",
)
place_s345_part(
    "servo_2000",
    mat_mul(mat_pos_mm(-28.0, 83.0, 316.0), mat_rot_deg((1, 0, 0), 90.0)),
    "servo_s5_cam_selector",
)
place_s345_part(
    "servo_2000",
    mat_mul(mat_pos_mm(-92.0, -83.0, 352.0), mat_rot_deg((1, 0, 0), -90.0)),
    "servo_s6_pitch_hood",
)

# Also re-verify Stage 2 occurrence transforms so deleting/recreating Stage 3-4 hardware never reverts Stage 2 parts!
QB_FACE_PLUS_X = mat_mul(mat_rot_deg((0, 0, 1), -90.0), QB_CENTERED)
STAR_ALONG_Y = mat_rot_deg((1, 0, 0), 90.0)
for s2_lbl, s2_mat in (
    ("qb_front_bumper_left", mat_mul(mat_pos_mm(210.0, 156.0, 48.0), QB_FACE_PLUS_X)),
    ("qb_front_bumper_right", mat_mul(mat_pos_mm(210.0, -156.0, 48.0), QB_FACE_PLUS_X)),
    ("front_funnel_bumper_left", mat_pos_mm(216.0, 156.0, 48.0)),
    ("front_funnel_bumper_right", mat_pos_mm(216.0, -156.0, 48.0)),
    ("star_roller_sleeve_1", mat_mul(mat_pos_mm(203.0, -62.0, 104.0), mat_rot_deg((0, 1, 0), 20.0), STAR_ALONG_Y)),
    ("star_roller_sleeve_2", mat_mul(mat_pos_mm(203.0, 0.0, 104.0), mat_rot_deg((0, 1, 0), 40.0), STAR_ALONG_Y)),
    ("star_roller_sleeve_3", mat_mul(mat_pos_mm(203.0, 62.0, 104.0), mat_rot_deg((0, 1, 0), 60.0), STAR_ALONG_Y)),
    ("qb_m7_intake_mount", mat_mul(mat_pos_mm(132.0, 103.0, 96.0), QB_FACE_MINUS_X)),
    ("motor_5203_m7_intake", mat_mul(mat_pos_mm(126.0, 103.0, 96.0), MOT_FACE_PLUS_X)),
    ("g407_lip_gate_bridge", mat_pos_mm(202.0, 0.0, 148.0)),
    ("servo_s1_lip_gate", mat_mul(mat_pos_mm(192.0, 148.0, 158.0), SERVO_IN_LEFT_UPRIGHT)),
    ("flower_depollinator_fork", mat_pos_mm(148.0, 0.0, 20.0)),
    ("servo_s2_depollinator", mat_pos_mm(96.0, -96.0, 50.0)),
    ("robot_sign_left_149", mat_mul(mat_pos_mm(84.0, 181.65, 140.0), mat_rot_deg((0, 0, 1), 180.0))),
    ("robot_sign_right_149", mat_pos_mm(84.0, -181.65, 140.0)),
):
    s2_occ = STAGE2_OCCS.get(s2_lbl)
    if s2_occ and s2_occ.isValid:
        s2_occ.isGrounded = False
        try:
            s2_occ.isGroundToParent = False
        except Exception:
            pass
        s2_occ.transform2 = s2_mat
        s2_occ.isLightBulbOn = True

if design.snapshots.hasPendingSnapshot:
    design.snapshots.add()


# ==============================================================================
# APPLY HIGH-CONTRAST FUSION 360 APPEARANCES TO STAGES 3, 4 & 5
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


app_violet = get_or_make_color("Hermes_Shroud_Violet", 167, 139, 250)
app_crimson = get_or_make_color("Hermes_CamArm_Crimson", 244, 63, 94)
app_orange = get_or_make_color("Hermes_Stinger_Orange", 251, 146, 60)
app_pollen = get_or_make_color("Hermes_Pollen_Yellow", 250, 204, 21)
app_gecko = get_or_make_color("Hermes_Gecko_Green", 34, 197, 94)
app_cyan = get_or_make_color("Hermes_Hood_Cyan", 56, 189, 248)
app_gold = get_or_make_color("Hermes_Turret_Gold", 245, 158, 11)
app_deck = get_or_make_color("Hermes_Deck_Aluminum", 203, 213, 225)
app_belt = get_or_make_color("Hermes_Belt_Rubber", 30, 41, 59)
app_brass = get_or_make_color("Hermes_Miter_Brass", 218, 165, 32)
app_steel = get_or_make_color("Hermes_Bright_Steel", 226, 232, 240)
app_bearing = get_or_make_color("Hermes_Dark_Bearing", 51, 65, 85)


def color_component(comp: fusion.Component, app_color: core.Appearance):
    for b_i in range(comp.bRepBodies.count):
        body = comp.bRepBodies.item(b_i)
        body.appearance = app_color
        for f_i in range(body.faces.count):
            body.faces.item(f_i).appearance = app_color


color_component(STATE["lib"]["mag_shroud"], app_violet)
color_component(STATE["lib"]["cam_arm"], app_crimson)
color_component(STATE["lib"]["stinger_cup"], app_orange)

for comp_obj in (s3_comp, s4_comp):
    for b_i in range(comp_obj.bRepBodies.count):
        body = comp_obj.bRepBodies.item(b_i)
        bname = body.name
        if "pollen_ball" in bname:
            body.appearance = app_pollen
        elif "nectar_ball" in bname:
            body.appearance = app_orange
        elif "shroud_purple" in bname:
            body.appearance = app_violet
        elif "tire_green" in bname or "limelight_3a_housing" in bname:
            body.appearance = app_gecko
        elif "tire_amber" in bname or "turret_side_plate" in bname or "turret_top_bridge" in bname:
            body.appearance = app_gold
        elif "tire_rose" in bname:
            body.appearance = app_crimson
        elif "hood_cyan" in bname:
            body.appearance = app_cyan
        elif "carbon_fiber" in bname:
            body.appearance = app_orange
        elif "deck_plate" in bname or "hub_silver" in bname or "pulley" in bname:
            body.appearance = app_deck
        elif "timing_belt" in bname:
            body.appearance = app_belt
        elif "miter_gear" in bname:
            body.appearance = app_brass
        elif "rex_axle" in bname:
            body.appearance = app_steel
        elif "bearing" in bname or "slewing" in bname or "mount" in bname or "stand" in bname:
            body.appearance = app_bearing
        elif "elevator_chute" in bname:
            body.appearance = app_pollen

for comp_c in design.allComponents:
    comp_c.isBodiesFolderLightBulbOn = True
    for b_idx in range(comp_c.bRepBodies.count):
        comp_c.bRepBodies.item(b_idx).isLightBulbOn = True

root.isJointsFolderLightBulbOn = False
if app.activeViewport:
    app.activeViewport.refresh()

# Compute exact overall robot bounding box across all visible occurrences and verify R102 (457.2 x 457.2 x 457.2 mm) cube compliance!
xs_min, ys_min, zs_min = [], [], []
xs_max, ys_max, zs_max = [], [], []
for i in range(root.occurrences.count):
    o = root.occurrences.item(i)
    if o.isLightBulbOn:
        bb = bbox_mm(o.boundingBox)
        if bb["size_mm"][0] > 0.1:
            xs_min.append(bb["min_mm"][0])
            ys_min.append(bb["min_mm"][1])
            zs_min.append(bb["min_mm"][2])
            xs_max.append(bb["max_mm"][0])
            ys_max.append(bb["max_mm"][1])
            zs_max.append(bb["max_mm"][2])

for comp_obj in (s3_comp, s4_comp):
    for b_i in range(comp_obj.bRepBodies.count):
        bb = bbox_mm(comp_obj.bRepBodies.item(b_i).boundingBox)
        xs_min.append(bb["min_mm"][0])
        ys_min.append(bb["min_mm"][1])
        zs_min.append(bb["min_mm"][2])
        xs_max.append(bb["max_mm"][0])
        ys_max.append(bb["max_mm"][1])
        zs_max.append(bb["max_mm"][2])

overall_size_mm = [
    round(max(xs_max) - min(xs_min), 2),
    round(max(ys_max) - min(ys_min), 2),
    round(max(zs_max) - min(zs_min), 2),
]

result = {
    "stage345_parts_placed": len(STAGE345_OCCS),
    "overall_min_mm": [round(min(xs_min), 2), round(min(ys_min), 2), round(min(zs_min), 2)],
    "overall_max_mm": [round(max(xs_max), 2), round(max(ys_max), 2), round(max(zs_max), 2)],
    "overall_size_mm": overall_size_mm,
    "overall_size_inches": [round(x / 25.4, 2) for x in overall_size_mm],
    "r102_18in_cube_compliant": all(x <= 457.2 for x in overall_size_mm),
}
