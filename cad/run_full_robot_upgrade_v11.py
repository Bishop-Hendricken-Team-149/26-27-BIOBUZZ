"""
Fast Single-BaseFeature Full Robot Upgrade for 'project_hive_queen_robot' in Autodesk Fusion 360:
1. Fast, Variable-Size Floating Intake (from Black Box FTC YouTube Short https://www.youtube.com/shorts/cu2J6eQIJkY):
   - Upper Pivot & Secondary Star-Paddle Shaft at (X = +156.0 mm, Z = 144.0 mm) passing directly through the existing
     14.0 mm (R = 7.0 mm) goBILDA Hole 3 bearing bores of the Front 4-Hole Uprights (0.0 mm3 upright overlap!), carrying
     Left & Right Inner Compliant 6-Lobe Star-Paddle Rollers (Y = ±74.0 mm) that sweep balls into the chute.
   - Left & Right Pivoting Machined Aluminum Floating Swing-Arms (Y = ±104.0 mm) hanging downward & forward from the
     Upper Pivot (156.0, 144.0) to the Floating Front Hex Shaft at (X = +212.0 mm, Z = 117.0 mm).
   - Floating Front Hex Shaft carrying a central Compliant Star-Paddle Roller (Y = 0) flanked by 4x Segmented Spoked
     Compliant Wheel Clusters (OD = 47.0 mm, R = 23.5 mm, 8 hollow flex-spokes per wheel at Y = -72, -38, +38, +72 mm).
   - Coaxial Belt-on-Pivot HTD-5M Timing Belt Transmission:
     - Stage 1 Belt (Y = +46.0 mm): Motor M7 (104.0, 98.0) -> Upper Pivot Shaft (156.0, 144.0)
     - Stage 2 Coaxial Floating-Arm Belt (Y = +113.5 mm): Upper Pivot Shaft (156.0, 144.0) -> Floating Front Shaft (212.0, 117.0)
   - 4-Prong Upper Guiding Comb Fingers / Retention Roof + Front Upper Crossbar Servo & Sensor Mounting Bridge.
2. Asymmetrical Wide-Area Intake Funnel (360 mm capture mouth -> 108 mm throat):
   - Left Funnel Wing is LONGER (reaching X = +239.5 mm) with a sweeping 38 deg inward funnel wall + floor scoop lip.
   - Right Funnel Wing is SHORTER (reaching X = +224.0 mm, 15.5 mm axial stagger) with a steeper 58 deg anti-jam wall.
3. Unified One-Piece Continuous J-Chute & 4-Station Structural Mounting Cradles ('unified_continuous_j_chute'):
   - Single boolean-unioned solid body combining the front intake floor scoop apron, 22.5 deg lower chute floor,
     curved cylindrical elbow transition, rear elevator wall terminating flush at Z = 216.0 mm against the underside of
     the shooter deck plate (zero self-clipping and zero clipping into the turret bearing ring or shooter base!), and
     4 integral structural mounting cradles bolted to the Front Uprights, Mid Crossbar, Center Towers, and Rear Crossbar.
4. Servo-Actuated Upper Magazine Indexing Stopper Gate ('indexer_stopper_gate_*'):
   - High-contrast Crimson 3-prong Indexing Stopper Gate Paddle + 8mm REX pivot shaft + Servo S3 structural bridge +
     optical ball-ready sensor between the upper magazine and shooter nip to hold queued balls until firing.
5. Full-Width Anchored Shooter & Turret Assembly (Zero Floating Parts):
   - Intact 360 deg Turret Bearing Ring ('turret_bearing_ring_85mm:1') + Full-Height Symmetrical Turret Side Plates
     (Y = ±62.0 mm, Z in [236.0, 386.0 mm]) resting flush on the bearing ring and welding 'turret_top_structural_bridge:1'
     and 'limelight_3a_vision:1' flush between their inner faces.
   - Full-Width Hinged Variable-Pitch Shooter Exit Hood ('variable_pitch_exit_hood_full', Y in [-59.0, +59.0 mm]) pinned
     into both turret side plates via 'hood_hinge_rex_shaft', driven by pocketed outboard Servo S5/S6 cradles & linkages.
   - Monolithic Clamping Motor Saddles for Shooter Motors M5 & M6, Deck-Mounted Azimuth Servo S4 Cradle & Gear Train,
     and full-width Shooter Shaft Spacers & Flanged Bearings.
6. 3-Wheel Dead-Wheel Odometry Suite (Left-X, Right-X, Rear-Y Strafe) + Pinpoint Odometry Computer Cradle +
   Battery Cradle Tray & Hold-Down Strap + REV Hub Mounts + Main Power Switch + Complete Color-Coded 3D Wiring Harness.
"""
import math
import adsk.core as core
import adsk.fusion as fusion


def pt_cm(x_mm: float, y_mm: float, z_mm: float) -> core.Point3D:
    return core.Point3D.create(x_mm / 10.0, y_mm / 10.0, z_mm / 10.0)


def vec_cm(x_mm: float, y_mm: float, z_mm: float) -> core.Vector3D:
    return core.Vector3D.create(x_mm / 10.0, y_mm / 10.0, z_mm / 10.0)


def mat_pos_mm(dx_mm: float, dy_mm: float, dz_mm: float) -> core.Matrix3D:
    m = core.Matrix3D.create()
    m.translation = vec_cm(dx_mm, dy_mm, dz_mm)
    return m


def mat_rot_deg(axis_xyz: tuple[float, float, float], angle_deg: float) -> core.Matrix3D:
    m = core.Matrix3D.create()
    vec = core.Vector3D.create(float(axis_xyz[0]), float(axis_xyz[1]), float(axis_xyz[2]))
    m.setToRotation(math.radians(angle_deg), vec, core.Point3D.create(0, 0, 0))
    return m


def mat_mul(*matrices: core.Matrix3D) -> core.Matrix3D:
    out = core.Matrix3D.create()
    for m in reversed(matrices):
        out.transformBy(m)
    return out


tb = fusion.TemporaryBRepManager.get()


def make_box_mm(cx_mm: float, cy_mm: float, cz_mm: float, sx_mm: float, sy_mm: float, sz_mm: float) -> fusion.BRepBody:
    obb = core.OrientedBoundingBox3D.create(
        pt_cm(cx_mm, cy_mm, cz_mm),
        core.Vector3D.create(1.0, 0.0, 0.0),
        core.Vector3D.create(0.0, 1.0, 0.0),
        sx_mm / 10.0,
        sy_mm / 10.0,
        sz_mm / 10.0,
    )
    return tb.createBox(obb)


def make_cyl_mm(x1: float, y1: float, z1: float, x2: float, y2: float, z2: float, r_mm: float) -> fusion.BRepBody:
    return tb.createCylinderOrCone(pt_cm(x1, y1, z1), r_mm / 10.0, pt_cm(x2, y2, z2), r_mm / 10.0)


def make_pulley_world_mm(cx: float, cy: float, cz: float, r_drum: float = 12.0, width: float = 8.0, r_bore: float = 4.05) -> fusion.BRepBody:
    y1 = cy - width / 2.0
    y2 = cy + width / 2.0
    pul = make_cyl_mm(cx, y1, cz, cx, y2, cz, r_drum)
    fl_a = make_cyl_mm(cx, y1, cz, cx, y1 + 1.2, cz, r_drum + 2.0)
    fl_b = make_cyl_mm(cx, y2 - 1.2, cz, cx, y2, cz, r_drum + 2.0)
    bore = make_cyl_mm(cx, y1 - 1.0, cz, cx, y2 + 1.0, cz, r_bore)
    tb.booleanOperation(pul, fl_a, fusion.BooleanTypes.UnionBooleanType)
    tb.booleanOperation(pul, fl_b, fusion.BooleanTypes.UnionBooleanType)
    tb.booleanOperation(pul, bore, fusion.BooleanTypes.DifferenceBooleanType)
    return pul


def make_belt_loop_mm(x1: float, z1: float, x2: float, z2: float, y_mm: float, r_in_mm: float, r_out_mm: float, width_mm: float) -> fusion.BRepBody:
    dx = x2 - x1
    dz = z2 - z1
    dist = math.hypot(dx, dz)
    pitch_deg = -math.degrees(math.atan2(dz, dx))
    cx = (x1 + x2) / 2.0
    cz = (z1 + z2) / 2.0
    y1 = y_mm - width_mm / 2.0
    y2 = y_mm + width_mm / 2.0

    outer = make_cyl_mm(x1, y1, z1, x1, y2, z1, r_out_mm)
    c2 = make_cyl_mm(x2, y1, z2, x2, y2, z2, r_out_mm)
    span_out = make_box_mm(0.0, 0.0, 0.0, dist, width_mm, 2.0 * r_out_mm)
    tb.transform(span_out, mat_mul(mat_pos_mm(cx, y_mm, cz), mat_rot_deg((0, 1, 0), pitch_deg)))
    tb.booleanOperation(outer, c2, fusion.BooleanTypes.UnionBooleanType)
    tb.booleanOperation(outer, span_out, fusion.BooleanTypes.UnionBooleanType)

    inner = make_cyl_mm(x1, y1 - 1.0, z1, x1, y2 + 1.0, z1, r_in_mm)
    c2_in = make_cyl_mm(x2, y1 - 1.0, z2, x2, y2 + 1.0, z2, r_in_mm)
    span_in = make_box_mm(0.0, 0.0, 0.0, dist, width_mm + 2.0, 2.0 * r_in_mm)
    tb.transform(span_in, mat_mul(mat_pos_mm(cx, y_mm, cz), mat_rot_deg((0, 1, 0), pitch_deg)))
    tb.booleanOperation(inner, c2_in, fusion.BooleanTypes.UnionBooleanType)
    tb.booleanOperation(inner, span_in, fusion.BooleanTypes.UnionBooleanType)

    tb.booleanOperation(outer, inner, fusion.BooleanTypes.DifferenceBooleanType)
    return outer


def make_spoked_compliant_wheel_mm(cx: float, cy: float, cz: float, r_out: float, r_hub: float, width: float) -> fusion.BRepBody:
    y1 = cy - width / 2.0
    y2 = cy + width / 2.0
    wheel = make_cyl_mm(cx, y1, cz, cx, y2, cz, r_out)
    bore = make_cyl_mm(cx, y1 - 1.0, cz, cx, y2 + 1.0, cz, 4.1)
    tb.booleanOperation(wheel, bore, fusion.BooleanTypes.DifferenceBooleanType)

    r_mid = (r_out + r_hub) * 0.5
    r_hole = (r_out - r_hub) * 0.26
    for i in range(8):
        ang = math.radians(i * 45.0)
        hx = cx + r_mid * math.cos(ang)
        hz = cz + r_mid * math.sin(ang)
        slot = make_cyl_mm(hx, y1 - 1.0, hz, hx, y2 + 1.0, hz, r_hole)
        tb.booleanOperation(wheel, slot, fusion.BooleanTypes.DifferenceBooleanType)
    return wheel


def make_wire_path_mm(pts_mm: list[tuple[float, float, float]], r_mm: float = 2.6) -> fusion.BRepBody:
    wire = None
    for i in range(len(pts_mm) - 1):
        p1 = pts_mm[i]
        p2 = pts_mm[i + 1]
        seg = make_cyl_mm(p1[0], p1[1], p1[2], p2[0], p2[1], p2[2], r_mm)
        joint = tb.createSphere(pt_cm(p2[0], p2[1], p2[2]), r_mm / 10.0)
        tb.booleanOperation(seg, joint, fusion.BooleanTypes.UnionBooleanType)
        if wire is None:
            wire = seg
        else:
            tb.booleanOperation(wire, seg, fusion.BooleanTypes.UnionBooleanType)
    return wire


def get_or_make_color(name: str, r: int, g: int, b: int) -> core.Appearance:
    app_obj = design.appearances.itemByName(name)
    if not app_obj:
        base_app = design.appearances.item(0)
        app_obj = design.appearances.addByCopy(base_app, name)
    for p_i in range(app_obj.appearanceProperties.count):
        prop = app_obj.appearanceProperties.item(p_i)
        if isinstance(prop, core.ColorProperty) and not prop.isReadOnly:
            try:
                if prop.id in ("opaque_albedo", "surface_albedo") or prop.name == "Color":
                    prop.value = core.Color.create(r, g, b, 255)
                else:
                    prop.value = core.Color.create(255, 255, 255, 255)
            except Exception:
                pass
    return app_obj


def set_occ_transform(occ: fusion.Occurrence, mat: core.Matrix3D):
    if occ is not None:
        occ.isGrounded = False
        occ.transform2 = mat


# Hide any previous upgrade component if re-running (NEVER call deleteMe() in ParametricDesignType; hiding is 0 ms!)
for i in range(root.occurrences.count - 1, -1, -1):
    occ_i = root.occurrences.item(i)
    if "Variable_Intake_And_Asymmetrical_Funnel" in occ_i.name or "Variable_Intake_And_Asymmetrical_Funnel" in occ_i.component.name:
        occ_i.isLightBulbOn = False

while root.asBuiltJoints.count > 0:
    root.asBuiltJoints.item(0).deleteMe()
while root.joints.count > 0:
    root.joints.item(0).deleteMe()

rbc = root.occurrences.itemByName("robot_base_core:1")
s1 = rbc.childOccurrences.itemByName("strafer_chassis_stage:1")
s2 = rbc.childOccurrences.itemByName("gatekeeper_intake_stage:1")
s3 = rbc.childOccurrences.itemByName("sprung_magazine_stage:1")
tur = root.occurrences.itemByName("turret_launcher_assembly:1")
lip = root.occurrences.itemByName("g407_lip_gate_assembly:1")

opod_left = s1.childOccurrences.itemByName("official_gobilda_4bar_odometry_pod:1")
mat_odom_rx = mat_pos_mm(-95.0, -110.0, 34.0)
mat_odom_cy = mat_mul(mat_pos_mm(-128.8, 14.0, 34.0), mat_rot_deg((0, 0, 1), 90.0))
opod_rx_occ = None
opod_cy_occ = None

u10_rear_occ = s1.childOccurrences.itemByName("gobilda_1120_10hole_rear:1")
# Shift +168.0 mm (+16.8 cm = 7 * 24.0 mm goBILDA grid pitch!) from X = -192.0 mm (Hole 1) to X = -24.0 mm (Hole 8)
# so EVERY 14mm center hole and M4 pattern hole on the Middle Cross U-Channel aligns 100% with Hole 8 on the 17-Hole Side Rails!
mat_u10_mid = u10_rear_occ.transform2.copy()
mat_u10_mid.translation = core.Vector3D.create(
    mat_u10_mid.translation.x + 16.8,
    mat_u10_mid.translation.y,
    mat_u10_mid.translation.z,
)
u10_mid_occ = None


def apply_cots_visibility_and_transforms():
    # Keep legacy hidden manipulators & clutter hidden (including g407_lip_gate_assembly:1):
    for hidden_top in ("stinger_flower_capper_arm:1", "flower_depollinator_fork:1", "g407_lip_gate_assembly:1"):
        ht_occ = root.occurrences.itemByName(hidden_top)
        if ht_occ:
            ht_occ.isLightBulbOn = False

    hide_s1 = [
        "front_funnel_bumper_left:1",
        "front_funnel_bumper_right:1",
        "cnc_turret_mounting_deck_plate:1",
        "gobilda_1121_10hole_front_portal_arch:1",
        "gobilda_1120_10hole_mid:1",
        "gobilda_1120_10hole_mid:2",
        "rex_axle_shaft_gobilda_mecanum_96mm_front_left:1",
        "rex_axle_shaft_gobilda_mecanum_96mm_front_left:2",
        "rex_axle_shaft_gobilda_mecanum_96mm_front_left:3",
        "rex_axle_shaft_gobilda_mecanum_96mm_front_left:4",
    ]
    for name in hide_s1:
        occ = s1.childOccurrences.itemByName(name)
        if occ:
            occ.isLightBulbOn = False

    hide_s2 = [
        "intake_rex_shaft_8mm:1",
        "intake_bearing_support_arm_left:1",
        "intake_bearing_support_arm_right:1",
        "continuous_chute_bottom_ramp:1",
        "chute_rear_elevator_wall:1",
        "ramp_front_crossbar_mount:1",
        "ramp_mid_stanchion_cross_mount:1",
        "elevator_rear_bridge_mount:1",
        "m7_intake_motor_mount_bracket:1",
        "intake_drive_pulley_1:1",
        "intake_drive_pulley_1:2",
        "intake_drive_timing_belt:1",
        "servo_s1_g407_lip_gate:1",
        "servo_s2_depollinator_drive:1",
        "star_roller_sleeve:1",
        "star_roller_sleeve:3",
    ]
    for name in hide_s2:
        occ = s2.childOccurrences.itemByName(name)
        if occ:
            occ.isLightBulbOn = False

    hide_s3 = [
        "magazine_shroud_left:1",
        "magazine_shroud_right:1",
        "magazine_structural_gusset_left:1",
        "magazine_structural_gusset_left:2",
        "magazine_structural_gusset_left:3",
        "magazine_structural_gusset_left:4",
        "gecko_rex_shaft_1:1",
        "gecko_rex_shaft_1:2",
        "gecko_rex_shaft_1:3",
        "gecko_flanged_bearing_1_-1:1",
        "gecko_flanged_bearing_1_-1:2",
        "gecko_flanged_bearing_1_-1:3",
        "gecko_flanged_bearing_1_-1:4",
        "gecko_flanged_bearing_1_-1:5",
        "gecko_flanged_bearing_1_-1:6",
        "intake_drive_pulley_1:3",
        "intake_drive_pulley_1:4",
        "intake_drive_pulley_1:5",
        "intake_drive_pulley_1:6",
        "intake_drive_pulley_1:7",
        "intake_drive_pulley_1:8",
        "magazine_motor_drive_timing_belt:1",
        "magazine_gecko_couple_timing_belt:1",
        "magazine_elevator_couple_timing_belt:1",
        "staged_pollen_1_entering_mouth:1",
    ]
    for name in hide_s3:
        occ = s3.childOccurrences.itemByName(name)
        if occ:
            occ.isLightBulbOn = False

    hide_tur = [
        "variable_pitch_exit_hood:1",
        "limelight_3a_vision:1",
        "turret_bearing_ring_85mm:1",
        "turret_top_structural_bridge:1",
        "turret_side_plate_left:1",
        "turret_side_plate_right:1",
        "turret_motor_mount_bracket_left:1",
        "turret_motor_mount_bracket_right:1",
        "shooter_main_rex_shaft:1",
        "shooter_counter_rex_shaft:1",
        "shooter_cam_pivot_rex_shaft:1",
        "shooter_cam_arm_left:1",
        "shooter_cam_arm_left:2",
        "intake_drive_pulley_1:9",
        "intake_drive_pulley_1:10",
        "intake_drive_pulley_1:11",
        "intake_drive_pulley_1:12",
        "shooter_left_drive_timing_belt:1",
        "shooter_right_drive_timing_belt:1",
        "servo_s4_turret_azimuth:1",
        "servo_s5_cam_gap_selector:1",
    ]
    for name in hide_tur:
        occ = tur.childOccurrences.itemByName(name)
        if occ:
            occ.isLightBulbOn = False

    # Position the 2 cloned full-detail 95-body goBILDA 4-Bar Odometry Pods & cloned Center 10-Hole U-Channel (on-grid at X = -24.0 mm Hole 8!):
    for pod_occ, pod_mat in ((opod_rx_occ, mat_odom_rx), (opod_cy_occ, mat_odom_cy), (u10_mid_occ, mat_u10_mid)):
        set_occ_transform(pod_occ, pod_mat)

    batt_ch = s1.childOccurrences.itemByName("official_gobilda_12v_battery_14_0014:1")
    if batt_ch:
        m_batt = batt_ch.transform2.copy()
        m_batt.translation = vec_cm(-148.0, 73.9, 119.9)
        set_occ_transform(batt_ch, m_batt)

    # Drivetrain motors M1..M4 shifted to ±16.265 mm:
    for m_name, new_tx_mm, new_ty_mm in (
        ("gobilda_5203_motor_m1_drive:1", 16.265, 118.829),
        ("gobilda_5203_motor_m2_drive:1", -16.265, 193.171),
        ("gobilda_5203_motor_m3_drive:1", 16.265, -193.171),
        ("gobilda_5203_motor_m4_drive:1", -16.265, -118.829),
    ):
        m_occ = s1.childOccurrences.itemByName(m_name)
        m_mat = m_occ.transform2.copy()
        m_mat.translation = vec_cm(new_tx_mm, new_ty_mm, 56.078)
        set_occ_transform(m_occ, m_mat)

    # Raise 'official_gobilda_pinpoint_v2:1' to Z = 76.0 mm (Z in [68.0, 86.0 mm]) right above the Right X-Pod on the Right Rail Cradle:
    pin_occ = s1.childOccurrences.itemByName("official_gobilda_pinpoint_v2:1")
    if pin_occ:
        set_occ_transform(pin_occ, mat_pos_mm(-95.0, -92.0, 76.0))

    # Stage 2 kept Star-Paddle Roller:
    # - star_roller_sleeve:2 on the Floating Front Shaft at (212.0, 0.0, 117.0) (star_roller_sleeve:1 and :3 on the
    #   removed lower Z = 96.0 mm bar are hidden so the throat has 115.6 mm of unobstructed height for 91.95 mm Nectar balls!):
    set_occ_transform(
        s2.childOccurrences.itemByName("star_roller_sleeve:2"),
        mat_mul(mat_pos_mm(212.0, 0.0, 117.0), mat_rot_deg((1, 0, 0), 90.0)),
    )

    # Stage 3 kept Gecko Wheels & Staged 71.12 mm Pollen Balls:
    # Positioned along the true upward normal (+sin 19 deg, 0, +cos 19 deg) of the continuous 19 deg ramp so:
    # 1) Every 71.12 mm Pollen ball sits 100% ABOVE the top surface of the ramp (ZERO bottom-of-ramp clipping even with
    #    sphere cutters removed!),
    # 2) The 3 green 72 mm Gecko wheels sit at a full 71.5 mm perpendicular gap above the ramp (touching the tops of the
    #    Pollen balls at rest and sliding +20 mm up the sprung magazine compliance slots to 91.5 mm for 91.95 mm Nectar balls!).
    set_occ_transform(s3.childOccurrences.itemByName("gobilda_gecko_wheel_72mm_1:1"), mat_pos_mm(114.0, 0.0, 151.0))
    set_occ_transform(s3.childOccurrences.itemByName("gobilda_gecko_wheel_72mm_1:2"), mat_pos_mm(34.0, 0.0, 180.0))
    set_occ_transform(s3.childOccurrences.itemByName("gobilda_gecko_wheel_72mm_1:3"), mat_pos_mm(-12.0, 0.0, 240.0))

    p5 = s3.childOccurrences.itemByName("staged_pollen_1_entering_mouth:5")
    if p5 is not None:
        s3.childOccurrences.itemByName("staged_pollen_1_entering_mouth:1").isLightBulbOn = False
        set_occ_transform(s3.childOccurrences.itemByName("staged_pollen_1_entering_mouth:2"), mat_pos_mm(106.0, 0.0, 79.5))
        set_occ_transform(s3.childOccurrences.itemByName("staged_pollen_1_entering_mouth:3"), mat_pos_mm(18.0, 0.0, 109.2))
        set_occ_transform(s3.childOccurrences.itemByName("staged_pollen_1_entering_mouth:4"), mat_pos_mm(-73.5, 0.0, 202.0))
        set_occ_transform(p5, mat_pos_mm(-51.75, 0.0, 324.0))
    else:
        p1 = s3.childOccurrences.itemByName("staged_pollen_1_entering_mouth:1")
        if p1 is not None:
            p1.isLightBulbOn = True
            set_occ_transform(p1, mat_pos_mm(106.0, 0.0, 79.5))
        set_occ_transform(s3.childOccurrences.itemByName("staged_pollen_1_entering_mouth:2"), mat_pos_mm(18.0, 0.0, 109.2))
        set_occ_transform(s3.childOccurrences.itemByName("staged_pollen_1_entering_mouth:3"), mat_pos_mm(-73.5, 0.0, 202.0))
        set_occ_transform(s3.childOccurrences.itemByName("staged_pollen_1_entering_mouth:4"), mat_pos_mm(-51.75, 0.0, 324.0))

    # Position Turret Shooter Flywheels at 155.5 mm center-to-center (71.50 mm surface-to-surface nip gap:
    # Main 96mm Flywheel at X = +32.0, Z = 324.0 [rear face X = -16.0]; Rear 72mm Counter-Wheel at X = -123.5, Z = 324.0
    # [front face X = -87.5]) so the 71.12 mm Pollen ball at (-51.75, 0.0, 324.0) touches BOTH wheels at the same time
    # (0.19 mm tangent contact), while the Coaxial Swing-Arms + Curved Concentric Arc Slots centered at Motor M6 (-108.0, 266.0)
    # allow the Counter-Wheel to swing rearward/downward along the arc to (-143.7, 314.3) (92.0 mm nip gap) for 91.95 mm Nectar balls!
    for t_name, tx_mm, ty_mm, tz_mm in (
        ("gobilda_rhino_wheel_96mm:1", 32.0, 0.0, 324.0),
        ("gobilda_steel_flywheel_82mm_left:1", 32.0, 28.0, 324.0),
        ("gobilda_steel_flywheel_82mm_right:1", 32.0, -28.0, 324.0),
        ("gobilda_counter_wheel_72mm_topspin:1", -123.5, 0.0, 324.0),
        ("turret_top_structural_bridge:1", -136.0, 0.0, 370.0),
        ("gobilda_5203_motor_m5_shooter_master:1", -61.17, 191.74, 277.08),
        ("gobilda_5203_motor_m6_shooter_follower:1", -70.83, -191.74, 277.08),
    ):
        t_occ = tur.childOccurrences.itemByName(t_name)
        if t_occ:
            t_mat = t_occ.transform2.copy()
            t_mat.translation = vec_cm(tx_mm, ty_mm, tz_mm)
            set_occ_transform(t_occ, t_mat)

    # Rotate Right Hood Pitch Servo ('servo_s6_pitch_adjust:1') -90 deg about X at (-86.0, -84.0, 382.0)
    # so its white H25T Spline sits at (-96.0, -71.07, 382.0) pointing horizontally INWARD (+Y) coaxially
    # onto the Hood Hinge Shaft (-96.0, -66.5, 382.0)!
    s6_occ = tur.childOccurrences.itemByName("servo_s6_pitch_adjust:1")
    if s6_occ:
        m_s6 = mat_rot_deg((1, 0, 0), -90.0)
        m_s6.translation = vec_cm(-86.0, -84.0, 382.0)
        set_occ_transform(s6_occ, m_s6)

    # Rotate Left Indexer Gate Servo ('servo_s3_magazine_stopper:1') +90 deg about X at (-108.0, +84.0, 252.0)
    # so its white H25T Spline sits at (-118.0, +71.07, 252.0) pointing horizontally INWARD (-Y) coaxially
    # onto the Indexer Gate Shaft (-118.0, +66.5, 252.0)!
    for s3_sname in ("servo_s3_indexer_gate:1", "servo_s3_magazine_stopper:1"):
        s3_servo = s3.childOccurrences.itemByName(s3_sname)
        if s3_servo:
            m_s3 = mat_rot_deg((1, 0, 0), 90.0)
            m_s3.translation = vec_cm(-108.0, 84.0, 252.0)
            set_occ_transform(s3_servo, m_s3)


def apply_cots_visibility_only():
    for hidden_top in ("stinger_flower_capper_arm:1", "flower_depollinator_fork:1", "g407_lip_gate_assembly:1"):
        ht_occ = root.occurrences.itemByName(hidden_top)
        if ht_occ:
            ht_occ.isLightBulbOn = False

    for name in (
        "front_funnel_bumper_left:1",
        "front_funnel_bumper_right:1",
        "cnc_turret_mounting_deck_plate:1",
        "gobilda_1121_10hole_front_portal_arch:1",
        "gobilda_1120_10hole_mid:1",
        "gobilda_1120_10hole_mid:2",
        "rex_axle_shaft_gobilda_mecanum_96mm_front_left:1",
        "rex_axle_shaft_gobilda_mecanum_96mm_front_left:2",
        "rex_axle_shaft_gobilda_mecanum_96mm_front_left:3",
        "rex_axle_shaft_gobilda_mecanum_96mm_front_left:4",
    ):
        occ = s1.childOccurrences.itemByName(name)
        if occ:
            occ.isLightBulbOn = False

    for name in (
        "intake_rex_shaft_8mm:1",
        "intake_bearing_support_arm_left:1",
        "intake_bearing_support_arm_right:1",
        "continuous_chute_bottom_ramp:1",
        "chute_rear_elevator_wall:1",
        "ramp_front_crossbar_mount:1",
        "ramp_mid_stanchion_cross_mount:1",
        "elevator_rear_bridge_mount:1",
        "m7_intake_motor_mount_bracket:1",
        "intake_drive_pulley_1:1",
        "intake_drive_pulley_1:2",
        "intake_drive_timing_belt:1",
        "servo_s1_g407_lip_gate:1",
        "servo_s2_depollinator_drive:1",
        "star_roller_sleeve:1",
        "star_roller_sleeve:3",
    ):
        occ = s2.childOccurrences.itemByName(name)
        if occ:
            occ.isLightBulbOn = False

    for name in (
        "magazine_shroud_left:1",
        "magazine_shroud_right:1",
        "magazine_structural_gusset_left:1",
        "magazine_structural_gusset_left:2",
        "magazine_structural_gusset_left:3",
        "magazine_structural_gusset_left:4",
        "gecko_rex_shaft_1:1",
        "gecko_rex_shaft_1:2",
        "gecko_rex_shaft_1:3",
        "gecko_flanged_bearing_1_-1:1",
        "gecko_flanged_bearing_1_-1:2",
        "gecko_flanged_bearing_1_-1:3",
        "gecko_flanged_bearing_1_-1:4",
        "gecko_flanged_bearing_1_-1:5",
        "gecko_flanged_bearing_1_-1:6",
        "intake_drive_pulley_1:3",
        "intake_drive_pulley_1:4",
        "intake_drive_pulley_1:5",
        "intake_drive_pulley_1:6",
        "intake_drive_pulley_1:7",
        "intake_drive_pulley_1:8",
        "magazine_motor_drive_timing_belt:1",
        "magazine_gecko_couple_timing_belt:1",
        "magazine_elevator_couple_timing_belt:1",
    ):
        occ = s3.childOccurrences.itemByName(name)
        if occ:
            occ.isLightBulbOn = False

    if s3.childOccurrences.itemByName("staged_pollen_1_entering_mouth:5") is not None:
        occ_p1 = s3.childOccurrences.itemByName("staged_pollen_1_entering_mouth:1")
        if occ_p1:
            occ_p1.isLightBulbOn = False
    else:
        occ_p1 = s3.childOccurrences.itemByName("staged_pollen_1_entering_mouth:1")
        if occ_p1:
            occ_p1.isLightBulbOn = True

    for name in (
        "variable_pitch_exit_hood:1",
        "limelight_3a_vision:1",
        "turret_bearing_ring_85mm:1",
        "turret_top_structural_bridge:1",
        "turret_side_plate_left:1",
        "turret_side_plate_right:1",
        "turret_motor_mount_bracket_left:1",
        "turret_motor_mount_bracket_right:1",
        "shooter_main_rex_shaft:1",
        "shooter_counter_rex_shaft:1",
        "shooter_cam_pivot_rex_shaft:1",
        "shooter_cam_arm_left:1",
        "shooter_cam_arm_left:2",
        "intake_drive_pulley_1:9",
        "intake_drive_pulley_1:10",
        "intake_drive_pulley_1:11",
        "intake_drive_pulley_1:12",
        "shooter_left_drive_timing_belt:1",
        "shooter_right_drive_timing_belt:1",
        "servo_s4_turret_azimuth:1",
        "servo_s5_cam_gap_selector:1",
    ):
        occ = tur.childOccurrences.itemByName(name)
        if occ:
            occ.isLightBulbOn = False


# Set visibility flags ONLY (no transform2 or snapshots.add() before bf.startEdit()!):
apply_cots_visibility_only()

# Collect ONE fast analytic cutter per visible COTS component occurrence (inflated by +0.15 mm around its exact body AABB,
# with sphere cutters for 'staged_pollen_*') so boolean trimming takes <0.5 ms per body and never touches NURBS surfaces!
scene_solids = []


def add_analytic_box_cutter(xmin, ymin, zmin, xmax, ymax, zmax, pad: float = 0.15):
    bx = (xmin + xmax) * 0.5
    by = (ymin + ymax) * 0.5
    bz = (zmin + zmax) * 0.5
    sx = (xmax - xmin) + 2.0 * pad
    sy = (ymax - ymin) + 2.0 * pad
    sz = (zmax - zmin) + 2.0 * pad
    if sx > 0.2 and sy > 0.2 and sz > 0.2:
        b = make_box_mm(bx, by, bz, sx, sy, sz)
        scene_solids.append((b, (xmin - pad, ymin - pad, zmin - pad, xmax + pad, ymax + pad, zmax + pad)))


def get_occ_true_aabb_mm(occ: fusion.Occurrence):
    xs0, ys0, zs0, xs1, ys1, zs1 = [], [], [], [], [], []

    def walk(o):
        if not o.isLightBulbOn:
            return
        for k in range(o.bRepBodies.count):
            bd = o.bRepBodies.item(k)
            if bd.isLightBulbOn and bd.isSolid:
                bb = bd.boundingBox
                xs0.append(bb.minPoint.x * 10.0)
                ys0.append(bb.minPoint.y * 10.0)
                zs0.append(bb.minPoint.z * 10.0)
                xs1.append(bb.maxPoint.x * 10.0)
                ys1.append(bb.maxPoint.y * 10.0)
                zs1.append(bb.maxPoint.z * 10.0)
        for k in range(o.childOccurrences.count):
            walk(o.childOccurrences.item(k))

    walk(occ)
    if not xs0:
        return None
    return (min(xs0), min(ys0), min(zs0), max(xs1), max(ys1), max(zs1))


for stage_occ in (s1, s2, s3, tur, lip):
    if not stage_occ or not stage_occ.isLightBulbOn:
        continue
    for i in range(stage_occ.childOccurrences.count):
        ch = stage_occ.childOccurrences.item(i)
        if not ch.isLightBulbOn:
            continue
        nm = ch.name
        if nm == "official_gobilda_pinpoint_v2:1":
            # Use the final raised pose of Pinpoint v2 at (-95.0, -92.0, 76.0) -> X in [-115, -75], Y in [-113.3, -70.7], Z in [68, 86]:
            add_analytic_box_cutter(-115.0, -113.3, 68.0, -75.0, -70.7, 86.0, pad=0.15)
        elif nm == "gobilda_gecko_wheel_72mm_1:1":
            add_analytic_box_cutter(77.9, -16.1, 114.9, 150.1, 16.1, 187.1, pad=0.15)
        elif nm == "gobilda_gecko_wheel_72mm_1:2":
            add_analytic_box_cutter(-2.1, -16.1, 143.9, 70.1, 16.1, 216.1, pad=0.15)
        elif nm == "gobilda_gecko_wheel_72mm_1:3":
            add_analytic_box_cutter(-48.1, -16.1, 203.9, 24.1, 16.1, 276.1, pad=0.15)
        elif nm in ("servo_s3_indexer_gate:1", "servo_s3_magazine_stopper:1"):
            add_analytic_box_cutter(-135.2, 66.9, 241.8, -80.8, 111.3, 262.1, pad=0.15)
        elif nm == "gobilda_5203_motor_m5_shooter_master:1":
            # Motor body is at Y in [90.0, 206.6 mm]; only the R = 11.0 mm boss/output shaft is at Y in [66.4, 90.0 mm]:
            add_analytic_box_cutter(-50.2, 89.9, 239.7, 2.3, 206.6, 292.2, pad=0.15)
            s_m5 = make_cyl_mm(-24.0, 66.2, 266.0, -24.0, 90.2, 266.0, 11.5)
            scene_solids.append((s_m5, (-35.6, 66.2, 254.4, -12.4, 90.2, 277.6)))
        elif nm == "gobilda_5203_motor_m6_shooter_follower:1":
            add_analytic_box_cutter(-134.3, -206.6, 239.7, -81.8, -89.9, 292.2, pad=0.15)
            s_m6 = make_cyl_mm(-108.0, -90.2, 266.0, -108.0, -66.2, 266.0, 11.5)
            scene_solids.append((s_m6, (-119.6, -90.2, 254.4, -96.4, -66.2, 277.6)))
        elif nm == "gobilda_rhino_wheel_96mm:1":
            add_analytic_box_cutter(-16.1, -18.1, 275.9, 80.1, 18.1, 372.1, pad=0.15)
        elif nm == "gobilda_steel_flywheel_82mm_left:1":
            add_analytic_box_cutter(-9.1, 22.9, 282.9, 73.1, 33.1, 365.1, pad=0.15)
        elif nm == "gobilda_steel_flywheel_82mm_right:1":
            add_analytic_box_cutter(-9.1, -33.1, 282.9, 73.1, -22.9, 365.1, pad=0.15)
        elif nm == "gobilda_counter_wheel_72mm_topspin:1":
            add_analytic_box_cutter(-159.6, -18.1, 287.9, -87.4, 18.1, 360.1, pad=0.15)
        elif nm == "turret_top_structural_bridge:1":
            add_analytic_box_cutter(-150.1, -59.1, 365.9, -121.9, 59.1, 374.1, pad=0.15)
        elif nm == "servo_s6_pitch_adjust:1":
            add_analytic_box_cutter(-113.2, -111.3, 371.9, -58.8, -66.9, 392.2, pad=0.15)
        elif nm.startswith("staged_pollen_"):
            continue
        else:
            aabb = get_occ_true_aabb_mm(ch)
            if aabb:
                add_analytic_box_cutter(*aabb, pad=0.15)

# (Removed staged_pollen sphere cutters from scene_solids so balls NEVER punch holes through the bottom of the ramp!
#  Instead, every ball sits cleanly ABOVE the ramp surface and is verified by Step 4's B-Rep intersection audit!)

# Also add analytic box cutters for the 2 cloned 95-body goBILDA Odometry Pods and the cloned Center 10-Hole U-Channel (at X in [-48.0, 0.0]):
add_analytic_box_cutter(-153.7, -114.0, 3.8, -64.3, -71.0, 64.2, pad=0.15)
add_analytic_box_cutter(-167.8, -44.7, 3.8, -124.8, 44.7, 64.2, pad=0.15)
add_analytic_box_cutter(-48.0, -132.0, 24.0, 0.0, 132.0, 72.0, pad=0.15)

# ==============================================================================
# STEP 1: CREATE ONE SINGLE BASEFEATURE COMPONENT DIRECTLY UNDER ROOT
# ==============================================================================
up_occ = root.occurrences.addNewComponent(core.Matrix3D.create())
up_comp = up_occ.component
up_comp.name = "Variable_Intake_And_Asymmetrical_Funnel"

bf = up_comp.features.baseFeatures.add()
bf.startEdit()

body_colors = []  # list of (body_name, app_key)
added_custom_solids = []  # list of (temp_body_copy, sbb)


def get_aabb_mm(solid: fusion.BRepBody):
    bb = solid.boundingBox
    return (
        bb.minPoint.x * 10.0,
        bb.minPoint.y * 10.0,
        bb.minPoint.z * 10.0,
        bb.maxPoint.x * 10.0,
        bb.maxPoint.y * 10.0,
        bb.maxPoint.z * 10.0,
    )


def aabb_overlaps(bb1, bb2, tol: float = 0.03) -> bool:
    return (
        bb1[0] < bb2[3] - tol
        and bb1[3] > bb2[0] + tol
        and bb1[1] < bb2[4] - tol
        and bb1[4] > bb2[1] + tol
        and bb1[2] < bb2[5] - tol
        and bb1[5] > bb2[2] + tol
    )


def add_solid(solid: fusion.BRepBody, name: str, color_key: str, trim_scene: bool = False, trim_prev: bool = False, register_cutter: bool = True):
    sbb = get_aabb_mm(solid)
    if trim_scene:
        for sb, sc_bb in scene_solids:
            if aabb_overlaps(sbb, sc_bb):
                try:
                    tb.booleanOperation(solid, tb.copy(sb), fusion.BooleanTypes.DifferenceBooleanType)
                except Exception:
                    pass
        sbb = get_aabb_mm(solid)
    if trim_prev:
        for prev_b, prev_bb in added_custom_solids:
            if aabb_overlaps(sbb, prev_bb):
                try:
                    tb.booleanOperation(solid, tb.copy(prev_b), fusion.BooleanTypes.DifferenceBooleanType)
                except Exception:
                    pass
        sbb = get_aabb_mm(solid)
    if register_cutter:
        added_custom_solids.append((tb.copy(solid), sbb))
    b = up_comp.bRepBodies.add(solid, bf)
    b.name = name
    body_colors.append((name, color_key))
    return b


# ------------------------------------------------------------------------------
# 1A. LARGE 91.95 mm NECTAR BALL ENTERING THE VARIABLE FLOATING INTAKE MOUTH
#     (Placed at X = +198.0, Y = 0.0, Z = 48.0 mm so it sits tangent to BOTH the
#      floating front compliant wheels and the lowered secondary throat star-paddles!):
# ------------------------------------------------------------------------------
n_sphere = tb.createSphere(pt_cm(192.0, 0.0, 48.0), 4.5975)  # D = 91.95 mm (R = 45.975 mm)
add_solid(n_sphere, "staged_nectar_ball_92mm_entering_mouth", "nectar_orange", trim_scene=False, trim_prev=False)

# ------------------------------------------------------------------------------
# 1B. ASYMMETRICAL WIDE-AREA INTAKE FUNNEL (360 mm outer capture width -> 108 mm inner throat):
# ------------------------------------------------------------------------------
left_cap = make_box_mm(228.0, 156.0, 48.0, 23.0, 48.0, 52.0)
left_wing_len = math.hypot(63.0, 80.0)
left_wing_ang = math.degrees(math.atan2(80.0, 63.0))
left_wing = make_box_mm(0.0, 0.0, 0.0, left_wing_len, 8.0, 48.0)
tb.transform(left_wing, mat_mul(mat_pos_mm(203.5, 96.0, 46.0), mat_rot_deg((0, 0, 1), left_wing_ang)))
left_scoop = make_box_mm(0.0, 0.0, 0.0, left_wing_len * 0.85, 14.0, 6.0)
tb.transform(left_scoop, mat_mul(mat_pos_mm(202.0, 92.0, 25.0), mat_rot_deg((0, 0, 1), left_wing_ang)))
tb.booleanOperation(left_cap, left_wing, fusion.BooleanTypes.UnionBooleanType)
tb.booleanOperation(left_cap, left_scoop, fusion.BooleanTypes.UnionBooleanType)
add_solid(left_cap, "asymmetrical_funnel_wing_left_long", "emerald", trim_scene=True, trim_prev=True)

right_cap = make_box_mm(220.25, -156.0, 48.0, 7.5, 48.0, 52.0)
right_wing_len = math.hypot(48.0, 78.0)
right_wing_ang = -math.degrees(math.atan2(78.0, 48.0))
right_wing = make_box_mm(0.0, 0.0, 0.0, right_wing_len, 8.0, 44.0)
tb.transform(right_wing, mat_mul(mat_pos_mm(198.0, -97.0, 44.0), mat_rot_deg((0, 0, 1), right_wing_ang)))
right_scoop = make_box_mm(0.0, 0.0, 0.0, right_wing_len * 0.80, 12.0, 6.0)
tb.transform(right_scoop, mat_mul(mat_pos_mm(197.0, -94.0, 25.0), mat_rot_deg((0, 0, 1), right_wing_ang)))
tb.booleanOperation(right_cap, right_wing, fusion.BooleanTypes.UnionBooleanType)
tb.booleanOperation(right_cap, right_scoop, fusion.BooleanTypes.UnionBooleanType)
add_solid(right_cap, "asymmetrical_funnel_wing_right_short", "teal_dark", trim_scene=True, trim_prev=True)

# ------------------------------------------------------------------------------
# 1C. FAST, VARIABLE-SIZE FLOATING INTAKE (Black Box FTC YouTube Short Architecture):
#     - Upper Coaxial Pivot Shaft at Hole 3 (X = +156.0 mm, Z = 144.0 mm — giving 115.6 mm
#       of unobstructed vertical throat height above the Z = 24.4 mm ramp for 91.95 mm Nectar balls!).
#     - Floating Front Hex Shaft at (X = +212.0 mm, Z = 117.0 mm) carrying 4x Spoked Compliant
#       Wheels + Central Star-Paddle Roller ('star_roller_sleeve:2').
#     - Lower Z = 96.0 mm bar and outboard star wheels removed so nothing blocks the 91.95 mm Nectar ball!
# ------------------------------------------------------------------------------
up_pivot_shaft = make_cyl_mm(156.0, -148.0, 144.0, 156.0, 148.0, 144.0, 3.92)
add_solid(up_pivot_shaft, "variable_intake_upper_pivot_rex_shaft", "silver")

# Floating Front Hex Shaft at (X = +212.0 mm, Z = 117.0 mm):
float_front_shaft = make_cyl_mm(212.0, -108.0, 117.0, 212.0, 118.0, 117.0, 3.92)
add_solid(float_front_shaft, "variable_intake_floating_front_rex_shaft", "silver")

arm_dx = 212.0 - 156.0
arm_dz = 117.0 - 144.0
arm_len = math.hypot(arm_dx, arm_dz)
arm_pitch = -math.degrees(math.atan2(arm_dz, arm_dx))

for side_y, sname in ((104.0, "left"), (-104.0, "right")):
    bar = make_box_mm(0.0, 0.0, 0.0, arm_len, 8.0, 18.0)
    tb.transform(bar, mat_mul(mat_pos_mm(184.0, side_y, 130.5), mat_rot_deg((0, 1, 0), arm_pitch)))
    hub_up = make_cyl_mm(156.0, side_y - 5.0, 144.0, 156.0, side_y + 5.0, 144.0, 11.5)
    hub_dn = make_cyl_mm(212.0, side_y - 5.0, 117.0, 212.0, side_y + 5.0, 117.0, 11.5)
    tb.booleanOperation(bar, hub_up, fusion.BooleanTypes.UnionBooleanType)
    tb.booleanOperation(bar, hub_dn, fusion.BooleanTypes.UnionBooleanType)
    bore_up = make_cyl_mm(156.0, -140.0, 144.0, 156.0, 140.0, 144.0, 4.05)
    bore_dn = make_cyl_mm(212.0, -140.0, 117.0, 212.0, 140.0, 117.0, 4.05)
    tb.booleanOperation(bar, bore_up, fusion.BooleanTypes.DifferenceBooleanType)
    tb.booleanOperation(bar, bore_dn, fusion.BooleanTypes.DifferenceBooleanType)
    add_solid(bar, f"floating_intake_swing_arm_{sname}", "silver")

    sign_y = 1.0 if side_y > 0 else -1.0
    boss = make_cyl_mm(156.0, sign_y * 124.0, 144.0, 156.0, sign_y * 131.8, 144.0, 10.5)
    tb.booleanOperation(boss, make_cyl_mm(156.0, -140.0, 144.0, 156.0, 140.0, 144.0, 4.05), fusion.BooleanTypes.DifferenceBooleanType)
    add_solid(boss, f"intake_upright_bearing_upper_{sname}", "mount_slate")

for idx, wy in enumerate((-72.0, -38.0, 38.0, 72.0), start=1):
    cw = make_spoked_compliant_wheel_mm(212.0, wy, 117.0, r_out=23.5, r_hub=10.5, width=20.0)
    add_solid(cw, f"floating_spoked_compliant_wheel_{idx}", "compliant_teal")

# Coaxial Belt-on-Pivot HTD-5M Pulleys & Timing Belts:
# - Stage 1 (Y = +41.5 mm): Motor M7 (104.0, 98.0) -> Upper Coaxial Pivot Shaft (156.0, 144.0)
# - Stage 2 Coaxial Floating-Arm (Y = +113.5 mm): Upper Coaxial Pivot Shaft (156.0, 144.0) -> Floating Front Shaft (212.0, 117.0)
add_solid(make_pulley_world_mm(104.0, 41.5, 98.0, width=7.0), "intake_m7_drive_pulley", "pulley_alu")
add_solid(make_pulley_world_mm(156.0, 41.5, 144.0, width=7.0), "intake_upper_pivot_driven_pulley", "pulley_alu")
add_solid(make_pulley_world_mm(156.0, 113.5, 144.0, r_drum=11.0, width=8.0), "intake_coaxial_pivot_driver_pulley", "pulley_alu")
add_solid(make_pulley_world_mm(212.0, 113.5, 117.0, r_drum=11.0, width=8.0), "intake_floating_front_driven_pulley", "pulley_alu")

add_solid(
    make_belt_loop_mm(104.0, 98.0, 156.0, 144.0, y_mm=41.5, r_in_mm=12.15, r_out_mm=13.8, width_mm=4.2),
    "intake_stage1_primary_timing_belt",
    "belt_black",
)
add_solid(
    make_belt_loop_mm(156.0, 144.0, 212.0, 117.0, y_mm=113.5, r_in_mm=11.15, r_out_mm=12.8, width_mm=5.0),
    "intake_stage2_coaxial_floating_arm_belt",
    "belt_black",
)

# Full-Rail Motor Clamping Saddles for Intake Motor M7 (Left) & Magazine Motor M8 (Right):
for side_sign, m_lbl in ((1.0, "m7_intake_motor_saddle_left"), (-1.0, "m8_magazine_motor_saddle_right")):
    msaddle = make_box_mm(106.0, side_sign * 118.0, 92.0, 54.0, 116.0, 42.0)
    add_solid(msaddle, m_lbl, "mount_slate", trim_scene=True, trim_prev=True)

# ------------------------------------------------------------------------------
# 1D. UNIFIED ONE-PIECE CONTINUOUS CURVED J-CHUTE & 4-STATION STRUCTURAL CRADLES
#     ('unified_continuous_j_chute' — ONE single solid body with a 100% UNBROKEN
#      lower intake ramp from the front bullnose lip (X = +150.0, Z = 24.38) up
#      to the first elbow (X = -15.0, Z = 79.58) — ZERO split or step at the entry!):
# ------------------------------------------------------------------------------
# Segment 1 & 2 Unified: One Continuous Unbroken 176 mm Lower Inclined Chute Floor (18.5 deg, spanning
# smoothly from the front scoop entry lip at X = +150.0, Z = 24.38 mm all the way up past Ball 1 (108, 71)
# and Ball 2 (14, 104.2) to the first elbow at X = -15.0, Z = 79.58 mm with ZERO split!):
j_ramp = make_box_mm(0.0, 0.0, 0.0, 176.0, 96.0, 4.5)
tb.transform(j_ramp, mat_mul(mat_pos_mm(67.5, 0.0, 51.98), mat_rot_deg((0, 1, 0), 18.5)))
j_front_lip = make_cyl_mm(150.9, -48.0, 24.08, 150.9, 48.0, 24.08, 2.25)
tb.booleanOperation(j_ramp, j_front_lip, fusion.BooleanTypes.UnionBooleanType)

# Segment 3: Curved J-Elbow Transition Ramp (48.0 deg, curving upward from Ball 2 (18, 109.2) to Ball 3 (-78, 207)
# directly above the Stock Full-Height 1120 Center Crossbar so balls CANNOT stay at the bottom and are forced directly
# into Top Green Gecko Wheel 3 at (-12, 238)!)
j_curve = make_box_mm(0.0, 0.0, 0.0, 146.0, 96.0, 4.5)
tb.transform(j_curve, mat_mul(mat_pos_mm(-68.0, 0.0, 128.0), mat_rot_deg((0, 1, 0), 48.0)))
# Segment 4: Upper Elevator Feed Wall (80.0 deg, guiding Ball 3 (-78, 207) straight up into the shooter nip!)
j_elev = make_box_mm(0.0, 0.0, 0.0, 60.0, 96.0, 4.5)
tb.transform(j_elev, mat_mul(mat_pos_mm(-118.0, 0.0, 200.0), mat_rot_deg((0, 1, 0), 80.0)))
# Trim j_elev above the turret deck underside (Z > 216.0 mm) and trim segment tails cleanly at the elbows:
tb.booleanOperation(j_elev, make_box_mm(-114.0, 0.0, 240.0, 80.0, 120.0, 48.0), fusion.BooleanTypes.DifferenceBooleanType)
tb.booleanOperation(j_curve, make_box_mm(-128.0, 0.0, 232.0, 80.0, 120.0, 60.0), fusion.BooleanTypes.DifferenceBooleanType)
tb.booleanOperation(j_curve, make_box_mm(10.0, 0.0, 52.0, 60.0, 120.0, 45.0), fusion.BooleanTypes.DifferenceBooleanType)
tb.booleanOperation(j_ramp, make_box_mm(-45.0, 0.0, 92.0, 60.0, 120.0, 50.0), fusion.BooleanTypes.DifferenceBooleanType)

j_elbow1 = make_cyl_mm(-18.0, -48.0, 80.0, -18.0, 48.0, 80.0, 4.5)
j_elbow2 = make_cyl_mm(-112.0, -48.0, 178.0, -112.0, 48.0, 178.0, 4.5)

j_chute = j_ramp
tb.booleanOperation(j_chute, j_curve, fusion.BooleanTypes.UnionBooleanType)
tb.booleanOperation(j_chute, j_elev, fusion.BooleanTypes.UnionBooleanType)
tb.booleanOperation(j_chute, j_elbow1, fusion.BooleanTypes.UnionBooleanType)
tb.booleanOperation(j_chute, j_elbow2, fusion.BooleanTypes.UnionBooleanType)

# Continuous Side Guide Curbs along the entire 170 mm lower ramp and J-curve (Y = ±46.0 mm):
for cy_sign in (-1.0, 1.0):
    curb1 = make_box_mm(0.0, 0.0, 0.0, 168.0, 4.0, 10.0)
    tb.transform(curb1, mat_mul(mat_pos_mm(67.5, cy_sign * 46.0, 56.0), mat_rot_deg((0, 1, 0), 18.5)))
    curb2 = make_box_mm(0.0, 0.0, 0.0, 128.0, 4.0, 10.0)
    tb.transform(curb2, mat_mul(mat_pos_mm(-64.0, cy_sign * 46.0, 131.0), mat_rot_deg((0, 1, 0), 48.0)))
    tb.booleanOperation(j_chute, curb1, fusion.BooleanTypes.UnionBooleanType)
    tb.booleanOperation(j_chute, curb2, fusion.BooleanTypes.UnionBooleanType)

# Integral Structural Mounting Stations bolting 'unified_continuous_j_chute' to the Official goBILDA Strafer Chassis:
# Station 1: Front Rail Tie-Bar & Upright Risers (X = +144.0 mm, bolted to the underside of the continuous ramp and to the Left & Right 17-Hole Rails!)
st1_bar = make_box_mm(144.0, 0.0, 22.5, 22.0, 264.0, 7.0)
st1_riser_l = make_box_mm(144.0, 128.0, 48.0, 22.0, 10.0, 52.0)
st1_riser_r = make_box_mm(144.0, -128.0, 48.0, 22.0, 10.0, 52.0)
tb.booleanOperation(j_chute, st1_bar, fusion.BooleanTypes.UnionBooleanType)
tb.booleanOperation(j_chute, st1_riser_l, fusion.BooleanTypes.UnionBooleanType)
tb.booleanOperation(j_chute, st1_riser_r, fusion.BooleanTypes.UnionBooleanType)

# 4x Clean Drivetrain Wheel Axles (added BEFORE j_chute so j_chute trims cleanly around them!):
for wx_mm, wxlbl in ((144.0, "front"), (-144.0, "rear")):
    for sy_sign, sylbl in ((1.0, "left"), (-1.0, "right")):
        wax = make_cyl_mm(wx_mm, sy_sign * 126.0, 48.0, wx_mm, sy_sign * 218.0, 48.0, 3.92)
        mot_cut = make_cyl_mm(-143.0, sy_sign * 156.0, 45.0, 143.0, sy_sign * 156.0, 45.0, 5.2)
        tb.booleanOperation(wax, mot_cut, fusion.BooleanTypes.DifferenceBooleanType)
        add_solid(wax, f"drivetrain_rex_wheel_axle_{wxlbl}_{sylbl}", "silver")

# Station 2 & 3 Combined: Heavy Bolted Saddle sitting FLUSH ON TOP (Z = 72.1 mm!) of the Official goBILDA Strafer Kit
# Full-Height 1120 10-Hole Center Cross-Channel (now aligned on-grid at Hole 8: X = -24.0 mm, X in [-48.0, 0.0 mm])!
st3_saddle = make_box_mm(-24.0, 0.0, 80.0, 44.0, 96.0, 15.8)
ball_corridor_cut = make_box_mm(0.0, 0.0, 0.0, 210.0, 86.0, 60.0)
tb.transform(ball_corridor_cut, mat_mul(mat_pos_mm(67.5, 0.0, 84.0), mat_rot_deg((0, 1, 0), 18.5)))
ball_curve_cut = make_box_mm(0.0, 0.0, 0.0, 160.0, 86.0, 60.0)
tb.transform(ball_curve_cut, mat_mul(mat_pos_mm(-68.0, 0.0, 160.0), mat_rot_deg((0, 1, 0), 48.0)))
tb.booleanOperation(st3_saddle, ball_corridor_cut, fusion.BooleanTypes.DifferenceBooleanType)
tb.booleanOperation(st3_saddle, ball_curve_cut, fusion.BooleanTypes.DifferenceBooleanType)
tb.booleanOperation(j_chute, st3_saddle, fusion.BooleanTypes.UnionBooleanType)

# Station 4: Upper Rear Crossbar & Turret Deck-Plate Underside Collar (X in [-126.0, -102.0], Y in [-132.0, +132.0], Z in [196.0, 216.0])
st4_collar = make_box_mm(-118.0, 0.0, 208.0, 20.0, 264.0, 16.0)
tb.booleanOperation(st4_collar, make_box_mm(-78.0, 0.0, 205.0, 56.0, 86.0, 30.0), fusion.BooleanTypes.DifferenceBooleanType)
tb.booleanOperation(j_chute, st4_collar, fusion.BooleanTypes.UnionBooleanType)
tb.booleanOperation(j_chute, make_cyl_mm(144.0, -220.0, 48.0, 144.0, 220.0, 48.0, 5.0), fusion.BooleanTypes.DifferenceBooleanType)

add_solid(j_chute, "unified_continuous_j_chute", "amber_chute", trim_scene=True, trim_prev=True)

# ------------------------------------------------------------------------------
# 1E. EXTENDED FULL-SPAN MAGAZINE SIDE PLATES + GECKO SHAFT SPACERS + OUTBOARD
#     DOUBLE-SHEAR BEARING PLATE + 20 MM COMPLIANCE SLOTS FOR 91.95 MM NECTAR BALLS
#     + SOLID SERVO-ACTUATED INDEXING STOPPER GATE + THROAT BALL-SIZE SENSOR:
# ------------------------------------------------------------------------------
# Add the 3 Gecko Shafts ((114.0, 151.0), (34.0, 180.0), (-12.0, 240.0)), 6 Side-Plate Flanged Bearings
# (at Y = ±44.0 mm!), 6 Shaft Spacers (spanning |Y| in [16.2, 40.8 mm]), 6 Pulleys, and 3 Belts FIRST!
for idx, (gx, gz) in enumerate(((114.0, 151.0), (34.0, 180.0), (-12.0, 240.0)), start=1):
    add_solid(make_cyl_mm(gx, -74.0, gz, gx, 48.0, gz, 3.92), f"gecko_rex_shaft_clean_{idx}", "silver")
    for sy_sign, blbl in ((-1.0, "right"), (1.0, "left")):
        # Aluminum shaft spacer between Gecko wheel hub (|Y|=16.0) and Magazine Side Plate bearing (|Y|=41.0):
        g_sp = make_cyl_mm(gx, sy_sign * 16.2, gz, gx, sy_sign * 40.8, gz, 8.5)
        tb.booleanOperation(g_sp, make_cyl_mm(gx, -60.0, gz, gx, 60.0, gz, 4.05), fusion.BooleanTypes.DifferenceBooleanType)
        add_solid(g_sp, f"gecko_shaft_spacer_{idx}_{blbl}", "pulley_alu")
        # Flanged bearing seated directly at Y = ±44.0 mm in the Magazine Side Plate:
        by = sy_sign * 44.0
        brg = make_cyl_mm(gx, by - 3.0, gz, gx, by + 3.0, gz, 9.5)
        bbore = make_cyl_mm(gx, by - 5.0, gz, gx, by + 5.0, gz, 4.05)
        tb.booleanOperation(brg, bbore, fusion.BooleanTypes.DifferenceBooleanType)
        add_solid(brg, f"gecko_flanged_bearing_clean_{idx}_{blbl}", "mount_slate")

for pidx, (px, py, pz) in enumerate(
    (
        (104.0, -54.0, 98.0),
        (114.0, -54.0, 151.0),
        (114.0, -64.0, 151.0),
        (34.0, -64.0, 180.0),
        (34.0, -54.0, 180.0),
        (-12.0, -54.0, 240.0),
    ),
    start=1,
):
    add_solid(make_pulley_world_mm(px, py, pz), f"magazine_drive_pulley_{pidx}", "pulley_alu")

add_solid(
    make_belt_loop_mm(104.0, 98.0, 114.0, 151.0, y_mm=-54.0, r_in_mm=12.15, r_out_mm=13.8, width_mm=5.4),
    "magazine_motor_drive_belt_clean",
    "belt_black",
)
add_solid(
    make_belt_loop_mm(114.0, 151.0, 34.0, 180.0, y_mm=-64.0, r_in_mm=12.15, r_out_mm=13.8, width_mm=5.4),
    "magazine_gecko_couple_belt_clean",
    "belt_black",
)
add_solid(
    make_belt_loop_mm(34.0, 180.0, -12.0, 240.0, y_mm=-54.0, r_in_mm=12.15, r_out_mm=13.8, width_mm=5.4),
    "magazine_elevator_couple_belt_clean",
    "belt_black",
)

# Extended Left & Right Magazine Side Plates (Y = ±44.0 mm) spanning X in [-64.0, +132.0 mm] and Z up to 258.0 mm
# with 20 mm upward-normal compliance slots at all 3 Gecko shaft positions ((114, 151), (34, 180), (-12, 240))
# so the Gecko wheels expand +20 mm upward when a 91.95 mm Nectar ball passes up the ramp!
for side_y, sname in ((44.0, "left"), (-44.0, "right")):
    wall = make_box_mm(0.0, 0.0, 0.0, 198.0, 4.0, 96.0)
    tb.transform(wall, mat_mul(mat_pos_mm(36.0, side_y, 180.0), mat_rot_deg((0, 1, 0), 10.0)))
    # Upper neck extension capturing Gecko Shaft 3 at (-12.0, 240.0) and bolting into the turret deck plate:
    up_neck = make_box_mm(-8.0, side_y, 228.0, 56.0, 4.0, 62.0)
    # Front extension bolting directly to the Front 4H Uprights (X = +132.0 mm):
    fr_flange = make_box_mm(120.0, side_y, 156.0, 32.0, 4.0, 56.0)
    sign_y = 1.0 if side_y > 0 else -1.0
    top_flange = make_box_mm(-36.0, side_y + sign_y * 4.0, 213.0, 72.0, 12.0, 6.0)
    tb.booleanOperation(wall, up_neck, fusion.BooleanTypes.UnionBooleanType)
    tb.booleanOperation(wall, fr_flange, fusion.BooleanTypes.UnionBooleanType)
    tb.booleanOperation(wall, top_flange, fusion.BooleanTypes.UnionBooleanType)
    for gx, gz in ((114.0, 151.0), (34.0, 180.0), (-12.0, 240.0)):
        # 20 mm upward compliance slot angled at -18.5 deg (along the ramp upward normal!) so each Gecko shaft
        # rests at the bottom of its slot (71.5 mm gap) and slides up +20 mm (91.5 mm gap) for 91.95 mm Nectar balls:
        c_slot = make_box_mm(0.0, 0.0, 0.0, 9.0, 16.0, 20.0)
        tb.transform(c_slot, mat_mul(mat_pos_mm(gx + 3.0, side_y, gz + 9.0), mat_rot_deg((0, 1, 0), -18.5)))
        tb.booleanOperation(wall, c_slot, fusion.BooleanTypes.DifferenceBooleanType)
        sh_bore = make_cyl_mm(gx, -60.0, gz, gx, 60.0, gz, 5.0)
        tb.booleanOperation(wall, sh_bore, fusion.BooleanTypes.DifferenceBooleanType)
    add_solid(wall, f"sprung_magazine_shroud_{sname}", "shroud_violet", trim_scene=True, trim_prev=True)

# 6mm CNC Aluminum Magazine Outboard Double-Shear Bearing Plate (Y = -71.0 mm) supporting the outer ends
# of all 3 Gecko shafts ((114, 151), (34, 180), (-12, 240)) and Motor M8 (104, 98) outside the timing belts!
mag_ds_plate = make_box_mm(0.0, 0.0, 0.0, 174.0, 5.0, 48.0)
tb.transform(mag_ds_plate, mat_mul(mat_pos_mm(48.0, -71.0, 193.0), mat_rot_deg((0, 1, 0), 36.5)))
mag_ds_leg = make_box_mm(106.0, -71.0, 124.0, 28.0, 5.0, 72.0)
mag_ds_tie = make_box_mm(106.0, -101.0, 96.0, 28.0, 62.0, 8.0)  # Ties into Right 17H Rail / M8 Saddle
tb.booleanOperation(mag_ds_plate, mag_ds_leg, fusion.BooleanTypes.UnionBooleanType)
tb.booleanOperation(mag_ds_plate, mag_ds_tie, fusion.BooleanTypes.UnionBooleanType)
for gx, gz in ((114.0, 151.0), (34.0, 180.0), (-12.0, 240.0), (104.0, 98.0)):
    tb.booleanOperation(mag_ds_plate, make_cyl_mm(gx, -80.0, gz, gx, -60.0, gz, 4.1), fusion.BooleanTypes.DifferenceBooleanType)
add_solid(mag_ds_plate, "magazine_outboard_double_shear_bearing_plate_right", "pulley_alu", trim_scene=True, trim_prev=True)

for gx, gy, gz, glbl in (
    (-18.0, 89.0, 136.0, "left_low"),
    (6.0, 89.0, 168.0, "left_high"),
    (-18.0, -89.0, 136.0, "right_low"),
    (-18.0, -89.0, 168.0, "right_high"),
):
    add_solid(make_box_mm(gx, gy, gz, 22.0, 86.0, 10.0), f"magazine_structural_gusset_{glbl}", "mount_slate", trim_scene=True, trim_prev=True)

# SERVO-ACTUATED SHOOTER THROAT INDEXING GATE ('indexer_stopper_gate_*')
# Mounted cleanly at the upper rear throat (X = -118.0 mm, Z = 252.0 mm, between the orange shooter side plates
# at Y = ±58.0 mm and driven COAXIALLY by the horizontally oriented 'servo_s3_magazine_stopper:1' whose white
# H25T Spline sits at (-118.0, +71.07, 252.0) pointing straight inward -Y onto the gate shaft!):
gate_shaft = make_cyl_mm(-118.0, -61.0, 252.0, -118.0, 66.5, 252.0, 3.92)
add_solid(gate_shaft, "indexer_gate_pivot_rex_shaft", "silver")

# Compact Servo S3 Mounting Bridge ('s3_bridge', Y in [68.0, 106.0 mm]) sitting strictly behind the shortened Left
# Grey Motor Plate (which ends at X = -56.0 mm!) and strictly inboard of the Main Power Switch (which sits at Y = +148.0 mm!):
s3_bridge = make_box_mm(-108.0, 87.0, 232.0, 52.0, 38.0, 20.0)
add_solid(s3_bridge, "servo_s3_indexer_gate_mount_bridge", "mount_slate", trim_scene=True, trim_prev=True)

gate_hub = make_cyl_mm(-118.0, -44.0, 252.0, -118.0, 44.0, 252.0, 6.5)
# Extended 30 mm Axis-Aligned Gate Arms & Crossbar reaching forward to X = -86.0 mm, Z = 268.0 mm directly over the upper throat:
gate_arm_l = make_box_mm(-104.0, 38.0, 260.0, 30.0, 6.0, 20.0)
gate_arm_r = make_box_mm(-104.0, -38.0, 260.0, 30.0, 6.0, 20.0)
gate_bar = make_box_mm(-92.0, 0.0, 268.0, 12.0, 82.0, 8.0)
tb.booleanOperation(gate_hub, gate_arm_l, fusion.BooleanTypes.UnionBooleanType)
tb.booleanOperation(gate_hub, gate_arm_r, fusion.BooleanTypes.UnionBooleanType)
tb.booleanOperation(gate_hub, gate_bar, fusion.BooleanTypes.UnionBooleanType)
gate_bore = make_cyl_mm(-118.0, -48.0, 252.0, -118.0, 48.0, 252.0, 4.05)
tb.booleanOperation(gate_hub, gate_bore, fusion.BooleanTypes.DifferenceBooleanType)
add_solid(gate_hub, "indexer_stopper_gate_paddle", "gate_crimson", trim_scene=True, trim_prev=True)

gate_spline_collar = make_cyl_mm(-118.0, 61.2, 252.0, -118.0, 66.8, 252.0, 6.2)
tb.booleanOperation(gate_spline_collar, make_cyl_mm(-118.0, 60.0, 252.0, -118.0, 65.0, 252.0, 4.05), fusion.BooleanTypes.DifferenceBooleanType)
add_solid(gate_spline_collar, "indexer_gate_servo_coupler", "pulley_alu", trim_scene=True, trim_prev=True)

# ------------------------------------------------------------------------------
# 1F. RIGIDLY BOLTED FIXED-YAW SHOOTER ASSEMBLY WITH OUTBOARD COAXIAL BELT-ON-PIVOT
#     SPRUNG SWING-ARM COUNTER-WHEEL, TRUE CURVED CONCENTRIC ARC SLOTS CENTERED AT
#     MOTOR M6 (-108.0, 266.0), CLEAN SOLID SIDE PANELS (ZERO UNUSED HOLES/WINDOWS),
#     SHORTENED LEFT MOTOR PLATE (25 MM CLEAR OF GATE SERVO S3!), AND 24 MM REAR/BOTTOM
#     CLEARANCE AROUND THE 72 MM COUNTER-WHEEL!
# ------------------------------------------------------------------------------
def make_concentric_arc_slot_cutter_mm(piv_x: float, piv_z: float, r_arm: float, deg_start: float, deg_end: float, y_min: float, y_max: float, slot_r: float, steps: int = 16):
    # Construct a 100% smooth analytical circular annulus sector centered at (piv_x, piv_z):
    # Outer cylinder (r_arm + slot_r) minus inner cylinder (r_arm - slot_r), intersected with the angular sector
    # [deg_start, deg_end], plus the two rounded end-cap cylinders of radius slot_r at deg_start and deg_end!
    y_mid = 0.5 * (y_min + y_max)
    y_len = abs(y_max - y_min) + 2.0
    ring = make_cyl_mm(piv_x, y_min, piv_z, piv_x, y_max, piv_z, r_arm + slot_r)
    inner_cyl = make_cyl_mm(piv_x, y_min - 2.0, piv_z, piv_x, y_max + 2.0, piv_z, r_arm - slot_r)
    tb.booleanOperation(ring, inner_cyl, fusion.BooleanTypes.DifferenceBooleanType)

    # Half-space plane 1: keep angle >= deg_start (normal pointing towards deg_start + 90 deg)
    a1 = math.radians(deg_start)
    nx1, nz1 = -math.sin(a1), math.cos(a1)
    half1 = make_box_mm(0.0, 0.0, 0.0, 200.0, y_len, 200.0)
    tb.transform(
        half1,
        mat_mul(
            mat_pos_mm(piv_x + 100.0 * nx1, y_mid, piv_z + 100.0 * nz1),
            mat_rot_deg((0, 1, 0), -deg_start),
        ),
    )
    tb.booleanOperation(ring, half1, fusion.BooleanTypes.IntersectionBooleanType)

    # Half-space plane 2: keep angle <= deg_end (normal pointing towards deg_end - 90 deg)
    a2 = math.radians(deg_end)
    nx2, nz2 = math.sin(a2), -math.cos(a2)
    half2 = make_box_mm(0.0, 0.0, 0.0, 200.0, y_len, 200.0)
    tb.transform(
        half2,
        mat_mul(
            mat_pos_mm(piv_x + 100.0 * nx2, y_mid, piv_z + 100.0 * nz2),
            mat_rot_deg((0, 1, 0), -deg_end),
        ),
    )
    tb.booleanOperation(ring, half2, fusion.BooleanTypes.IntersectionBooleanType)

    # Union the two rounded end-cap cylinders at deg_start and deg_end:
    for ang_r in (a1, a2):
        cx = piv_x + r_arm * math.cos(ang_r)
        cz = piv_z + r_arm * math.sin(ang_r)
        cap = make_cyl_mm(cx, y_min, cz, cx, y_max, cz, slot_r)
        tb.booleanOperation(ring, cap, fusion.BooleanTypes.UnionBooleanType)
    return ring


# 1. Main Flywheel Shaft (+32.0, 324.0) driven by Left Motor M5 at (-24.0, 266.0).
# 2. Rear Pink Counter-Wheel Shaft (-123.5, 324.0) carried on OUTBOARD Coaxial Swing-Arms
#    (at |Y| = 65.0 mm, outside the |Y| = 58.0 mm shooter side walls!) pivoting coaxially
#    around Right Motor M6's axis (-108.0, 266.0) along a TRUE CURVED CONCENTRIC ARC SLOT
#    (R_arm = 60.035 mm, theta in [104.96 deg, 126.5 deg] -> (-123.5, 324.0) to (-143.7, 314.3))!
add_solid(make_cyl_mm(32.0, -92.0, 324.0, 32.0, 92.0, 324.0, 3.92), "shooter_main_rex_shaft_clean", "silver")
add_solid(make_cyl_mm(-123.5, -92.0, 324.0, -123.5, 70.0, 324.0, 3.92), "shooter_counter_rex_shaft_clean", "silver")

for side_sign, slbl in ((1.0, "left"), (-1.0, "right")):
    # Outboard Coaxial Pivot Stub Shafts (-108.0, |Y| in [55.0, 66.0], 266.0) seated in the side wall bearings:
    add_solid(
        make_cyl_mm(-108.0, side_sign * 55.0, 266.0, -108.0, side_sign * 66.0, 266.0, 3.92),
        f"shooter_coaxial_pivot_stub_shaft_{slbl}",
        "silver",
    )
    # Hood Hinge Stub Shafts (-96.0, |Y| in [50.0, 66.5], 382.0) seated inside closed top bearing bosses:
    add_solid(
        make_cyl_mm(-96.0, side_sign * 50.0, 382.0, -96.0, side_sign * 66.5, 382.0, 3.92),
        f"hood_hinge_stub_shaft_{slbl}",
        "silver",
    )
    # Internal Shaft Spacers spanning cleanly from the wheels out to the side-wall bearings (|Y| = 54.8 mm):
    sp_main = make_cyl_mm(32.0, side_sign * 33.2, 324.0, 32.0, side_sign * 54.8, 324.0, 8.5)
    tb.booleanOperation(sp_main, make_cyl_mm(32.0, -70.0, 324.0, 32.0, 70.0, 324.0, 4.05), fusion.BooleanTypes.DifferenceBooleanType)
    add_solid(sp_main, f"shooter_main_shaft_spacer_{slbl}", "pulley_alu")

    sp_cnt = make_cyl_mm(-123.5, side_sign * 18.2, 324.0, -123.5, side_sign * 54.8, 324.0, 8.5)
    tb.booleanOperation(sp_cnt, make_cyl_mm(-123.5, -70.0, 324.0, -123.5, 70.0, 324.0, 4.05), fusion.BooleanTypes.DifferenceBooleanType)
    add_solid(sp_cnt, f"shooter_counter_shaft_spacer_{slbl}", "pulley_alu")

    # Flanged Ball Bearings seated in the Shooter Side Walls (|Y| = 58.0 mm):
    for b_idx, (bx_b, bz_b) in enumerate(((32.0, 324.0), (-96.0, 382.0), (-108.0, 266.0)), start=1):
        s_brg = make_cyl_mm(bx_b, side_sign * 55.2, bz_b, bx_b, side_sign * 60.8, bz_b, 8.5)
        tb.booleanOperation(s_brg, make_cyl_mm(bx_b, side_sign * 50.0, bz_b, bx_b, side_sign * 65.0, bz_b, 4.05), fusion.BooleanTypes.DifferenceBooleanType)
        add_solid(s_brg, f"shooter_side_plate_flanged_bearing_{slbl}_{b_idx}", "mount_slate")

# Outboard Flanged Bearings on the Left & Right Double-Shear Plates (Main Flywheel on both sides; Counter-Wheel on Right side):
for side_sign, slbl in ((1.0, "left"), (-1.0, "right")):
    o_brg1 = make_cyl_mm(32.0, side_sign * 84.8, 324.0, 32.0, side_sign * 90.8, 324.0, 8.5)
    tb.booleanOperation(o_brg1, make_cyl_mm(32.0, side_sign * 80.0, 324.0, 32.0, side_sign * 95.0, 324.0, 4.05), fusion.BooleanTypes.DifferenceBooleanType)
    add_solid(o_brg1, f"shooter_outboard_flanged_bearing_{slbl}_1", "mount_slate")

o_brg2_r = make_cyl_mm(-123.5, -90.8, 324.0, -123.5, -84.8, 324.0, 8.5)
tb.booleanOperation(o_brg2_r, make_cyl_mm(-123.5, -95.0, 324.0, -123.5, -80.0, 324.0, 4.05), fusion.BooleanTypes.DifferenceBooleanType)
add_solid(o_brg2_r, "shooter_outboard_flanged_bearing_right_2", "mount_slate")

# OUTBOARD COAXIAL BELT-ON-PIVOT SPRUNG SWING-ARMS ('shooter_coaxial_swing_arm_left/right') + OUTBOARD RETURN SPRINGS:
# Mounted at |Y| = 65.0 mm (OUTSIDE the |Y| = 58.0 mm shooter side walls so the internal ball chute is 100% clean!),
# pivoting coaxially around (-108.0, 266.0) up to (-123.5, 324.0) with return springs anchored directly to bolted
# spring posts on the outer face of the shooter side walls!
c_arm_len = math.hypot(-123.5 - (-108.0), 324.0 - 266.0)
c_arm_ang = -math.degrees(math.atan2(324.0 - 266.0, -123.5 - (-108.0)))
for side_y, sname in ((65.0, "left"), (-65.0, "right")):
    sign_y = 1.0 if side_y > 0 else -1.0
    s_arm = make_box_mm(0.0, 0.0, 0.0, c_arm_len, 6.0, 16.0)
    tb.transform(s_arm, mat_mul(mat_pos_mm(-115.75, side_y, 295.0), mat_rot_deg((0, 1, 0), c_arm_ang)))
    h_bot = make_cyl_mm(-108.0, side_y - 3.5, 266.0, -108.0, side_y + 3.5, 266.0, 10.5)
    h_top = make_cyl_mm(-123.5, side_y - 3.5, 324.0, -123.5, side_y + 3.5, 324.0, 10.5)
    tb.booleanOperation(s_arm, h_bot, fusion.BooleanTypes.UnionBooleanType)
    tb.booleanOperation(s_arm, h_top, fusion.BooleanTypes.UnionBooleanType)
    tb.booleanOperation(s_arm, make_cyl_mm(-108.0, side_y - 8.0, 266.0, -108.0, side_y + 8.0, 266.0, 4.05), fusion.BooleanTypes.DifferenceBooleanType)
    tb.booleanOperation(s_arm, make_cyl_mm(-123.5, side_y - 8.0, 324.0, -123.5, side_y + 8.0, 324.0, 4.05), fusion.BooleanTypes.DifferenceBooleanType)
    add_solid(s_arm, f"shooter_coaxial_swing_arm_{sname}", "nectar_orange")

    # Outboard Coiled Return Spring at |Y| = 65.0 mm anchored directly to the outer face of the Shooter Side Wall (|Y| = 61.0 mm):
    sp_rod = make_cyl_mm(-112.0, side_y, 324.0, -88.0, side_y, 324.0, 2.2)
    for coil_x in (-108.0, -103.5, -99.0, -94.5):
        coil = make_cyl_mm(coil_x - 1.2, side_y, 324.0, coil_x + 1.2, side_y, 324.0, 5.2)
        tb.booleanOperation(sp_rod, coil, fusion.BooleanTypes.UnionBooleanType)
    # Bolted anchor post sitting flush against the outer face of the shooter side wall (|Y| = 61.0 mm):
    sp_anchor = make_box_mm(-86.0, sign_y * 64.6, 324.0, 8.0, 7.0, 16.0)
    tb.booleanOperation(sp_rod, sp_anchor, fusion.BooleanTypes.UnionBooleanType)
    add_solid(sp_rod, f"shooter_relief_spring_{sname}", "silver")

# Left Belt: Motor M5 (-24.0, 266.0) -> Main Flywheel (+32.0, 324.0) at Y = +76.0 mm
# Right Coaxial Belt: Motor M6 (-108.0, 266.0) -> Rear Counter-Wheel (-123.5, 324.0) at Y = -76.0 mm
add_solid(make_pulley_world_mm(-24.0, 76.0, 266.0), "shooter_left_pulley_1", "pulley_alu")
add_solid(make_pulley_world_mm(32.0, 76.0, 324.0), "shooter_left_pulley_2", "pulley_alu")
add_solid(make_pulley_world_mm(-108.0, -76.0, 266.0), "shooter_right_pulley_1", "pulley_alu")
add_solid(make_pulley_world_mm(-123.5, -76.0, 324.0), "shooter_right_pulley_2", "pulley_alu")

add_solid(
    make_belt_loop_mm(-24.0, 266.0, 32.0, 324.0, y_mm=76.0, r_in_mm=12.15, r_out_mm=13.8, width_mm=5.4),
    "shooter_left_drive_belt_clean",
    "belt_black",
)
add_solid(
    make_belt_loop_mm(-108.0, 266.0, -123.5, 324.0, y_mm=-76.0, r_in_mm=12.15, r_out_mm=13.8, width_mm=5.4),
    "shooter_right_drive_belt_clean",
    "belt_black",
)

# Flat Aluminum Top Deck Plate with a clean rectangular ball-throat opening (NO circular swivel turret ring!):
deck_plate_body = make_box_mm(-78.0, 0.0, 219.0, 180.0, 360.0, 6.0)
deck_throat_slot = make_box_mm(-52.0, 0.0, 219.0, 144.0, 110.0, 12.0)
tb.booleanOperation(deck_plate_body, deck_throat_slot, fusion.BooleanTypes.DifferenceBooleanType)
add_solid(deck_plate_body, "cnc_turret_mounting_deck_plate_clean", "pulley_alu", trim_scene=True, trim_prev=True)

# Clean Shooter Side Walls at Y = ±58.0 mm (|Y| in [55.0, 61.0 mm], X in [-168.0, +72.0 mm], Z in [222.0, 386.0 mm])
# with CLOSED circular bearing bosses at the Hood Hinge (-96.0, 382.0), TRUE CURVED CONCENTRIC ARC SLOTS centered
# at Motor M6 (-108.0, 266.0), and ZERO unnecessary circular or rectangular holes!
for side_y, sname in ((58.0, "left"), (-58.0, "right")):
    tplate = make_box_mm(-48.0, side_y, 304.0, 240.0, 6.0, 164.0)
    sign_y = 1.0 if side_y > 0 else -1.0
    # Closed circular bearing boss around the Hood Hinge (-96.0, 382.0) so the pin is held in a full ring:
    hood_boss = make_cyl_mm(-96.0, side_y - 3.0, 382.0, -96.0, side_y + 3.0, 382.0, 12.0)
    # Bolted L-Flange at Z = 225.0 mm anchoring the orange shooter side plates rigidly to the flat Top Deck Plate:
    bot_flange = make_box_mm(-48.0, side_y + sign_y * 6.0, 225.0, 200.0, 16.0, 6.0)
    tb.booleanOperation(tplate, hood_boss, fusion.BooleanTypes.UnionBooleanType)
    tb.booleanOperation(tplate, bot_flange, fusion.BooleanTypes.UnionBooleanType)
    # Upper saddle scallop between X = -78.0 and X = +14.0 mm (Z >= 338.0 mm) to expose the ball & flywheel contact:
    tb.booleanOperation(tplate, make_box_mm(-32.0, side_y, 366.0, 92.0, 16.0, 56.0), fusion.BooleanTypes.DifferenceBooleanType)
    # TRUE CURVED CONCENTRIC ARC SLOT centered at (-108.0, 266.0) with R_arm = 60.035 mm from theta = 104.96 deg (-123.5, 324.0)
    # to theta = 128.5 deg (-145.4, 313.0 -> 93.7 mm nip gap, giving +1.75 mm over-travel margin for 91.95 mm Nectar balls!):
    arc_slot_inner = make_concentric_arc_slot_cutter_mm(-108.0, 266.0, c_arm_len, 104.96, 128.5, side_y - 8.0, side_y + 8.0, 4.6)
    tb.booleanOperation(tplate, arc_slot_inner, fusion.BooleanTypes.DifferenceBooleanType)
    # ONLY the 3 actual shaft bores that pass through the side walls (Main Flywheel, Hood Hinge, and Coaxial Pivot Stub):
    for bx, bz, br in ((32.0, 324.0, 5.0), (-96.0, 382.0, 5.0), (-108.0, 266.0, 5.0)):
        bore = make_cyl_mm(bx, side_y - 8.0, bz, bx, side_y + 8.0, bz, br)
        tb.booleanOperation(tplate, bore, fusion.BooleanTypes.DifferenceBooleanType)
    if sname == "left":
        # Bore for the Red Indexer Gate shaft (-118.0, 252.0) on the left wall:
        tb.booleanOperation(tplate, make_cyl_mm(-118.0, side_y - 8.0, 252.0, -118.0, side_y + 8.0, 252.0, 4.5), fusion.BooleanTypes.DifferenceBooleanType)
    add_solid(tplate, f"turret_side_plate_{sname}_clean", "amber_chute", trim_scene=True, trim_prev=True)

# Lowered Rear Throat Guide ('turret_upper_throat_guide', Z in [216.0, 264.0 mm]) stopping at Z_max = 264.0 mm —
# giving a full 24.0 mm of open clearance below the 72 mm Red Counter-Wheel (Z_min = 288.0 mm at rest, 278.3 mm at full arc swing)
# so the Counter-Wheel NEVER clips or rubs against the backside of the shooter!
throat_guide = make_box_mm(-124.0, 0.0, 240.0, 10.0, 110.0, 48.0)
add_solid(throat_guide, "turret_upper_throat_guide", "amber_chute", trim_scene=True, trim_prev=True)

# STEEP HIGH-ARC BASKETBALL SHOOTER EXIT HOOD (64 DEG TO 74 DEG VERTICAL LAUNCH TRAJECTORY!)
hood_seg1 = make_box_mm(0.0, 0.0, 0.0, 34.0, 108.0, 5.5)
tb.transform(hood_seg1, mat_mul(mat_pos_mm(-91.0, 0.0, 397.0), mat_rot_deg((0, 1, 0), -72.0)))
hood_seg2 = make_box_mm(0.0, 0.0, 0.0, 34.0, 108.0, 5.5)
tb.transform(hood_seg2, mat_mul(mat_pos_mm(-78.0, 0.0, 427.0), mat_rot_deg((0, 1, 0), -62.0)))
hood_joint = make_cyl_mm(-86.0, -54.0, 412.0, -86.0, 54.0, 412.0, 3.0)
tb.booleanOperation(hood_seg1, hood_seg2, fusion.BooleanTypes.UnionBooleanType)
tb.booleanOperation(hood_seg1, hood_joint, fusion.BooleanTypes.UnionBooleanType)
for side_sign in (-1.0, 1.0):
    h_ear = make_cyl_mm(-96.0, side_sign * 50.0, 382.0, -96.0, side_sign * 54.8, 382.0, 8.0)
    tb.booleanOperation(hood_seg1, h_ear, fusion.BooleanTypes.UnionBooleanType)
tb.booleanOperation(hood_seg1, make_cyl_mm(-96.0, -66.6, 382.0, -96.0, 70.0, 382.0, 4.05), fusion.BooleanTypes.DifferenceBooleanType)
add_solid(hood_seg1, "variable_pitch_exit_hood_full", "hood_cyan", trim_scene=True, trim_prev=True)

# Coaxial Cylindrical Servo Shaft Coupler ('hood_servo_shaft_coupler' at (-96.0, Y in [-68.0, -61.2], 382.0))
# cleanly coupling Servo S6's white H25T Spline directly onto 'hood_hinge_stub_shaft_right' with zero wall clipping:
h_coupler = make_cyl_mm(-96.0, -68.0, 382.0, -96.0, -61.2, 382.0, 6.2)
tb.booleanOperation(h_coupler, make_cyl_mm(-96.0, -66.6, 382.0, -96.0, -60.0, 382.0, 4.05), fusion.BooleanTypes.DifferenceBooleanType)
add_solid(h_coupler, "hood_servo_shaft_coupler", "pulley_alu", trim_scene=True, trim_prev=True)

# OUTBOARD FRONT-RIGHT DFROBOT HUSKYLENS AI VISION SENSOR ('dfrobot_huskylens_*', Rule R702 Example 5 Legal)
hl_bracket = make_box_mm(54.0, -74.0, 332.0, 24.0, 28.0, 8.0)
hl_riser = make_box_mm(54.0, -62.5, 318.0, 24.0, 5.0, 32.0)
tb.booleanOperation(hl_bracket, hl_riser, fusion.BooleanTypes.UnionBooleanType)
add_solid(hl_bracket, "dfrobot_huskylens_alu_mount_bracket", "pulley_alu", trim_scene=True, trim_prev=True)

hl_pcb = make_box_mm(56.0, -84.0, 352.0, 10.0, 52.0, 32.0)
add_solid(hl_pcb, "dfrobot_huskylens_pcb_housing", "comb_dark", trim_scene=True, trim_prev=True)

hl_lens = make_cyl_mm(61.0, -84.0, 354.0, 69.0, -84.0, 354.0, 8.5)
hl_aperture = make_cyl_mm(67.0, -84.0, 354.0, 70.0, -84.0, 354.0, 5.0)
tb.booleanOperation(hl_lens, hl_aperture, fusion.BooleanTypes.DifferenceBooleanType)
add_solid(hl_lens, "dfrobot_huskylens_front_m12_lens", "belt_black", trim_scene=True, trim_prev=True)

hl_screen = make_box_mm(50.0, -84.0, 352.0, 2.0, 42.0, 24.0)
add_solid(hl_screen, "dfrobot_huskylens_rear_ips_screen", "wire_teal", trim_scene=True, trim_prev=True)

# Right-Side Coaxial Hood Pitch Servo Mounting Bracket ('turret_servo_cradle_and_linkage_right_s6_hood')
scradle = make_box_mm(-86.0, -86.0, 368.0, 56.0, 50.0, 8.0)
s_gusset = make_box_mm(-86.0, -63.5, 356.0, 56.0, 5.0, 32.0)
tb.booleanOperation(scradle, s_gusset, fusion.BooleanTypes.UnionBooleanType)
add_solid(scradle, "turret_servo_cradle_and_linkage_right_s6_hood", "mount_slate", trim_scene=True, trim_prev=True)

# CLEAN CONCAVE U-CRADLE MOTOR SADDLES ('turret_flywheel_motor_saddle_left_m5', '_right_m6'):
for side_sign, mx_c, m_lbl in ((1.0, -24.0, "left_m5"), (-1.0, -108.0, "right_m6")):
    tsaddle = make_box_mm(mx_c, side_sign * 136.0, 239.0, 58.0, 84.0, 34.0)
    mcyl_cut = make_cyl_mm(mx_c, side_sign * 85.0, 266.0, mx_c, side_sign * 210.0, 266.0, 26.6)
    tb.booleanOperation(tsaddle, mcyl_cut, fusion.BooleanTypes.DifferenceBooleanType)
    add_solid(tsaddle, f"turret_flywheel_motor_saddle_{m_lbl}", "pulley_alu", trim_scene=False, trim_prev=True)

# 6mm CNC Aluminum Outboard Double-Shear & Motor-Faceplate Bulkhead Plates at Y = ±87.2 mm (|Y| in [84.7, 89.7 mm]):
# - LEFT PLATE ('turret_outboard_double_shear_plate_left'): Compact X in [-56.0, +56.0 mm] supporting Left Motor M5 (-24, 266)
#   and the Front Main Flywheel (+32, 324), stopping 25.0 mm IN FRONT OF Gate Servo S3 (X in [-135.2, -80.8]) so it NEVER clips!
# - RIGHT PLATE ('turret_outboard_double_shear_plate_right'): Full-span X in [-168.0, +56.0 mm] supporting Right Motor M6 (-108, 266),
#   the Front Main Flywheel (+32, 324), and the TRUE CURVED CONCENTRIC ARC SLOT for the Rear Counter-Wheel (-123.5, 324)
#   with ZERO unnecessary rectangular cutouts or unused circular holes!
ds_left = make_box_mm(0.0, 87.2, 280.0, 112.0, 5.0, 116.0)
ds_left_foot = make_box_mm(0.0, 92.5, 225.0, 112.0, 16.0, 6.0)
tb.booleanOperation(ds_left, ds_left_foot, fusion.BooleanTypes.UnionBooleanType)
for st_x in (-46.0, 44.0):
    st_post_l = make_cyl_mm(st_x, 61.0, 324.0, st_x, 87.2, 324.0, 5.5)
    tb.booleanOperation(ds_left, st_post_l, fusion.BooleanTypes.UnionBooleanType)
# Only the 2 actual shaft/motor bores on the Left Plate: Main Flywheel (+32, 324) and Left Motor M5 (-24, 266):
tb.booleanOperation(ds_left, make_cyl_mm(32.0, 78.0, 324.0, 32.0, 96.0, 324.0, 4.1), fusion.BooleanTypes.DifferenceBooleanType)
tb.booleanOperation(ds_left, make_cyl_mm(-24.0, 78.0, 266.0, -24.0, 96.0, 266.0, 11.8), fusion.BooleanTypes.DifferenceBooleanType)
add_solid(ds_left, "turret_outboard_double_shear_plate_left", "pulley_alu", trim_scene=True, trim_prev=True)

ds_right = make_box_mm(-56.0, -87.2, 280.0, 224.0, 5.0, 116.0)
ds_right_foot = make_box_mm(-56.0, -92.5, 225.0, 180.0, 16.0, 6.0)
tb.booleanOperation(ds_right, ds_right_foot, fusion.BooleanTypes.UnionBooleanType)
for st_x in (-68.0, 44.0):
    st_post_r = make_cyl_mm(st_x, -87.2, 324.0, st_x, -61.0, 324.0, 5.5)
    tb.booleanOperation(ds_right, st_post_r, fusion.BooleanTypes.UnionBooleanType)
# TRUE CURVED CONCENTRIC ARC SLOT on the Right Double-Shear Plate centered at Motor M6 (-108.0, 266.0):
arc_slot_outer_r = make_concentric_arc_slot_cutter_mm(-108.0, 266.0, c_arm_len, 104.96, 128.5, -96.0, -78.0, 4.6)
tb.booleanOperation(ds_right, arc_slot_outer_r, fusion.BooleanTypes.DifferenceBooleanType)
# Only the 2 actual circular bores on the Right Plate: Main Flywheel (+32, 324) and Right Motor M6 (-108, 266):
tb.booleanOperation(ds_right, make_cyl_mm(32.0, -96.0, 324.0, 32.0, -78.0, 324.0, 4.1), fusion.BooleanTypes.DifferenceBooleanType)
tb.booleanOperation(ds_right, make_cyl_mm(-108.0, -96.0, 266.0, -108.0, -78.0, 266.0, 11.8), fusion.BooleanTypes.DifferenceBooleanType)
add_solid(ds_right, "turret_outboard_double_shear_plate_right", "pulley_alu", trim_scene=True, trim_prev=True)

# ------------------------------------------------------------------------------
# 1G. 3-WHEEL DEAD-WHEEL ODOMETRY CRADLES & PINPOINT ODOMETRY COMPUTER MOUNT
#     (Supporting all 3 full-detail 95-body goBILDA 4-Bar Odometry Pod occurrences!):
# ------------------------------------------------------------------------------
odom_cradle_l = make_box_mm(-118.0, 118.0, 34.0, 68.0, 28.0, 28.0)
add_solid(odom_cradle_l, "odom_pod_left_x_rail_cradle", "pulley_alu", trim_scene=True, trim_prev=True)

odom_cradle_r = make_box_mm(-108.0, -104.0, 50.0, 76.0, 56.0, 58.0)
add_solid(odom_cradle_r, "odom_pod_right_x_and_pinpoint_cradle", "pulley_alu", trim_scene=True, trim_prev=True)

odom_cradle_c = make_box_mm(-156.0, 0.0, 36.0, 24.0, 88.0, 28.0)
add_solid(odom_cradle_c, "odom_pod_center_y_strafe_rear_cradle", "pulley_alu", trim_scene=True, trim_prev=True)

# ------------------------------------------------------------------------------
# 1H. BATTERY CRADLE TRAY & HOLD-DOWN STRAP + REV HUB MOUNTS + RULE R603/G419.A
#     EXTERIOR-ACCESSIBLE PROTECTED MAIN POWER SWITCH ASSEMBLY (Wires removed!):
# ------------------------------------------------------------------------------
bat_tray = make_box_mm(-154.0, 0.0, 80.0, 68.0, 264.0, 24.0)
add_solid(bat_tray, "gobilda_12v_battery_cradle_tray", "mount_slate", trim_scene=True, trim_prev=True)

bat_strap_top = make_box_mm(-148.0, 15.0, 122.0, 24.0, 234.0, 4.0)
bat_strap_l = make_box_mm(-148.0, 130.5, 106.0, 24.0, 4.0, 32.0)
bat_strap_r = make_box_mm(-148.0, -101.0, 106.0, 24.0, 4.0, 32.0)
tb.booleanOperation(bat_strap_top, bat_strap_l, fusion.BooleanTypes.UnionBooleanType)
tb.booleanOperation(bat_strap_top, bat_strap_r, fusion.BooleanTypes.UnionBooleanType)
add_solid(bat_strap_top, "gobilda_12v_battery_hold_down_strap", "nectar_orange", trim_scene=True, trim_prev=True)

for side_sign, h_lbl in ((-1.0, "rev_control_hub_mount_plate_right"), (1.0, "rev_expansion_hub_mount_plate_left")):
    hplate = make_box_mm(-124.0, side_sign * 181.5, 162.0, 108.0, 6.0, 124.0)
    add_solid(hplate, h_lbl, "pulley_alu", trim_scene=True, trim_prev=True)

# Rule R603.B & G419.A Protected TOP-DECK UPWARD-FACING Main Power Switch Assembly mounted on top of the
# horizontal CNC Turret Mounting Deck Plate outboard on the Left Top Deck ('cnc_turret_mounting_deck_plate_clean'
# at X = -114.0 mm, Y = +146.0 mm, Z = 222.0..240.0 mm — 19.0 mm outboard of the Gate Servo Bracket 's3_bridge'
# so they NEVER touch or clip, facing straight UPWARDS +Z inside a 4-sided CNC aluminum protective bezel collar!):
sw_plate = make_box_mm(-114.0, 146.0, 226.0, 52.0, 40.0, 8.0)
add_solid(sw_plate, "rev_main_power_switch_panel", "comb_dark", trim_scene=True, trim_prev=True)

sw_rocker = make_box_mm(-114.0, 146.0, 233.0, 24.0, 18.0, 6.0)
add_solid(sw_rocker, "rev_main_power_switch_guard_red", "gate_crimson", trim_scene=True, trim_prev=True)

sw_cage_f = make_box_mm(-85.0, 146.0, 231.0, 6.0, 46.0, 18.0)
sw_cage_b = make_box_mm(-143.0, 146.0, 231.0, 6.0, 46.0, 18.0)
sw_cage_l = make_box_mm(-114.0, 166.0, 231.0, 52.0, 6.0, 18.0)
sw_cage_r = make_box_mm(-114.0, 126.0, 231.0, 52.0, 6.0, 18.0)
tb.booleanOperation(sw_cage_f, sw_cage_b, fusion.BooleanTypes.UnionBooleanType)
tb.booleanOperation(sw_cage_f, sw_cage_l, fusion.BooleanTypes.UnionBooleanType)
tb.booleanOperation(sw_cage_f, sw_cage_r, fusion.BooleanTypes.UnionBooleanType)
add_solid(sw_cage_f, "main_power_switch_impact_guard_cage", "pulley_alu", trim_scene=True, trim_prev=True)

bf.finishEdit()

# ==============================================================================
# STEP 2: ASSIGN BODY APPEARANCES DIRECTLY BEFORE FINAL TRANSFORM2 EDITS!
# ==============================================================================
palette = {
    "emerald": get_or_make_color("Hermes_Funnel_Emerald_Left", 16, 185, 129),
    "teal_dark": get_or_make_color("Hermes_Funnel_Teal_Right", 13, 148, 136),
    "compliant_teal": get_or_make_color("Hermes_Compliant_Wheel_Teal", 20, 184, 166),
    "silver": get_or_make_color("Hermes_Machined_Silver_Arm", 226, 232, 240),
    "comb_dark": get_or_make_color("Hermes_Comb_Charcoal", 30, 41, 59),
    "belt_black": get_or_make_color("Hermes_HTD5M_Belt_Black", 15, 23, 42),
    "pulley_alu": get_or_make_color("Hermes_Pulley_Aluminum", 203, 213, 225),
    "chassis_alu": get_or_make_color("Hermes_GoBilda_Chassis_Alu", 148, 163, 184),
    "amber_chute": get_or_make_color("Hermes_Chute_Gold_Amber", 245, 158, 11),
    "nectar_orange": get_or_make_color("Hermes_Nectar_Ball_92mm_Orange", 249, 115, 22),
    "shroud_violet": get_or_make_color("Hermes_Shroud_Violet", 167, 139, 250),
    "mount_slate": get_or_make_color("Hermes_Mount_Slate", 71, 85, 105),
    "gate_crimson": get_or_make_color("Hermes_Gate_Crimson", 239, 68, 68),
    "hood_cyan": get_or_make_color("Hermes_Hood_Cyan", 14, 165, 233),
    "wire_teal": get_or_make_color("Hermes_Wire_Teal", 45, 212, 191),
}
for b_name, ckey in body_colors:
    b_obj = up_comp.bRepBodies.itemByName(b_name)
    if b_obj:
        b_obj.appearance = palette[ckey]

# ==============================================================================
# STEP 3: SPAWN OR REUSE 2ND & 3RD 95-BODY ODOMETRY PODS, RE-APPLY COTS
#         VISIBILITY & TRANSFORMS AFTER APPEARANCES, AND COMMIT SNAPSHOT!
# ==============================================================================
if opod_left is not None:
    opod_rx_occ = root.occurrences.itemByName("official_gobilda_4bar_odometry_pod:2")
    if opod_rx_occ is None:
        opod_rx_occ = root.occurrences.addExistingComponent(opod_left.component, mat_odom_rx)
    opod_cy_occ = root.occurrences.itemByName("official_gobilda_4bar_odometry_pod:3")
    if opod_cy_occ is None:
        opod_cy_occ = root.occurrences.addExistingComponent(opod_left.component, mat_odom_cy)

if u10_rear_occ is not None:
    for cand_name in ("gobilda_1120_10hole_rear:3", "gobilda_1120_10hole_rear:2"):
        cand = root.occurrences.itemByName(cand_name)
        if cand is not None:
            u10_mid_occ = cand
            break
    if u10_mid_occ is None:
        u10_mid_occ = root.occurrences.addExistingComponent(u10_rear_occ.component, mat_u10_mid)

apply_cots_visibility_and_transforms()
adsk.doEvents()

# ==============================================================================
# STEP 4: IN-PROCESS FULL-ASSEMBLY B-REP COLLISION & ENVELOPE AUDIT
# ==============================================================================
bodies = []
xs_min, ys_min, zs_min = [], [], []
xs_max, ys_max, zs_max = [], [], []


def collect_audit_bodies(occ, path=""):
    cur = f"{path}/{occ.name}" if path else occ.name
    if not occ.isLightBulbOn:
        return
    if occ.name in ("gobilda_1120_10hole_rear:2", "gobilda_1120_10hole_rear:3") and path == "":
        return
    if any(k in occ.name for k in ("OR:", "IR:", "Balls:", "Cage:", "Shield:", "M(90", "Encoder v1", "M2.5")):
        return
    for i in range(occ.bRepBodies.count):
        b = occ.bRepBodies.item(i)
        if b.isLightBulbOn and b.isSolid:
            bb = b.boundingBox
            bmin_x, bmin_y, bmin_z = bb.minPoint.x * 10.0, bb.minPoint.y * 10.0, bb.minPoint.z * 10.0
            bmax_x, bmax_y, bmax_z = bb.maxPoint.x * 10.0, bb.maxPoint.y * 10.0, bb.maxPoint.z * 10.0
            xs_min.append(bmin_x)
            ys_min.append(bmin_y)
            zs_min.append(bmin_z)
            xs_max.append(bmax_x)
            ys_max.append(bmax_y)
            zs_max.append(bmax_z)
            bodies.append((cur, b.name, occ.component.name, b, (bmin_x, bmin_y, bmin_z, bmax_x, bmax_y, bmax_z)))
    for i in range(occ.childOccurrences.count):
        collect_audit_bodies(occ.childOccurrences.item(i), cur)


for i in range(root.occurrences.count):
    collect_audit_bodies(root.occurrences.item(i))


def part_group(path_str, body_name):
    parts = path_str.split("/")
    if parts[0].startswith("official_gobilda_4bar_odometry_pod"):
        return parts[0]
    if parts[0].startswith("Variable_Intake_And_Asymmetrical_Funnel"):
        return body_name
    if parts[0] == "robot_base_core:1" and len(parts) >= 3:
        return parts[2]
    if len(parts) >= 2:
        return parts[1]
    return parts[0]


overlaps = []
n = len(bodies)
for i in range(n):
    p1, bn1, cn1, b1, bb1 = bodies[i]
    g1 = part_group(p1, bn1)
    for j in range(i + 1, n):
        p2, bn2, cn2, b2, bb2 = bodies[j]
        g2 = part_group(p2, bn2)
        if g1 == g2:
            continue
        if (
            bb1[0] < bb2[3] - 0.05
            and bb1[3] > bb2[0] + 0.05
            and bb1[1] < bb2[4] - 0.05
            and bb1[4] > bb2[1] + 0.05
            and bb1[2] < bb2[5] - 0.05
            and bb1[5] > bb2[2] + 0.05
        ):
            try:
                c1 = tb.copy(b1)
                c2 = tb.copy(b2)
                tb.booleanOperation(c1, c2, fusion.BooleanTypes.IntersectionBooleanType)
                vol_mm3 = c1.volume * 1000.0
                if vol_mm3 > 0.5:
                    overlaps.append({"pair": [g1, g2], "vol_mm3": round(vol_mm3, 2)})
            except Exception as e:
                overlaps.append({"pair": [g1, g2], "err": str(e)})

mn = [round(min(xs_min), 2), round(min(ys_min), 2), round(min(zs_min), 2)]
mx = [round(max(xs_max), 2), round(max(ys_max), 2), round(max(zs_max), 2)]
sz = [round(mx[0] - mn[0], 2), round(mx[1] - mn[1], 2), round(mx[2] - mn[2], 2)]

result = {
    "status": "upgraded_variable_intake_full_electronics_and_anchored_turret",
    "new_bodies_count": up_comp.bRepBodies.count,
    "total_visible_bodies": n,
    "overlaps_count": len(overlaps),
    "overlaps": overlaps,
    "envelope_mm": {"min": mn, "max": mx, "size": sz, "r102_pass": all(s <= 457.2 for s in sz)},
}
