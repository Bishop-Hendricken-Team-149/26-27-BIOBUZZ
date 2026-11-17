"""
Fast in-place Stage 1 Builder for Autodesk Fusion 360 (via HermesFusionBridge):
Updates/places the FTC Team 149 BIOBUZZ 'Project HIVE-QUEEN' Stage 1 Strafer Drivetrain & Tower Superstructure
in <0.5s by updating existing occurrence transforms in-place instead of deleting timeline nodes.
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

# Clear any existing As-Built Joints so the joint solver never locks occurrences to old relative offsets!
while root.asBuiltJoints.count > 0:
    root.asBuiltJoints.item(0).deleteMe()

# Delete drivetrain_hardware and obsolete mid-throat bridge occurrences first before setting transforms
for del_k in ("drivetrain_hardware", "upper_turret_bridge_mid_10h", "qb_upper_bridge_mid_left", "qb_upper_bridge_mid_right"):
    if del_k in STAGE1_OCCS and STAGE1_OCCS[del_k] and STAGE1_OCCS[del_k].isValid:
        STAGE1_OCCS[del_k].deleteMe()
        del STAGE1_OCCS[del_k]

# Create Native B-Rep Drivetrain Hardware Component FIRST before setting transform2 on STEP occurrences!
hw_occ = root.occurrences.addNewComponent(core.Matrix3D.create())
hw_comp = hw_occ.component
hw_comp.name = "Strafer_Bevel_And_Axle_Hardware"
STAGE1_OCCS["drivetrain_hardware"] = hw_occ

tb = fusion.TemporaryBRepManager.get()
base_feat = hw_comp.features.baseFeatures.add()
base_feat.startEdit()

for wx, x_sign in ((144.0, 1.0), (-144.0, -1.0)):
    for side in (1.0, -1.0):
        p1 = core.Point3D.create(wx / 10.0, (side * 134.0) / 10.0, 4.8)
        p2 = core.Point3D.create(wx / 10.0, (side * 212.0) / 10.0, 4.8)
        axle_solid = tb.createCylinderOrCone(p1, 0.4, p2, 0.4)
        b_ax = hw_comp.bRepBodies.add(axle_solid, base_feat)
        b_ax.name = f"rex_axle_{int(wx)}_{int(side)}"

        for wall_y, fl_dir in ((132.0, -1.0), (180.0, 1.0)):
            fp1 = core.Point3D.create(wx / 10.0, (side * wall_y) / 10.0, 4.8)
            fp2 = core.Point3D.create(wx / 10.0, (side * (wall_y + fl_dir * 2.0)) / 10.0, 4.8)
            flange = tb.createCylinderOrCone(fp1, 0.85, fp2, 0.85)
            tb.booleanOperation(flange, tb.copy(axle_solid), fusion.BooleanTypes.DifferenceBooleanType)
            b_brg = hw_comp.bRepBodies.add(flange, base_feat)
            b_brg.name = f"flanged_bearing_{int(wx)}_{int(side * wall_y)}"

        gp1 = core.Point3D.create((wx - x_sign * 11.5) / 10.0, (side * 156.0) / 10.0, 4.8)
        gp2 = core.Point3D.create((wx - x_sign * 5.8) / 10.0, (side * 156.0) / 10.0, 4.8)
        gear_mot = tb.createCylinderOrCone(gp1, 1.15, gp2, 0.65)
        mot_bore = tb.createCylinderOrCone(
            core.Point3D.create((wx - x_sign * 13.0) / 10.0, (side * 156.0) / 10.0, 4.8),
            0.61,
            core.Point3D.create((wx - x_sign * 5.0) / 10.0, (side * 156.0) / 10.0, 4.8),
            0.61,
        )
        tb.booleanOperation(gear_mot, mot_bore, fusion.BooleanTypes.DifferenceBooleanType)
        b_gm = hw_comp.bRepBodies.add(gear_mot, base_feat)
        b_gm.name = f"miter_gear_motor_{int(wx)}_{int(side)}"

        ap1 = core.Point3D.create(wx / 10.0, (side * 144.0) / 10.0, 4.8)
        ap2 = core.Point3D.create(wx / 10.0, (side * 149.5) / 10.0, 4.8)
        gear_ax = tb.createCylinderOrCone(ap1, 1.15, ap2, 0.60)
        tb.booleanOperation(gear_ax, tb.copy(axle_solid), fusion.BooleanTypes.DifferenceBooleanType)
        b_ga = hw_comp.bRepBodies.add(gear_ax, base_feat)
        b_ga.name = f"miter_gear_axle_{int(wx)}_{int(side)}"

base_feat.finishEdit()

# Hide any library template occurrences that aren't in STAGE1_OCCS
tracked_occs = {occ.name for occ in STAGE1_OCCS.values() if occ and occ.isValid}
for i in range(root.occurrences.count):
    occ = root.occurrences.item(i)
    if occ.name not in tracked_occs:
        occ.isLightBulbOn = False


def place_part(lib_key: str, world_mat: core.Matrix3D, label: str) -> fusion.Occurrence:
    existing = STAGE1_OCCS.get(label)
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
    STAGE1_OCCS[label] = occ
    return occ


def u_chan_centered_mat(length_mm: float) -> core.Matrix3D:
    return mat_pos_mm(0.0, -length_mm / 2.0, 21.5)


U10_LOW_CENTERED = mat_mul(mat_rot_deg((0, 1, 0), -90.0), mat_pos_mm(18.0, -132.0, 21.5))
QB_CENTERED = mat_pos_mm(0.0, -6.0, 0.0)
MOT_FACE_PLUS_X = mat_mul(mat_rot_deg((0, 0, 1), -90.0), mat_pos_mm(37.171, -101.735, 11.078))
MEC_CENTERED = mat_mul(mat_rot_deg((1, 0, 0), 90.0), mat_pos_mm(0.0, 0.0, 7.515))

# 1. Base 17-Hole Side U-Channels (Open-Top 'U' facing +Z so motors & gears drop in!)
u17_open_top_along_x = mat_mul(
    mat_rot_deg((1, 0, 0), 180.0),
    mat_rot_deg((0, 0, 1), 90.0),
    u_chan_centered_mat(432.0),
)

left_rail = place_part("u17", mat_mul(mat_pos_mm(0.0, 156.0, 48.0), u17_open_top_along_x), "left_rail_17h")
left_rail.isGrounded = True

right_rail = place_part("u17", mat_mul(mat_pos_mm(0.0, -156.0, 48.0), u17_open_top_along_x), "right_rail_17h")

# 2. Base Crossbars + Quad Block Mounts
rear_cross = place_part(
    "u10",
    mat_mul(mat_pos_mm(-192.0, 0.0, 48.0), u_chan_centered_mat(264.0)),
    "rear_crossbar_10h",
)
qb_rear_l = place_part(
    "quad_block",
    mat_mul(mat_pos_mm(-192.0, 126.0, 48.0), mat_rot_deg((0, 0, 1), 180.0), QB_CENTERED),
    "qb_rear_cross_left",
)
qb_rear_r = place_part(
    "quad_block",
    mat_mul(mat_pos_mm(-192.0, -126.0, 48.0), QB_CENTERED),
    "qb_rear_cross_right",
)
mid_cross = place_part(
    "u10_low",
    mat_mul(mat_pos_mm(-48.0, 0.0, 30.0), U10_LOW_CENTERED),
    "mid_belly_brace_10h_low",
)

# 3. Vertical U-Channel Tower Stanchions + Quad Block Mounts
#    - Rear 5H at X = -168.0 (Hole 2, uncovering Rear Bevel Gears at X=-144!)
#    - Mid 5H at X = 0.0 (Hole 9, giving 120mm clear span [-144..-24] for 103mm REV Hubs at X=-84!)
#    - Front 4H at X = +192.0 (Hole 17, uncovering Front Bevel Gears at X=+144!)
u5_vert_mat = mat_mul(mat_rot_deg((0, 0, 1), 90.0), mat_rot_deg((1, 0, 0), 90.0), u_chan_centered_mat(144.0))
u4_vert_mat = mat_mul(mat_rot_deg((0, 0, 1), 90.0), mat_rot_deg((1, 0, 0), 90.0), u_chan_centered_mat(120.0))
QB_FACE_DOWN = mat_mul(mat_rot_deg((1, 0, 0), -90.0), QB_CENTERED)

for tx, tname in ((-168.0, "rear"), (0.0, "mid")):
    for ty, sname in ((156.0, "left"), (-156.0, "right")):
        place_part("u5", mat_mul(mat_pos_mm(tx, ty, 144.0), u5_vert_mat), f"tower_5h_{tname}_{sname}")
        place_part("quad_block", mat_mul(mat_pos_mm(tx, ty, 78.0), QB_FACE_DOWN), f"qb_tower_base_{tname}_{sname}")

for ty, sname in ((156.0, "left"), (-156.0, "right")):
    place_part("u4", mat_mul(mat_pos_mm(192.0, ty, 132.0), u4_vert_mat), f"front_upright_4h_{sname}")
    place_part("quad_block", mat_mul(mat_pos_mm(192.0, ty, 78.0), QB_FACE_DOWN), f"qb_front_base_{sname}")

# 4. Upper Bridges (Rear 10H at X=-168, Z=192; Mid 10H at X=0, Z=192; Front 10H Low-Side Arch at X=+192, Z=198)
upper_rear_bridge = place_part(
    "u10",
    mat_mul(mat_pos_mm(-168.0, 0.0, 192.0), u_chan_centered_mat(264.0)),
    "upper_turret_bridge_10h",
)
place_part(
    "quad_block",
    mat_mul(mat_pos_mm(-168.0, 126.0, 192.0), mat_rot_deg((0, 0, 1), 180.0), QB_CENTERED),
    "qb_upper_bridge_left",
)
place_part(
    "quad_block",
    mat_mul(mat_pos_mm(-168.0, -126.0, 192.0), QB_CENTERED),
    "qb_upper_bridge_right",
)

U10_LOW_ARCH_TOP_WEB = mat_mul(mat_rot_deg((0, 1, 0), 90.0), mat_pos_mm(18.0, -132.0, 21.5))
front_arch = place_part(
    "u10_low",
    mat_mul(mat_pos_mm(192.0, 0.0, 198.0), U10_LOW_ARCH_TOP_WEB),
    "front_portal_arch_10h_low",
)

# 5. 4x Drive Motors (5203-2402-0019) + 4x Motor Quad Blocks (1201-0043-0002)
QB_FACE_PLUS_X = mat_mul(mat_rot_deg((0, 0, 1), -90.0), QB_CENTERED)
QB_FACE_MINUS_X = mat_mul(mat_rot_deg((0, 0, 1), 90.0), QB_CENTERED)

drive_configs = [
    ("m1_front_left", 116.0, 156.0, 0.0, 122.0, QB_FACE_MINUS_X),
    ("m2_rear_left", -116.0, 156.0, 180.0, -122.0, QB_FACE_PLUS_X),
    ("m3_front_right", 116.0, -156.0, 0.0, 122.0, QB_FACE_MINUS_X),
    ("m4_rear_right", -116.0, -156.0, 180.0, -122.0, QB_FACE_PLUS_X),
]
for mname, mx, my, yaw, qbx, qb_mat in drive_configs:
    place_part(
        "motor_5203",
        mat_mul(mat_pos_mm(mx, my, 48.0), mat_rot_deg((0, 0, 1), yaw), MOT_FACE_PLUS_X),
        f"motor_5203_{mname}",
    )
    place_part(
        "quad_block",
        mat_mul(mat_pos_mm(qbx, my, 48.0), qb_mat),
        f"qb_motor_mount_{mname}",
    )

# 6. 4x Official 96mm Mecanum Wheels (True Holonomic 'X' Roller Pattern!)
wheel_configs = [
    ("wheel_front_left", "mec_left", 144.0, 203.5, 0.0),
    ("wheel_front_right", "mec_right", 144.0, -203.5, 180.0),
    ("wheel_rear_left", "mec_right", -144.0, 203.5, 0.0),
    ("wheel_rear_right", "mec_left", -144.0, -203.5, 180.0),
]
for wname, wkey, wx, wy, wyaw in wheel_configs:
    w_mat = mat_mul(mat_pos_mm(wx, wy, 48.0), mat_rot_deg((0, 0, 1), wyaw), MEC_CENTERED)
    place_part(wkey, w_mat, wname)

# 8. Official Electronics & Localization Suite (Protected Inboard Mounting!)
place_part(
    "rev_ctrl",
    mat_mul(mat_pos_mm(-84.0, 131.5, 144.0), mat_rot_deg((0, 0, 1), 180.0)),
    "rev_control_hub",
)
place_part(
    "rev_exp",
    mat_pos_mm(-84.0, -131.5, 144.0),
    "rev_expansion_hub",
)

batt_mat = mat_mul(
    mat_pos_mm(-192.0, 0.0, 116.0),
    mat_rot_deg((0, 0, 1), 90.0),
    mat_pos_mm(-58.9, 0.0, 0.0),
)
place_part("battery", batt_mat, "gobilda_12v_battery")
place_part("pinpoint", mat_pos_mm(-48.0, -96.0, 44.0), "gobilda_pinpoint_v2")
place_part("odo_pod", mat_pos_mm(-106.0, -127.75, 30.3), "gobilda_4bar_odometry_pod")

if design.snapshots.hasPendingSnapshot:
    design.snapshots.add()


# Helper to create/get a custom RGB appearance in design.appearances
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


app_yellow = get_or_make_color("Hermes_YellowJacket_Gold", 245, 180, 15)
app_brass = get_or_make_color("Hermes_Miter_Brass", 218, 165, 32)
app_steel = get_or_make_color("Hermes_Bright_Steel", 226, 232, 240)
app_bearing = get_or_make_color("Hermes_Dark_Bearing", 51, 65, 85)
app_orange = get_or_make_color("Hermes_REV_Orange", 234, 88, 12)
app_purple = get_or_make_color("Hermes_Pinpoint_Purple", 147, 51, 234)
app_qb = get_or_make_color("Hermes_QuadBlock_BlueGray", 71, 85, 105)

# Color-code Drivetrain Hardware bodies (axles, bearings, brass bevel gears)
hw_comp_ref = STAGE1_OCCS["drivetrain_hardware"].component
for b_i in range(hw_comp_ref.bRepBodies.count):
    body = hw_comp_ref.bRepBodies.item(b_i)
    if "miter_gear" in body.name:
        body.appearance = app_brass
    elif "rex_axle" in body.name:
        body.appearance = app_steel
    elif "flanged_bearing" in body.name:
        body.appearance = app_bearing

# Color-code 5203 Motor Jacket Shield & Quad Blocks so they pop visually inside the open-top rails
mot_comp_ref = STATE["lib"]["motor_5203"]
for o_i in range(mot_comp_ref.occurrences.count):
    ch = mot_comp_ref.occurrences.item(o_i)
    if "shield" in ch.name.lower():
        for b_i in range(ch.component.bRepBodies.count):
            b_sh = ch.component.bRepBodies.item(b_i)
            b_sh.appearance = app_yellow
            for f_i in range(b_sh.faces.count):
                b_sh.faces.item(f_i).appearance = app_yellow

for b_i in range(STATE["lib"]["quad_block"].bRepBodies.count):
    STATE["lib"]["quad_block"].bRepBodies.item(b_i).appearance = app_qb

for b_i in range(STATE["lib"]["pinpoint"].bRepBodies.count):
    STATE["lib"]["pinpoint"].bRepBodies.item(b_i).appearance = app_purple

# Ground the primary reference rail and hide Joint Icon clutter in the 3D viewport!
STAGE1_OCCS["left_rail_17h"].isGrounded = True
root.isJointsFolderLightBulbOn = False
if design.snapshots.hasPendingSnapshot:
    design.snapshots.add()

result = {
    "stage1_parts_placed": len(STAGE1_OCCS),
    "as_built_joints": root.asBuiltJoints.count,
    "joints_folder_hidden": not root.isJointsFolderLightBulbOn,
}
