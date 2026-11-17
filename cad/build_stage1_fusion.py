"""
Stage 1 Builder for Autodesk Fusion 360 (via HermesFusionBridge):
Builds the FTC Team 149 BIOBUZZ 'Project HIVE-QUEEN' Stage 1 Strafer Drivetrain & Tower Superstructure
using real vendor STEP components cached in STATE['lib'], exact 24mm/8mm goBILDA grid transforms,
custom B-Rep bevel gears & flanged bearings, and native Fusion 360 As-Built Joints (Rigid + Revolute).
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


# 1. Hide the hidden library template occurrences (indices 0..N that were imported into STATE['lib'])
lib_comp_names = {c.name for c in STATE["lib"].values()}
to_delete = []
kept_lib_occs = {}
for i in range(root.occurrences.count):
    occ = root.occurrences.item(i)
    comp = occ.component
    if comp.name in lib_comp_names and comp.name not in kept_lib_occs:
        occ.isLightBulbOn = False
        kept_lib_occs[comp.name] = occ
    else:
        to_delete.append(occ)

for occ in to_delete:
    occ.deleteMe()

# Also delete any previous custom root bodies or joints from earlier runs
while root.joints.count > 0:
    root.joints.item(0).deleteMe()
while root.asBuiltJoints.count > 0:
    root.asBuiltJoints.item(0).deleteMe()
while root.bRepBodies.count > 0:
    root.bRepBodies.item(0).deleteMe()


def place_part(lib_key: str, world_mat: core.Matrix3D, label: str) -> fusion.Occurrence:
    comp = STATE["lib"][lib_key]
    occ = root.occurrences.addExistingComponent(comp, world_mat)
    occ.isLightBulbOn = True
    STATE.setdefault("stage1_occs", {})[label] = occ
    return occ


# Canonical centering transforms (from native STEP frame -> centered part datum):
# - Standard goBILDA 1120 U-Channel of length L:
#   Native has X in [-24, +24], Y in [0, L], Z in [-45.5, +2.5] (bore center at Z = -21.5).
#   T_u_center(L) puts geometric center at (0,0,0) with length along Y, width along X, web on +Z [+21.5..+24.0].
def u_chan_centered_mat(length_mm: float) -> core.Matrix3D:
    return mat_pos_mm(0.0, -length_mm / 2.0, 21.5)


# - Low-Side goBILDA 1121-0010-0264 U-Channel (264mm):
#   Native has X in [-24, -12], Y in [0, 264], Z in [-45.5, +2.5].
#   Rotating -90° about Y maps native -X [-24..-12] -> +Z [+12..+24] (or +90° about Y maps -X -> -Z).
#   Specifically: Pos(18.0, -132.0, 21.5) centers it in [-6..+6, -132..+132, -24..+24] with web at X = -6..-3.5.
#   Then RotY(-90°) puts web at Z = -6..-3.5 (bottom) and flanges rising to Z = +6 (U opening UP), width in X [-24..+24], length in Y [-132..+132]!
U10_LOW_CENTERED = mat_mul(mat_rot_deg((0, 1, 0), -90.0), mat_pos_mm(18.0, -132.0, 21.5))

# - Quad Block 1201-0043-0002:
#   Native has X in [-21.5, +21.5], Y in [0.0, +12.0] (pattern face at Y=0, side M4 holes at Y=8.0), Z in [-21.5, +21.5].
#   Translating by (0, -6.0, 0) centers it in X[-21.5..+21.5], Y[-6..+6], Z[-21.5..+21.5] with pattern face at Y = -6.0!
QB_CENTERED = mat_pos_mm(0.0, -6.0, 0.0)

# - Motor 5203-2402-0019:
#   Translating by (+37.171, -101.735, +11.078) puts front gearbox faceplate at (0,0,0), shaft along +Y [0..+23.57], body along -Y [-116.56..0].
#   Rotating -90° around Z maps shaft -> +X [0..+23.57] and body -> -X [-116.56..0]!
MOT_FACE_PLUS_X = mat_mul(mat_rot_deg((0, 0, 1), -90.0), mat_pos_mm(37.171, -101.735, 11.078))

# - 96mm Mecanum Wheels (mec_left / mec_right):
#   Native has Z in [-30.171, +15.144] (midpoint at Z = -7.515).
#   Pos(0, 0, +7.515) centers wheel at (0,0,0) with axle along Z; RotX(90°) aligns axle along Y!
MEC_CENTERED = mat_mul(mat_rot_deg((1, 0, 0), 90.0), mat_pos_mm(0.0, 0.0, 7.515))

STAGE1_OCCS = {}
STATE["stage1_occs"] = STAGE1_OCCS

# ==============================================================================
# 1. BASE 17-HOLE SIDE U-CHANNELS (X in [-216, +216], Y = ±156, Z = 48 [24..72])
# ==============================================================================
# Rotate centered U17 by 90° around Z so length Y -> X [-216..+216] and width X -> Y [-24..+24]
u17_along_x = mat_mul(mat_rot_deg((0, 0, 1), 90.0), u_chan_centered_mat(432.0))

left_rail = place_part("u17", mat_mul(mat_pos_mm(0.0, 156.0, 48.0), u17_along_x), "left_rail_17h")
left_rail.isGrounded = True  # Ground the left main chassis rail as the assembly root reference!

right_rail = place_part("u17", mat_mul(mat_pos_mm(0.0, -156.0, 48.0), u17_along_x), "right_rail_17h")

# ==============================================================================
# 2. BASE CROSSBARS (Rear 10H at X=-192, Z=48; Mid 10H Low-Side at X=+48, Z=30)
#    + QUAD BLOCK MOUNTS BOLTING THEM TO THE SIDE RAILS!
# ==============================================================================
# Rear 10H Crossbar (Hole 1 of 17H rails: X = -192.0, Y in [-132, +132], Z = 48.0)
rear_cross = place_part(
    "u10",
    mat_mul(mat_pos_mm(-192.0, 0.0, 48.0), u_chan_centered_mat(264.0)),
    "rear_crossbar_10h",
)

# 2x Quad Blocks inside the ends of rear_cross (Y = +126 and Y = -126, pattern faces flush at Y = ±132.0!)
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

# Mid 10H Low-Side Belly Cross-Brace at X = +48.0 (Hole 11 of 17H rails), Z = 30.0 (Z in [24, 36]), Y in [-132, +132]
mid_cross = place_part(
    "u10_low",
    mat_mul(mat_pos_mm(48.0, 0.0, 30.0), U10_LOW_CENTERED),
    "mid_belly_brace_10h_low",
)

# ==============================================================================
# 3. VERTICAL U-CHANNEL TOWER STANCHIONS (on exact 24mm grid holes!)
#    - Rear 5H (144mm) Stanchions at X = -144.0 (Hole 3), Y = ±156.0, Z in [72.0, 216.0] (center Z = 144.0)
#    - Mid 5H (144mm) Stanchions at X = -24.0 (Hole 8), Y = ±156.0, Z in [72.0, 216.0] (center Z = 144.0)
#    - Front 4H (120mm) Stanchions at X = +168.0 (Hole 16), Y = ±156.0, Z in [72.0, 192.0] (center Z = 132.0)
#    + 6x QUAD BLOCK MOUNTS tucked inside the bottom of each stanchion (Z in [72.0, 84.0])!
# ==============================================================================
# Orient vertical U-channel so its length Y -> +Z, and its open U faces INWARD (toward Y=0) or along X!
# Wait: if we rotate centered U-channel by +90° around X:
#   native Y [-L/2..+L/2] -> +Z [-L/2..+L/2]
#   native Z [-24..+24] -> -Y (web at Y = -24, open U facing +Y)
#   native X [-24..+24] -> +X [-24..+24]
# Even better: if we also rotate 90° around Z so the two side walls of the vertical stanchion are at Y = ±24 (flush with the base rail's Y = ±24 walls!),
# then the open U of the vertical stanchion faces along X (with its side walls at Y = ±132 and Y = ±180, coplanar with the base rail walls!)
u5_vert_mat = mat_mul(mat_rot_deg((0, 0, 1), 90.0), mat_rot_deg((1, 0, 0), 90.0), u_chan_centered_mat(144.0))
u4_vert_mat = mat_mul(mat_rot_deg((0, 0, 1), 90.0), mat_rot_deg((1, 0, 0), 90.0), u_chan_centered_mat(120.0))

# Quad Block with pattern face pointing DOWN (-Z at Z = -6.0) to sit at Z = 78.0 (Z in [72.0, 84.0] inside vertical stanchion):
# Native pattern face is at Y = -6.0 (normal -Y); rotating -90° around X maps -Y -> -Z!
QB_FACE_DOWN = mat_mul(mat_rot_deg((1, 0, 0), -90.0), QB_CENTERED)

for tx, tname in ((-144.0, "rear"), (-24.0, "mid")):
    for ty, sname in ((156.0, "left"), (-156.0, "right")):
        place_part("u5", mat_mul(mat_pos_mm(tx, ty, 144.0), u5_vert_mat), f"tower_5h_{tname}_{sname}")
        place_part("quad_block", mat_mul(mat_pos_mm(tx, ty, 78.0), QB_FACE_DOWN), f"qb_tower_base_{tname}_{sname}")

for ty, sname in ((156.0, "left"), (-156.0, "right")):
    place_part("u4", mat_mul(mat_pos_mm(168.0, ty, 132.0), u4_vert_mat), f"front_upright_4h_{sname}")
    place_part("quad_block", mat_mul(mat_pos_mm(168.0, ty, 78.0), QB_FACE_DOWN), f"qb_front_base_{sname}")

# ==============================================================================
# 4. UPPER BRIDGES (Rear 10H Crossbar at X=-144, Z=192 + Front 10H Low-Side Arch at X=+168, Z=198)
# ==============================================================================
upper_rear_bridge = place_part(
    "u10",
    mat_mul(mat_pos_mm(-144.0, 0.0, 192.0), u_chan_centered_mat(264.0)),
    "upper_turret_bridge_10h",
)
place_part(
    "quad_block",
    mat_mul(mat_pos_mm(-144.0, 126.0, 192.0), mat_rot_deg((0, 0, 1), 180.0), QB_CENTERED),
    "qb_upper_bridge_left",
)
place_part(
    "quad_block",
    mat_mul(mat_pos_mm(-144.0, -126.0, 192.0), QB_CENTERED),
    "qb_upper_bridge_right",
)

# Front Portal Arch (1121-0010-0264 Low-Side U-Channel) capping the top of the Front Uprights at X=+168, Z=198 (Z in [192, 204])
front_arch = place_part(
    "u10_low",
    mat_mul(mat_pos_mm(168.0, 0.0, 198.0), U10_LOW_CENTERED),
    "front_portal_arch_10h_low",
)

# ==============================================================================
# 5. 4x DRIVE MOTORS (5203-2402-0019) + 4x MOTOR QUAD BLOCKS (1201-0043-0002)
#    Mounted inside the Left & Right 17H Side Rails:
#    - Quad Blocks at X = ±122.0 (spanning |X| in [116.0, 128.0], bolted to the |X|=128.0 M4 holes of Hole 4/14!)
#    - Motor Faceplates bolted flush to the inner face of the Quad Blocks at X = ±116.0!
#      -> Motor bodies extend INWARD along X in [0.0, 116.0] (100% clear of center X=0!)
#      -> Motor 8mm REX shafts extend OUTWARD along X in [116.0, 139.57] (through the Quad Block bore right up to the bevel gear at |X|=136, stopping 4.4mm clear of the wheel axle at |X|=144.0!)
# ==============================================================================
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

# ==============================================================================
# 6. 4x OFFICIAL 96mm MECANUM WHEELS (3606-0000-0096 & 3606-0100-0096)
#    Centered at X = ±144.0 (Holes 3 & 15), Y = ±204.0 (|Y| in [181.3, 226.7], 1.3mm clear of rail at |Y|=180, total width = 453.4mm <= 457.2mm R102!)
# ==============================================================================
wheel_configs = [
    ("wheel_front_left", "mec_left", 144.0, 204.0),
    ("wheel_front_right", "mec_right", 144.0, -204.0),
    ("wheel_rear_left", "mec_right", -144.0, 204.0),
    ("wheel_rear_right", "mec_left", -144.0, -204.0),
]
for wname, wkey, wx, wy in wheel_configs:
    place_part(wkey, mat_mul(mat_pos_mm(wx, wy, 48.0), MEC_CENTERED), wname)

# ==============================================================================
# 7. OFFICIAL ELECTRONICS & LOCALIZATION SUITE (REV Control Hub, Expansion Hub,
#    12V Slim Battery, goBILDA Pinpoint v2, and goBILDA 4-Bar Odometry Pod)
# ==============================================================================
# REV Control Hub (REV-31-1595) & Expansion Hub (REV-31-1153):
# Native bbox: X in [-51.5, +51.5], Y in [0.05, 29.6] (mounting backplate at Y=0!), Z in [-71.5, +71.5]
# Mount vertically against the outer face of the Rear Tower Stanchions (X = -144.0, Y = ±180.0, Z = 144.0):
place_part(
    "rev_ctrl",
    mat_pos_mm(-144.0, 180.5, 144.0),
    "rev_control_hub",
)
place_part(
    "rev_exp",
    mat_mul(mat_pos_mm(-144.0, -180.5, 144.0), mat_rot_deg((0, 0, 1), 180.0)),
    "rev_expansion_hub",
)

# Official 12V Battery (14-0014):
# Native bbox: X in [-55.8, +173.6] (center X = +58.9, length = 229.4), Y in [-22.7, +22.7], Z in [-43.8, 0.0] (bottom at Z = -43.8).
# Orient length along Y centered at Y=0, sitting FLUSH on top of Rear Base Crossbar (Z = 72.0, so Z in [72.0, 115.8]) at X = -192.0!
batt_mat = mat_mul(
    mat_pos_mm(-192.0, 0.0, 116.0),
    mat_rot_deg((0, 0, 1), 90.0),
    mat_pos_mm(-58.9, 0.0, 0.0),
)
place_part("battery", batt_mat, "gobilda_12v_battery")

# Official goBILDA Pinpoint v2 Odometry Computer (3110-0002-0002):
# Native bbox: X in [-20, +20], Y in [-21.3, +21.3], Z in [-8.0, +10.0] (bottom at Z = -8.0).
# Mount flush on top of Mid Belly Brace (X = +48.0, Y = -96.0, Z = 44.0 -> Z_min = 36.0 == top of mid_belly_brace_10h_low!)
place_part("pinpoint", mat_pos_mm(48.0, -96.0, 44.0), "gobilda_pinpoint_v2")

# Official goBILDA 4-Bar Odometry Pod (3110-0001-0001):
# Native bbox: X in [-58.7, +30.8], Y in [-4.2, +39.2], Z in [-30.3, +30.3]
# Mount inside the right inner channel wall (X = -72.0, Y = -131.0, Z = 36.0)
place_part("odo_pod", mat_pos_mm(-72.0, -131.0, 36.0), "gobilda_4bar_odometry_pod")

# ==============================================================================
# 8. CREATE NATIVE FUSION 360 AS-BUILT JOINTS (Rigid Structural + Revolute Wheels)
# ==============================================================================
ab_joints = root.asBuiltJoints
rigid_pairs = [
    ("left_rail_17h", "qb_rear_cross_left"),
    ("qb_rear_cross_left", "rear_crossbar_10h"),
    ("rear_crossbar_10h", "qb_rear_cross_right"),
    ("qb_rear_cross_right", "right_rail_17h"),
    ("left_rail_17h", "mid_belly_brace_10h_low"),
    ("left_rail_17h", "qb_tower_base_rear_left"),
    ("qb_tower_base_rear_left", "tower_5h_rear_left"),
    ("left_rail_17h", "qb_tower_base_mid_left"),
    ("qb_tower_base_mid_left", "tower_5h_mid_left"),
    ("right_rail_17h", "qb_tower_base_rear_right"),
    ("qb_tower_base_rear_right", "tower_5h_rear_right"),
    ("right_rail_17h", "qb_tower_base_mid_right"),
    ("qb_tower_base_mid_right", "tower_5h_mid_right"),
    ("left_rail_17h", "qb_front_base_left"),
    ("qb_front_base_left", "front_upright_4h_left"),
    ("right_rail_17h", "qb_front_base_right"),
    ("qb_front_base_right", "front_upright_4h_right"),
    ("tower_5h_rear_left", "upper_turret_bridge_10h"),
    ("front_upright_4h_left", "front_portal_arch_10h_low"),
]

created_joints = 0
for a_name, b_name in rigid_pairs:
    inp = ab_joints.createInput(STAGE1_OCCS[a_name], STAGE1_OCCS[b_name], None)
    inp.setAsRigidJointMotion()
    j = ab_joints.add(inp)
    j.name = f"Rigid_{a_name}_to_{b_name}"
    created_joints += 1

result = {
    "stage1_parts_placed": len(STAGE1_OCCS),
    "as_built_joints_created": created_joints,
}
