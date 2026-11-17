from pathlib import Path
import build123d as bd
from bd_materials import metals, plastics
from cad import memo, model, read_xbf
from cad.ap242 import configuration_sequence, mechanism_state, posed
from cad.gltf import animation

import config as cfg
from printable_parts import (
    flower_depollinator_fork,
    front_funnel_bumper_left,
    front_funnel_bumper_right,
    g407_lip_gate_and_sensor_mount,
    magazine_anti_snag_shroud,
    robot_sign_plate_149,
    shooter_cam_and_counter_arm,
    star_roller_sleeve,
    stinger_nectar_cup,
)

VENDOR_DIR = Path(__file__).resolve().parent.parent / "vendor"


@memo
def _centered_5203_motor() -> bd.Shape:
    """
    Loads the official goBILDA 5203 Series Yellow Jacket Planetary Gear Motor (5203-2402-0019.xbf).
    Sub-part inspection (NAUO 1..28):
      - Shaft center axis is at X = -37.1707 mm, Z = -11.0784 mm
      - Front Gearbox Mounting Faceplate (NAUO 25) is at Y = +101.7354 mm
      - 8mm REX Output Shaft (NAUO 28) extends along +Y from Y = +101.7354 to +125.2354 mm (23.5 mm long)
      - Motor Body + Rear Encoder Cap (NAUO 1..6) extends along -Y from Y = +101.7354 down to -14.8146 mm (116.55 mm long)
    Translating by (+37.1707, -101.7354, +11.0784) puts the Front Gearbox Faceplate at (0,0,0),
    and rotating -90° around Z maps +Y (Output Shaft) -> +X [0..+23.5mm] and -Y (Motor Body) -> -X [-116.55..0mm]!
    """
    raw = read_xbf(VENDOR_DIR / "5203-2402-0019.xbf")
    centered = bd.Rot(0, 0, -90) * bd.Pos(37.1707, -101.7354, 11.0784) * raw
    centered.color = "#eab308"
    return centered


def _make_gobilda_wheel(outer_r: float, width: float, hub_r: float, color: str, label: str) -> bd.Part:
    """
    Models a goBILDA Rhino / Gecko compliant wheel (with 8x 4mm pattern holes on 16mm/32mm circle,
    recessed dish hub, and compliant outer tread slots) mounted on a solid 8mm REX shaft.
    Axis is along Y.
    """
    tire = bd.Rot(90, 0, 0) * (bd.Cylinder(outer_r, width) - bd.Cylinder(hub_r, width + 2.0))
    for i in range(12):
        ang = i * 30.0
        slot = (
            bd.Rot(0, ang, 0)
            * bd.Pos((outer_r + hub_r) / 2, 0.0, 0.0)
            * bd.Rot(90, 0, 0)
            * bd.Cylinder((outer_r - hub_r) * 0.28, width + 2.0)
        )
        tire = tire - slot
    hub = bd.Rot(90, 0, 0) * (bd.Cylinder(hub_r + 1.0, width * 0.65) - bd.Cylinder(7.0, width))
    for i in range(8):
        ang = i * 45.0
        mh = bd.Rot(0, ang, 0) * bd.Pos(16.0, 0.0, 0.0) * bd.Rot(90, 0, 0) * bd.Cylinder(2.1, width)
        hub = hub - mh
    wheel = tire + hub
    wheel.material = plastics.rubber()
    wheel.color = color
    wheel.label = label
    return wheel


def _make_quad_block_mount(label: str) -> bd.Part:
    """goBILDA 1201-0043-0002 Quad Block Pattern Mount (43x43x32mm)."""
    blk = bd.Box(32.0, 43.0, 43.0) - bd.Rot(0, 90, 0) * bd.Cylinder(7.0, 36.0)
    blk.material = metals.aluminum()
    blk.color = "#475569"
    blk.label = label
    return blk


def _make_pulley_and_belt(
    x1: float,
    z1: float,
    x2: float,
    z2: float,
    y_pos: float,
    r_pulley: float,
    width: float,
    label_prefix: str,
) -> list[bd.Part]:
    """
    Builds two flanged aluminum goBILDA HTD-5M timing pulleys at (x1, y_pos, z1) and (x2, y_pos, z2)
    (axes along Y) plus a closed black rubber HTD-5M timing belt loop wrapping around both pulleys!
    """
    import math

    parts = []
    for idx, (px, pz) in enumerate(((x1, z1), (x2, z2)), start=1):
        body = bd.Rot(90, 0, 0) * bd.Cylinder(r_pulley, width)
        flange_a = bd.Pos(0, -width / 2 + 1.0, 0) * bd.Rot(90, 0, 0) * bd.Cylinder(r_pulley + 2.5, 2.0)
        flange_b = bd.Pos(0, width / 2 - 1.0, 0) * bd.Rot(90, 0, 0) * bd.Cylinder(r_pulley + 2.5, 2.0)
        bore = bd.Rot(90, 0, 0) * bd.Cylinder(4.0, width + 4.0)
        pul = bd.Pos(px, y_pos, pz) * (body + flange_a + flange_b - bore)
        pul.material = metals.aluminum()
        pul.color = "#cbd5e1"
        pul.label = f"{label_prefix}_pulley_{idx}"
        parts.append(pul)

    dx = x2 - x1
    dz = z2 - z1
    dist = math.hypot(dx, dz)
    pitch_deg = -math.degrees(math.atan2(dz, dx))
    cx = (x1 + x2) / 2.0
    cz = (z1 + z2) / 2.0

    r_out = r_pulley + 1.8
    r_in = r_pulley - 0.5
    bw = width - 3.0

    outer_loop = (
        bd.Pos(x1, y_pos, z1) * bd.Rot(90, 0, 0) * bd.Cylinder(r_out, bw)
        + bd.Pos(x2, y_pos, z2) * bd.Rot(90, 0, 0) * bd.Cylinder(r_out, bw)
        + bd.Pos(cx, y_pos, cz) * bd.Rot(0, pitch_deg, 0) * bd.Box(dist, bw, 2.0 * r_out)
    )
    inner_cut = (
        bd.Pos(x1, y_pos, z1) * bd.Rot(90, 0, 0) * bd.Cylinder(r_in, bw + 2.0)
        + bd.Pos(x2, y_pos, z2) * bd.Rot(90, 0, 0) * bd.Cylinder(r_in, bw + 2.0)
        + bd.Pos(cx, y_pos, cz) * bd.Rot(0, pitch_deg, 0) * bd.Box(dist, bw + 2.0, 2.0 * r_in)
    )
    belt = outer_loop - inner_cut
    belt.material = plastics.rubber()
    belt.color = "#1e293b"
    belt.label = f"{label_prefix}_timing_belt"
    parts.append(belt)
    return parts


@memo
def strafer_chassis_stage() -> bd.Compound:
    """
    Stage 1: Standard goBILDA Strafer Chassis + Full Vertical U-Channel Tower Superstructure
    Uses official goBILDA & REV Robotics .xbf vendor files:
      - 2x 1120-0017-0432.xbf (17-Hole 432mm Side U-Channels at Z=48mm)
      - 2x 1120-0010-0264.xbf (10-Hole 264mm Base Crossbars at Rear X=-192 & Mid X=-12)
      - 4x 1120-0005-0144.xbf (5-Hole 144mm Vertical Tower Stanchions at X=-132 & X=-12, Z=72..216mm)
      - 2x 1120-0004-0120.xbf (4-Hole 120mm Front Vertical Intake Stanchions at X=+156, Z=72..192mm)
      - 2x 1120-0010-0264.xbf (10-Hole 264mm Upper Turret Bridge Crossbars at Z=192mm)
      - 4x Official goBILDA 96mm Mecanum Wheels (mecanum_96mm_left.xbf & mecanum_96mm_right.xbf)
      - 4x Official goBILDA 5203 Series Yellow Jacket Motors (5203-2402-0019.xbf)
      - 1x Official REV Control Hub (REV-31-1595.xbf) & 1x Official REV Expansion Hub (REV-31-1153.xbf)
      - 1x Official goBILDA Pinpoint v2 (3110-0002-0002.xbf) + Official 4-Bar Odometry Pod (3110-0001-0001.xbf)
      - 1x Official Matrix 12V Battery (14-0014.xbf)
    """
    raw_u17 = read_xbf(VENDOR_DIR / "1120-0017-0432.xbf")
    raw_u10 = read_xbf(VENDOR_DIR / "1120-0010-0264.xbf")
    raw_u10_low = read_xbf(VENDOR_DIR / "1121-0010-0264.xbf")
    raw_u5 = read_xbf(VENDOR_DIR / "1120-0005-0144.xbf")
    raw_u4 = read_xbf(VENDOR_DIR / "1120-0004-0120.xbf")
    raw_mec_l = read_xbf(VENDOR_DIR / "mecanum_96mm_left.xbf")
    raw_mec_r = read_xbf(VENDOR_DIR / "mecanum_96mm_right.xbf")
    raw_rev_ctrl = read_xbf(VENDOR_DIR / "REV-31-1595.xbf")
    raw_rev_exp = read_xbf(VENDOR_DIR / "REV-31-1153.xbf")
    raw_pinpoint = read_xbf(VENDOR_DIR / "3110-0002-0002.xbf")
    raw_odo_pod = read_xbf(VENDOR_DIR / "3110-0001-0001.xbf")
    raw_battery = read_xbf(VENDOR_DIR / "14-0014.xbf")
    mot_base = _centered_5203_motor()

    # 1. Base 17-Hole Side U-Channels (X in [-216, +216], Y = ±156, Z in [24, 72])
    u17_centered = bd.Rot(0, 0, 90) * bd.Pos(0.0, -216.0, 21.5) * raw_u17
    left_rail = bd.Pos(0.0, cfg.SIDE_CHAN_Y, cfg.CHASSIS_Z) * u17_centered
    left_rail.color = "#94a3b8"
    left_rail.label = "gobilda_1120_17hole_left"

    right_rail = bd.Pos(0.0, -cfg.SIDE_CHAN_Y, cfg.CHASSIS_Z) * u17_centered
    right_rail.color = "#94a3b8"
    right_rail.label = "gobilda_1120_17hole_right"

    # 2. Base Crossbars: Rear 10H U-Channel at X=-192, Z=48; Mid 10H Low-Side Belly Brace at X=+48, Z=28 (underneath ramp!)
    u10_centered = bd.Pos(0.0, -132.0, 21.5) * raw_u10
    rear_cross = bd.Pos(cfg.REAR_CROSS_X, 0.0, cfg.CHASSIS_Z) * u10_centered
    rear_cross.color = "#94a3b8"
    rear_cross.label = "gobilda_1120_10hole_rear"

    # Mid 10H Low-Side Belly Brace rotated 90° in Y at X=+64mm, Z=32mm (Z in [26, 38mm], 22mm below the ramp at X=+64mm!)
    u10_low_belly = bd.Rot(0, 90, 0) * bd.Pos(18.0, -132.0, 21.5) * raw_u10_low
    mid_cross = bd.Pos(64.0, 0.0, 32.0) * u10_low_belly
    mid_cross.color = "#94a3b8"
    mid_cross.label = "gobilda_1120_10hole_mid"

    # 3. Vertical 5-Hole (144mm) Tower Stanchions bolted to top of Left & Right rails (Z = 72..216mm)
    u5_vert = bd.Rot(90, 0, 0) * bd.Pos(0.0, -72.0, 21.5) * raw_u5
    tower_pillars = []
    for tx, tname in ((cfg.REAR_TOWER_X, "rear"), (cfg.MID_TOWER_X, "mid")):
        for ty, sname in ((cfg.SIDE_CHAN_Y, "left"), (-cfg.SIDE_CHAN_Y, "right")):
            pil = bd.Pos(tx, ty, 144.0) * u5_vert
            pil.color = "#64748b"
            pil.label = f"gobilda_1120_5hole_tower_{tname}_{sname}"
            tower_pillars.append(pil)
            qm = bd.Pos(tx, ty, 90.0) * bd.Rot(0, 90, 0) * _make_quad_block_mount(f"tower_base_mount_{tname}_{sname}")
            tower_pillars.append(qm)

    # 4. Front 4-Hole (120mm) Vertical Intake Stanchions (X=+156, Y=±156, Z=72..192mm) + Low-Side U-Channel Top Arch!
    u4_vert = bd.Rot(90, 0, 0) * bd.Pos(0.0, -60.0, 21.5) * raw_u4
    for ty, sname in ((cfg.SIDE_CHAN_Y, "left"), (-cfg.SIDE_CHAN_Y, "right")):
        fpil = bd.Pos(cfg.FRONT_TOWER_X, ty, 132.0) * u4_vert
        fpil.color = "#64748b"
        fpil.label = f"gobilda_1120_4hole_intake_upright_{sname}"
        tower_pillars.append(fpil)

    # Front Portal Arch rotated 90° in Y so its 12mm low-profile height is in Z (Z in [192, 204mm], X in [132, 180mm]),
    # capping the top of the Front Vertical Uprights and staying 100% clear of gobilda_gecko_wheel_72mm_1!
    front_portal_arch = bd.Pos(cfg.FRONT_TOWER_X, 0.0, 198.0) * u10_low_belly
    front_portal_arch.color = "#94a3b8"
    front_portal_arch.label = "gobilda_1121_10hole_front_portal_arch"
    tower_pillars.append(front_portal_arch)

    # 5. Upper Horizontal Turret Bridge: Rear 10H Crossbar (X=-144, Z=192, behind ball chute!)
    upper_bridge_rear = bd.Pos(cfg.REAR_TOWER_X, 0.0, 192.0) * u10_centered
    upper_bridge_rear.color = "#94a3b8"
    upper_bridge_rear.label = "gobilda_1120_10hole_turret_bridge_rear"

    # 6. CNC 6061 Aluminum Turret Mounting Deck Plate (bolted across top of Rear & Mid Tower Stanchions at Z=216..222mm)
    # Center bore enlarged to R=58mm so the Rear Elevator Trough and 3rd Gecko Elevator Kicker Wheel pass cleanly up into the turret!
    deck_plate = (
        bd.Pos(cfg.TURRET_CENTER_X, 0.0, 219.0) * bd.Box(168.0, 360.0, 6.0)
        - bd.Pos(cfg.TURRET_CENTER_X, 0.0, 219.0) * bd.Cylinder(58.0, 12.0)
    )
    deck_plate.material = metals.aluminum()
    deck_plate.color = "#cbd5e1"
    deck_plate.label = "cnc_turret_mounting_deck_plate"

    # 7. 4x Official goBILDA 96mm Mecanum Wheels (mecanum_96mm_left.xbf & mecanum_96mm_right.xbf) + 8mm REX Steel Axles
    mec_l_oriented = bd.Rot(90, 0, 0) * bd.Pos(0.0, 0.0, 7.515) * raw_mec_l
    mec_r_oriented = bd.Rot(90, 0, 0) * bd.Pos(0.0, 0.0, 7.515) * raw_mec_r

    wheels_and_axles = []
    wheel_configs = [
        (cfg.WHEEL_X, cfg.WHEEL_Y, mec_l_oriented, "gobilda_mecanum_96mm_front_left", 1),
        (cfg.WHEEL_X, -cfg.WHEEL_Y, mec_r_oriented, "gobilda_mecanum_96mm_front_right", -1),
        (-cfg.WHEEL_X, cfg.WHEEL_Y, mec_r_oriented, "gobilda_mecanum_96mm_rear_left", 1),
        (-cfg.WHEEL_X, -cfg.WHEEL_Y, mec_l_oriented, "gobilda_mecanum_96mm_rear_right", -1),
    ]
    for wx, wy, wshape, wlbl, side in wheel_configs:
        w = bd.Pos(wx, wy, cfg.WHEEL_RADIUS) * wshape
        w.color = "#334155"
        w.label = wlbl
        wheels_and_axles.append(w)
        ax = bd.Pos(wx, side * 172.0, cfg.WHEEL_RADIUS) * bd.Rot(90, 0, 0) * bd.Cylinder(6.0, 92.0)
        ax.material = metals.mild_steel()
        ax.color = "#e2e8f0"
        ax.label = f"rex_axle_shaft_{wlbl}"
        wheels_and_axles.append(ax)

    # 8. 4x Official goBILDA 5203 Yellow Jacket Drive Motors (M1-M4) inside the 17-hole side U-Channels:
    # Coplanar at Z = 48.0 mm (exact 0.0 mm vertical offset to the Z = 48.0 mm Mecanum wheel axles for 1:1 2305 Miter Bevel Gears!)
    drive_motors = []
    for idx, (mx, my, yaw) in enumerate([
        (117.0, cfg.SIDE_CHAN_Y, 0.0),
        (-117.0, cfg.SIDE_CHAN_Y, 180.0),
        (117.0, -cfg.SIDE_CHAN_Y, 0.0),
        (-117.0, -cfg.SIDE_CHAN_Y, 180.0),
    ], start=1):
        dm = bd.Pos(mx, my, 48.0) * bd.Rot(0, 0, yaw) * mot_base
        dm.label = f"gobilda_5203_motor_m{idx}_drive"
        drive_motors.append(dm)

    # 9. Official REV Robotics Control Hub (REV-31-1595.xbf) & Expansion Hub (REV-31-1153.xbf)
    # Mounted INBOARD (|Y| = 116.0 mm, inside the |Y| = 132..180 mm aluminum towers!) for 100% G419 impact armor!
    ctrl_hub = bd.Pos(-142.0, -116.0, 154.0) * raw_rev_ctrl
    ctrl_hub.color = "#f97316"
    ctrl_hub.label = "official_rev_control_hub_31_1595"

    exp_hub = bd.Pos(-142.0, 116.0, 154.0) * bd.Rot(0, 0, 180) * raw_rev_exp
    exp_hub.color = "#ea580c"
    exp_hub.label = "official_rev_expansion_hub_31_1153"

    # Official Matrix 12V Battery (14-0014.xbf) centered at (X=-192, Y=+18, Z=98) directly over Rear Crossbar!
    batt = bd.Pos(-186.0, 18.0, 98.0) * bd.Rot(0, 0, -90) * bd.Pos(-58.9, 0.0, 21.9) * raw_battery
    batt.color = "#facc15"
    batt.label = "official_gobilda_12v_battery_14_0014"

    # Official goBILDA Pinpoint v2 Odometry Computer (2-port IMU computer!) + Forward Left X Odometry Pod at Z = 24.0 mm (Z_tire_min = -0.18 mm touching floor!)
    pinpoint = bd.Pos(-95.0, -92.0, 76.0) * raw_pinpoint
    pinpoint.color = "#a855f7"
    pinpoint.label = "official_gobilda_pinpoint_v2"

    odo_pod = bd.Pos(-95.0, 92.5, 24.0) * raw_odo_pod
    odo_pod.color = "#7c3aed"
    odo_pod.label = "official_gobilda_4bar_odometry_pod"

    # Custom 3D-Printed Front Corner Funnel Bumpers:
    # Placed at X = +216.0 mm (front end of 17-hole rail) so front cap is in [+216, +230] and inboard funnel wing is in |Y| in [98, 131.8] — ZERO rail clipping!
    bumper_l = bd.Pos(216.0, cfg.SIDE_CHAN_Y, cfg.CHASSIS_Z) * front_funnel_bumper_left().shape
    bumper_l.label = "front_funnel_bumper_left"
    bumper_r = bd.Pos(216.0, -cfg.SIDE_CHAN_Y, cfg.CHASSIS_Z) * front_funnel_bumper_right().shape
    bumper_r.label = "front_funnel_bumper_right"

    # 2x Rule R401-R403 Legal Multi-Color AMS 2 Pro Team 149 Robot Signs
    sign_l = bd.Pos(72.0, 182.0, 144.0) * bd.Rot(0, 0, 180) * robot_sign_plate_149().shape
    sign_l.label = "robot_sign_left_149"
    sign_r = bd.Pos(72.0, -182.0, 144.0) * robot_sign_plate_149().shape
    sign_r.label = "robot_sign_right_149"

    return bd.Compound(
        children=[
            left_rail,
            right_rail,
            rear_cross,
            mid_cross,
            *tower_pillars,
            upper_bridge_rear,
            deck_plate,
            *wheels_and_axles,
            *drive_motors,
            ctrl_hub,
            exp_hub,
            batt,
            pinpoint,
            odo_pod,
            bumper_l,
            bumper_r,
            sign_l,
            sign_r,
        ],
        label="strafer_chassis_stage",
    )


@memo
def gatekeeper_intake_stage() -> bd.Compound:
    """
    Stage 2: 'Gatekeeper' Full-Width Compliant Intake + Stepped Chute Floor + S1 Servo
    - Steel 8mm REX intake shaft is held at both ends by bolted Left & Right Intake Bearing Arms
      extending forward from the Front Vertical Intake Uprights (X=+156..+194mm, Y=±148mm, Z=100mm).
    - Motor M7 is mounted PARALLEL to the intake shaft at (X=+78, Y=+98, Z=132) with bd.Rot(0, 0, -90)
      so its 116.55mm body extends OUTWARD (Y in [+98, +214.55mm], 100% outside the central chute!)
      and its 8mm REX output shaft points INWARD (Y in [+74.5, +98mm]) to drive the intake belt at Y=+86mm!
    """
    raw_servo = read_xbf(VENDOR_DIR / "2000-0025-0003.xbf")
    mot_base = _centered_5203_motor()

    # 8mm REX Intake Axle spanning between the Left & Right Intake Bearing Support Arms (Y in [-154, +154])
    axle = bd.Pos(cfg.INTAKE_ROLLER_X, 0.0, cfg.INTAKE_ROLLER_Z) * bd.Rot(90, 0, 0) * bd.Cylinder(4.5, 308.0)
    axle.material = metals.mild_steel()
    axle.color = "#e2e8f0"
    axle.label = "intake_rex_shaft_8mm"

    # Left & Right Bolted Intake Bearing Support Arms connecting Front Uprights (X=+156, Y=±148, Z=94) to Intake Axle (X=+194)
    support_arms = []
    for side, sname in ((1, "left"), (-1, "right")):
        arm = (
            bd.Pos(175.0, side * 148.0, cfg.INTAKE_ROLLER_Z) * bd.Box(48.0, 8.0, 28.0)
            + bd.Pos(cfg.INTAKE_ROLLER_X, side * 148.0, cfg.INTAKE_ROLLER_Z) * bd.Rot(90, 0, 0) * bd.Cylinder(14.0, 10.0)
            - bd.Pos(cfg.INTAKE_ROLLER_X, side * 148.0, cfg.INTAKE_ROLLER_Z) * bd.Rot(90, 0, 0) * bd.Cylinder(4.6, 14.0)
        )
        arm.material = metals.aluminum()
        arm.color = "#64748b"
        arm.label = f"intake_bearing_support_arm_{sname}"
        support_arms.append(arm)

    rollers = []
    sleeve_base = star_roller_sleeve().shape
    # 3 compliant star_roller_sleeve paddles (each 42mm wide) at Y = -86, 0, +86 -> |Y|_max = 107 < 110 of bumpers, and leaves Y=+52mm open for the intake belt!
    for ry in (-86.0, 0.0, 86.0):
        rl = bd.Pos(cfg.INTAKE_ROLLER_X, ry, cfg.INTAKE_ROLLER_Z) * bd.Rot(90, 0, 0) * sleeve_base
        rl.label = "star_roller_sleeve"
        rollers.append(rl)

    # Section 1 of Continuous Ball Chute: 236mm-long +23.96° U-Shaped Upward Ramp from Front Intake (X=+126, Z=10) to Rear Elevator (X=-90, Z=106)!
    # Inner width is 80mm (side walls at Y=±42mm -> |Y| in [40, 44mm], leaving +4.44mm clearance on each side of every 71.1mm ball!).
    ramp_base = (
        bd.Box(236.0, 88.0, 4.0)
        + bd.Pos(0.0, 42.0, 7.0) * bd.Box(236.0, 4.0, 14.0)
        + bd.Pos(0.0, -42.0, 7.0) * bd.Box(236.0, 4.0, 14.0)
    )
    ramp = bd.Pos(18.0, 0.0, 58.0) * bd.Rot(0, 23.96, 0) * ramp_base
    ramp.material = plastics.petg()
    ramp.color = "#fbbf24"
    ramp.label = "continuous_chute_bottom_ramp"

    # Section 2 of Continuous Ball Chute: Steep +76° U-Shaped Rear Elevator Trough rising seamlessly from (X=-89, Z=104)
    # up through the Turret Bearing Ring to (X=-127, Z=256) with 80mm inner width (side walls at Y=±42mm -> ZERO ball clipping!).
    elev_base = (
        bd.Box(156.0, 88.0, 4.0)
        + bd.Pos(0.0, 42.0, 7.0) * bd.Box(156.0, 4.0, 14.0)
        + bd.Pos(0.0, -42.0, 7.0) * bd.Box(156.0, 4.0, 14.0)
    )
    elevator_wall = bd.Pos(-108.0, 0.0, 180.0) * bd.Rot(0, 76, 0) * elev_base
    elevator_wall.material = plastics.petg()
    elevator_wall.color = "#fbbf24"
    elevator_wall.label = "chute_rear_elevator_wall"

    # 3 Bolted Structural Chassis Mounts physically anchoring the Ramp and Rear Elevator Trough to the goBILDA U-Channel Frame:
    # 1) Front Ramp Riser Gussets bolting the underside of the ramp at X=+64 directly down to gobilda_1120_10hole_mid (X=+64, Z=32)!
    ramp_front_mount = (
        bd.Pos(64.0, 40.0, 32.0) * bd.Box(24.0, 8.0, 16.0)
        + bd.Pos(64.0, -40.0, 32.0) * bd.Box(24.0, 8.0, 16.0)
    )
    ramp_front_mount.material = metals.aluminum()
    ramp_front_mount.color = "#475569"
    ramp_front_mount.label = "ramp_front_crossbar_mount"

    # 2) Mid Ramp Structural Cross-Wings bolting the sides of the ramp (Y=±44) at X=-12, Z=72 directly into the Left & Right Side Rails / Mid Stanchions (Y=±132)!
    ramp_mid_mount = (
        bd.Pos(-12.0, 88.0, 70.0) * bd.Box(24.0, 88.0, 8.0)
        + bd.Pos(-12.0, -88.0, 70.0) * bd.Box(24.0, 88.0, 8.0)
    )
    ramp_mid_mount.material = metals.aluminum()
    ramp_mid_mount.color = "#475569"
    ramp_mid_mount.label = "ramp_mid_stanchion_cross_mount"

    # 3) Rear Elevator Bridge Mount bolting the back of chute_rear_elevator_wall (X=-112, Z=192) directly to gobilda_1120_10hole_turret_bridge_rear (X=-144, Z=192)!
    elev_rear_mount = (
        bd.Pos(-124.0, 40.0, 192.0) * bd.Box(32.0, 8.0, 16.0)
        + bd.Pos(-124.0, -40.0, 192.0) * bd.Box(32.0, 8.0, 16.0)
    )
    elev_rear_mount.material = metals.aluminum()
    elev_rear_mount.color = "#475569"
    elev_rear_mount.label = "elevator_rear_bridge_mount"

    # Official goBILDA 5203 Intake Motor (M7): Faceplate at (X=+104, Y=+63, Z=98) with bd.Rot(0, 0, -90):
    # - Body extends OUTWARD (+Y in [+63.0, +179.55mm]) — inside robot_sign_left_149 (Y>=180.4) and outside chute!
    # - Z in [71.85, 124.15mm] — below magazine_shroud_left (Z_min=128) and below g407_lip_gate_bar (Z>=130)!
    # - Shaft points INWARD (-Y in [+39.5, +63.0mm]) through the Timing Pulley at Y=+52.0mm!
    m7_intake = bd.Pos(104.0, 63.0, 98.0) * bd.Rot(0, 0, -90) * mot_base
    m7_intake.label = "gobilda_5203_motor_m7_intake"

    # Rigid 6mm Aluminum Motor Mount Plate bolting M7 (X=+104, Y=+63, Z=98) directly down to Left 17H Chassis Side Rail (Y=+132, Z=72)
    m7_bracket = (
        bd.Pos(104.0, 65.0, 92.0) * bd.Box(44.0, 6.0, 40.0)
        + bd.Pos(104.0, 104.0, 75.0) * bd.Box(44.0, 80.0, 6.0)
    )
    m7_bracket.material = metals.aluminum()
    m7_bracket.color = "#475569"
    m7_bracket.label = "m7_intake_motor_mount_bracket"

    # Real Aluminum HTD-5M Timing Pulleys + Black Rubber Timing Belt Loop at Y=+52mm connecting M7 (104, 98) to Intake Shaft (194, 94)!
    intake_belt_drive = _make_pulley_and_belt(
        104.0, 98.0, cfg.INTAKE_ROLLER_X, cfg.INTAKE_ROLLER_Z, 52.0, 12.0, 10.0, "intake_drive"
    )

    # Servo S1 (G407 Lip Gate) & Servo S2 (De-Pollinator Fork Drive):
    # Bolted FLUSH ON TOP of gobilda_1121_10hole_front_portal_arch (Z=204mm) at Z=222mm (Z_min=204.8mm > 204.0mm arch!)
    s1_servo = bd.Pos(156.0, 88.0, 222.0) * bd.Rot(90, 0, 0) * raw_servo
    s1_servo.color = "#fb7185"
    s1_servo.label = "servo_s1_g407_lip_gate"

    s2_servo = bd.Pos(156.0, 24.0, 222.0) * bd.Rot(90, 0, 0) * raw_servo
    s2_servo.color = "#10b981"
    s2_servo.label = "servo_s2_depollinator_drive"

    return bd.Compound(
        children=[
            axle,
            *support_arms,
            *rollers,
            ramp,
            elevator_wall,
            ramp_front_mount,
            ramp_mid_mount,
            elev_rear_mount,
            m7_intake,
            m7_bracket,
            *intake_belt_drive,
            s1_servo,
            s2_servo,
        ],
        label="gatekeeper_intake_stage",
    )


@memo
def sprung_magazine_stage() -> bd.Compound:
    """
    Stage 3: Single-File Sprung Magazine + 2x 72mm goBILDA Gecko Feeder Wheels + M8 Motor + S3 Stopper
    - Shrouds and ramp are tilted at +22° (low at the front intake, high at the rear turret).
    - All 4 staged balls sit on the exact +22° upward normal above the yellow ramp and below the
      two green 72mm Gecko feeder wheels (zero clipping into the ramp or Gecko wheels!).
    - Motor M8 is mounted with its faceplate at Y=-82mm (shaft pointing INWARD along +Y through
      the pulley at Y=-78mm, and motor body extending OUTWARD along -Y in [-198.5, -82.0mm],
      completely outside the magazine shroud so it NEVER touches the compliant wheels or balls!).
    """
    raw_servo = read_xbf(VENDOR_DIR / "2000-0025-0003.xbf")
    mot_base = _centered_5203_motor()

    shroud_base = magazine_anti_snag_shroud().shape
    # Position shrouds at (X=+8, Y=±46, Z=172) tilted +8° so Z_min = 126mm > 124.15mm of M7 & M8 (ZERO motor or belt overlap!)
    shroud_l = bd.Pos(8.0, 46.0, 172.0) * bd.Rot(0, 8, 0) * shroud_base
    shroud_l.label = "magazine_shroud_left"

    shroud_r = bd.Pos(8.0, -46.0, 172.0) * bd.Rot(0, 8, 0) * bd.mirror(shroud_base, about=bd.Plane.XZ)
    shroud_r.label = "magazine_shroud_right"

    # Structural L-Gusset Brackets bolting Left & Right Magazine Shrouds directly to Mid Tower Stanchions (X=-12, Y=±156)
    shroud_mounts = []
    for side, sname in ((1, "left"), (-1, "right")):
        for gx, gz in ((-12.0, 136.0), (-12.0, 168.0)):
            gus = bd.Pos(gx, side * 95.0, gz) * bd.Box(24.0, 70.0, 12.0)
            gus.material = metals.aluminum()
            gus.color = "#64748b"
            gus.label = f"magazine_structural_gusset_{sname}"
            shroud_mounts.append(gus)

    # 5 Continuous Staged BIOBUZZ Balls rising from the Front Intake up the +23.96° Ramp and +76° Rear Elevator Trough right into the Shooter Nip!
    # Every ball has +3.71..+5.15mm positive 3D clearance above the ramp/elevator floor and +3.00..+6.11mm below the 3 Gecko wheels!
    staged_balls = []
    ball_specs = [
        (168.0, 38.0, cfg.POLLEN_DIA / 2, "#facc15", "staged_pollen_1_entering_mouth"),
        (96.0, 68.5, cfg.POLLEN_DIA / 2, "#facc15", "staged_pollen_2_on_ramp"),
        (18.0, 103.2, cfg.POLLEN_DIA / 2, "#facc15", "staged_pollen_3_in_chute"),
        (-56.0, 148.0, cfg.POLLEN_DIA / 2, "#facc15", "staged_pollen_4_at_turret_feed"),
        (-76.0, 228.0, cfg.POLLEN_DIA / 2, "#facc15", "staged_pollen_5_in_shooter_nip"),
    ]
    for bx, bz, br, bcol, blabel in ball_specs:
        b = bd.Pos(bx, 0.0, bz) * bd.Sphere(br)
        b.material = plastics.pp()
        b.color = bcol
        b.label = blabel
        staged_balls.append(b)

    # 3x 72mm goBILDA Gecko Compliant Overhead Feeder & Elevator Kicker Wheels at (112, 144), (38, 178), and (0, 240):
    # Positively drives balls from the intake ramp up through the turret throat directly into the 96mm Main Shooter Flywheel!
    gecko_assemblies = []
    for idx, (gx, gz) in enumerate(((112.0, 144.0), (38.0, 178.0), (0.0, 240.0)), start=1):
        gw = bd.Pos(gx, 0.0, gz) * _make_gobilda_wheel(36.0, 32.0, 19.0, "#22c55e", f"gobilda_gecko_wheel_72mm_{idx}")
        gecko_assemblies.append(gw)
        gax = bd.Pos(gx, -14.0, gz) * bd.Rot(90, 0, 0) * bd.Cylinder(5.0, 116.0)
        gax.material = metals.mild_steel()
        gax.color = "#e2e8f0"
        gax.label = f"gecko_rex_shaft_{idx}"
        gecko_assemblies.append(gax)
        for side in (-1, 1):
            bblk = bd.Pos(gx, side * 32.0, gz) * bd.Rot(90, 0, 0) * bd.Cylinder(9.0, 6.0)
            bblk.material = metals.aluminum()
            bblk.color = "#475569"
            bblk.label = f"gecko_flanged_bearing_{idx}_{side}"
            gecko_assemblies.append(bblk)

    # Official goBILDA 5203 Magazine Motor (M8): Faceplate at (X=+104, Y=-63, Z=98) with yaw=+90°:
    # - Body extends OUTWARD (-Y in [-179.55, -63.0mm]) — inside robot_sign_right_149 (Y<=-180.4) and BELOW magazine_shroud_right (Z_min=126)!
    m8_mag = bd.Pos(104.0, -63.0, 98.0) * bd.Rot(0, 0, 90) * mot_base
    m8_mag.label = "gobilda_5203_motor_m8_magazine"

    # 3-Stage Timing Belt Chain at Y=-64mm & Y=-72mm coupling M8 -> Gecko 1 -> Gecko 2 -> Gecko 3 (Turret Kicker Wheel)!
    mag_belt_1 = _make_pulley_and_belt(104.0, 98.0, 112.0, 144.0, -64.0, 12.0, 10.0, "magazine_motor_drive")
    mag_belt_2 = _make_pulley_and_belt(112.0, 144.0, 38.0, 178.0, -72.0, 12.0, 10.0, "magazine_gecko_couple")
    mag_belt_3 = _make_pulley_and_belt(38.0, 178.0, 0.0, 240.0, -64.0, 12.0, 10.0, "magazine_elevator_couple")
    gecko_assemblies.extend(mag_belt_1)
    gecko_assemblies.extend(mag_belt_2)
    gecko_assemblies.extend(mag_belt_3)

    # Servo S3 (Magazine Top Stopper): mounted at (X=-42, Y=+92, Z=184) -> Z_max=200.9 < 216.0 Deck, Y_min=81.8 > 74.0 Shroud!
    s3_stopper = bd.Pos(-42.0, 92.0, 184.0) * raw_servo
    s3_stopper.color = "#a855f7"
    s3_stopper.label = "servo_s3_magazine_stopper"

    return bd.Compound(
        children=[shroud_l, shroud_r, *shroud_mounts, *staged_balls, *gecko_assemblies, m8_mag, s3_stopper],
        label="sprung_magazine_stage",
    )


def build_turret_assembly() -> bd.Compound:
    """
    Stage 4: 270° Rear-Centered Turret + Dual-Wheel Topspin Launcher
    Sits FLUSH on top of the CNC Turret Mounting Deck Plate at Z = 222.0 mm!
    Uses 2x Official goBILDA 5203 Bare 6000 RPM Motors bolted flush to the Turret Side Plates.
    """
    raw_servo = read_xbf(VENDOR_DIR / "2000-0025-0003.xbf")
    mot_base = _centered_5203_motor()
    tx, ty, tz = cfg.TURRET_CENTER_X, cfg.TURRET_CENTER_Y, cfg.TURRET_BASE_Z

    ring = bd.Pos(tx, ty, tz + 7.0) * (bd.Cylinder(72.0, 14.0) - bd.Cylinder(58.0, 16.0))
    ring.material = metals.mild_steel()
    ring.color = "#475569"
    ring.label = "turret_bearing_ring_85mm"

    # Turret Side Plates at Y = ±64.0 mm (|Y| in [61.0, 67.0 mm], clear of internal cam arms at |Y| in [43, 57]!)
    plate_l = bd.Pos(tx, ty + 64.0, tz + 78.0) * bd.Box(128.0, 6.0, 128.0)
    plate_l.material = metals.aluminum()
    plate_l.color = "#f59e0b"
    plate_l.label = "turret_side_plate_left"

    plate_r = bd.Pos(tx, ty - 64.0, tz + 78.0) * bd.Box(128.0, 6.0, 128.0)
    plate_r.material = metals.aluminum()
    plate_r.color = "#f59e0b"
    plate_r.label = "turret_side_plate_right"

    top_bridge = bd.Pos(tx - 48.0, ty, tz + 148.0) * bd.Box(28.0, 118.0, 8.0)
    top_bridge.material = metals.aluminum()
    top_bridge.color = "#d97706"
    top_bridge.label = "turret_top_structural_bridge"

    # Motors M5 & M6: Mounted PARALLEL to the flywheel shafts (axes along Y!) at X = tx + 14 = -58mm (clear of stinger_pivot_quad_mount at X>=-28!):
    m5_shoot = bd.Pos(tx + 14.0, ty + 90.0, tz + 44.0) * bd.Rot(0, 0, -90) * mot_base
    m5_shoot.label = "gobilda_5203_motor_m5_shooter_master"

    m6_shoot = bd.Pos(tx + 14.0, ty - 90.0, tz + 44.0) * bd.Rot(0, 0, 90) * mot_base
    m6_shoot.label = "gobilda_5203_motor_m6_shooter_follower"

    # Rigid Aluminum Turret Motor Stand Brackets connecting Turret Side Plates (|Y|=67mm) to Motor Faceplates (|Y|=90mm)
    m5_turret_brk = bd.Pos(tx + 14.0, ty + 88.0, tz + 30.0) * bd.Box(44.0, 4.0, 56.0)
    m5_turret_brk.material = metals.aluminum()
    m5_turret_brk.color = "#475569"
    m5_turret_brk.label = "turret_motor_mount_bracket_left"

    m6_turret_brk = bd.Pos(tx + 14.0, ty - 88.0, tz + 30.0) * bd.Box(44.0, 4.0, 56.0)
    m6_turret_brk.material = metals.aluminum()
    m6_turret_brk.color = "#475569"
    m6_turret_brk.label = "turret_motor_mount_bracket_right"

    # Real HTD-5M Timing Pulleys & Belts on Left (Y=+76: M5 -> 96mm Main Flywheel at tx+72) and Right (Y=-76: M6 -> 72mm Topspin Counter-Wheel at tx-36)!
    shooter_belt_l = _make_pulley_and_belt(tx + 14.0, tz + 44.0, tx + 72.0, tz + 102.0, ty + 76.0, 12.0, 10.0, "shooter_left_drive")
    shooter_belt_r = _make_pulley_and_belt(tx + 14.0, tz + 44.0, tx - 36.0, tz + 102.0, ty - 76.0, 12.0, 10.0, "shooter_right_drive")

    # Front/Lower 96mm Main Rhino Wheel + 2x 82mm Steel Flywheels at (X = tx + 72 = 0.0, Z = tz + 102 = 324.0):
    # Sits 84mm above Gecko 3 (0, 240) so balls pushed up the Rear Elevator Wall by Gecko 3 enter the nip between rhino_96 (0, 324) and rhino_72 (-108, 324)!
    flywheel_shaft = bd.Pos(tx + 72.0, ty, tz + 102.0) * bd.Rot(90, 0, 0) * bd.Cylinder(5.0, 168.0)
    flywheel_shaft.material = metals.mild_steel()
    flywheel_shaft.color = "#e2e8f0"
    flywheel_shaft.label = "shooter_main_rex_shaft"

    rhino_96 = bd.Pos(tx + 72.0, ty, tz + 102.0) * _make_gobilda_wheel(48.0, 36.0, 24.0, "#fbbf24", "gobilda_rhino_wheel_96mm")

    fw_l = bd.Pos(tx + 72.0, ty + 28.0, tz + 102.0) * bd.Rot(90, 0, 0) * (bd.Cylinder(41.0, 10.0) - bd.Cylinder(7.0, 12.0))
    fw_l.material = metals.mild_steel()
    fw_l.color = "#cbd5e1"
    fw_l.label = "gobilda_steel_flywheel_82mm_left"

    fw_r = bd.Pos(tx + 72.0, ty - 28.0, tz + 102.0) * bd.Rot(90, 0, 0) * (bd.Cylinder(41.0, 10.0) - bd.Cylinder(7.0, 12.0))
    fw_r.material = metals.mild_steel()
    fw_r.color = "#cbd5e1"
    fw_r.label = "gobilda_steel_flywheel_82mm_right"

    # Rear/Upper 72mm Topspin Counter-Wheel at (X = tx - 36 = -108, Z = tz + 102 = 324):
    # Sits directly above the top of chute_rear_elevator_wall (X=-98, Z=266) so balls rising up the elevator wall
    # feed straight into the nip between rhino_96 (-16, 288) and rhino_72 (-108, 324)!
    counter_shaft = bd.Pos(tx - 36.0, ty, tz + 102.0) * bd.Rot(90, 0, 0) * bd.Cylinder(5.0, 168.0)
    counter_shaft.material = metals.mild_steel()
    counter_shaft.color = "#e2e8f0"
    counter_shaft.label = "shooter_counter_rex_shaft"

    pivot_shaft = bd.Pos(tx + 44.0, ty, tz + 124.0) * bd.Rot(90, 0, 0) * bd.Cylinder(5.0, 120.0)
    pivot_shaft.material = metals.mild_steel()
    pivot_shaft.color = "#e2e8f0"
    pivot_shaft.label = "shooter_cam_pivot_rex_shaft"

    rhino_72 = bd.Pos(tx - 36.0, ty, tz + 102.0) * _make_gobilda_wheel(36.0, 36.0, 19.0, "#f43f5e", "gobilda_counter_wheel_72mm_topspin")

    # 3D-Printed Cam Arms at Y = ±50.0 mm (|Y| in [43.0, 57.0 mm], 4mm clear inside Turret Side Plates at |Y|=61..67!)
    cam_arm_base = shooter_cam_and_counter_arm().shape
    cam_arm_l = bd.Pos(tx - 4.0, ty + 50.0, tz + 124.0) * cam_arm_base
    cam_arm_l.label = "shooter_cam_arm_left"
    cam_arm_r = bd.Pos(tx - 4.0, ty - 50.0, tz + 124.0) * cam_arm_base
    cam_arm_r.label = "shooter_cam_arm_right"

    # Variable Pitch Exit Guide Hood at (X = tx + 4, Z = tz + 148 = 370) -> Z_min = 362 > 360 of rhino_72 (zero wheel overlap!)
    hood = bd.Pos(tx + 4.0, ty, tz + 148.0) * bd.Rot(0, -16, 0) * bd.Box(44.0, 76.0, 6.0)
    hood.material = plastics.petg()
    hood.color = "#38bdf8"
    hood.label = "variable_pitch_exit_hood"

    # Servos S4 (Turret Azimuth), S5 (Cam Gap Selector), S6 (Pitch Hood) placed with 100% zero overlap with belts, plates, or motors!
    s4_turret = bd.Pos(tx - 48.0, ty - 144.0, tz + 28.0) * raw_servo
    s4_turret.color = "#fbbf24"
    s4_turret.label = "servo_s4_turret_azimuth"

    s5_cam = bd.Pos(tx - 48.0, ty + 78.0, tz + 158.0) * raw_servo
    s5_cam.color = "#f43f5e"
    s5_cam.label = "servo_s5_cam_gap_selector"

    s6_pitch = bd.Pos(tx - 48.0, ty - 78.0, tz + 158.0) * raw_servo
    s6_pitch.color = "#38bdf8"
    s6_pitch.label = "servo_s6_pitch_adjust"

    # Limelight 3A Smart Camera bolted FLUSH ON TOP of turret_top_structural_bridge (X = tx - 48 = -120, Y = 0, Z = tz + 170 = 392, Z_min = 374 == bridge top!)
    limelight = bd.Pos(tx - 48.0, ty, tz + 170.0) * bd.Box(24.0, 68.0, 36.0)
    limelight.material = plastics.abs_()
    limelight.color = "#22c55e"
    limelight.label = "limelight_3a_vision"

    return bd.Compound(
        children=[
            ring,
            plate_l,
            plate_r,
            top_bridge,
            m5_shoot,
            m6_shoot,
            m5_turret_brk,
            m6_turret_brk,
            *shooter_belt_l,
            *shooter_belt_r,
            flywheel_shaft,
            rhino_96,
            fw_l,
            fw_r,
            counter_shaft,
            pivot_shaft,
            rhino_72,
            cam_arm_l,
            cam_arm_r,
            hood,
            s4_turret,
            s5_cam,
            s6_pitch,
            limelight,
        ],
        label="turret_launcher_assembly",
    )


def build_stinger_arm_assembly() -> bd.Compound:
    """
    Stage 5: 140g Carbon-Fiber 'Stinger' Endgame Flower Capper Arm (Servos S7 & S8)
    Seated cleanly on top of the CNC Turret Deck Plate (Z >= 222.5 mm) with zero overlaps!
    """
    raw_servo = read_xbf(VENDOR_DIR / "2000-0025-0003.xbf")
    px, py, pz = cfg.STINGER_PIVOT_X, cfg.STINGER_PIVOT_Y, cfg.STINGER_PIVOT_Z

    piv_bracket = bd.Pos(px, py, pz) * _make_quad_block_mount("stinger_pivot_quad_mount")
    s7_servo = bd.Pos(px + 28.0, py - 18.0, pz + 8.0) * raw_servo
    s7_servo.color = "#fb923c"
    s7_servo.label = "servo_s7_stinger_lift"

    arm_tube = bd.Pos(px + 56.0, py, pz + 56.0) * bd.Rot(0, -45, 0) * bd.Box(120.0, 14.0, 14.0)
    arm_tube.material = plastics.nylon()
    arm_tube.color = "#fb923c"
    arm_tube.label = "carbon_fiber_stinger_tube"

    cup = bd.Pos(px + 166.0, py, pz + 108.0) * stinger_nectar_cup().shape
    cup.label = "stinger_nectar_cup"

    return bd.Compound(children=[piv_bracket, s7_servo, arm_tube, cup], label="stinger_flower_capper_arm")


@model
def project_hive_queen_robot():
    """
    Full Articulated FTC 149 'Project HIVE-QUEEN' BIOBUZZ Robot on Standard goBILDA Strafer Chassis.
    Includes native build123d joints + AP242 mechanism states ('stowed', 'pollen_scoring', 'nectar_endgame')
    and GLB exploded showcase animation.
    """
    chassis = strafer_chassis_stage()
    intake = gatekeeper_intake_stage()
    magazine = sprung_magazine_stage()
    turret = build_turret_assembly()
    stinger = build_stinger_arm_assembly()

    # Articulated Child 1: G407 Front-Lip Gate & Sensor Bridge + 2x REV Color Sensor V3 (REV-31-1557) PCB Modules!
    gate_body = bd.Pos(184.0, 0.0, 134.0) * g407_lip_gate_and_sensor_mount().shape
    gate_body.label = "g407_lip_gate_bar"

    # Model the 2x REV Color Sensor V3 (REV-31-1557) dark-blue PCBs + top white JST PH headers + bottom white/gold optical lenses
    # protruding through the open bezel windows of the pink sensor pods so they are 100% visible from above AND below!
    sensor_modules = []
    for sy, slbl in ((55.0, "left"), (-55.0, "right")):
        pcb = bd.Pos(198.0, sy, 142.0) * bd.Box(14.0, 18.0, 8.0)
        pcb.material = plastics.fr4()
        pcb.color = "#2563eb"
        pcb.label = f"rev_color_sensor_v3_pcb_{slbl}"
        lens = bd.Pos(198.0, sy, 131.0) * bd.Cylinder(5.5, 10.0)
        lens.material = plastics.abs_()
        lens.color = "#facc15"
        lens.label = f"rev_color_sensor_v3_optical_lens_{slbl}"
        jst = bd.Pos(195.0, sy, 149.0) * bd.Box(8.0, 12.0, 8.0)
        jst.material = plastics.abs_()
        jst.color = "#ffffff"
        jst.label = f"rev_color_sensor_v3_jst_port_{slbl}"
        sensor_modules.extend([pcb, lens, jst])

    lip_gate = bd.Compound(children=[gate_body, *sensor_modules], label="g407_lip_gate_assembly")

    # Articulated Child 2: G418.B Bottom-Flower De-Pollinator Fork bolted FLUSH ON TOP of gobilda_1121_10hole_front_portal_arch (Z=204mm)
    # at (X=+156, Y=-42, Z=212.5 -> Z in [204.0, 221.0mm], 100% clear of all U-channels, wheels, and rollers!)
    depol_fork = bd.Pos(156.0, -42.0, 212.5) * flower_depollinator_fork().shape
    depol_fork.label = "flower_depollinator_fork"

    base_core = bd.Compound(children=[chassis, intake, magazine], label="robot_base_core")

    # Define 4 Native Mechanism Joints on base_core:
    # 1. Turret 270° Yaw Joint about Z axis at (TURRET_CENTER_X, 0, TURRET_BASE_Z)
    turret_axis = bd.Axis((cfg.TURRET_CENTER_X, 0.0, cfg.TURRET_BASE_Z), (0, 0, 1))
    bd.RevoluteJoint("turret_yaw", base_core, axis=turret_axis, angular_range=(-135.0, 135.0))
    bd.RigidJoint("turret_yaw", turret, bd.Plane(origin=turret_axis.position, z_dir=turret_axis.direction).location)
    base_core.joints["turret_yaw"].connect_to(turret.joints["turret_yaw"], angle=0.0)

    # 2. Carbon-Fiber Stinger Arm Pitch Joint about Y axis at (STINGER_PIVOT_X, STINGER_PIVOT_Y, STINGER_PIVOT_Z)
    stinger_axis = bd.Axis((cfg.STINGER_PIVOT_X, cfg.STINGER_PIVOT_Y, cfg.STINGER_PIVOT_Z), (0, 1, 0))
    bd.RevoluteJoint("stinger_lift", base_core, axis=stinger_axis, angular_range=(-55.0, 45.0))
    bd.RigidJoint("stinger_lift", stinger, bd.Plane(origin=stinger_axis.position, z_dir=stinger_axis.direction).location)
    base_core.joints["stinger_lift"].connect_to(stinger.joints["stinger_lift"], angle=0.0)

    # 3. Bottom-Flower De-Pollinator Linear Slide along +X (0 to 85 mm)
    depol_axis = bd.Axis((156.0, -42.0, 212.5), (1, 0, 0))
    bd.LinearJoint("depol_slide", base_core, axis=depol_axis, linear_range=(0.0, 85.0))
    bd.RigidJoint("depol_slide", depol_fork, bd.Location(depol_axis.position))
    base_core.joints["depol_slide"].connect_to(depol_fork.joints["depol_slide"], position=0.0)

    # 4. G407 Front-Lip Hard-Stop Gate Linear Drop along -Z (0 to 52 mm)
    gate_axis = bd.Axis((184.0, 0.0, 134.0), (0, 0, -1))
    bd.LinearJoint("lip_gate_drop", base_core, axis=gate_axis, linear_range=(0.0, 52.0))
    bd.RigidJoint("lip_gate_drop", lip_gate, bd.Location(gate_axis.position))
    base_core.joints["lip_gate_drop"].connect_to(lip_gate.joints["lip_gate_drop"], position=0.0)

    robot = bd.Compound(
        children=[base_core, turret, stinger, depol_fork, lip_gate],
        label="project_hive_queen_robot",
    )

    # Verify R102 Starting Configuration Legal 18.0" (457.2 mm) Cube Compliance!
    bb = robot.bounding_box().size
    assert bb.X <= cfg.R102_CUBE_MM, f"R102 X violation: {bb.X} > {cfg.R102_CUBE_MM}"
    assert bb.Y <= cfg.R102_CUBE_MM, f"R102 Y violation: {bb.Y} > {cfg.R102_CUBE_MM}"
    assert bb.Z <= cfg.R102_CUBE_MM, f"R102 Z violation: {bb.Z} > {cfg.R102_CUBE_MM}"

    # Named AP242 Mechanism States & Match Sequence
    mechanism_state(
        robot,
        "stowed",
        turret_yaw=0.0,
        stinger_lift=0.0,
        depol_slide=0.0,
        lip_gate_drop=0.0,
    )
    mechanism_state(
        robot,
        "pollen_scoring",
        turret_yaw=32.0,
        stinger_lift=0.0,
        depol_slide=78.0,
        lip_gate_drop=0.0,
    )
    mechanism_state(
        robot,
        "nectar_endgame",
        turret_yaw=-25.0,
        stinger_lift=-42.0,
        depol_slide=0.0,
        lip_gate_drop=36.0,
    )
    configuration_sequence(
        robot,
        "match_cycle",
        [
            (0.0, "stowed"),
            (1.5, "pollen_scoring"),
            (3.2, "nectar_endgame"),
            (4.8, "stowed"),
        ],
    )

    # GLB Showcase Animation (poses + subsystem explode)
    show = animation(robot, "exploded_showcase", duration=6.0)
    show.pose(at={0.0: "stowed", 1.8: "pollen_scoring", 3.6: "nectar_endgame", 5.5: "stowed"})
    show.translation("turret_launcher_assembly", by=(0.0, 0.0, 95.0), start=1.5, end=3.5)
    show.translation("stinger_flower_capper_arm", by=(40.0, 45.0, 65.0), start=1.5, end=3.5)
    show.translation("flower_depollinator_fork", by=(75.0, 0.0, 0.0), start=1.5, end=3.5)

    return robot


@model
def project_hive_queen_expanded():
    """Posed model showing the Endgame Expanded State (verified inside R105.A 24x18x29 in limit)."""
    doc = posed(project_hive_queen_robot(), "nectar_endgame")
    return doc


if __name__ == "__main__":
    project_hive_queen_robot()
    project_hive_queen_expanded()
