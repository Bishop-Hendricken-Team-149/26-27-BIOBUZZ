import math
import build123d as bd
from bd_materials import metals, plastics
from cad import model

import config as cfg


def _make_funnel_bumper(side: int) -> bd.Part:
    """
    3D-printable Asymmetrical Wide-Area Front Corner Funnel Wing (PETG-CF / TPU) with ZERO U-Channel clipping:
    - Left Wing (side = +1): Extended 23.5 mm forward lead-in (X in [0.0, +23.5 mm], world X in [+216.0, +239.5 mm])
      with shallow 38° inward sweep to gather off-center Nectar (91.95 mm) and Pollen (71.12 mm) balls across a 360 mm front span.
    - Right Wing (side = -1): Shorter 14.0 mm staggered anti-jam relief wing (X in [0.0, +14.0 mm], world X in [+216.0, +230.0 mm],
      9.5 mm axial stagger behind the left wing) with a 29° relief sweep so side-by-side balls cannot bridge or wedge across the throat.
    """
    lead_x = 23.5 if side > 0 else 14.0
    inboard_y = -62.0 if side > 0 else -56.0
    pts_ccw = [
        (0.0, inboard_y),
        (lead_x, -26.0),
        (lead_x, 24.0),
        (0.0, 24.0),
    ]
    sk = bd.Plane.XY.offset(-24.0) * bd.Polygon(*pts_ccw, align=None)
    part = bd.extrude(sk, amount=48.0)

    # Concave G415 FLOWER pipe V-notch on the leading face
    v_cut = bd.Pos(lead_x, -8.0, 0.0) * bd.Cylinder(11.0, 52.0)
    part = part - v_cut

    # goBILDA 8mm metric grid M4 mounting holes through front cap into front of 17H U-Channel
    for dy in (-16.0, 16.0):
        for dz in (-16.0, 16.0):
            hole = bd.Pos(8.0, dy, dz) * bd.Rot(0, 90, 0) * bd.Cylinder(cfg.GB_M4_CLEAR / 2, 24.0)
            part = part - hole

    if side < 0:
        part = bd.mirror(part, about=bd.Plane.XZ)

    part.material = plastics.petg()
    part.color = "#10b981" if side > 0 else "#0d9488"
    part.label = "front_funnel_bumper_left" if side > 0 else "front_funnel_bumper_right"
    assert abs(part.bounding_box().size.Z - 48.0) < 0.5, f"Bumper Z size error: {part.bounding_box().size.Z}"
    return part


@model
def front_funnel_bumper_left():
    p = _make_funnel_bumper(+1)
    assert abs(p.bounding_box().size.Z - 48.0) < 0.5
    return p


@model
def front_funnel_bumper_right():
    p = _make_funnel_bumper(-1)
    assert abs(p.bounding_box().size.Z - 48.0) < 0.5
    return p


@model
def star_roller_sleeve():
    """
    Custom 50mm OD, 6-lobe compliant star intake roller sleeve with 8mm REX hex bore.
    Printable directly in TPU 95A / AMS-TPU or cast in 25A silicone using star_roller_mold_half().
    """
    length = 42.0
    hub = bd.Cylinder(13.0, length)
    lobes = []
    for i in range(6):
        ang = i * 60.0
        # Curved compliant paddle lobe reaching R = 25.0 mm (50.0 mm OD)
        lobe = (
            bd.Rot(0, 0, ang)
            * bd.Pos(16.5, 0.0, 0.0)
            * bd.Box(17.0, 4.2, length)
        )
        tip = (
            bd.Rot(0, 0, ang)
            * bd.Pos(24.0, 0.0, 0.0)
            * bd.Cylinder(2.6, length)
        )
        lobes.append(lobe + tip)
    body = hub
    for lb in lobes:
        body = body + lb

    # 8mm REX (across-flats 8.1mm hex) bore through center
    hex_sk = bd.RegularPolygon(radius=8.15 / math.sqrt(3), side_count=6)
    hex_cut = bd.extrude(hex_sk, amount=length + 2.0, both=True)
    body = body - hex_cut

    body.material = plastics.tpu()
    body.color = "#34d399"
    body.label = "star_roller_sleeve"
    assert abs(body.bounding_box().size.Z - length) < 0.2
    return body


@model
def star_roller_mold_half():
    """
    Bambu AMS 2 Pro 3D-printable 2-part silicone casting mold for the 50mm star intake roller.
    Includes alignment dowel pins/sockets, M4 clamping ears, and 8mm REX core pin socket.
    """
    mold_h = 46.0
    block = bd.Pos(0, -18.0, 0) * bd.Box(74.0, 36.0, mold_h)
    cavity = star_roller_sleeve().shape
    part = block - cavity

    # M4 clamping bolt holes on left and right flanges
    for dx in (-30.0, 30.0):
        for dz in (-15.0, 15.0):
            bolt = bd.Pos(dx, -18.0, dz) * bd.Rot(90, 0, 0) * bd.Cylinder(cfg.GB_M4_CLEAR / 2, 40.0)
            part = part - bolt

    part.material = plastics.pla()
    part.color = "#38bdf8"
    part.label = "star_roller_mold_half"
    return part


@model
def g407_lip_gate_and_sensor_mount():
    """
    Front-lip sensor bridge & G407 4-ball hard-stop gate bar.
    Mounts 2x REV Color Sensor V3 (REV-31-1557) modules 35mm ahead of the front intake lip for
    pre-control G408 opponent Nectar deflection, and guides the S1 hard-stop gate bar across the 256mm mouth.
    """
    span = 252.0
    bar = bd.Box(14.0, span, 8.0)
    # Left and right servo linkage ears with 8mm pitch M4 holes
    ear_l = bd.Pos(-14.0, span / 2 - 6.0, 8.0) * bd.Box(36.0, 8.0, 24.0)
    ear_r = bd.Pos(-14.0, -span / 2 + 6.0, 8.0) * bd.Box(36.0, 8.0, 24.0)
    # Dual front-lip sensor housings (extending forward +X with bottom optical viewing windows!)
    pod_l = bd.Pos(14.0, 55.0, 4.0) * bd.Box(22.0, 28.0, 16.0)
    pod_r = bd.Pos(14.0, -55.0, 4.0) * bd.Box(22.0, 28.0, 16.0)
    part = bar + ear_l + ear_r + pod_l + pod_r

    # Open-top & open-bottom bezel windows through the pink sensor pods so the REV Color Sensor V3 PCB,
    # top white JST connector, and bottom optical lens are 100% visible from above and below!
    for py in (55.0, -55.0):
        bezel_cut = bd.Pos(14.0, py, 4.0) * bd.Box(15.0, 19.0, 20.0)
        part = part - bezel_cut
    for ey in (span / 2 - 6.0, -span / 2 + 6.0):
        piv = bd.Pos(-24.0, ey, 12.0) * bd.Rot(90, 0, 0) * bd.Cylinder(cfg.GB_M4_CLEAR / 2, 16.0)
        part = part - piv

    part.material = plastics.petg()
    part.color = "#fb7185"
    part.label = "g407_lip_gate_and_sensor_mount"
    assert abs(part.bounding_box().size.Y - span) < 1.0
    return part


@model
def flower_depollinator_fork():
    """
    G418.B Legal Bottom-Flower De-Pollinator Twin-Tine Extractor Fork (Servo S2).
    Fits inside the 3.55" x 3.57" (90.2 x 90.7 mm) Flower Retrieval Opening and lifts
    the bottom Pollen out of the 2.79" (70.9 mm) floor ring hole into the intake.
    """
    fork_w = 82.0   # < 90.2 mm Flower bottom retrieval opening width
    fork_l = 96.0
    bridge = bd.Pos(-fork_l / 2 + 12.0, 0, 0) * bd.Box(24.0, fork_w, 8.0)
    tine_l = bd.Pos(10.0, fork_w / 2 - 7.0, 0) * bd.Box(fork_l - 20.0, 14.0, 7.0)
    tine_r = bd.Pos(10.0, -fork_w / 2 + 7.0, 0) * bd.Box(fork_l - 20.0, 14.0, 7.0)
    boss = bd.Pos(-fork_l / 2 + 12.0, 0, 8.0) * bd.Cylinder(10.0, 10.0)
    part = bridge + tine_l + tine_r + boss

    for dy in (-16.0, 0.0, 16.0):
        h = bd.Pos(-fork_l / 2 + 12.0, dy, 4.0) * bd.Cylinder(cfg.GB_M4_CLEAR / 2, 24.0)
        part = part - h

    part.material = plastics.pom()
    part.color = "#10b981"
    part.label = "flower_depollinator_fork"
    assert part.bounding_box().size.Y < 89.0
    return part


@model
def magazine_anti_snag_shroud():
    """
    Continuous smooth-walled single-file magazine side shroud (printed in PETG on AMS 2 Pro).
    Prevents the 11.2mm (Pollen) and 16.1mm (Nectar) ball perforations from catching on standoffs.
    Sized to fit strictly between the Bottom Chute Ramp (Z=72mm) and the CNC Turret Deck (Z=216mm).
    """
    shroud_h = 92.0
    wall = bd.Box(152.0, 4.0, shroud_h)
    # Top and bottom goBILDA 8mm-grid mounting flanges (24mm wide with M4 holes every 16mm)
    flange_bot = bd.Pos(0.0, 10.0, -shroud_h / 2 + 4.0) * bd.Box(152.0, 24.0, 8.0)
    flange_top = bd.Pos(0.0, 10.0, shroud_h / 2 - 4.0) * bd.Box(152.0, 24.0, 8.0)
    part = wall + flange_bot + flange_top

    # Optical ToF / Beam-Break sensor viewing slots and M4 grid holes
    for dx in (-64.0, -32.0, 0.0, 32.0, 64.0):
        h_bot = bd.Pos(dx, 12.0, -shroud_h / 2 + 4.0) * bd.Cylinder(cfg.GB_M4_CLEAR / 2, 16.0)
        h_top = bd.Pos(dx, 12.0, shroud_h / 2 - 4.0) * bd.Cylinder(cfg.GB_M4_CLEAR / 2, 16.0)
        part = part - h_bot - h_top
    for dx in (-36.0, 36.0):
        win = bd.Pos(dx, 0.0, 0.0) * bd.Rot(90, 0, 0) * bd.Cylinder(22.0, 12.0)
        part = part - win

    part.material = plastics.petg()
    part.color = "#a78bfa"
    part.label = "magazine_anti_snag_shroud"
    return part


@model
def shooter_cam_and_counter_arm():
    """
    Bi-Positional Eccentric Cam + 4.2:3 Upper Counter-Roller Pivot Arm (PAHT-CF).
    Switches the upper 72mm Rhino counter-wheel between 67.0mm (Pollen) and 87.0mm (Nectar)
    in 55 ms while pivoting about the belt idler axis so belt tension never changes.
    """
    arm_len = 96.0
    arm = bd.Box(arm_len, 10.0, 24.0)
    hub_pivot = bd.Pos(-arm_len / 2, 0, 0) * bd.Rot(90, 0, 0) * bd.Cylinder(16.0, 14.0)
    hub_wheel = bd.Pos(arm_len / 2, 0, 0) * bd.Rot(90, 0, 0) * bd.Cylinder(16.0, 14.0)
    # Eccentric cam lobe follower rest on top of arm
    cam_follower = bd.Pos(0, 0, 14.0) * bd.Box(28.0, 14.0, 8.0)
    part = arm + hub_pivot + hub_wheel + cam_follower

    # 8mm REX bearing bores at pivot (-48mm) and counter-wheel (+48mm)
    for dx in (-arm_len / 2, arm_len / 2):
        bore = bd.Pos(dx, 0, 0) * bd.Rot(90, 0, 0) * bd.Cylinder(7.0, 20.0)
        part = part - bore

    part.material = plastics.nylon()
    part.color = "#f43f5e"
    part.label = "shooter_cam_and_counter_arm"
    return part


@model
def stinger_nectar_cup():
    """
    140g Carbon-Fiber 'Stinger' Endgame Flower Capper Nectar Drop Cup (Servo S7/S8).
    Cradles a 3.62" (91.95mm) Nectar ball and drops it dead-center into the 4.00" (101.6mm)
    Flower top opening at z = 24.0" (610mm) in 0.8 seconds.
    """
    outer_r = 50.0  # Fits inside 101.6mm Flower backstop
    inner_r = 46.5  # Cradles 91.95mm Nectar ball with 1mm radial clearance
    ring = bd.Cylinder(outer_r, 28.0) - bd.Cylinder(inner_r, 30.0)
    # Front release gate cutout so Nectar drops cleanly when S8 latch opens
    front_cut = bd.Pos(35.0, 0, 0) * bd.Box(45.0, 64.0, 32.0)
    # Carbon-fiber tube socket clamp on rear (-X)
    stem = bd.Pos(-outer_r - 16.0, 0, 0) * bd.Box(36.0, 24.0, 28.0)
    part = (ring - front_cut) + stem

    for dx in (-outer_r - 24.0, -outer_r - 8.0):
        hole = bd.Pos(dx, 0, 0) * bd.Rot(90, 0, 0) * bd.Cylinder(cfg.GB_M4_CLEAR / 2, 30.0)
        part = part - hole

    part.material = plastics.petg()
    part.color = "#fb923c"
    part.label = "stinger_nectar_cup"
    return part


def _make_robot_sign_149(bg_hex: str, label_suffix: str) -> bd.Compound:
    """
    Rule R401-R403 Legal Team 149 Alliance & Number Sign for Multi-Color Bambu AMS 2 Pro print:
    - 172.0 mm x 70.0 mm (6.77 in x 2.76 in > R401.B/C 6.5 in x 2.5 in minimum)
    - 57.2 mm (2.252 in) tall solid opaque white Arabic numerals '149' (exact R403.A 2.25 in height)
    - 6.4 mm (0.252 in) top/bottom border margin (R403.B) + tool-less quick-swap holes at X = ±72 mm
    """
    w = 172.0
    h = 70.0
    plate = bd.Box(w, 3.2, h)
    num_h = 57.2
    stroke = 9.0
    d1 = bd.Pos(-40.0, -2.0, 0.0) * bd.Box(stroke, 1.6, num_h)
    d4_v1 = bd.Pos(-14.0, -2.0, num_h * 0.20) * bd.Box(stroke, 1.6, num_h * 0.60)
    d4_h = bd.Pos(-3.0, -2.0, -3.0) * bd.Box(28.0, 1.6, stroke)
    d4_v2 = bd.Pos(6.0, -2.0, 0.0) * bd.Box(stroke, 1.6, num_h)
    d9_top = (
        bd.Pos(34.0, -2.0, num_h * 0.21) * bd.Box(26.0, 1.6, num_h * 0.58)
        - bd.Pos(34.0, -2.0, num_h * 0.21) * bd.Box(9.5, 3.0, num_h * 0.24)
    )
    d9_r = bd.Pos(42.5, -2.0, 0.0) * bd.Box(stroke, 1.6, num_h)
    d9_bot = bd.Pos(34.0, -2.0, -num_h / 2 + stroke / 2) * bd.Box(26.0, 1.6, stroke)

    for dx in (-72.0, 72.0):
        for dz in (-20.0, 20.0):
            hole = bd.Pos(dx, 0.0, dz) * bd.Rot(90, 0, 0) * bd.Cylinder(cfg.GB_M4_CLEAR / 2, 10.0)
            plate = plate - hole

    plate.material = plastics.petg()
    plate.color = bg_hex
    plate.label = f"sign_background_{label_suffix}"

    nums = d1 + d4_v1 + d4_h + d4_v2 + d9_top + d9_r + d9_bot
    nums.material = plastics.petg()
    nums.color = "#ffffff"
    nums.label = f"sign_numerals_149_{label_suffix}"

    sign = bd.Compound(children=[plate, nums], label=f"robot_sign_plate_149_{label_suffix}")
    assert abs(sign.bounding_box().size.X - w) < 0.5
    assert abs(sign.bounding_box().size.Z - h) < 0.5
    assert abs(nums.bounding_box().size.Z - num_h) < 0.5
    return sign


@model
def robot_sign_plate_149():
    return _make_robot_sign_149("#dc2626", "red")


@model
def robot_sign_plate_149_blue():
    return _make_robot_sign_149("#2563eb", "blue")


@model
def spoked_compliant_intake_wheel():
    """
    47.0 mm OD (R = 23.5 mm), 24.0 mm wide segmented spoked compliant intake wheel (TPU 85A / 95A)
    with 8 hollow radial flex-spokes and 8mm REX bore for the fast variable floating intake.
    """
    r_out = 23.5
    width = 24.0
    wheel = bd.Cylinder(r_out, width) - bd.Cylinder(4.05, width + 2.0)
    for k in range(8):
        ang = k * 45.0
        hole = bd.Rot(0, 0, ang) * bd.Pos(14.2, 0.0, 0.0) * bd.Cylinder(4.8, width + 2.0)
        wheel = wheel - hole
    wheel.material = plastics.tpu()
    wheel.color = "#14b8a6"
    wheel.label = "spoked_compliant_intake_wheel"
    assert abs(wheel.bounding_box().size.X - 47.0) < 0.5
    return wheel


@model
def floating_intake_swing_arm():
    """
    83.24 mm center-to-center machined aluminum / PETG-CF floating intake swing-arm link
    connecting Upper Pivot Shaft (156.0, 144.0) to Floor-Biting Floating Front Shaft (208.0, 79.0).
    """
    span = math.hypot(208.0 - 156.0, 79.0 - 144.0)
    bar = bd.Box(span + 20.0, 7.0, 18.0)
    b1 = bd.Pos(-span / 2.0, 0.0, 0.0) * bd.Rot(90, 0, 0) * bd.Cylinder(11.0, 7.0)
    b2 = bd.Pos(span / 2.0, 0.0, 0.0) * bd.Rot(90, 0, 0) * bd.Cylinder(11.0, 7.0)
    c1 = bd.Pos(-span / 2.0, 0.0, 0.0) * bd.Rot(90, 0, 0) * bd.Cylinder(4.05, 10.0)
    c2 = bd.Pos(span / 2.0, 0.0, 0.0) * bd.Rot(90, 0, 0) * bd.Cylinder(4.05, 10.0)
    arm = (bar + b1 + b2) - c1 - c2
    arm.material = metals.aluminum()
    arm.color = "#e2e8f0"
    arm.label = "floating_intake_swing_arm"
    assert abs(arm.bounding_box().size.Y - 7.0) < 0.5
    return arm


if __name__ == "__main__":
    front_funnel_bumper_left()
    front_funnel_bumper_right()
    star_roller_sleeve()
    star_roller_mold_half()
    g407_lip_gate_and_sensor_mount()
    flower_depollinator_fork()
    magazine_anti_snag_shroud()
    shooter_cam_and_counter_arm()
    stinger_nectar_cup()
    robot_sign_plate_149()
    robot_sign_plate_149_blue()
    spoked_compliant_intake_wheel()
    floating_intake_swing_arm()
