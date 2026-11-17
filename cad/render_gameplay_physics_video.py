"""
Generates a 1080p Physics-Grounded Gameplay & Subsystem Animation Video (hive_queen_gameplay_physics.mp4)
for FTC Team 149 BIOBUZZ ('Project HIVE-QUEEN'), illustrating the exact CAD geometry, force vectors
(F_normal, F_traction, m*g, F_Magnus, F_drag), wheel speeds, nip compression, and RK4 ballistic trajectory
across all 5 robot functions in gameplay.
"""
import math
import os
import subprocess
from PIL import Image, ImageDraw, ImageFont

W, H = 1920, 1080
FPS = 30
DURATION_S = 15.0
TOTAL_FRAMES = int(FPS * DURATION_S)

OUT_DIR = r"C:\Users\danu0\Downloads\26-27-BIOBUZZ\cad\tmp\anim_frames"
os.makedirs(OUT_DIR, exist_ok=True)
MP4_PATH = r"C:\Users\danu0\Downloads\26-27-BIOBUZZ\cad\tmp\hive_queen_gameplay_physics.mp4"


def load_font(size: int, bold: bool = False):
    candidates = [
        r"C:\Windows\Fonts\consolab.ttf" if bold else r"C:\Windows\Fonts\consola.ttf",
        r"C:\Windows\Fonts\segoeui.ttf",
        r"C:\Windows\Fonts\arial.ttf",
    ]
    for p in candidates:
        if os.path.exists(p):
            return ImageFont.truetype(p, size)
    return ImageFont.load_default()


FONT_TITLE = load_font(22, True)
FONT_H2 = load_font(19, True)
FONT_BODY = load_font(16, False)
FONT_BOLD = load_font(16, True)
FONT_SM = load_font(14, True)
FONT_XS = load_font(13, False)


# Left panel: True-to-scale XZ Cross-Section of Robot (X in [-240, +260 mm], Z in [-20, +430 mm])
def mm_to_px_robot(x_mm: float, z_mm: float) -> tuple[float, float]:
    scale = 1.76
    cx_px = 510.0 + x_mm * scale
    cz_px = 945.0 - z_mm * scale
    return (cx_px, cz_px)


# Right-top panel: Zoomed-in Shooter Nip & Hood Physics Diagram (confined strictly to right sub-box x in [1500..1880], y in [150..525]!)
def mm_to_px_nip(x_mm: float, z_mm: float) -> tuple[float, float]:
    scale = 1.32
    cx_px = 1672.0 + (x_mm - (-68.0)) * scale
    cz_px = 362.0 - (z_mm - 306.0) * scale
    return (cx_px, cz_px)


# Right-bottom panel: Full FTC Field Ballistic Trajectory (RK4 with Gravity + Drag + Topspin Magnus Dip)
def m_to_px_field(x_m: float, z_m: float) -> tuple[float, float]:
    scale_x = 265.0
    scale_z = 210.0
    return (1135.0 + x_m * scale_x, 955.0 - z_m * scale_z)


def draw_arrow(draw: ImageDraw.ImageDraw, p1, p2, color, width=4, head_len=14):
    x1, y1 = p1
    x2, y2 = p2
    draw.line([p1, p2], fill=color, width=width)
    ang = math.atan2(y2 - y1, x2 - x1)
    a1 = ang + math.radians(150)
    a2 = ang - math.radians(150)
    p_l = (x2 + head_len * math.cos(a1), y2 + head_len * math.sin(a1))
    p_r = (x2 + head_len * math.cos(a2), y2 + head_len * math.sin(a2))
    draw.polygon([p2, p_l, p_r], fill=color)


def draw_wheel(draw: ImageDraw.ImageDraw, center_px, r_px, angle_rad, tire_color, hub_color, cw=True):
    cx, cy = center_px
    draw.ellipse([cx - r_px, cy - r_px, cx + r_px, cy + r_px], fill=tire_color, outline=(255, 255, 255), width=2)
    hr = r_px * 0.52
    draw.ellipse([cx - hr, cy - hr, cx + hr, cy + hr], fill=hub_color, outline=(40, 50, 65), width=2)
    # 6 spokes + spin arrows
    for k in range(6):
        a = angle_rad + k * (math.pi / 3.0)
        sx = cx + (r_px * 0.85) * math.cos(a)
        sy = cy + (r_px * 0.85) * math.sin(a)
        draw.line([(cx, cy), (sx, sy)], fill=(30, 41, 59), width=2)
    # Spin direction arc arrow
    arc_a = -0.6 if not cw else 0.6
    ax1 = cx + r_px * 0.72 * math.cos(-0.4)
    ay1 = cy + r_px * 0.72 * math.sin(-0.4)
    ax2 = cx + r_px * 0.72 * math.cos(-0.4 + arc_a)
    ay2 = cy + r_px * 0.72 * math.sin(-0.4 + arc_a)
    draw_arrow(draw, (ax1, ay1), (ax2, ay2), (255, 255, 255), width=2, head_len=8)


# Continuous 21° Ramp -> Curved Rear Wall (X=-92..-114) -> 67.8mm Diagonal Nip Path!
# Parameter s in [0.0, 1.0]:
#   s = 0.00: Floor Flower in front of intake (X=235, Z=35.6)
#   s = 0.24: Parked Slot 1 (Bottom of Magazine, X=123.65, Z=90.03)
#   s = 0.38: Parked Slot 2 (Middle of Magazine, X=52.70, Z=117.26)
#   s = 0.52: Parked Slot 3 (Top of Magazine, HELD AGAINST CLOSED SERVO S3 GATE at X=-18.25, Z=144.50!)
#   s = 0.68: Pinched between Gecko 3 Kicker (+4, 200.5, R=22) & Curved Rear Guide Wall (X=-96, Z=192) -> X=-54, Z=198 (ZERO floating!)
#   s = 0.82: Pinched in 67.8mm Diagonal Nip between Main 96mm Wheel (-10,284,R=48) & Counter 48mm Wheel (-126,362,R=24) -> X=-74, Z=328!
#   s = 1.00: Launched at +48° out the unobstructed front window!
def pos_from_path_s(s: float, hood_deg: float = 48.0) -> tuple[float, float]:
    if s <= 0.14:
        k = s / 0.14
        return (235.0 - 50.0 * k, 35.6)
    elif s <= 0.24:
        k = (s - 0.14) / 0.10
        return (185.0 - 61.35 * k, 35.6 + 54.43 * k)
    elif s <= 0.52:
        k = (s - 0.24) / 0.28
        return (123.65 - 141.90 * k, 90.03 + 54.47 * k)
    elif s <= 0.68:
        k = (s - 0.52) / 0.16
        # Curved pinch handoff between Gecko 3 (+4, 200.5, R=22) and Curved Rear Guide Wall (X=-92..-106)
        return (-18.25 - 38.0 * k, 144.50 + 58.0 * k)
    elif s <= 0.84:
        k = (s - 0.68) / 0.16
        # Direct bridge from Gecko 3 into Main 96mm Wheel (-10, 284) & Counter 48mm Wheel (-126, 362)
        return (-56.25 - 17.75 * math.sin(math.pi * k) + 8.0 * k, 202.50 + 135.5 * k)
    else:
        k = (s - 0.84) / 0.16
        dt = k * 0.020
        vx_mm_s = 7400.0 * math.cos(math.radians(hood_deg))
        vz_mm_s = 7400.0 * math.sin(math.radians(hood_deg))
        bx = -48.0 + vx_mm_s * dt
        bz = 338.0 + vz_mm_s * dt - 0.5 * 9810.0 * (dt**2)
        return (bx, bz)


# Compute the 3 balls' positions (s1, s2, s3), S3 Gate state, and Hood Angle at time t in [0, 15s]:
#   0.0 - 3.6s (STAGE 1 & 2 — INTAKE & INDEX): Balls 3, 2, 1 are swept in and STOP at s=0.52, 0.38, 0.24 behind CLOSED Servo S3 Gate!
#   3.6 - 7.5s (STAGE 3 — DRIVE & AIM HOOD, BALLS HELD STATIONARY!): All 3 balls stay 100% locked at s=0.52, 0.38, 0.24 while S6 tilts the Hood (36°->54°->48°) and M5/M6 spin up!
#   7.5 - 12.0s (STAGE 4 — SERVO S3 GATE OPENS & FIRES 3-BALL BURST!): S3 Gate swings OPEN (+55°) and Gecko 3 feeds Ball 3, Ball 2, Ball 1 through the 67.8mm Nip!
#   12.0 - 15.0s (STAGE 5 — STINGER ENDGAME CAP): S3 Gate closes; Stinger arm caps the 20.5" Tall Flower!
def get_match_state(t: float):
    if t < 3.6:
        p = t / 3.6
        s3 = min(0.52, 0.02 + 0.58 * p)
        s2 = min(0.38, max(0.0, -0.10 + 0.54 * p))
        s1 = min(0.24, max(0.0, -0.22 + 0.50 * p))
        return (s1, s2, s3, False, 40.0, "STAGE 1–2 (0–3.6s): Intake Sweeps 3 Balls In -> STOPPED & HELD by Closed Servo S3 Gate!")
    elif t < 7.5:
        k = (t - 3.6) / 3.9
        hood_aim = 48.0 + 9.0 * math.sin(2.0 * math.pi * k) * (1.0 - 0.6 * k)
        return (0.24, 0.38, 0.52, False, hood_aim, "STAGE 3 (3.6–7.5s): 3 Balls HELD STATIONARY by S3 Gate While Driving, Aiming Hood (S6) & Spinning Flywheel!")
    elif t < 12.0:
        k = (t - 7.5) / 4.5
        s3 = min(1.0, 0.52 + 1.15 * k)
        s2 = min(1.0, 0.38 + max(0.0, (k - 0.22) * 1.15))
        s1 = min(1.0, 0.24 + max(0.0, (k - 0.44) * 1.15))
        return (s1, s2, s3, True, 48.0, "STAGE 4 (7.5–12.0s): Servo S3 Gate OPENS -> Gecko 3 Feeds Held Balls Into 67.8mm Shooter Nip!")
    else:
        return (0.24, 0.38, 0.52, False, 48.0, "STAGE 5 (12.0–15.0s): Servo S3 Gate Closed | 140g Carbon-Fiber Stinger Arm (S7) Caps Tall Flower!")


# Precompute RK4 Full-Field Ballistic Trajectory (Topspin Magnus effect drops ball steeply into Beehive High Goal at X=2.22m, Z=1.00m!)
def compute_rk4_trajectory(v0=6.35, theta_deg=48.0, topspin_rad_s=95.0):
    dt = 0.005
    x, z = 0.0, 0.35
    vx = v0 * math.cos(math.radians(theta_deg))
    vz = v0 * math.sin(math.radians(theta_deg))
    g = 9.81
    k_drag = 0.055
    k_mag = 0.0022
    pts = [(x, z, vx, vz)]
    for _ in range(260):
        v = math.hypot(vx, vz)
        ax = -k_drag * v * vx + k_mag * topspin_rad_s * vz
        az = -g - k_drag * v * vz - k_mag * topspin_rad_s * vx
        x += vx * dt
        z += vz * dt
        vx += ax * dt
        vz += az * dt
        pts.append((x, z, vx, vz))
        if (x >= 2.28 and z <= 1.02) or z < 0.05:
            break
    return pts


TRAJ_POLLEN = compute_rk4_trajectory(6.35, 48.0, 95.0)
TRAJ_NECTAR = compute_rk4_trajectory(6.15, 50.0, 75.0)

for f_idx in range(TOTAL_FRAMES):
    t = f_idx / FPS
    img = Image.new("RGB", (W, H), (15, 23, 42))
    draw = ImageDraw.Draw(img)

    # Background panels (Two-line header so main title and active phase NEVER overlap!)
    draw.rounded_rectangle([24, 16, 1920 - 24, 92], radius=12, fill=(30, 41, 59), outline=(71, 85, 105), width=2)
    draw.text(
        (44, 24),
        "FTC TEAM 149 BIOBUZZ — PROJECT HIVE-QUEEN: PHYSICS-GROUNDED GAMEPLAY & SHOOTER SIMULATION",
        fill=(56, 189, 248),
        font=FONT_TITLE,
    )

    s1_val, s2_val, s3_val, s3_gate_open, hood_aim_deg, status_str = get_match_state(t)
    draw.text((44, 58), status_str, fill=(250, 204, 21), font=FONT_BOLD)

    # Left Main Viewport Panel (True-to-Scale XZ Robot Cross-Section)
    draw.rounded_rectangle([24, 102, 1068, 995], radius=14, fill=(19, 29, 49), outline=(51, 65, 85), width=2)
    draw.text(
        (44, 114),
        "1. TRUE-TO-SCALE XZ CAD CROSS-SECTION (mm) & ACTIVE FORCE VECTORS",
        fill=(148, 163, 184),
        font=FONT_H2,
    )

    # Draw Floor Mat (Z = 0 mm) & Coordinate Grid
    _, floor_y = mm_to_px_robot(0, 0)
    draw.line([(40, floor_y), (1050, floor_y)], fill=(100, 116, 139), width=4)
    for gx in range(-200, 250, 50):
        px, _ = mm_to_px_robot(gx, 0)
        draw.line([(px, 150), (px, floor_y)], fill=(30, 41, 59), width=1)
        draw.text((px - 18, floor_y + 8), f"X={gx}", fill=(100, 116, 139), font=FONT_XS)
    for gz in range(50, 400, 50):
        _, py = mm_to_px_robot(0, gz)
        draw.line([(40, py), (1050, py)], fill=(30, 41, 59), width=1)
        draw.text((44, py - 14), f"Z={gz}mm", fill=(100, 116, 139), font=FONT_XS)

    # Draw 18" R102 Starting Cube Envelope (X in [-216, +232], Z = 457.2 mm)
    x_l, z_t = mm_to_px_robot(-216.0, 410.0)
    x_r, _ = mm_to_px_robot(232.0, 0.0)
    draw.rectangle([x_l, z_t, x_r, floor_y], outline=(56, 189, 248), width=1)

    # Draw Base 17-Hole U-Channel Rail (X in [-216, +216], Z in [24, 72]) + Rear/Mid/Front Uprights
    r_p1 = mm_to_px_robot(-216.0, 72.0)
    r_p2 = mm_to_px_robot(216.0, 24.0)
    draw.rectangle([r_p1, r_p2], fill=(51, 65, 85), outline=(148, 163, 184), width=2)
    # Uprights: Rear (-192..-144, Z 72..216), Mid (-24..+24, Z 72..216), Front (+168..+216, Z 72..204)
    for ux1, ux2, uz2, ulbl in ((-192, -144, 216, "Rear 5H"), (-24, 24, 216, "Mid 5H"), (168, 216, 204, "Front 4H")):
        u1 = mm_to_px_robot(ux1, uz2)
        u2 = mm_to_px_robot(ux2, 72.0)
        draw.rectangle([u1, u2], fill=(60, 75, 95), outline=(148, 163, 184), width=2)

    # Turret Deck Plate (X in [-192, +14], Z in [216, 222]) + Slewing Ring + Rotating Base (Z in [222, 236])
    d1 = mm_to_px_robot(-192.0, 222.0)
    d2 = mm_to_px_robot(-142.0, 216.0)
    draw.rectangle([d1, d2], fill=(203, 213, 225), outline=(255, 255, 255), width=1)
    # Turret Side Plate outline (X in [-156, -12], Z in [236, 354])
    tp1 = mm_to_px_robot(-156.0, 354.0)
    tp2 = mm_to_px_robot(-12.0, 236.0)
    draw.rectangle([tp1, tp2], outline=(245, 158, 11), width=2)

    # Continuous +21° Ball Chute Ramp (from (170, 31.6) to (-62, 120.4)) + Rear Elevator Backplate (X=-134, Z=126..276)
    ramp_pts = [
        mm_to_px_robot(170.0, 31.6),
        mm_to_px_robot(-62.0, 120.4),
        mm_to_px_robot(-64.0, 115.0),
        mm_to_px_robot(168.0, 26.2),
    ]
    draw.polygon(ramp_pts, fill=(251, 191, 36), outline=(255, 255, 255))

    # Rear Elevator Curved Scoop & Vertical Backplate at X = -114 mm (68mm pinch channel opposite Gecko 4 & 5 at X=-22, R=24 -> tread X=-46mm!)
    backplate_pts = [
        mm_to_px_robot(-62.0, 120.4),
        mm_to_px_robot(-114.0, 142.0),
        mm_to_px_robot(-114.0, 326.0),
        mm_to_px_robot(-120.0, 326.0),
        mm_to_px_robot(-120.0, 138.0),
        mm_to_px_robot(-64.0, 115.0),
    ]
    draw.polygon(backplate_pts, fill=(148, 163, 184), outline=(255, 255, 255))
    bp_lbl_p = mm_to_px_robot(-228.0, 205.0)
    draw.text(bp_lbl_p, "Rear Backplate\n(X=-114mm)", fill=(226, 232, 240), font=FONT_XS)

    # Floor-Level G418.B De-Pollinator Fork (Z = 20 mm) animated extension in Stage 1!
    phase_idx = 1 if t < 3.6 else (3 if t < 7.5 else (4 if t < 12.0 else 5))
    fork_ext_mm = 32.0 * math.sin(math.pi * (t / 3.6)) if phase_idx == 1 else 0.0
    fk1 = mm_to_px_robot(100.0 + fork_ext_mm, 25.0)
    fk2 = mm_to_px_robot(196.0 + fork_ext_mm, 17.0)
    draw.rectangle([fk1, fk2], fill=(16, 185, 129), outline=(255, 255, 255), width=2)
    if phase_idx == 1:
        # Draw Field Flower bottom opening at X = 215..255 mm
        fl1 = mm_to_px_robot(218.0, 110.0)
        fl2 = mm_to_px_robot(256.0, 0.0)
        draw.rectangle([fl1, fl2], outline=(244, 63, 94), width=3)
        draw.text(mm_to_px_robot(195.0, 132.0), "Field Flower\n3.55\" Bottom", fill=(244, 63, 94), font=FONT_XS)
        draw_arrow(
            draw,
            mm_to_px_robot(145.0, 21.0),
            mm_to_px_robot(210.0 + fork_ext_mm, 21.0),
            (16, 185, 129),
            width=4,
        )

    # G407 Lip Gate Bridge (X = 202 mm, Z = 148 mm)
    gate_lift = 18.0 if phase_idx in (2, 3) else 0.0
    gt1 = mm_to_px_robot(186.0, 160.0 + gate_lift)
    gt2 = mm_to_px_robot(214.0, 142.0 + gate_lift)
    draw.rectangle([gt1, gt2], fill=(251, 113, 133), outline=(255, 255, 255), width=2)
    draw.text(mm_to_px_robot(220.0, 162.0 + gate_lift), "G407 Lip Gate\n+ Color Sensor", fill=(251, 113, 133), font=FONT_XS)

    # Rotating Wheels in Left Cross-Section:
    # 1) Front Star Roller (X = +192, Z = 104, R = 33 mm)
    star_c = mm_to_px_robot(192.0, 104.0)
    star_r = 33.0 * 1.76
    for b_i in range(6):
        ba = -t * 7.0 + b_i * (math.pi / 3.0)
        bx = star_c[0] + star_r * math.cos(ba)
        by = star_c[1] + star_r * math.sin(ba)
        draw.line([star_c, (bx, by)], fill=(52, 211, 153), width=7)
    draw.ellipse([star_c[0] - 12, star_c[1] - 12, star_c[0] + 12, star_c[1] + 12], fill=(203, 213, 225))

    # 2) 3x Gecko Feeder & Throat Kicker Shafts (14.7mm CLEAR of Main 96mm Shooter Wheel — ZERO wheel clipping!):
    #    (136.3, 156.4, R=30), (69.1, 182.2, R=30), (4.0, 200.5, R=22)
    #    Note: During Stage 3 (3.6-7.5s, Aiming Mode), M8 & Gecko wheels STOP spinning while S3 Gate holds the 3 balls!
    gecko_spin = -t * 8.0 if (t < 3.6 or 7.5 <= t < 12.0) else -3.6 * 8.0
    for gx, gz, gr in (
        (136.3, 156.4, 30.0),
        (69.1, 182.2, 30.0),
        (4.0, 200.5, 22.0),
    ):
        draw_wheel(draw, mm_to_px_robot(gx, gz), gr * 1.76, gecko_spin, (34, 197, 94), (203, 213, 225), cw=False)

    # Draw Timing Belts linking Gecko 1 <-> Gecko 2 <-> Gecko 3
    draw.line([mm_to_px_robot(136.3, 156.4), mm_to_px_robot(69.1, 182.2)], fill=(250, 204, 21), width=3)
    draw.line([mm_to_px_robot(69.1, 182.2), mm_to_px_robot(4.0, 200.5)], fill=(250, 204, 21), width=3)

    # Draw SERVO S3 INDEXING STOPPER GATE at (X = -46.0, Z = 178.0):
    # - CLOSED (Red Bar blocking chute) during Stages 1-3 (0..7.5s) to HOLD all 3 balls stationary while driving & aiming!
    # - OPEN (Green Bar swung up +55°) during Stage 4 (7.5..12.0s) to fire the 3 held balls into the shooter!
    gate_piv = mm_to_px_robot(-46.0, 184.0)
    gate_end = mm_to_px_robot(-20.0, 204.0) if s3_gate_open else mm_to_px_robot(-46.0, 136.0)
    gate_col = (34, 197, 94) if s3_gate_open else (244, 63, 94)
    draw.line([gate_piv, gate_end], fill=gate_col, width=8)
    draw.ellipse([gate_piv[0] - 6, gate_piv[1] - 6, gate_piv[0] + 6, gate_piv[1] + 6], fill=(255, 255, 255))
    draw.text(
        mm_to_px_robot(-108.0, 196.0),
        "S3 Gate: OPEN (Firing!)" if s3_gate_open else "S3 Gate: CLOSED (Holding 3 Balls!)",
        fill=gate_col,
        font=FONT_XS,
    )

    # 3) Lower-Front Main 96mm Shooter Flywheel (-10.0, 284.0, R=48.0) & Upper-Rear Counter 48mm Wheel (-126.0, 362.0, R=24.0)
    #    14.7mm CLEAR above Gecko 3 (4.0, 200.5, R=22) -> ZERO compliant wheel clipping!
    draw_wheel(draw, mm_to_px_robot(-10.0, 284.0), 48.0 * 1.76, -t * 16.0, (245, 158, 11), (226, 232, 240), cw=False)
    draw_wheel(draw, mm_to_px_robot(-126.0, 362.0), 24.0 * 1.76, t * 16.0, (244, 63, 94), (226, 232, 240), cw=True)

    # 4) Raised Upper-Rear Cyan Deflector Hood (tilting live with hood_aim_deg during Stage 3 Aiming Mode!)
    hood_dz = (hood_aim_deg - 48.0) * 1.2
    hood_arc_pts = [
        mm_to_px_robot(-112.0, 384.0),
        mm_to_px_robot(-88.0, 396.0 + hood_dz * 0.5),
        mm_to_px_robot(-56.0, 400.0 + hood_dz),
    ]
    draw.line(hood_arc_pts, fill=(56, 189, 248), width=9)
    draw.text(mm_to_px_robot(-228.0, 386.0), f"Hood S6 ({hood_aim_deg:.0f}°)", fill=(56, 189, 248), font=FONT_XS)

    # 5) Stage 5 Carbon-Fiber Stinger Endgame Arm (Pivot at X=-4, Z=244)
    stinger_ang_deg = -38.0 if phase_idx < 5 else -38.0 + 76.0 * (0.5 - 0.5 * math.cos(math.pi * ((t - 12.0) / 3.0)))
    st_rad = math.radians(stinger_ang_deg)
    piv_px = mm_to_px_robot(-4.0, 244.0)
    tip_x = -4.0 + 145.0 * math.cos(st_rad)
    tip_z = 244.0 - 145.0 * math.sin(st_rad)
    tip_px = mm_to_px_robot(tip_x, tip_z)
    draw.line([piv_px, tip_px], fill=(251, 146, 60), width=8)
    draw.ellipse([tip_px[0] - 18, tip_px[1] - 18, tip_px[0] + 18, tip_px[1] + 18], outline=(251, 146, 60), width=4)
    if phase_idx == 5:
        draw.text((tip_px[0] - 30, tip_px[1] - 42), "Stinger Caps 20.5\" Tall Flower!", fill=(251, 146, 60), font=FONT_BOLD)

    # Draw the 3 Staged/Firing Balls (HELD STATIONARY during Stage 3 Aiming, then FIRED in Stage 4!)
    for s_ball in (s1_val, s2_val, s3_val):
        bx_mm, bz_mm = pos_from_path_s(s_ball, hood_aim_deg)
        b_px = mm_to_px_robot(bx_mm, bz_mm)
        br_px = 35.56 * 1.76
        sq_r = 0.88 if 0.66 <= s_ball <= 0.85 else 1.0
        draw.ellipse(
            [b_px[0] - br_px * sq_r, b_px[1] - br_px, b_px[0] + br_px * sq_r, b_px[1] + br_px],
            fill=(250, 204, 21),
            outline=(255, 255, 255),
            width=3,
        )
        if 3.6 <= t < 7.5 and abs(s_ball - 0.52) < 0.02:
            draw.text((b_px[0] - 38, b_px[1] - 10), "HELD", fill=(15, 23, 42), font=FONT_BOLD)
        elif 0.72 <= s_ball <= 0.88:
            draw_arrow(draw, b_px, (b_px[0] + 66, b_px[1] - 72), (244, 63, 94), width=5)
            draw.text((b_px[0] + 42, b_px[1] - 82), "v_exit = 7.4 m/s (48°)", fill=(244, 63, 94), font=FONT_BOLD)

    # Re-draw Curved Hood on top of the ball so the hood visibly wraps OVER the ball!
    draw.line(hood_arc_pts, fill=(56, 189, 248), width=9)

    # ==========================================================================
    # RIGHT-TOP PANEL: ZOOMED-IN SHOOTER NIP & LAUNCH MECHANICS EXPLAINER
    # ==========================================================================
    draw.rounded_rectangle([1084, 102, 1896, 545], radius=14, fill=(19, 29, 49), outline=(56, 189, 248), width=2)
    draw.text(
        (1104, 114),
        "2. HOW THE BALL IS SHOT OUT: DUAL-WHEEL NIP + 42° DEFLECTOR HOOD",
        fill=(56, 189, 248),
        font=FONT_H2,
    )
    # Dedicated right sub-frame for the graphic (x in [1495, 1882], y in [150, 532])
    draw.rounded_rectangle([1495, 148, 1882, 532], radius=10, fill=(15, 23, 42), outline=(51, 65, 85), width=1)

    # Draw Gecko 3 Kicker (+4, 200.5, R=22 — 14.7mm clear of Main Wheel!), Main 96mm Wheel (-10, 284, R=48), and Counter 48mm Wheel (-126, 362, R=24) in Zoomed Panel!
    g5_c = mm_to_px_nip(4.0, 200.5)
    m_c = mm_to_px_nip(-10.0, 284.0)
    c_c = mm_to_px_nip(-126.0, 362.0)
    draw_wheel(draw, g5_c, 22.0 * 1.32, gecko_spin, (34, 197, 94), (203, 213, 225), cw=False)
    draw_wheel(draw, m_c, 48.0 * 1.32, -t * 16.0, (245, 158, 11), (226, 232, 240), cw=False)
    draw_wheel(draw, c_c, 24.0 * 1.32, t * 16.0, (244, 63, 94), (226, 232, 240), cw=True)

    # Draw S3 Stopper Gate in Zoomed Panel!
    ng_piv = mm_to_px_nip(-46.0, 184.0)
    ng_end = mm_to_px_nip(-20.0, 204.0) if s3_gate_open else mm_to_px_nip(-46.0, 142.0)
    draw.line([ng_piv, ng_end], fill=gate_col, width=7)

    # Draw Rear Elevator Backplate (X=-114) & Raised Upper Cyan Deflector Hood (78mm gap to Main Wheel!) in Zoomed Panel
    draw.line([mm_to_px_nip(-114.0, 185.0), mm_to_px_nip(-114.0, 326.0)], fill=(148, 163, 184), width=5)
    nip_hood_pts = [
        mm_to_px_nip(-112.0, 384.0),
        mm_to_px_nip(-86.0, 396.0 + hood_dz * 0.5),
        mm_to_px_nip(-52.0, 400.0 + hood_dz),
    ]
    draw.line(nip_hood_pts, fill=(56, 189, 248), width=8)

    # In Zoomed Panel, show Ball 3 held behind S3 Gate during Stages 1-3, then fired through Gecko 3 -> 67.8mm Nip in Stage 4!
    nx, nz = pos_from_path_s(s3_val, hood_aim_deg)
    nb_px = mm_to_px_nip(nx, nz)
    nbr_px = 35.56 * 1.32
    squish_x = 0.89 if 0.65 <= s3_val <= 0.84 else 1.0
    squish_z = 1.05 if 0.65 <= s3_val <= 0.84 else 1.0
    draw.ellipse(
        [nb_px[0] - nbr_px * squish_x, nb_px[1] - nbr_px * squish_z, nb_px[0] + nbr_px * squish_x, nb_px[1] + nbr_px * squish_z],
        fill=(250, 204, 21),
        outline=(255, 255, 255),
        width=3,
    )
    if s3_gate_open:
        draw_arrow(draw, nb_px, (nb_px[0] + 50, nb_px[1] - 54), (34, 197, 94), width=4)
    draw.line(nip_hood_pts, fill=(56, 189, 248), width=8)

    # Labels inside Left Column of Zoomed Nip Panel (x = 1100..1480 — zero overlap with right sub-frame!)
    draw.text((1100, 158), "1. Servo S3 Indexing Stopper Gate:", fill=(244, 63, 94), font=FONT_SM)
    draw.text((1100, 180), "   • CLOSED while driving & aiming hood", fill=(226, 232, 240), font=FONT_XS)
    draw.text((1100, 198), "     (holds up to 3 balls in magazine!).", fill=(226, 232, 240), font=FONT_XS)

    draw.text((1100, 232), "2. Fire Trigger (S3 Opens + Gecko 3):", fill=(34, 197, 94), font=FONT_SM)
    draw.text((1100, 254), "   • S3 swings OPEN (+55°) & Gecko 3", fill=(226, 232, 240), font=FONT_XS)
    draw.text((1100, 272), "     feeds ball into 96mm Main Wheel.", fill=(226, 232, 240), font=FONT_XS)

    draw.text((1100, 306), "3. Zero Compliant-Wheel Clipping:", fill=(245, 158, 11), font=FONT_SM)
    draw.text((1100, 328), "   • Gecko 3 (+4,200.5) has 14.7mm solid", fill=(226, 232, 240), font=FONT_XS)
    draw.text((1100, 346), "     clearance to 96mm Wheel (-10,284)!", fill=(226, 232, 240), font=FONT_XS)

    draw.text((1100, 380), "4. 67.8mm Diagonal Nip + 78mm Hood:", fill=(56, 189, 248), font=FONT_SM)
    draw.text((1100, 402), "   • Zero crossbars in front of hood", fill=(226, 232, 240), font=FONT_XS)
    draw.text((1100, 420), "   • Unobstructed +48° launch window!", fill=(226, 232, 240), font=FONT_XS)
    draw.text((1100, 456), "5. Dual 82mm Steel Inertia Discs:", fill=(250, 204, 21), font=FONT_SM)
    draw.text((1100, 478), "   KE = ½ I ω² = 18.4 J (<4% RPM drop)", fill=(226, 232, 240), font=FONT_XS)

    # ==========================================================================
    # RIGHT-BOTTOM PANEL: RK4 FULL-FIELD BALLISTIC TRAJECTORY (GRAVITY + DRAG + MAGNUS)
    # ==========================================================================
    draw.rounded_rectangle([1084, 560, 1896, 995], radius=14, fill=(19, 29, 49), outline=(51, 65, 85), width=2)
    draw.text(
        (1104, 572),
        "3. RK4 FIELD BALLISTIC TRAJECTORY (Gravity + Drag + Topspin Magnus Dip)",
        fill=(148, 163, 184),
        font=FONT_H2,
    )

    # Draw Field Floor & High Beehive Goal at X = 2.15..2.38 m, Z = 0.85..1.15 m
    f_p1 = m_to_px_field(0.0, 0.0)
    f_p2 = m_to_px_field(2.55, 0.0)
    draw.line([f_p1, f_p2], fill=(100, 116, 139), width=3)
    goal_top = m_to_px_field(2.12, 1.12)
    goal_bot = m_to_px_field(2.38, 0.82)
    draw.rectangle([goal_top, goal_bot], fill=(245, 158, 11), outline=(250, 204, 21), width=3)
    draw.text((goal_top[0] - 32, goal_bot[1] + 8), "BEEHIVE HIGH GOAL", fill=(250, 204, 21), font=FONT_XS)

    # Draw Trajectory Curves (Pollen 71.1mm in Yellow, Nectar 91.95mm in Crimson)
    for idx in range(len(TRAJ_POLLEN) - 1):
        p_a = m_to_px_field(TRAJ_POLLEN[idx][0], TRAJ_POLLEN[idx][1])
        p_b = m_to_px_field(TRAJ_POLLEN[idx + 1][0], TRAJ_POLLEN[idx + 1][1])
        draw.line([p_a, p_b], fill=(250, 204, 21), width=3)
    for idx in range(len(TRAJ_NECTAR) - 1):
        p_a = m_to_px_field(TRAJ_NECTAR[idx][0], TRAJ_NECTAR[idx][1])
        p_b = m_to_px_field(TRAJ_NECTAR[idx + 1][0], TRAJ_NECTAR[idx + 1][1])
        draw.line([p_a, p_b], fill=(244, 63, 94), width=2)

    # Animate flying ball along TRAJ_POLLEN
    t_idx = int(((t * 1.2) % 1.0) * (len(TRAJ_POLLEN) - 1))
    bx_m, bz_m, bvx, bvz = TRAJ_POLLEN[t_idx]
    fb_px = m_to_px_field(bx_m, bz_m)
    draw.ellipse([fb_px[0] - 11, fb_px[1] - 11, fb_px[0] + 11, fb_px[1] + 11], fill=(250, 204, 21), outline=(255, 255, 255), width=2)
    # Velocity & Gravity vectors on flying ball
    draw_arrow(draw, fb_px, (fb_px[0] + bvx * 6.5, fb_px[1] - bvz * 6.5), (34, 197, 94), width=3, head_len=9)
    draw_arrow(draw, fb_px, (fb_px[0], fb_px[1] + 32), (56, 189, 248), width=3, head_len=9)

    draw.text(
        (1108, 612),
        "m dv/dt = m g - ½ ρ C_d A |v| v + S (ω × v)   |   KE_flywheel = ½ I ω² = 18.4 J (2x 82mm Steel Discs)",
        fill=(226, 232, 240),
        font=FONT_XS,
    )

    # Bottom Telemetry Bar
    draw.rounded_rectangle([24, 1008, 1920 - 24, 1064], radius=10, fill=(30, 41, 59), outline=(71, 85, 105), width=1)
    draw.text(
        (44, 1025),
        f"LIVE TELEMETRY  |  {status_str}  |  S3 Gate: {'OPEN' if s3_gate_open else 'CLOSED (HOLDING)'}",
        fill=(52, 211, 153),
        font=FONT_BOLD,
    )

    img.save(os.path.join(OUT_DIR, f"frame_{f_idx:04d}.png"))

# Encode to H.264 MP4 via ffmpeg
cmd = [
    'C:\\Users\\danu0\\AppData\\Local\\uv\\cache\\archive-v0\\K9L9q6S_NY9kZWlx\\Lib\\site-packages\\imageio_ffmpeg\\binaries\\ffmpeg-win-x86_64-v9.0.2.exe',
    "-y",
    "-framerate",
    str(FPS),
    "-i",
    os.path.join(OUT_DIR, "frame_%04d.png"),
    "-c:v",
    "libx264",
    "-pix_fmt",
    "yuv420p",
    "-crf",
    "18",
    MP4_PATH,
]
subprocess.run(cmd, check=True)
print("Saved MP4:", MP4_PATH, "Size MB:", round(os.path.getsize(MP4_PATH) / (1024 * 1024), 2))
