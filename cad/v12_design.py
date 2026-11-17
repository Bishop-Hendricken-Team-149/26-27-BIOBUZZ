"""
HIVE-QUEEN v12 design constants & pure-math geometry (no Fusion imports).

Shared by cad/run_full_robot_upgrade.py (builder) and cad/verify_v12_gates.py (gate audit) so
every coordinate used by the CAD build is the same one the verification checks.

Frame: +X front, +Y left, +Z up, Z = 0 field tiles. Units: mm.
"""
import math

# ----------------------------------------------------------------------------
# Game pieces (BIOBUZZ TU07 section 9.8)
# ----------------------------------------------------------------------------
NECTAR_D = 91.95
POLLEN_D = 71.12
NECTAR_R = NECTAR_D / 2.0
POLLEN_R = POLLEN_D / 2.0

# ----------------------------------------------------------------------------
# HTD-5M timing belt helpers (B7: exact closed-loop COTS lengths)
# ----------------------------------------------------------------------------
HTD_PITCH = 5.0


def htd_pd(teeth: int, pitch: float = HTD_PITCH) -> float:
    return teeth * pitch / math.pi


def belt_length(c: float, t1: int, t2: int, pitch: float = HTD_PITCH) -> float:
    d1, d2 = htd_pd(t1, pitch), htd_pd(t2, pitch)
    return 2.0 * c + math.pi * (d1 + d2) / 2.0 + (d1 - d2) ** 2 / (4.0 * c)


def dist2(a, b) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def polar(c, r, deg):
    return (c[0] + r * math.cos(math.radians(deg)), c[1] + r * math.sin(math.radians(deg)))


def ang(c, p) -> float:
    return math.degrees(math.atan2(p[1] - c[1], p[0] - c[0]))


# ----------------------------------------------------------------------------
# Ball corridor / J-chute (B1)
# ----------------------------------------------------------------------------
CORRIDOR_HALF = 51.0          # 102.0 mm clear width
SHROUD_IN, SHROUD_OUT = 51.0, 55.0
FLOOR_T = 4.5                 # chute floor thickness
CURB_H = 12.0                 # curb height above floor surface
RAMP_DEG = 23.0               # must clear the X in [-48, 0] mid cross-channel (top Z = 72)
LIP = (184.0, 7.0)            # wedge lip (B3). v12.2: ramp raised 3.5 mm so the lip clears foam-tile seams
LIP_CUT_Z = 5.0               # wedge underside trimmed flat 5 mm above the tiles (was 1.5: would snag tile seams)
TH = math.radians(RAMP_DEG)
N = (math.sin(TH), math.cos(TH))   # ramp upward normal in (x, z)
TAN = math.tan(TH)
WALL_X = -110.0               # inner face of vertical rear elevator wall
WALL_T = 4.5
WALL_TOP_Z = 276.0            # stays below the counter-wheel swing envelope
BEND_R = 100.0                # J-bend floor radius


def ramp_z(x: float) -> float:
    return LIP[1] + TAN * (LIP[0] - x)


BEND_C = (WALL_X + BEND_R, LIP[1] + (BEND_R - ((WALL_X + BEND_R) - LIP[0]) * N[0]) / N[1])
RAMP_TANGENT = (BEND_C[0] - BEND_R * N[0], BEND_C[1] - BEND_R * N[1])
BEND_A0 = math.degrees(math.atan2(-N[1], -N[0])) % 360.0   # ~247 deg (ramp side)
BEND_A1 = 180.0                                             # wall side


def h_above_ramp(p) -> float:
    return (p[0] - LIP[0]) * N[0] + (p[1] - LIP[1]) * N[1]


def ramp_ball_center(foot_x: float, r: float):
    return (foot_x + r * N[0], ramp_z(foot_x) + r * N[1])


def bend_ball_center(a_deg: float, r: float):
    return polar(BEND_C, BEND_R - r, a_deg)


# Ball path (centre line) sampler for both sizes: ramp -> bend -> wall -> nip.
def path_points(r: float, step: float = 4.0):
    pts = []
    x = LIP[0] + 12.0
    while x > RAMP_TANGENT[0]:
        pts.append(ramp_ball_center(x, r))
        x -= step
    a = BEND_A0
    while a > BEND_A1:
        pts.append(bend_ball_center(a, r))
        a -= step * 57.3 / (BEND_R - r)
    z = BEND_C[1]
    while z < 330.0:
        pts.append((WALL_X + r, z))
        z += step
    return pts


# ----------------------------------------------------------------------------
# Drivetrain (B6)
# ----------------------------------------------------------------------------
AXLE_X = 144.0                # rear axles (hole 15 of the 17-hole rail). Kept for back-compat = AXLE_X_REAR
AXLE_X_REAR = 144.0
# v12.4: FRONT axles move out one 24 mm hole (hole 16). With both pairs at 144 the two drive motors on
# each side met back-to-back with 0 mm between their encoder caps, leaving no room for the radial power
# and encoder cable exits. Front at 168 gives a 24 mm gap. The rear can't move: an axle at -168 lands on
# the rear crossbar (X -216..-168). Wheelbase 312 mm, centred 12 mm forward.
AXLE_X_FRONT = 168.0
DRIVE_MOTOR_FACE_DX = 30.0   # v12.6: gearbox face sits 30.0 mm from the axle (2315 miter: shaft tip 6 mm short of the axle)
DRIVE_MOTOR_BACK_GAP_MM = (AXLE_X_FRONT - 144.0) + (AXLE_X_REAR - 144.0) - 2 * (DRIVE_MOTOR_FACE_DX - 27.6)   # 19.2 mm
AXLE_Z = 48.0
RAIL_Y_IN, RAIL_Y_OUT = 132.0, 180.0
RAIL_WALL_T = 2.5
BEARING_OD_R = 6.95      # 1611-0514-4008: 14 mm OD seated in the 14 mm channel hole
BEARING_ID_R = 4.0
BEARING_W = 4.0
BEARING_FL_R = 8.0
BEARING_FL_T = 1.0
DRIVE_MOTOR_DX = 0.44 + 2.4    # v12.6: +2.4 mm toward centre so the gearbox face is axle - 30.0 (real 2315 miter pinion)
MITER_D0, MITER_D1 = 8.0, 16.0   # (v12.5 hand model, superseded by the goBILDA 2315 STEP in v12.6)
# v12.6: goBILDA 2315-4008-0030 steel miter (MOD 1.25, 30T, PD 37.5). Cone apex measured from the vendor STEP
# (tip cone 47.7 deg and root cone 41.8 deg both meet at local X -35.43): mounting distance 20.56 mm from the
# back face, which leaves 0.94 mm to the 1120 channel wall for goBILDA's thrust washer.
MITER_STEP = "2315-4008-0030 assembly.STEP"
MITER_APEX_LOCAL = (-35.43, 22.37819, 5.0264)   # bore axis from the STEP's r4.0/r7.5 cylinders
MITER_APEX_Y = 156.0
# REX fits (v12.6): the pinion bore must sit on the motor shaft's hex flats (measured at -3.48 deg in the pinion
# frame at all 4 corners); the axle turns freely, so the axle gear takes whatever phase meshes: 9.3-9.7 deg gives
# zero tooth overlap at every corner (offline scan, cad/tmp/miter_phase2.py). The axle is REX (flats 3.48 mm,
# corners r 3.95) aligned to that bore.
MITER_PINION_PHASE = -3.48
MITER_AXLE_PHASE = 9.45
REX_FLAT, REX_CORNER = 3.48, 3.95
MITER_UNDERCUT = 0.5

# ----------------------------------------------------------------------------
# Floating intake (B3)
# ----------------------------------------------------------------------------
INTAKE_PIVOT = (156.0, 144.0)
INTAKE_ARM_R = 80.0                  # = 260 mm HTD-5M belt with 20T/20T (exact)
INTAKE_SHAFT_Z_LOW = 76.5
INTAKE_SHAFT_Z_HIGH = 104.0
INTAKE_SHAFT_LOW = (INTAKE_PIVOT[0] + math.sqrt(INTAKE_ARM_R ** 2 - (INTAKE_PIVOT[1] - INTAKE_SHAFT_Z_LOW) ** 2), INTAKE_SHAFT_Z_LOW)
INTAKE_SHAFT_HIGH = (INTAKE_PIVOT[0] + math.sqrt(INTAKE_ARM_R ** 2 - (INTAKE_PIVOT[1] - INTAKE_SHAFT_Z_HIGH) ** 2), INTAKE_SHAFT_Z_HIGH)
INTAKE_WHEEL_R = 23.5
INTAKE_WHEEL_YS = (-72.0, -38.0, 0.0, 38.0, 72.0)   # v12.1: centre star roller -> 5th compliant wheel (star blades swept into gecko 1)
INTAKE_ARM_Y = 104.0                 # arms at |Y| in [100, 108]
INTAKE_ARM_T = 8.0
INTAKE_ARM_W = 18.0
INTAKE_TAIL = 18.0
INTAKE_STAGE2_Y = 92.0               # left, inboard of the left arm
INTAKE_STAGE1_Y = 60.0               # left, on M7 shaft
M7_SHAFT = None                      # filled below (belt-exact)

# ----------------------------------------------------------------------------
# Magazine (B2): coaxial belt-on-pivot swing arms
# ----------------------------------------------------------------------------
GECKO_R = 36.0
GECKO_HALF_W = 16.0
H_POLLEN = POLLEN_D + GECKO_R - 3.0      # gecko centre height above floor at rest (3 mm bite)
H_NECTAR = NECTAR_D + GECKO_R - 5.0      # lifted (5 mm bite on Nectar)
M8_SHAFT = (104.0, 121.0)                # raised from Z 98 -> 121 so the encoder clears the FL/FR wheels
# v12.6: magazine belts are HTD-3M 6 mm (33T/33T printed pulleys, PD 31.51 mm). 3M belts come in 3 mm steps,
# so the centres move: Gates 213-3M-06 (71T) for b1/b3 (57.5 -> 57.0) and 258-3M-06 (86T, 32T pulleys) for b2 (80 -> 81) -- exact.
MAG_PITCH, MAG_T, MAG_T2 = 3.0, 33, 32     # b2 uses 32T/32T so its 81.0 mm centre takes a standard Gates 258-3M-06
G1_R = 57.0                              # 213 mm HTD-3M belt (71T), 33T/33T
_hp = h_above_ramp(M8_SHAFT)
G1_A_REST = 90.0 - RAMP_DEG - math.degrees(math.acos((H_POLLEN - _hp) / G1_R))
G1_A_LIFT = 90.0 - RAMP_DEG - math.degrees(math.acos((H_NECTAR - _hp) / G1_R))
G1_REST = polar(M8_SHAFT, G1_R, G1_A_REST)
G1_LIFT = polar(M8_SHAFT, G1_R, G1_A_LIFT)

G2_R = 57.0                              # 213 mm HTD-3M belt (71T)
B2_C = 81.0                              # M8 -> J2: exact 258 mm (86T) Gates 258-3M-06 on 32T/32T (79.5 for 33T is below the reachable minimum ~79.8)


def _g2_layout(bend_a):
    rest = polar(BEND_C, BEND_R - H_POLLEN, bend_a)
    lift = polar(BEND_C, BEND_R - H_NECTAR, bend_a)
    mid = ((rest[0] + lift[0]) / 2.0, (rest[1] + lift[1]) / 2.0)
    ch = dist2(rest, lift)
    u = (math.cos(math.radians(bend_a)), math.sin(math.radians(bend_a)))
    t = (-u[1], u[0])
    d = math.sqrt(G2_R ** 2 - (ch / 2.0) ** 2)
    return rest, lift, (mid[0] + d * t[0], mid[1] + d * t[1])


# v12.2: solve the gecko-2 bend angle so M8 -> J2 is EXACTLY B2_C (was a fixed 210.5 deg that only
# happened to give 80.002 mm before the ramp was raised 3.5 mm for tile clearance)
_lo, _hi = 195.0, 225.0
for _ in range(80):
    _m = 0.5 * (_lo + _hi)
    if (dist2(M8_SHAFT, _g2_layout(_m)[2]) - B2_C) * (dist2(M8_SHAFT, _g2_layout(_lo)[2]) - B2_C) <= 0:
        _hi = _m
    else:
        _lo = _m
G2_BEND_A = 0.5 * (_lo + _hi)
G2_REST, G2_LIFT, J2 = _g2_layout(G2_BEND_A)
G2_A_REST = ang(J2, G2_REST)
G2_A_LIFT = ang(J2, G2_LIFT)

# Right-side Y planes (mirror for left where used)
PLANE_A = (56.5, 63.5)    # belts b1/b3 (centre 60)
PLANE_B = (64.5, 70.5)    # swing arms
PLANE_C = (71.5, 78.5)    # belt b2 (centre 75)
PLANE_D = (79.6, 85.4)    # torsion spring
MAG_DS_Y = (86.0, 91.0)   # outboard double-shear plate (J2 outer bearing, M8 gearbox collar)

# Indexer gate (S3) below the bend floor, 3 fingers through floor slits
GATE_A = 237.0
GATE_PIVOT = polar(BEND_C, 125.0, GATE_A)
GATE_FINGER_TIP_R = 66.0
GATE_FINGER_YS = (-25.0, 0.0, 25.0)
GATE_RETRACT_DEG = 60.0     # +rotation (CCW in XZ, away from the held ball) that drops every finger tip below the floor (r > 104.5)

# ----------------------------------------------------------------------------
# Shooter (B4/B5)
# ----------------------------------------------------------------------------
FRONT_WHEEL = (32.0, 324.0)       # gobilda Rhino 96 mm + 2x steel 82 mm inertia discs
FRONT_WHEEL_D = 96.0
REAR_WHEEL_REST = (-123.5, 324.0) # 72 mm counter-wheel
REAR_WHEEL_D = 72.0
MOTOR_FREE_RPM = 1150.0           # 5203-2402-0005 (M5, M6)
FRONT_T_MOTOR, FRONT_T_WHEEL = 20, 20   # 1:1
REAR_T_MOTOR, REAR_T_WHEEL = 32, 16     # 2:1 overdrive -> topspin (v_rear > v_front)
_c5 = None
# M5 placed so the 20T/20T belt is exactly 260 mm (52T)
_dirv = (FRONT_WHEEL[0] - (-24.0), FRONT_WHEEL[1] - 266.0)
_dl = math.hypot(*_dirv)
M5_C = (260.0 - math.pi * htd_pd(20)) / 2.0
M5_SHAFT = (FRONT_WHEEL[0] - M5_C * _dirv[0] / _dl, FRONT_WHEEL[1] - M5_C * _dirv[1] / _dl)


def _solve_c(L, t1, t2, c0=60.0):
    c = c0
    for _ in range(60):
        f = belt_length(c, t1, t2) - L
        df = (belt_length(c + 1e-4, t1, t2) - belt_length(c - 1e-4, t1, t2)) / 2e-4
        c -= f / df
    return c


M6_C = _solve_c(245.0, REAR_T_MOTOR, REAR_T_WHEEL)     # 49T exact
_d6 = (15.5, -58.0)
_l6 = math.hypot(*_d6)
M6_SHAFT = (REAR_WHEEL_REST[0] + M6_C * _d6[0] / _l6, REAR_WHEEL_REST[1] + M6_C * _d6[1] / _l6)
COUNTER_ARM_R = M6_C
COUNTER_A_REST = ang(M6_SHAFT, REAR_WHEEL_REST)
COUNTER_A_MAX = 128.5
COUNTER_SLOT_R = 4.6              # 9.2 mm wide arc slot
HOOD_HINGE = (-116.0, 382.0)      # moved 20 mm rearward, clear of the Nectar exit column
HOOD_DEG_FROM_VERTICAL = 20.0
HOOD_LEN = 60.0
SIDE_PLATE_Y = (55.0, 61.0)
SWING_ARM_Y = (62.0, 68.0)
SPRING_Y = (69.0, 75.0)
M6_DY_OUT = 5.0                   # M6 moved 5 mm outboard so its shaft tip clears the swing-arm hub
DS_LEFT_Y = (85.0, 90.0)
DS_RIGHT_Y = (90.0, 95.0)


def surface_speed(rpm, d_mm):
    return math.pi * d_mm / 1000.0 * rpm / 60.0


V_FRONT = surface_speed(MOTOR_FREE_RPM * FRONT_T_MOTOR / FRONT_T_WHEEL, FRONT_WHEEL_D)
V_REAR = surface_speed(MOTOR_FREE_RPM * REAR_T_MOTOR / REAR_T_WHEEL, REAR_WHEEL_D)
V_EXIT = 0.5 * (V_FRONT + V_REAR)

# ----------------------------------------------------------------------------
# M7 intake motor: belt-exact 20T/20T stage-1 to the upper pivot (215 mm)
# ----------------------------------------------------------------------------
_v7 = (INTAKE_PIVOT[0] - M8_SHAFT[0], INTAKE_PIVOT[1] - M8_SHAFT[1])
_l7 = math.hypot(*_v7)
M7_C = (215.0 - math.pi * htd_pd(20)) / 2.0
M7_SHAFT = (INTAKE_PIVOT[0] - M7_C * _v7[0] / _l7, INTAKE_PIVOT[1] - M7_C * _v7[1] / _l7)
M78_DY_OUT = 17.0                 # M7/M8 moved 17 mm outboard: shaft tips leave the 102 mm corridor

# ----------------------------------------------------------------------------
# Belt ledger (B7) — every closed loop with its centre distance
# ----------------------------------------------------------------------------
BELTS = {
    "intake_stage1_m7_to_pivot": (dist2(M7_SHAFT, INTAKE_PIVOT), 20, 20, 5.0),
    "intake_stage2_pivot_to_floating_shaft": (INTAKE_ARM_R, 20, 20, 5.0),
    "magazine_b1_m8_to_gecko1": (G1_R, MAG_T, MAG_T, MAG_PITCH),
    "magazine_b2_m8_to_j2": (dist2(M8_SHAFT, J2), MAG_T2, MAG_T2, MAG_PITCH),
    "magazine_b3_j2_to_gecko2": (G2_R, MAG_T, MAG_T, MAG_PITCH),
    "shooter_left_m5_to_front_wheel": (dist2(M5_SHAFT, FRONT_WHEEL), FRONT_T_MOTOR, FRONT_T_WHEEL, 5.0),
    "shooter_right_m6_to_counter_wheel": (dist2(M6_SHAFT, REAR_WHEEL_REST), REAR_T_MOTOR, REAR_T_WHEEL, 5.0),
}
# real belt / pulley widths (v12.6): HTD-5M 9 mm belt on 12.0 mm pulleys, HTD-3M 6 mm belt on 8.5 mm pulleys
BELT_SPEC = {5.0: dict(belt_w=9.0, thick=1.6, pulley_w=12.0, flange=1.0, flange_r=2.2, name="htd5m"),
             3.0: dict(belt_w=6.0, thick=1.0, pulley_w=8.5, flange=0.9, flange_r=1.5, name="htd3m")}


def belt_ledger():
    out = {}
    for k, (c, t1, t2, p) in BELTS.items():
        L = belt_length(c, t1, t2, p)
        Ln = round(L / p) * p
        out[k] = {"c_mm": round(c, 3), "pulleys": f"{t1}T/{t2}T", "pitch": p, "L_mm": round(L, 3), "cots_L_mm": Ln, "teeth": int(round(Ln / p)), "err_mm": round(L - Ln, 3)}
    return out


# ----------------------------------------------------------------------------
# Envelope targets
# ----------------------------------------------------------------------------
R102_MAX = (448.0, 448.0, 450.0)
R105_MAX = (609.6, 457.0, 736.6)

# ----------------------------------------------------------------------------
# Front-lip G407/G408 blocker (S1) and endgame capper (S7)
# ----------------------------------------------------------------------------
BLOCKER_PIVOT = (156.0, 168.0)          # goBILDA 14 mm hole, one pitch above the intake pivot
BLOCKER_BAR_DEPLOYED = (218.0, 52.0)
BLOCKER_L = dist2(BLOCKER_PIVOT, BLOCKER_BAR_DEPLOYED)
BLOCKER_A_DEPLOYED = ang(BLOCKER_PIVOT, BLOCKER_BAR_DEPLOYED)
BLOCKER_A_STOWED = math.degrees(math.acos((222.0 - BLOCKER_PIVOT[0]) / BLOCKER_L))
BLOCKER_ARM_Y = (112.0, 118.0)
# (v12.2) FLOWER capper removed -- no CAPPER_* geometry


# ----------------------------------------------------------------------------
# Staged balls (audit-visible) and magazine sensors
# ----------------------------------------------------------------------------
def _seg_dist(p, a, b):
    ax, az = b[0] - a[0], b[1] - a[1]
    t = max(0.0, min(1.0, ((p[0] - a[0]) * ax + (p[1] - a[1]) * az) / (ax * ax + az * az)))
    return math.hypot(p[0] - a[0] - t * ax, p[1] - a[1] - t * az)


GATE_TIP = polar(BEND_C, GATE_FINGER_TIP_R, GATE_A)


def _staged():
    pts = path_points(POLLEN_R, step=0.5)
    held = None
    for p in pts:
        if _seg_dist(p, GATE_PIVOT, GATE_TIP) < POLLEN_R + 3.0 + 0.3:
            break
        held = p
    balls = [held]
    keep_out = [(G1_REST, POLLEN_R + GECKO_R + 0.3), (G2_REST, POLLEN_R + GECKO_R + 0.3),
                (INTAKE_SHAFT_LOW, POLLEN_R + INTAKE_WHEEL_R + 0.5)]
    for p in reversed(pts[:pts.index(held)]):
        if len(balls) >= 3:
            break
        if dist2(p, balls[-1]) < POLLEN_D + 0.4:
            continue
        if any(dist2(p, c) < r for c, r in keep_out):
            continue
        if p[0] > 170.0:
            break
        balls.append(p)
    return balls


_sb = _staged()
STAGED_BALLS = [(f"staged_pollen_{k + 1}_queued", c, POLLEN_R) for k, c in enumerate(_sb)]
STAGED_BALLS.append(("staged_nectar_ball_92mm_approaching_mouth", (268.0, NECTAR_R), NECTAR_R))
# v12.6: count by slot occupancy. With the queue held against the closed indexer gate, POLLEN slots sit one
# ball pitch apart along the ramp: slot 1 = held ball (throat Color V3), slot 2 = ballCount1, slot 3 =
# ballCount2, slot 4 = at the mouth (seen by the lip Color V3). The old spots straddled slots (bc2 saw the
# held ball, bc1 sat between slots 2 and 3), so the code read 'full' with one ball aboard.
_sp = math.hypot(_sb[1][0] - _sb[0][0], _sb[1][1] - _sb[0][1])
_su = ((_sb[1][0] - _sb[0][0]) / _sp, (_sb[1][1] - _sb[0][1]) / _sp)
_bc1 = (_sb[1][0], _sb[1][1])
_bc2 = (_sb[1][0] + _sp * _su[0], _sb[1][1] + _sp * _su[1])
QUEUE_SLOTS = [(_sb[0][0] + k * _sp * _su[0], _sb[0][1] + k * _sp * _su[1]) for k in range(4)]
SENSOR_SPOTS = [(_bc1[0], _bc1[1], "magazine_ball_count_sensor_1_slot2"),
                (_bc2[0], _bc2[1], "magazine_ball_count_sensor_2_slot3"),
                (_sb[0][0], _sb[0][1], "throat_ball_size_color_sensor_rev_31_1557")]
SENSOR_WINDOWS = [(x, z) for x, z, _ in SENSOR_SPOTS]

# ----------------------------------------------------------------------------
# Last-ball hand-off (v12.1): no room for a 3rd gecko on the wall (gecko 2 sits at the bend
# centre and the front shooter wheel is above it), so the last ball must coast from gecko 2's
# last contact to the counter-wheel grab. M8 -> 435 RPM gives 2.6x margin (sliding, no spin credit).
# ----------------------------------------------------------------------------
M8_RPM = 1150.0                                           # 5203-2402-0005 (was 435: a lone POLLEN stalled 19 mm short)
# Intake (v12.2 readiness review): both stages are 20T:20T, so roller RPM = motor RPM.
# 312 RPM gave 0.77 m/s at the 47 mm wheels, half the 1.57 m/s drive speed (it would bulldoze balls).
M7_RPM = 1150.0                                           # 5203-2402-0005
DRIVE_RPM, DRIVE_WHEEL_D = 312.0, 96.0
DRIVE_TOP_MS = DRIVE_RPM / 60.0 * math.pi * DRIVE_WHEEL_D / 1000.0
INTAKE_SURFACE_MS = M7_RPM / 60.0 * math.pi * 2 * INTAKE_WHEEL_R / 1000.0
INTAKE_TO_DRIVE_RATIO = INTAKE_SURFACE_MS / DRIVE_TOP_MS
GECKO_SURFACE_MS = M8_RPM / 60.0 * math.pi * 2 * GECKO_R / 1000.0
# A ball pinched between a moving gecko and the FIXED chute wall rolls at HALF the gecko surface speed.
# On the vertical wall there is no normal force, so its spin cannot be converted to height (plain projectile).
BALL_FEED_MS = GECKO_SURFACE_MS / 2.0
LAST_BALL_COAST_MM = BALL_FEED_MS ** 2 / (2 * 9.81) * 1000.0
_wall_ball_x = WALL_X + POLLEN_R
G2_TOP_CONTACT_Z = G2_REST[1] + math.sqrt((POLLEN_R + GECKO_R) ** 2 - (G2_REST[0] - _wall_ball_x) ** 2)
COUNTER_GRAB_Z = REAR_WHEEL_REST[1] - math.sqrt((POLLEN_R + REAR_WHEEL_D / 2) ** 2 - (REAR_WHEEL_REST[0] - _wall_ball_x) ** 2)
UNPOWERED_GAP_MM = COUNTER_GRAB_Z - G2_TOP_CONTACT_Z

if __name__ == "__main__":
    import json
    print(json.dumps({
        "BEND_C": BEND_C, "RAMP_TANGENT": RAMP_TANGENT, "BEND_A0": BEND_A0,
        "M8": M8_SHAFT, "G1_REST": G1_REST, "G1_LIFT": G1_LIFT, "G1_A": (G1_A_REST, G1_A_LIFT),
        "J2": J2, "G2_REST": G2_REST, "G2_LIFT": G2_LIFT, "G2_A": (G2_A_REST, G2_A_LIFT),
        "M7": M7_SHAFT, "M5": M5_SHAFT, "M6": M6_SHAFT, "COUNTER_A_REST": COUNTER_A_REST,
        "INTAKE_LOW": INTAKE_SHAFT_LOW, "INTAKE_HIGH": INTAKE_SHAFT_HIGH, "GATE_PIVOT": GATE_PIVOT,
        "V": (V_FRONT, V_REAR, V_EXIT), "BLOCKER": (BLOCKER_L, BLOCKER_A_DEPLOYED, BLOCKER_A_STOWED),
        "belts": belt_ledger(),
    }, indent=1, default=lambda o: round(o, 3)))


# ---------------------------------------------------------------- v12.5 real device models + mounts
# Every device that used to be a plain box is now the vendor STEP (cad/vendor/), placed by its real mounting
# holes on a printed/modelled mount with real screw holes. Frames: (file, occurrence name, local X, Y, Z axes
# in world, origin in mm). REV sensor cases: local Y = depth (sensing face at Y 0), long axis = local X,
# ears = local X +/-16 through the face, lens centre at local X = AP_DX.
REV_COLOR_AP_DX = 0.69      # REV-31-1557 lens insert spans local X -10.0..11.38
REV_DIST_AP_DX = 0.55       # REV-31-1505 lens insert spans local X -10.0..11.1
SENSOR_FACE_Y = 61.05       # pad face Y 61.0 (shroud outer face 55 + 6 mm pad), 0.05 seat gap
LIP_SENSOR_AP = (167.0, 43.5)   # lip Color V3 lens (X, Z), looking across the mouth (-Y) from the left curb pad
_bc1s, _bc2s, _thr = SENSOR_SPOTS
INJECTOR_T = (-10.0, 96.25, 207.95)   # base plate (local Z -8) against the deck underside (Z 216)
INJECTOR_HOLES = [(INJECTOR_T[0] + sx * 16.0, INJECTOR_T[1] + sy * 16.0) for sx in (-1, 1) for sy in (-1, 1)]
HUSKY_T = (75.55, -59.22, 365.9)      # lens at (Y -86, Z 354) facing +X; screen (3.5 mm behind the PCB) clears the turret plate (X 72)
HUSKY_EAR_YS = (HUSKY_T[1] - 18.3, HUSKY_T[1] - 34.2)
HUSKY_EAR_Z = HUSKY_T[2] - 40.5
# v12.6: the HuskyLens block becomes an L-bracket: a 5 mm flange runs back along the right turret double-shear
# plate's outboard face (Y -95) and 2x M3 x 12 go through flange + 5 mm plate into nylock nuts on the inboard side.
HUSKY_FLANGE_X = (40.0, 56.05)
HUSKY_PLATE_SCREWS = ((45.0, 319.5), (51.0, 330.5))   # (X, Z)
# v12.6: goBILDA 12V NiMH Nested Battery 3100-0012-0020 (goBILDA's replacement for the discontinued Matrix
# 14-0014; Table 12-4 legal; XT30). 224.0 x 42.38 x 43.59 mm, lying in the rear bay like the Matrix did:
# It rides in goBILDA's own 1209-0001-0001 Battery Mount (profile-matched, hook-and-loop strap = tool-free swaps),
# bolted on the rear crossbar's top web: mount centred at (X -192, Y 12) so its counterbored M4 holes at
# (+/-8, +/-32) and (+/-16, +/-40) land on the crossbar's own holes (all 8 checked). Battery on the mount floor (Z 80).
BATT_MOUNT_T = (-192.0, 12.0, 72.005)
# The pack is 122.4 mm long (cells + wrap, STEP local X -99.7..22.7); the leads run ~100 mm on from its end.
# Pack centred in the mount (offline fit: zero overlap with the mount's profile walls, +/-1.25 mm play), leads to +Y.
BATT_T = (-190.5, 50.5, 123.615)
BATT_BOX = (-213.2, -170.8, -49.2, 73.2, 80.015, 123.615)
# goBILDA 1613-0516-0008 thrust bearing behind each axle miter gear (as in the Strafer kit): measured in the
# vendor STEPs it nests in the gear's dished back with its outer washer on the rail wall, zero overlap.
THRUST_STEP = "1613-0516-0008.STEP"
# The gear's dish: 3.5 mm seat + an r 8.0 pocket 3.95 mm deep; the 4.96 mm bearing (small washer in the pocket,
# large washer on the wall) therefore stands 1.01 mm proud of the gear back vs 0.94 mm at the theoretical apex.
# The bearing sets the gear (goBILDA's stated purpose), so the axle gear sits 0.08 mm deeper into mesh; the
# zero-overlap phase window at that seat is still 9.35-9.6 deg (cad/tmp/miter_phase3.py).
THRUST_SEAT_SHIFT = 0.09
# v12.6: S6 moves 6.3 mm outboard so the 17 mm servo-to-REX coupler fits between its case and the side plate.
_C45 = math.sqrt(0.5)                    # exact 45 deg axis component (Fusion rejects non-orthonormal frames)
S6_SHIFT = 6.3
S6_FACE_Y = -72.19 - S6_SHIFT          # servo case face around the spline (spline tip 4.1 mm beyond)
S6_COUPLER_REX_END = S6_FACE_Y + 0.1 - 1.32 + 10.32   # bottom of the coupler's REX socket
S3_SHAFT_END = 59.3 + 1.32 - 10.32       # indexer shaft ends at the coupler's REX socket bottom
S1_SHAFT_END = 184.94 + 1.32 - 10.32
DS_STANDOFFS = {"left": [(-46.0, 330.0), (50.0, 262.0)], "right": [(-60.0, 240.0), (50.0, 262.0)]}
SADDLE_SCREW_DX = 32.5                  # M5/M6 saddle: 8 mm base ears, 4x M4 x 12 into tapped deck holes
STOP_PLATE_SCREWS = [(-16.0, -8.0), (16.0, -8.0)]   # intake stop plates -> upright web holes (offsets from the pivot)

VENDOR_DEVICES = [
    ("REV-31-1505.STEP", "rev_2m_distance_sensor_ball_count_1", (1, 0, 0), (0, 1, 0), (0, 0, 1), (_bc1s[0] - REV_DIST_AP_DX, SENSOR_FACE_Y, _bc1s[1])),
    ("REV-31-1505.STEP", "rev_2m_distance_sensor_ball_count_2", (1, 0, 0), (0, 1, 0), (0, 0, 1), (_bc2s[0] - REV_DIST_AP_DX, SENSOR_FACE_Y, _bc2s[1])),
    ("REV-31-1557.STEP", "rev_color_sensor_v3_throat", (0, 0, 1), (0, 1, 0), (-1, 0, 0), (_thr[0], SENSOR_FACE_Y, _thr[1] - REV_COLOR_AP_DX)),
    ("REV-31-1557.STEP", "rev_color_sensor_v3_lip", (1, 0, 0), (0, 1, 0), (0, 0, 1), (LIP_SENSOR_AP[0] - REV_COLOR_AP_DX, SENSOR_FACE_Y, LIP_SENSOR_AP[1])),
    ("3125-0001-0001.step", "gobilda_servo_power_injector_3125", (1, 0, 0), (0, -1, 0), (0, 0, -1), INJECTOR_T),
    ("REV-31-1387.STEP", "rev_main_power_switch_31_1387", (1, 0, 0), (0, 0, 1), (0, -1, 0), (-114.0, 146.0, 251.0)),
    ("SEN0305/SEN0305.stp", "dfrobot_huskylens_sen0305", (0, -1, 0), (0, 0, -1), (1, 0, 0), HUSKY_T),
    ("1209-0001-0001-REV0.step", "gobilda_nested_battery_mount_1209", (1, 0, 0), (0, 1, 0), (0, 0, 1), BATT_MOUNT_T),
    ("4001-0025-4008 assembly.STEP", "gobilda_servo_rex_coupler_s3", (1, 0, 0), (0, -1, 0), (0, 0, -1), (GATE_PIVOT[0] + 27.51, 59.3 + 1.32, GATE_PIVOT[1] + 8.46)),
    ("4001-0025-4008 assembly.STEP", "gobilda_servo_rex_coupler_s1", (1, 0, 0), (0, -1, 0), (0, 0, -1), (BLOCKER_PIVOT[0] + 27.51, 184.94 + 1.32, BLOCKER_PIVOT[1] + 8.46)),
    ("4001-0025-4008 assembly.STEP", "gobilda_servo_rex_coupler_s6", (1, 0, 0), (0, 1, 0), (0, 0, 1), (HOOD_HINGE[0] + 27.51, S6_FACE_Y + 0.1 - 1.32, HOOD_HINGE[1] - 8.46)),
    # v12.6: real flywheels. goBILDA 3628-0032-0082 plates (82 x 6 mm, 152 g, 1651 g.cm2) bolt in pairs through the 32 mm
    # pattern of a goBILDA 1313-1632-4008 Hyper Hub (REX bore, flats at 0/60/120 deg): plate A | hub arms | plate B on the
    # hub shoulder, 4x M4 x 25 + hex nuts in the plates' counterbores. Plates are clocked 45 deg so their holes meet the arms.
    ("1313-1632-4008.step", "gobilda_1313_flywheel_hub_left", (1, 0, 0), (0, 0, 1), (0, -1, 0), (FRONT_WHEEL[0], 25.0, FRONT_WHEEL[1])),
    ("1313-1632-4008.step", "gobilda_1313_flywheel_hub_right", (1, 0, 0), (0, 0, -1), (0, 1, 0), (FRONT_WHEEL[0], -25.0, FRONT_WHEEL[1])),
    ("3628-0032-0082.step", "gobilda_flywheel_plate_left_a", (_C45, 0, _C45), (0, 1, 0), (-_C45, 0, _C45), (FRONT_WHEEL[0], 25.0, FRONT_WHEEL[1])),
    ("3628-0032-0082.step", "gobilda_flywheel_plate_left_b", (_C45, 0, _C45), (0, 1, 0), (-_C45, 0, _C45), (FRONT_WHEEL[0], 37.0, FRONT_WHEEL[1])),
    ("3628-0032-0082.step", "gobilda_flywheel_plate_right_a", (_C45, 0, _C45), (0, 1, 0), (-_C45, 0, _C45), (FRONT_WHEEL[0], -19.0, FRONT_WHEEL[1])),
    ("3628-0032-0082.step", "gobilda_flywheel_plate_right_b", (_C45, 0, _C45), (0, 1, 0), (-_C45, 0, _C45), (FRONT_WHEEL[0], -31.0, FRONT_WHEEL[1])),
    ("3100-0012-0020.STEP", "gobilda_12v_nimh_nested_battery_3100", (0, 1, 0), (-1, 0, 0), (0, 0, 1), BATT_T),
]
