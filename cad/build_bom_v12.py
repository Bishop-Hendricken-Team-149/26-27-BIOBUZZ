"""
Regenerate the HIVE-QUEEN v12 BOM from the as-built Fusion part list (cad/out/v12_cad_parts.json, written by
cad/tmp/extract_parts.py) + the exact belt ledger (cad/v12_design.py) + shop inventory (cad/Inventory(Motion).csv).

  python cad/build_bom_v12.py            -> cad/out/HIVE_QUEEN_149_BOM_CHECKLIST.csv (utf-8-sig)
  uv run --with openpyxl python cad/build_bom_v12.py   -> also .xlsx
"""
import collections
import csv
import json
import math
import os
import re

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "out")
parts = json.load(open(os.path.join(OUT, "v12_cad_parts.json")))
L = {"__name__": "v12_design"}
exec(open(os.path.join(HERE, "v12_design.py"), encoding="utf-8").read(), L)

inv = {}
with open(os.path.join(HERE, "Inventory(Motion).csv"), encoding="utf-8-sig") as f:
    for row in csv.reader(f):
        if len(row) >= 2 and row[0].strip():
            try:
                inv[row[0].strip()] = int(row[1])
            except ValueError:
                inv[row[0].strip()] = 0

custom = {c["name"]: c for c in parts["custom"]}
cots = collections.Counter(re.sub(r":\d+$", "", p.split("/")[-1]) for p in parts["cots"])
names = list(custom)


def having(*subs, excl=()):
    return [n for n in names if any(s in n for s in subs) and not any(e in n for e in excl)]


def ylen(n):
    b = custom[n]["bbox_mm"]
    return round(b[4] - b[1], 1)


rows = []


def add(status, cat, stage, item, vendor, sku, qty, cad, notes):
    rows.append(["✅" if status.startswith("In Stock") else "[ ]", status, cat, stage, item, vendor, sku, qty, cad, notes])


def stock_status(have, need):
    if have >= need:
        return "In Stock (Motion Inv)", f"IN STOCK: have {have} in Inventory(Motion).csv (uses {need})"
    if have > 0:
        return "Partial - To Order", f"have {have} in Inventory(Motion).csv -> ORDER {need - have}"
    return "To Order", "not in Inventory(Motion).csv"


C1, C2, C3, C4, C5 = ("1. Structural Channels & Mounts", "2. Motors, Servos & Electronics", "3. Motion Hardware (Wheels/Gears/Belts/Bearings)",
                      "4. Custom 3D-Printed & CNC Parts", "5. BIOBUZZ Game Elements (CAD Ref)")

# ---------------------------------------------------------------- 1. structure
add("To Order", C1, "Stage 1 - Drivetrain", "17-Hole U-Channel (432 mm)", "goBILDA", "1120-0017-0432", cots["gobilda_1120_17hole_left"], "gobilda_1120_17hole_left:1/:2", "Left & right side rails (left rail grounded in Fusion)")
add("To Order", C1, "Stage 1 - Drivetrain", "10-Hole U-Channel (264 mm)", "goBILDA", "1120-0010-0264", cots["gobilda_1120_10hole_rear"], "gobilda_1120_10hole_rear:1/:2/:3", "Rear crossbar, upper rear crossbar, on-grid mid cross-channel at X=-24 mm (J-chute saddle + Pinpoint plate)")
add("To Order", C1, "Stage 1 - Drivetrain", "5-Hole U-Channel (144 mm) towers", "goBILDA", "1120-0005-0144", cots["gobilda_1120_5hole_tower_rear_left"], "gobilda_1120_5hole_tower_*", "Mid & rear deck support towers")
add("To Order", C1, "Stage 2 - Intake", "4-Hole U-Channel (120 mm) uprights", "goBILDA", "1120-0004-0120", cots["gobilda_1120_4hole_intake_upright_left"], "gobilda_1120_4hole_intake_upright_*", "Front intake uprights; 14 mm holes at Z=144 (intake pivot) and Z=168 (G407 blocker pivot)")
add("To Order", C1, "All", "Tower base / quad block pattern mounts", "goBILDA", "1201-0043-0002", cots["tower_base_mount_rear_left"], "tower_base_mount_rear_left:*", "Tower-to-rail mounts. Same SKU as the drive-motor quad blocks in section 4: order both quantities")

# ---------------------------------------------------------------- 2. motors / servos / electronics
st, nt = stock_status(inv.get("312 RPM", 0), 4)
add(st, C2, "Stage 1", "5203 Yellow Jacket Gearmotor 312 RPM (M1-M4 drive)", "goBILDA", "5203-2402-0019", 4,
    "gobilda_5203_motor_m1..m4_drive", nt + f". Drive via 1:1 miter gears inside the rails; top speed {L['DRIVE_TOP_MS']:.2f} m/s")
st, nt = stock_status(inv.get("1,150 RPM", 0), 4)
add(st, C2, "Stage 2/3/4", "5203 Yellow Jacket Gearmotor 1150 RPM (M5 front wheel, M6 counter-wheel, M7 intake, M8 magazine)", "goBILDA", "5203-2402-0005", 4,
    "gobilda_5203_motor_m5_shooter_master, m6_shooter_follower, m7_intake, m8_magazine", nt + f". M8: a pinched ball moves at half the gecko surface speed -> {L['BALL_FEED_MS']:.2f} m/s, coasts {L['LAST_BALL_COAST_MM']:.0f} mm across the {L['UNPOWERED_GAP_MM']:.0f} mm wall gap (435 RPM stalled a lone POLLEN 19 mm short). Topspin: v_front {L['V_FRONT']:.2f} m/s (20T:20T), v_rear {L['V_REAR']:.2f} m/s (32T:16T). M7 intake 1:1:1 -> {L['INTAKE_SURFACE_MS']:.2f} m/s roller = {L['INTAKE_TO_DRIVE_RATIO']:.1f}x drive speed (the CAD M7 body is the longer 312 RPM gearbox, so real clearances only grow)")
servos = sorted(k for k in cots if k.startswith("servo_"))
st, nt = stock_status(inv.get("Torque servos", 0), len(servos))
add(st, C2, "Stages 2/3/4/5", "2000 Series Dual Mode Torque Servo (S1 lip blocker, S3 indexer gate, S6 hood)", "goBILDA", "2000-0025-0003", len(servos), ", ".join(servos), nt)
add("To Order", C2, "Electronics", "REV Control Hub", "REV Robotics", "REV-31-1595", cots["official_rev_control_hub_31_1595"], "official_rev_control_hub_31_1595 + rev_control_hub_mount_plate_right", "Inboard right bay, armored behind the 17-hole rail (R502/R505)")
add("To Order", C2, "Electronics", "REV Expansion Hub", "REV Robotics", "REV-31-1153", cots["official_rev_expansion_hub_31_1153"], "official_rev_expansion_hub_31_1153 + rev_expansion_hub_mount_plate_left", "Inboard left bay")
add("To Order", C2, "Electronics", "goBILDA 12V NiMH Nested Battery (3000 mAh, XT30, 20 A fuse) - 1 on the robot + 2 charged spares", "goBILDA", "3100-0012-0020", cots["gobilda_12v_nimh_nested_battery_3100"] + 2, "gobilda_12v_nimh_nested_battery_3100", "goBILDA's replacement for the discontinued Matrix 14-0014; Table 12-4 legal. Rides in the goBILDA battery mount on the rear crossbar; XT30 plugs straight into the REV switch cable. Buy 3+ for match rotation")
add("To Order", C2, "Electronics", "Battery Mount for 12V NiMH Nested Battery (incl. hook-and-loop strap, 4x M4x10 + nylock)", "goBILDA", "1209-0001-0001", cots["gobilda_nested_battery_mount_1209"], "gobilda_nested_battery_mount_1209 (vendor STEP)", "On the rear crossbar top web at (X -192, Y 12): its 4 counterbored M4 holes at (+/-8, +/-32) land on crossbar holes; the strap makes battery swaps tool-free")
add("To Order", C2, "Odometry", "Pinpoint Odometry Computer v2", "goBILDA", "3110-0002-0002", 1, "official_gobilda_pinpoint_v2 + pinpoint_v2_mount_plate", "2 encoder ports -> exactly 2 pods (B7). Spacer plate: 3x M4 x 10 at its X -40 end into the mid cross-channel (nylocks inside). Pinpoint: 2x M4 x 16 + nylock at the overhanging corners, 2x M4 x 14 thread-forming into the 3.3 mm pilot holes of the printed PETG spacer (locating only; the nylocked corners and the 3 channel screws carry it)")
add("To Order", C2, "Odometry", "Swingarm Odometry Pod (48 mm wheel)", "goBILDA", "3110-0001-0001", cots["official_gobilda_4bar_odometry_pod"], "official_gobilda_4bar_odometry_pod:1 (left X) + :2 (rear-centre Y)", "Tyres touch the tiles at Z -0.15 (tight measurement); brackets clamp the pivot-shaft ends; v111326's redundant right X-pod dropped")
col = sorted(k for k in cots if k.startswith("rev_color_sensor_v3"))
add("To Order", C2, "Sensors", "Color Sensor V3 (G407/G408 lip + throat ball-size/colour classifier)", "REV Robotics", "REV-31-1557", len(col), ", ".join(col), "TeamCode: lipColor, throatColor. Vendor STEP placed on printed pads (2x M3 x 8 into inserts each)")
cnt = sorted(k for k in cots if k.startswith("rev_2m_distance_sensor"))
add("To Order", C2, "Sensors", "2m Distance Sensor (magazine ball count: entry counter + 4th-slot full)", "REV Robotics", "REV-31-1505", len(cnt), ", ".join(cnt), "TeamCode: ballCount1, ballCount2; on left-shroud pads, looking through 12 mm windows (2x M3 x 8 each)")
add("To Order", C2, "Vision", "HuskyLens AI camera (I2C; spec choice over Limelight 3A, which is also legal per R702 Table 12-9)", "DFRobot", "SEN0305", cots["dfrobot_huskylens_sen0305"], "dfrobot_huskylens_sen0305 + dfrobot_huskylens_mount_block", "Front-right of the shooter; 2x M3 x 10 through its ears into the printed L-bracket, whose flange is held to the right turret double-shear plate by 2x M3 x 12 + M3 nylock nuts (inboard side)")
add("To Order", C2, "Power", "Servo Power Module (6 V injector for 3 torque servos)", "goBILDA", "3125-0001-0001", cots["gobilda_servo_power_injector_3125"], "gobilda_servo_power_injector_3125", "Hung under the deck: 4x M4 x 12 through its base into tapped M4 holes in the deck")
add("To Order", C2, "Power", "Main Power Switch (upward-facing, guarded - R603.B)", "REV Robotics", "REV-31-1387", cots["rev_main_power_switch_31_1387"], "rev_main_power_switch_31_1387 + rev_main_power_switch_housing + cage", "Face-up on a printed housing (3x M3 x 8 into inserts) inside the 4-sided impact cage; leads drop through the deck")

# ---------------------------------------------------------------- 3. motion hardware
add("To Order", C3, "Stage 1 - Drivetrain", "96 mm Mecanum wheel set (2 LH + 2 RH)", "goBILDA", "3606-0001-0096", cots["gobilda_mecanum_96mm_front_left"] + cots["gobilda_mecanum_96mm_front_right"], "gobilda_mecanum_96mm_*", f"Inventory has {inv.get('omni wheels', 0)}x omni wheels (not mecanum)")
nmit = cots["gobilda_2315_miter_gear_30t"]
st, nt = stock_status(inv.get("Bevel Gears (big)", 0), nmit)
add(st, C3, "Stage 1 - Drivetrain", "1:1 steel miter gears, MOD 1.25 30T, 8 mm REX (goBILDA 2315)", "goBILDA", "2315-4008-0030", nmit, "gobilda_2315_miter_gear_30t:1..:8 (vendor STEP)",
    nt + "; v12.6 real model: pinion on the motor shaft, gear on the REX axle, cone apex at Y=+/-156 mm (mounting distance 20.56 mm), back face 0.94 mm off the rail wall for a thrust washer. Inventory 'Bevel Gears (big)' must be checked against 2315-4008-0030")
add("To Order", C3, "Stage 1 - Drivetrain", "Thrust bearing, 8 mm ID (behind each axle miter gear)", "goBILDA", "1613-0516-0008", cots["gobilda_1613_thrust_bearing_8mm"],
    "gobilda_1613_thrust_bearing_8mm:1..:4 (vendor STEP)", "Nests in the 2315 gear's dished back against the rail wall, as in goBILDA's Strafer kit; takes the miter spreading force")
brg = having("bearing", excl=("stop_plate",))
st, nt = stock_status(inv.get("Bearings (rec)", 0), len(brg))
add(st, C3, "All", "8 mm REX flanged ball bearing (14 mm OD)", "goBILDA", "1611-0514-4008", len(brg), ", ".join(brg), nt + "; every swing-arm hub carries its own bearing (Gate 7)")
# shafts with a stock cut plan
shafts = sorted(((n, ylen(n)) for n in having("rex_wheel_axle", "rex_shaft", "stub_shaft", "pivot_stub", "jackshaft", "j2_pivot_pin", "rex_stub", "blocker_pivot_stub", "hinge_stub", excl=("coupler",))), key=lambda t: -t[1])
stock = []
for k, v in inv.items():
    m = re.match(r"(\d+)mm shaft", k)
    if m:
        stock += [int(m.group(1))] * v
stock.sort(reverse=True)
bins = [[s, s, []] for s in stock]
unplaced = []
for n, ln in shafts:
    need = math.ceil(ln + 2.0)
    for b in sorted(bins, key=lambda b: b[1]):
        if b[1] >= need:
            b[1] -= need
            b[2].append(f"{n}={ln}")
            break
    else:
        unplaced.append((n, ln))
used = [b for b in bins if b[2]]
plan = " | ".join(f"{b[0]} mm stock -> {', '.join(b[2])}" for b in used)
add("In Stock (Motion Inv)" if not unplaced else "Partial - To Order", C3, "All", "8 mm REX shaft, cut to length", "goBILDA", "2100-0008 series", len(shafts),
    ", ".join(f"{n} ({ln} mm)" for n, ln in shafts), f"Cut plan from {len(used)} stock shafts (2 mm kerf): {plan}" + (f" | ORDER: {unplaced}" if unplaced else ""))
pul = collections.Counter(("3M" if n.startswith("magazine_") else "5M", re.search(r"(\d+)t$", n).group(1)) for n in having("pulley"))
for (pt, t), q in sorted(pul.items()):
    names_ = ", ".join(n for n in having("pulley") if n.endswith(f"_{t}t") and (n.startswith("magazine_") == (pt == "3M")))
    pd_ = L["htd_pd"](int(t), 3.0 if pt == "3M" else 5.0)
    if pt == "5M" and t == "16":
        add("To Order", C3, "Stage 4", "HTD-5M 16T pulley, 8 mm REX bore, 9 mm belt", "goBILDA", "3412-0008-0016 (HTD-5M 16T)", q, names_, f"PD {pd_:.2f} mm")
    else:
        add("To Print", C3, "Stages 2/3/4", f"HTD-{pt} {t}T timing pulley, 8 mm REX bore, {'6' if pt == '3M' else '9'} mm belt (printed)",
            "Bambu PETG-CF (print 100 % infill, REX bore + 2x M3 set screws)", f"HTD-{pt} {t}T, PD {pd_:.2f} mm, {'8.5' if pt == '3M' else '12.0'} mm wide", q, names_,
            "goBILDA sells no " + (f"HTD-3M REX pulley" if pt == "3M" else f"{t}T HTD-5M pulley") + ". The CAD body is a toothless envelope: generate the printable pulley with an HTD pulley generator (e.g. the 'Parametric Pulley' OpenSCAD library, HTD 3mm/5mm profiles) at this tooth count, width and an 8 mm REX bore with 2x M3 set screws [%.1f cm3 total]" % (q * 3.14159 * (pd_ / 2 / 10) ** 2 * (0.85 if pt == "3M" else 1.2)))
belts = L["belt_ledger"]()
blen = collections.Counter(int(round(v["cots_L_mm"])) for v in belts.values())
for ln, q in sorted(blen.items()):
    which = [k for k, v in belts.items() if int(round(v["cots_L_mm"])) == ln]
    p_ = belts[which[0]]["pitch"]
    bodies_ = [n for n in having("htd5m", "htd3m") if f"_{ln}mm_" in n]
    if p_ == 3.0:
        add("To Order", C3, "Stage 3 - Magazine", f"HTD-3M closed-loop belt {ln} mm ({ln // 3} T), 6 mm wide", "Gates PowerGrip HTD (MDS of Michigan)", f"{ln}-3M-06", q,
            ", ".join(bodies_) + " (ledger: " + ", ".join(which) + ")", "v12.6: magazine belts are HTD-3M 6 mm (fits the existing 7 mm lanes)")
        continue
    add("To Order", C3, "Stages 2/4", f"HTD-5M closed-loop belt {ln} mm ({ln // 5} T), 9 mm wide", "goBILDA", f"3416-0009-{ln:04d}", q, ", ".join(bodies_) + " (ledger: " + ", ".join(which) + ")",
        f"Exact C-C: max |err| {max(abs(belts[k]['err_mm']) for k in which):.3f} mm (< 0.25 mm). Inventory has {inv.get('5m belt', 0)}x '5m belt' of unknown length")
add("In Stock (Motion Inv)", C3, "Stage 4 - Shooter", "96 mm Rhino high-grip wheel (front)", "goBILDA", "3614-0014-0096", cots["gobilda_rhino_wheel_96mm"], "gobilda_rhino_wheel_96mm", f"Boot Wheel big x{inv.get('Boot Wheel big', 0)} in inventory")
_fly = [d[1] for d in L["VENDOR_DEVICES"] if d[1].startswith("gobilda_flywheel_plate")]
_fhub = [d[1] for d in L["VENDOR_DEVICES"] if d[1].startswith("gobilda_1313_flywheel_hub")]
add("To Order", C3, "Stage 4 - Shooter", "Steel Flywheel 82 mm (152 g, 1651 g-cm2) - two per side", "goBILDA", "3628-0032-0082", len(_fly), ", ".join(_fly),
    "v12.6: replaces the source model's solid 82 x 10 mm REX disc (~415 g, not a real product). Plate A | 1313 hub arms | plate B on the hub shoulder, "
    "4x M4 x 25 + nylocks per side through the 32 mm pattern (plates clocked 45 deg). ~4.0 J stored at 1050 RPM: a NECTAR shot costs ~15 %")
add("To Order", C3, "Stage 4 - Shooter", "Hyper Hub, 8 mm REX bore, 16 + 32 mm patterns (flywheel plate hub)", "goBILDA", "1313-1632-4008", len(_fhub), ", ".join(_fhub),
    "REX bore locks to the REX main shaft; 2x M4 x 14 clamp screws included")
st, nt = stock_status(inv.get("gobuild compliant wheels medium", 0), 1)
add(st, C3, "Stage 4 - Shooter", "72 mm compliant counter-wheel", "goBILDA", "3613 series", 1, "gobilda_counter_wheel_72mm_topspin", nt + "; on coaxial swing arms (M6 axis), arc slot R=61.18 mm")
st, nt = stock_status(inv.get("Stretch intake wheel big", 0), cots["gobilda_gecko_wheel_72mm_1"])
add(st, C3, "Stage 3 - Magazine", "72 mm gecko feeder wheels", "goBILDA", "3613 series", cots["gobilda_gecko_wheel_72mm_1"], "gobilda_gecko_wheel_72mm_1:1/:2", nt)
add("To Order", C3, "Stage 3/4", "Hyper Hub, 8 mm REX bore (mounts the 14 mm-bore Rhino + Gecko wheels on REX shaft)", "goBILDA", "1310-0016-4008",
    cots["gobilda_rhino_wheel_96mm"] + cots["gobilda_gecko_wheel_72mm_1"] + cots["gobilda_counter_wheel_72mm_topspin"], "(not modelled) one per 14 mm-bore wheel: Rhino, 2 geckos, 72 mm counter-wheel", "goBILDA's 14 mm-bore wheels have no REX bore")
_cpl = [d[1] for d in L["VENDOR_DEVICES"] if d[1].startswith("gobilda_servo_rex_coupler")]
add("To Order", C3, "Stages 3/4/5", "Clamping servo-to-shaft coupler, 25T spline to 8 mm REX (servo torque path)", "goBILDA", "4001-0025-4008", len(_cpl), ", ".join(_cpl),
    "v12.6: S1 -> G407 blocker REX stub, S3 -> indexer REX shaft, S6 -> hood REX stub. Gate/hood bores are REX, so the servo really drives them (before, they spun freely on round shafts)")
spr = having("extension_spring")
add("To Order", C3, "Stages 3/4", "Extension spring assortment (gecko 1 + 2 shooter return springs)", "Amazon", "AMZ-B0GJZ8Q32D", 1, ", ".join(spr),
    f"{len(spr)} springs in the CAD: shooter 6 OD x ~24 mm hook-to-hook (start with the kit's 0.22 x 1.0 in), gecko 1 6 OD x 30 mm (start with 0.18 x 1.18 in). "
    "The rate is a bench-tune: counter-wheel squeeze on a POLLEN and gecko return must both hold - swap sizes from the kit")
add("To Order", C3, "Stage 3", "Torsion spring, 90 deg legs, <= 18 mm OD, >= 8.2 mm ID, <= 5.8 mm body (gecko 2 return on J2)", "McMaster-Carr", "9271K series (pick by these limits)", 1, "gecko2_torsion_return_spring",
    "Sized from the CAD envelope (18 x 5.8 mm on the 8 mm J2 pin). McMaster shows prices only on its site: price at order time")
pins = having("spring_arm_pin", "spring_fixed_pin")
add("To Order", C3, "Stages 3/4", "Steel dowel pins, metric h8 (spring anchors): 6 x 12, 6 x 14 (x3 each side), 5 x 24", "McMaster-Carr", "98381A series (18-8 SS metric dowels)", len(pins), ", ".join(pins),
    "CAD lengths 12.5/13.5/14 mm (6 mm) and 23.5 mm (5 mm): buy the next standard length that does not reach a belt plane (|Y| 72-80). "
    "Amazon dowel kits stop at 10/20/30 mm - too long here. Price at order time")
stops = having("hard_stop_pin")
add("In Stock (cut)", C3, "Stage 2", "Intake hard-stop pins, cut from 8 mm REX shaft stock: 2x 32 mm + 2x 24 mm", "goBILDA", "2100-0008 series offcuts", len(stops), ", ".join(stops),
    "REX (8 mm across the corners) fits the round 8 mm upright holes; retain with a 2910 clamping collar or Loctite")
spc = having("spacer", "collar", "coupler", excl=("sign_rail_bolt_spacer",))   # the sign-rail spacers are printed (PRINT table)
add("To Order", C3, "All", "8 mm REX spacers / collars / shaft couplers", "goBILDA", "1500/2910 series", len(spc), ", ".join(f"{n} ({ylen(n)} mm)" for n in spc), "")
_dr = json.load(open(os.path.join(HERE, "tmp", "drill.json"))) if os.path.exists(os.path.join(HERE, "tmp", "drill.json")) else {}
_br = _dr.get("by_radius", {})
# v12.6: fasteners are counted from the as-built CAD: every hole the drill kept (cad/tmp/drill.json "detail": body, kind,
# radius, clearance-side grip) plus the screws modelled explicitly in the builder (list below, each with its source),
# then +30 % spares, rounded up to whole packs by the pricing pass.
_det = _dr.get("detail", [])
_fx = collections.Counter()
_why = collections.defaultdict(list)


def _m3_len(grip):
    return next(l for l in (8, 10, 12, 16, 20) if l >= grip + 5.0)


for _n, _key, _r in _det:
    if _key.startswith("cots:"):
        if _r == 2.8:
            _fx["M4 insert"] += 1; _fx["M4x12"] += 1; _why["M4x12"].append("COTS->printed")
        elif _r == 2.0:
            _fx["M3 insert"] += 1; _fx["M3x10"] += 1
        elif _r == 2.25:
            _fx["M4x16"] += 1; _fx["M4 nylock"] += 1; _why["M4x16"].append("COTS->aluminium")
        elif _r == 1.7:
            _fx["M3x12"] += 1; _fx["M3 nylock"] += 1
    elif _key.startswith("joint:"):
        _g = float(_key.rsplit("|", 1)[1]) if "|" in _key else 6.0
        if _r == 2.0:
            _fx["M3 insert"] += 1
        elif _r == 1.7:
            _fx[f"M3x{_m3_len(_g)}"] += 1
EXPLICIT = [  # (fastener, qty, where) -- screws the builder models outside the drill pass
    ("M4x25", 8, "flywheel plate pairs through the 1313 hubs"), ("M4 nylock", 8, "flywheel plate pairs"),
    ("M4x35", 2, "left DS plate standoffs (5 + 24 tube + 6 tapped side plate)"), ("M4x40", 2, "right DS plate standoffs (5 + 29 + 6)"),
    ("M4x12", 8, "flywheel motor saddle ears into the tapped deck"),
    ("M4x16", 2, "left intake stop plate to the upright web"), ("M4x60", 2, "right intake stop plate through its bosses to the web"), ("M4 nylock", 4, "stop plates"),
    ("M4x12", 4, "M7 sleeve into the gearbox's tapped side holes"), ("M4x10", 4, "M8 sleeve into the gearbox's tapped side holes"),
    ("M4x12", 8, "battery mount 1209 to the rear crossbar (its 4 included screws become spares)"), ("M4 nylock", 8, "battery mount"),
    ("M4x12", 5, "Pinpoint spacer: 3 into the channel + 2 overhang corners"), ("M4 nylock", 5, "Pinpoint"), ("M4x14", 2, "Pinpoint corners thread-formed into the PETG spacer"),
    ("M3x12", 2, "HuskyLens L-bracket flange through the DS plate"), ("M3 nylock", 2, "HuskyLens flange"),
    ("M3x10", 4, "S3 servo ears (S1/S6 ears and the HuskyLens ears are drilled holes, counted above)"), ("M3 insert", 4, "S3 servo ears"),
    ("M3x10", 3, "main switch plate"), ("M3 insert", 3, "main switch plate"),
    ("M3x8", 8, "Color V3 + 2m Distance sensors (2 each)"), ("M3 insert", 8, "sensor pads"),
    ("M4x16", 4, "right sign rails to the mid tower wall"), ("M4x60", 4, "left sign rails across the open tower channel through printed spacer tubes"), ("M4 nylock", 8, "sign rails"),
    ("M4x8", 32, "drive quad blocks: 4 to the motor face + 4 to the channel each"),
    ("M4x8", 40, "goBILDA channel-to-channel / bracket joints (allowance: ~20 joints x 2)"), ("M4 nylock", 40, "channel joints"),
]
for _f, _q, _w in EXPLICIT:
    _fx[_f] += _q
    _why[_f].append(f"{_q} {_w}")
FAST_SKU = {"M4x8": "2800-0004-0008", "M4x10": "2800-0004-0010", "M4x12": "2800-0004-0012", "M4x14": "2800-0004-0014",
            "M4x16": "2800-0004-0016", "M4x25": "2800-0004-0025", "M4x35": "2800-0004-0035", "M4x40": "2800-0004-0040",
            "M4x60": "2800-0004-0060", "M4 nylock": "2812-0004-0007", "M3x8": "2800-0003-0008", "M3x10": "2800-0003-0010",
            "M3x12": "2800-0003-0012", "M3x16": "AMZ-B0FGV5FCBN", "M3x20": "AMZ-B0FGV5FCBN", "M3 nylock": "AMZ-B0CX16JZ6Z",
            "M4 insert": "AMZ-B07YSV66Y5", "M3 insert": "AMZ-B08K1BVGN9"}
for _f in sorted(_fx, key=lambda k: (k[:2], len(k), k)):
    _need = math.ceil(_fx[_f] * 1.3)
    _drill = _fx[_f] - sum(q for f, q, w in EXPLICIT if f == _f)
    _src = (f"{_drill} from drilled holes" if _drill else "") + ("; " if _drill and _why[_f] else "") + "; ".join(w for w in _why[_f] if not w.startswith("COTS"))
    _vend = "goBILDA" if FAST_SKU[_f][:4].isdigit() else ("Amazon" if FAST_SKU[_f].startswith("AMZ") else "Ruthex (Amazon)")
    _item = {"M4 insert": "M4 heat-set insert, 8.1 mm long (5.6 mm hole)", "M3 insert": "M3 heat-set insert, 5.7 mm long (4.0 mm hole)",
             "M4 nylock": "M4 nylon-insert lock nut", "M3 nylock": "M3 nylon-insert lock nut"}.get(_f, f"{_f.replace('x', ' x ')} mm socket head cap screw")
    add("To Order", C3, "Hardware", _item, _vend, FAST_SKU[_f], _need, "CAD-derived (cad/tmp/drill.json + EXPLICIT list in build_bom_v12.py)",
        f"{_fx[_f]} in the CAD + 30 % spares. {_src}".strip())
add("To Order", C3, "Hardware", "M3 x 4 cup-point set screws (printed HTD pulleys, 2 per pulley)", "Amazon", "AMZ-B012T9JTTW", 26, "cad/pulleys/README.md (13 pulleys)",
    "Grub screw on a REX flat through the captive-nut slot; M3 hex nuts from the M3 kit")

# ---------------------------------------------------------------- 3b. wiring + charging (v12.6; runs from docs/06, lengths
# chosen from what the vendors sell: each cable >= the estimated run; motors 5203 have 3.5 mm bullet leads + a JST-XH encoder header)
C6 = "6. Wiring & Charging"
WIRE = [
    ("Motor power adaptor, JST VH to 3.5 mm bullet (100 mm) - one per motor", "goBILDA", "3801-0613-0100", 8, "M1-M8 -> hub motor ports"),
    ("3.5 mm bullet extension, 300 mm - one per motor (runs 243-380 mm)", "goBILDA", "3802-1213-0300", 8, "M1-M8"),
    ("Encoder cable JST PH to JST XH, 300 mm (runs <= 300 mm)", "goBILDA", "3801-0709-0300", 5, "M2, M4, M5, M7, M8"),
    ("Encoder cable JST PH to JST XH, 600 mm (runs 300-600 mm)", "goBILDA", "3801-0709-0600", 5, "M1 (380), M3 (380), M6 (338) + odometry pods X (492 mm) and Y (408 mm) -> Pinpoint"),
    ("Servo extension 300 mm", "goBILDA", "3802-1718-0300", 4, "S3 (186 mm) + 3 injector signal inputs (234 mm)"),
    ("Servo extension 600 mm", "goBILDA", "3802-1718-0600", 1, "S1 (474 mm)"),
    ("Servo extension 900 mm", "goBILDA", "3802-1718-0900", 1, "S6 hood (682 mm)"),
    ("XT30 extension cable 50 cm (2-pack)", "REV Robotics", "REV-31-1394-PK2", 4, "switch -> Expansion Hub -> Control Hub -> servo power module (3 used, 1 spare)"),
    ("RS485 communication cable 50 cm (2-pack)", "REV Robotics", "REV-31-1418-PK2", 2, "Control Hub <-> Expansion Hub (1 used, 1 spare)"),
    ("JST PH 4-pin sensor cable 30 cm (4-pack)", "REV Robotics", "REV-31-1407-PK4", 4, "Pinpoint (237), throat Color V3 (176), ballCount1 slot 2 (292) + spare"),
    ("JST PH 4-pin sensor cable 50 cm (4-pack)", "REV Robotics", "REV-31-1408-PK4", 4, "ballCount2 slot 3 (400), lip Color V3 (474) + spares"),
    ("JST PH 4-pin sensor cable 100 cm (4-pack)", "REV Robotics", "REV-31-1409-PK4", 4, "HuskyLens (508): over 50 cm, so 100 cm routed in a service loop + spares. "
     "HuskyLens' own Gravity 4-pin lead is NOT REV-pinned: re-pin one 100 cm cable to HuskyLens order (+, -, SDA/T, SCL/R) and label it"),
    ("12V NiMH charger, XT30 (plugs straight into the Nested battery)", "goBILDA", "3101-0012-0001", 1, "charging 3 batteries between matches"),
    ("20 A MINI blade fuses (25-pack) - the Nested battery takes ATM/mini, not ATO", "Amazon", "AMZ-B0DS29R9HW", 25, "battery spare fuses"),
]
for _it, _v, _sku, _q, _w in WIRE:
    add("To Order", C6, "Wiring", _it, _v, _sku, _q, "docs/06_WIRING_PLAN.md", _w)
add("Verify Owned", C6, "Driver Station", "REV Driver Hub + 2 gamepads (R7xx legal list)", "REV Robotics", "REV-31-1596", 1, "-", "Not priced: the team's existing driver station kit")
add("Verify Owned", C6, "Tools", "Soldering iron with heat-set insert tips (M3 + M4), M3/M4 taps (JLCCNC taps the plate holes), 2.5/3 mm hex drivers", "-", "-", 1, "-", "Needed for the inserts and the PETG thread-forming pilots")

# ---------------------------------------------------------------- 4. custom parts
PRINT = {
    "unified_continuous_j_chute": ("To Print (split)", "Bambu PETG-CF (amber)", "J-chute (ramp+bend+wall+curbs+4 mounting stations), 308.7 x 274.5 x 263.8 mm. Print it whole on an H2D, or on a 256 mm bed print the 4-piece kit in cad/3MF/v12/_chute_split/ (part1 upper wall, part2 bend+lower wall, part3L/R front ramp; every piece fits 250 mm flat). Each piece carries its own frame mounting station; seams are perpendicular or parallel to ball travel. Bond flush with epoxy, sand any lip >0.3 mm, then check a NECTAR rolls through"),
    "sprung_magazine_shroud_": ("To Print", "Bambu PETG (violet)", "Concentric arc slots about J2/M8; left has 3 sensor windows"),
    "asymmetrical_funnel_wing_": ("To Print", "Bambu PETG/TPU", "X_max 228 (left) / 222 (right)"),
    "g409_sloped_deck_shed_shield_": ("To Print", "Bambu PETG (clear blue)", "40 deg hollow roofs (G409)"),
    "floating_intake_swing_arm_": ("To Print", "Bambu PETG-CF", "Bearing seats both hubs + tail hard stop"),
    "floating_spoked_compliant_wheel_": ("To Print", "Bambu TPU 95A (teal)", "47 mm spoked compliant wheels (5; the centre one replaces v12.0's star roller, whose blades swept into gecko 1)"),
    "gecko1_coaxial_swing_arm_on_m8": ("To Print", "Bambu PETG-CF (orange)", "Coaxial on the M8 shaft"),
    "gecko2_coaxial_swing_arm_": ("To Print", "Bambu PETG-CF (orange)", "Coaxial on J2"),
    "shooter_coaxial_swing_arm_": ("To CNC / Print", "6061 Al or PETG-CF", "Coaxial on the M6 axis (0 belt stretch)"),
    "variable_pitch_exit_hood_full": ("To Print", "Bambu PETG (cyan)", "Hinge X=-116 coaxial with S6"),
    "g407_front_lip_blocker_gate": ("To Print", "Bambu PETG (crimson)", "Pivot (156,168) on the upright 14 mm hole"),
    "indexer_stopper_gate_3prong_fingers": ("To Print", "Bambu PETG (crimson)", "3 fingers through floor slits"),
    "intake_upright_bearing_and_stop_plate_": ("To Print", "Bambu PETG-CF", "Pivot bearing + upper stop pin; v12.6: 2x M4 to the upright web (right side through 2 printed bosses)"),
    "turret_flywheel_motor_saddle_": ("To Print", "Bambu PETG-CF", "v12.6: 8 mm base ears, 4x M4 x 12 into tapped deck holes (it used to rest on the deck unscrewed)"),
    "m7_intake_motor_clamp_saddle_left": ("To Print", "Bambu PETG-CF", "v12.6: cradle + sleeve over the M7 gearbox, 4x M4 x 12 into its tapped side holes (M7 was held by nothing)"), "m8_magazine_motor_clamp_saddle_right": ("To Print", "Bambu PETG-CF", ""),
    "odom_pod_": ("To Print", "Bambu PETG-CF", "Swingarm pod brackets: clamp both ends of each pod's 8 mm REX pivot shaft (REX holes); the pod body swings freely between them"), "pinpoint_v2_mount_plate": ("To Print", "Bambu PETG", "3.1 mm spacer under the Pinpoint: 100% infill around the holes (4 walls) so the M4 pilot holes thread-form cleanly"),
    "gobilda_12v_battery_": ("To Print", "Bambu PETG / TPU strap", ""), "rev_control_hub_mount_plate_right": ("To Print", "Bambu PETG", ""),
    "rev_expansion_hub_mount_plate_left": ("To Print", "Bambu PETG", ""), "main_power_switch_impact_guard_cage": ("To Print", "Bambu PETG-CF", ""),
    "turret_servo_cradle_s6_hood": ("To Print", "Bambu PETG-CF", ""), "dfrobot_huskylens_mount_block": ("To Print", "Bambu PETG-CF", "L-bracket: 2x M3 inserts for the HuskyLens ears; 5 mm flange bolts to the DS plate's outboard face (2x M3 x 12 + nylocks)"),
    "rev_main_power_switch_housing": ("To Print", "Bambu PETG-CF", "Switch plate on top (3x M3 inserts); body inside the cavity"),
    "robot_sign_": ("To Print", "Bambu AMS multi-colour (red or blue + white)", "174 x 78 mm plate: 7.25 mm visible border inside the 3 mm rail lips (R403), 57.5 mm numerals, slide rails (R401-R403); red & blue .3mf"),
    "turret_side_plate_foot_bracket_": ("To Print", "Bambu PETG-CF", "L-bracket side plate <-> deck (v12.6: replaces the machined foot so the plate stays a flat 2D part)"),
    "sign_rail_bolt_spacer_": ("To Print", "Bambu PETG-CF", "4 spacer tubes inside the left mid tower: the left sign rails bolt across the open channel to its far wall (M4 x 60)"),
    "turret_ds_standoff_tube_": ("To Print", "Bambu PETG-CF", "11 mm OD standoff tubes DS plate -> side plate: M4 x 35 (left) / x 40 (right) through-bolts into tapped side-plate holes"),
    "turret_side_plate_": ("To CNC", "5052-H32 Al 6 mm (JLCCNC)", "Flat 2D plate: bearing holes + concentric arc slot; M4 tapped holes for the DS standoffs. DXF in cad/CNC/"),
    "turret_outboard_double_shear_plate_": ("To CNC", "5052-H32 Al 5 mm (JLCCNC)", "Flat 2D plate (bosses replaced by printed standoff tubes); clearance holes only. DXF in cad/CNC/"),
    "magazine_outboard_double_shear_plate_right": ("To Print", "Bambu PETG-CF (or CNC from 6061 block)", "Plate + square sleeve over the M8 gearbox; 4x M4 x 10 SHCS into the gearbox's tapped side holes hold it"),
    "s1_servo_guard_cage": ("To Print", "Bambu PETG-CF", "S1 mount + impact guard: S1's ears screw into its ribs (4x M3 inserts); its tab bolts to the upright's front wall"),
    "drive_motor_quad_block_mount_": ("To Order", "goBILDA 1201-0043-0002 Quad Block Pattern Mount", "Holds each drive motor inside the 1120 rail (motor face: 4x M4 x 8; to the channel walls: M4 x 8). Modelled as a placeholder block"),
    "s3_indexer_servo_bracket": ("To Print", "Bambu PETG-CF", "S3 L-bracket: flange on the mid cross-channel, 4x M3 inserts for S3's ears, spline clearance hole"),
    "cnc_turret_mounting_deck_plate_clean": ("To CNC", "5052-H32 Al 6 mm (JLCCNC)", "Throat cut-out |Y|<=55 + gecko-2 swing relief; M4 tapped holes for the flywheel motor saddles. DXF in cad/CNC/"),
}
hardware = set(brg) | set(n for n, _ in shafts) | set(having("pulley", "htd5m", "htd3m", "spring", "spacer", "collar", "coupler", "_pin", "sensor",
                                                                             "huskylens_pcb", "huskylens_front", "power_injector", "power_switch_panel", "staged_", excl=("sign_rail_bolt_spacer",)))
done = set(hardware)
for key, (status, proc, note) in PRINT.items():
    hits = [n for n in names if n.startswith(key) and n not in done]
    if not hits:
        continue
    done.update(hits)
    vol = sum(custom[n]["vol_cm3"] for n in hits)
    item = key.strip("_").replace("_", " ")
    add(status, C4, "Custom", item, proc, ".3mf / .step in cad/3MF/v12 + cad/STEP/project_hive_queen_robot_v12.step", len(hits), ", ".join(hits), f"{note} [{vol:.1f} cm3 total]".strip())

# ---------------------------------------------------------------- 5. game elements
add("Field Element", C5, "Game Element", "NECTAR ball 91.95 mm", "FIRST", "BIOBUZZ-NECTAR", 1, "staged_nectar_ball_92mm_approaching_mouth", "Audit reference sphere (never a cutter)")
add("Field Element", C5, "Game Element", "POLLEN ball 71.12 mm", "FIRST", "BIOBUZZ-POLLEN", 2, "staged_pollen_1_queued, staged_pollen_2_queued", "Queued behind the S3 gate (pollen 1 held, 0.7 mm clear of gecko 2)")

# ---------------------------------------------------------------- 6. links + prices (cad/bom_prices.py, read from live pages)
PR = {"__name__": "bom_prices", "__file__": os.path.join(HERE, "bom_prices.py")}
exec(open(os.path.join(HERE, "bom_prices.py"), encoding="utf-8").read(), PR)


def buy_units(r):
    st, qty, notes = r[1], r[7], r[9]
    if st.startswith("In Stock") or st.startswith("Field"):
        return 0
    m = re.search(r"ORDER (\d+)", notes)
    return int(m.group(1)) if (st.startswith("Partial") and m) else int(qty)


grand = 0.0
for r in rows:
    link = price = pack = packs = cost = None
    basis = ""
    e = PR["BY_SKU"].get(r[6]) or (PR["HYPER_HUB"] if r[6] == "1310-0016-4008" else None)
    if e is None and "1201-0043-0002" in r[5]:          # drive-motor quad blocks (generated from the PRINT table)
        e, r[6] = PR["BY_SKU"]["1201-0043-0002"], "1201-0043-0002"
    if r[4].startswith("8 mm REX spacers"):
        kinds = collections.Counter(next((k for k in PR["SPACER_KINDS"] if k in n), "spacer") for n in r[8].split(", ") if n)
        parts_, cost = [], 0.0
        for k, nk in kinds.items():
            sk = PR["SPACER_KINDS"][k]
            npk = math.ceil(nk / sk["pack"])
            cost += npk * sk["price"]
            parts_.append(f"{nk}x {k}: {sk['sku']} ${sk['price']:.2f}/{sk['pack']} -> {npk} pack(s) {sk['url']}")
        link, basis = PR["SPACER_KINDS"]["collar"]["url"], "; ".join(parts_)
        r[6] = "2910-0920-4008 / 1522-0010-0060 / 4007-4008-4008"
    elif e:
        link, price, pack = e["url"], e["price"], e["pack"]
        units = buy_units(r)
        packs = math.ceil(units / pack) if units else 0
        cost = round(packs * price, 2)
        basis = f"${price:.2f} per {'pack of ' + str(pack) if pack > 1 else 'unit'}"
        if units == 0 and not r[1].startswith("Field"):
            basis += " (in stock: nothing to buy)"
        if e.get("sku"):
            r[9] = (r[9] + " | " if r[9] else "") + f"[was {r[6]}]"
            r[6] = e["sku"]
        if e.get("note"):
            basis += ". " + e["note"]
    elif r[1].startswith("To Print") or r[1].startswith("To CNC / Print"):
        m = re.search(r"\[([\d.]+) cm3 total\]", r[9])
        proc = r[5]
        key = next((k for k in ("PETG-CF", "TPU", "clear", "AMS", "PETG") if k in proc), "PETG")
        f = PR["FILAMENT"][key]
        if m:
            grams = float(m.group(1)) * f["density"] * PR["FILL"]
            cost = round(grams / 1000.0 * f["per_kg"], 2)
            link, price, pack, packs = f["url"], f["per_kg"], 1, None
            basis = f"filament only: ~{grams:.0f} g of {key} at ${f['per_kg']:.2f}/kg (est. {int(PR['FILL'] * 100)} % of solid volume)"
            if r[1].startswith("To CNC / Print"):
                basis += f"; CNC option: quote at {PR['CNC']['url']}"
    elif r[1].startswith("To CNC"):
        link, basis = PR["CNC"]["url"], PR["CNC"]["note"]
    if link and "amazon.com" in link and r[5] == "goBILDA":     # v12.6: bought on Amazon (e.g. the Goodyear 260-5M-9 belt)
        r[5] = "Amazon"
    r += [link or "", price if price is not None else ("quote" if r[1].startswith("To CNC") else ""), pack or "", packs if packs is not None else "",
          cost if cost is not None else "", basis + (f" [checked {PR['CHECKED']}]" if link else "")]
    if isinstance(cost, (int, float)):
        grand += cost
unpriced = [r[4] for r in rows if r[-2] == "" and not r[1].startswith(("Field",))]

# coverage check: every custom body must be referenced by some row
listed = " ".join(r[8] for r in rows)
uncovered = [n for n in names if n not in listed and not n.startswith(("drivetrain_axle_bearing", "drivetrain_miter"))]
hdr = ["Checked", "Status", "Category", "Stage / Subsystem", "Item / Part Name", "Vendor / Process", "SKU / Part Number / Spec", "Qty", "CAD Occurrence / Body Names", "Notes",
       "Link", "Price per pack (USD)", "Pack qty", "Packs to buy", "Line cost (USD)", "Price basis / notes"]
csv_path = os.path.join(OUT, "HIVE_QUEEN_149_BOM_CHECKLIST.csv")
with open(csv_path, "w", newline="", encoding="utf-8-sig") as f:
    w = csv.writer(f)
    w.writerow(hdr)
    w.writerows(rows)
    w.writerow([""] * 13 + ["TOTAL to buy", round(grand, 2), "excl. tax, shipping, CNC quotes"])
print("rows", len(rows), "custom bodies", len(names), "cots occ", sum(cots.values()), "uncovered", uncovered)
print("total to buy $%.2f" % grand, "unpriced", unpriced)
try:
    import openpyxl
    from openpyxl.styles import Alignment, Font, PatternFill
    wb = openpyxl.Workbook()
    ws = wb.active
    ws.title = "HIVE-QUEEN v12 BOM"
    ws.append(hdr)
    for r in rows:
        ws.append(r)
    ws.append([""] * 13 + ["TOTAL to buy", round(grand, 2), "excl. tax, shipping, CNC quotes"])
    for row in ws.iter_rows(min_row=2):
        if row[10].value and str(row[10].value).startswith("http"):
            row[10].hyperlink = row[10].value
            row[10].style = "Hyperlink"
    for c in ws[1]:
        c.font = Font(bold=True, color="FFFFFF")
        c.fill = PatternFill("solid", fgColor="1F2937")
    fills = {"In Stock": "DCFCE7", "Partial": "FEF9C3", "To Order": "FEE2E2", "To Print": "E0E7FF", "To CNC": "E0F2FE", "Field": "F3F4F6"}
    for row in ws.iter_rows(min_row=2):
        for k, col in fills.items():
            if str(row[1].value).startswith(k):
                for c in row:
                    c.fill = PatternFill("solid", fgColor=col)
        for c in row:
            c.alignment = Alignment(wrap_text=True, vertical="top")
    for col, wdt in zip("ABCDEFGHIJKLMNOP", (8, 20, 30, 20, 48, 26, 30, 6, 60, 70, 50, 12, 8, 8, 12, 80)):
        ws.column_dimensions[col].width = wdt
    ws.freeze_panes = "A2"
    wb.save(os.path.join(OUT, "HIVE_QUEEN_149_BOM_CHECKLIST.xlsx"))
    print("xlsx written")
except ImportError:
    print("openpyxl missing - csv only")
