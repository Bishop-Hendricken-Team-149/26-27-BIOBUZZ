# 05 — HIVE-QUEEN v12.2 Competition Readiness Checklist

**CAD:** Fusion 360 `BHHS FTC 149 / Default Project / FTC 26-27 / HIVE QUEEN v111426m` (v12.6 production pass) · **Manual:** BIOBUZZ TU07

This file separates **what has been verified in CAD/analysis** (done, with numbers) from **what can only be verified on the real robot** (checklist for the build/pit crew). CAD can prove geometry, not friction, wiring, or tuning.

---

See also [`06_WIRING_PLAN.md`](06_WIRING_PLAN.md) (every cable, port and length) and [`07_GAME_SCENARIO_REVIEW.md`](07_GAME_SCENARIO_REVIEW.md) (match scenarios and how each is handled).

## 1. Verified in CAD / analysis

### 1.1 Geometry gates (`cad/verify_v12_gates.py` → `cad/out/v12_gate_report.json`, re-run on the saved document)
| Gate | Result |
|---|---|
| 1 Unfiltered B-Rep overlaps | **0** across 1,808 visible bodies (no masking cutters) |
| 2 Single-piece custom bodies | 175/175 single lump, none < 0.25 cm³ |
| 3 R102 starting cube (18 in = 457 mm) | **444.0 × 443.7 × 439.5 mm** |
| 4 R105 expansion (18 × 24 × 29 in) | **464.8 × 443.7 × 439.5 mm** (intake on its upper hard stop) |
| 5 NECTAR path | Ø91.95 mm sphere clears all 10 path stations |
| 6 Floor contact | 4 wheels + 2 odometry-pod **tyres** touch the tiles, measured with tight (oriented) bounding boxes; intake wheels reach Z 53 mm. Before v12.4 this gate used loose boxes and passed pods that were 6 mm up. |
| 7 Keyholes / arm bearings | pass |
| 8 Topspin | rear 8.67 m/s > front 5.78 m/s |
| 9 Mechanism sweeps | **6 mechanisms swept through full travel, 0 collisions**: intake low→high, gecko 1/2 lift, counter-wheel, indexer gate 0→60°, G407 blocker |
| 10 Support chain | every custom part connects to the chassis through ≤ 0.15 mm contacts |
| 11 Device screw paths (v12.5) | every sensor, the main switch, the servo injector and the HuskyLens: each mounting hole has a matching hole with material around it in a printed or CNC part (2/2, 2/2, 2/2, 2/2, 3/3, 4/4, 2/2) |
| Fastening audit (v12.6, `cad/fastening_audit.py`) | every non-moving custom part has a real screw path into another part; contact alone never counts. **v111426m: 63 parts checked, 0 unfastened; 48 have real screw paths and 15 are moving parts (swing arms, gates, hood, compliant wheels) that ride bearings or REX shafts.** Whole-token classification (the old substring skip list hid 4 unscrewed parts, and the sign slide rails were only resting on the mid towers - now 2x M4 each). Custom-to-custom joints are now also part of Gate 11 (HuskyLens bracket → turret plate 5/2) |
| Production pass (v111426m) | Flywheel saddles, intake stop plates and the M7 cradle are screwed down; S1/S3/S6 drive their mechanisms through goBILDA 4001 servo-to-REX couplers; the 5 CNC plates are flat 2D (`cad/CNC/`, verified by `cad/export_cnc.py`); `cad/cots_mount_audit.py`: every COTS device has screw paths |
| Belts | 7 belts (4 HTD-5M 9 mm, 3 HTD-3M 6 mm in the magazine), all exact (0.000 mm) |

Every mechanism also stays inside R105 at its **mechanical** extreme, not just its software limit (R105.B). That includes the G407 blocker at any angle: X ≤ 291.5 mm, well inside 610 mm.

### 1.2 Rules audit (robot construction, Section 12)
| Rule | Status |
|---|---|
| R102 / R105 | ✅ see above |
| R104 weight | no limit. Estimate **12.39 kg (27.3 lb)** before fasteners and wiring (`cad/mass_cg_v12.py`, v111426m): CG 128 mm high, 43/57 front/rear axle load, 1.04 g needed to tip (more than the tyres can grip); 46 deg to tip backward |
| R203 / R204 | ✅ backdrivable 312 RPM drive (pushable unpowered); no floor-grabbing |
| R401–R403 signs | ✅ 174 × 78 mm plates (min 165 × 64) with 57.5 mm white numerals (≈ 2.25 in) and a 7.25 mm border visible inside the rail lips (min 6). One on each side, 180° apart. Red/blue plates swap on slide rails, so the opposite colour is never visible |
| R501 motors | ✅ 8 × goBILDA 5203 (Yellow Jacket 5203 series is listed) |
| R502 servos | ✅ goBILDA 2000-0025-0002 Dual Mode (25-2 torque) is listed |
| R503 counts | ✅ **8/8 motors, 3/8 servos** |
| R505 / R608 power regulation | ✅ Control Hub + Expansion Hub (1 motor per port); goBILDA 3125-0001-0001 servo injector (≤ 2 servos per port), powered **only** from the XT30 by the main battery |
| R601 battery | ⚠️ **fixed in the BOM**: it listed goBILDA `3100-0012-0030`, which is **not** on the legal list. v12.5 used **Matrix 14-0014**, which is now discontinued. **v12.6: goBILDA 12V NiMH Nested Battery 3100-0012-0020**, goBILDA's replacement (Table 12-4, manual p. 132), in goBILDA's 1209-0001-0001 battery mount (hook-and-loop strap, tool-free swaps) on the rear crossbar |
| R603 power switch | ✅ REV-31-1387 (real model), face-up on a printed housing inside a 4-sided cage on the deck (cage top Z 264 > rocker Z 261); reachable from above |
| R702 / R707 / R708 vision | ✅ HuskyLens on I2C (FTC SDK has a native driver); a UVC webcam on USB is the only USB device besides the Expansion Hub. (Limelight 3A is also legal; HuskyLens was the spec's choice) |
| R710 lasers | ✅ REV 2m Distance sensors use a Class 1 940 nm (invisible) laser |
| G407 / G408 | ✅ lip colour sensor + S1 blocker + software guard |
| G409 | ✅ 40° shed roofs keep spilled balls from resting on the deck |
| G410 | ✅ moot: no FLOWER mechanism |

### 1.3 Performance analysis
| Item | Number | Verdict |
|---|---|---|
| Drive top speed | 312 RPM, 96 mm mecanum → **1.57 m/s** | ok |
| Intake roller speed | **M7 changed to 1150 RPM** → 2.83 m/s = **1.8× drive speed**. The original 312 RPM gave 0.77 m/s, half of drive speed, and would have bulldozed balls | fixed |
| Ball path, lone ball from the intake | Whole-path simulation. A pinched ball moves at half the gecko speed. At **435 RPM a lone POLLEN stalled at Z ≈ 253, 19 mm short of the shooter**, and staging at 0.5 power couldn't even reach the gate. **M8 changed to 1150 RPM**: a lone POLLEN reaches the nip at 1.91 m/s (4.45× energy margin), and staging power is 0.6 (needs ≥ 0.37) | fixed |
| CG / stability | CG (−15.8, 1.3, **115.8**) mm; static tip angles 48–60°; it takes **1.11 g** to tip, beyond mecanum traction | ok |
| Axle loads | 44.5% front / 55.5% rear | ok for mecanum |
| Shooter reach | Exit speed needed (drag model): 4.8 m/s at 0.75 m, 5.0 at 1.0 m, 5.3 at 1.25 m, 5.5 at 1.5 m, 5.7 at 1.75 m (shooter exit to CELL lip). Available at the 1050 RPM cap is about 4.95–5.6 m/s | **reliable window 0.75–1.25 m** from the lip (hood range 62–72° also caps long shots). The robot centre should be about **1.3–1.8 m from the HIVE pivot line (sweet spot ≈ 1.55 m / 61 in)**. Auto launches from D ≈ 1.07 m |
| Printability | 60 print files (one .3mf per printed part, the 4 HTD pulleys in `pulleys/`, red + blue multi-colour signs), all watertight and single-body (`cad/print_audit.py`). All fit a 256 mm bed except the J-chute, which now has a **4-piece split kit** (`cad/3MF/v12/_chute_split/`, every piece ≤ 148.7 mm, volume conserved exactly). Each piece carries its own frame mounting station | fixed |

### 1.4 Starting configuration (G304 / §10.3.4)
* **3 preload POLLEN in the magazine, the 4th on the tiles touching the front funnel** (legal: preloads may sit on the tiles contacting the robot).
  * A 4th POLLEN inside the robot would lift the floating intake until the robot is **456.1 mm long against the 457 mm limit**, 0.9 mm of margin, and balls aren't perfectly round.
* The auto runs the intake during LEAVE to pick up the 4th, then fires all 4 for tip #1. 3 NECTAR + 4 POLLEN = 223.6 g, above the 199 g must-tip line; with only 3 POLLEN it's 198.7 g, which is not guaranteed.

### 1.5 Deliberate design decisions
* **No FLOWER capper** (v12.2): an extra HIVE tip (20 pts + RP progress) beats a cap (about 7 pts, 0 RP).
* **No de-pollinator.** The RPs are alliance-wide tip counts for the whole match (Pollinator 1 = 4 tips, Pollinator 2 = 7, Swarm = LEAVE + PARK ≥ 16), and every tip spills its balls back onto the field. **Ball supply recirculates, so the limit is cycle time, not FLOWER POLLEN.**
  * POLLEN has to be lifted over a 0.4 in ring through a 3.55 × 3.57 in opening, and our floating intake can't reach it.
  * A rake would be a new, untested mechanism in the most crowded part of the robot. Revisit only if scouting shows auto tips are starved for balls.
* **Two geckos, not three**: no third position fits (§6.1 of doc 02).

---

## 1b. How every custom part is fastened (v12.3)

`cad/drill_fastener_holes.py` runs after the builder, before the gates. It transfer-drills every COTS hole a custom part covers, and adds M3 joints wherever two custom parts share a flat face. It never drills into the ball path or the 102 mm corridor walls.

**Totals:** 131 drilled holes in 24 of 26 structural parts, plus 8 modelled in the builder (S3 ear inserts and the gearbox-sleeve screws).
* 57 × Ø5.6 (M4 heat-set insert; goes straight through on thin parts, so use an M4 bolt + nyloc there)
* 45 × Ø4.0 (M3 insert)
* 29 × Ø3.4 (M3 clearance)

Gates 1–10 re-pass with the holes in.

| Part | Holes | How it is held |
|---|---|---|
| `cnc_turret_mounting_deck_plate_clean` | 25 | Bolts to the chassis channels (M4 through + nyloc); also takes the shroud flanges, G409 roofs and switch cage |
| `unified_continuous_j_chute` | 15 | M4 inserts at its 4 mounting stations into the chassis channels; split kit pieces bonded with epoxy first |
| `turret_side_plate_right_clean` | 9 | M4 to the deck + double-shear plate and HuskyLens bracket joints |
| `s3_indexer_servo_bracket` | 8 | **S3's mount**: base flange M4 into the mid cross-channel top web; 4 M3 inserts for S3's ears |
| `rev_expansion_hub_mount_plate_left` | 8 | M4 to the channels; hub screws to the plate with its own M3 pattern |
| `s1_servo_guard_cage` | 8 | **S1's mount**: S1 ears into the cage ribs (M3 inserts); the tab bolts through the upright's front wall (M4) |
| `turret_side_plate_left_clean` | 5 | M4 to the deck + double-shear plate joints |
| `turret_servo_cradle_s6_hood` | 5 | M3 to the right side plate; **S6 ears screw into its two posts** (M3 inserts) |
| `magazine_outboard_double_shear_plate_right` | 4 | Square sleeve over the M8 gearbox, 4× M4 SHCS into the gearbox's tapped side holes |
| `m7_intake_motor_clamp_saddle_left` | 4 | M4 to the rail top web |
| `m8_magazine_motor_clamp_saddle_right` | 4 | M4 to the rail top web |
| `odom_pod_center_y_strafe_rear_cradle` | 4 | M4 to the rear crossbar. Clamps both ends of the swingarm pod's 8 mm REX pivot shaft; the pod swings freely |
| `gobilda_12v_battery_cradle_tray` | 4 | M4 to the rear channels |
| `rev_control_hub_mount_plate_right` | 4 | M4 to the channels; hub screws to the plate with its own M3 pattern |
| `asymmetrical_funnel_wing_left_long` | 4 | Tab on the side rail's top web, M4 |
| `asymmetrical_funnel_wing_right_short` | 4 | Tab on the side rail's top web, M4 |
| `sprung_magazine_shroud_left` | 3 | Top flange screws up into the deck (M3 inserts in the flange, clearance in the deck) |
| `sprung_magazine_shroud_right` | 3 | Top flange screws up into the deck (M3 inserts in the flange, clearance in the deck) |
| `dfrobot_huskylens_mount_block` | 2 + joints | 2× M3 inserts for the HuskyLens ears; M3 joints to the turret double-shear plate |
| `rev_main_power_switch_housing` | 3 + joints | 3× M3 inserts for the switch plate; M3 joints to the deck |
| Shroud / chute sensor pads | 2 per sensor | 2× M3 inserts per sensor, stopping at the wall (built into `sprung_magazine_shroud_left` and `unified_continuous_j_chute`) |
| Deck (servo injector) | 4 | 4× **tapped M4** in the aluminium deck for the injector base |
| `main_power_switch_impact_guard_cage` | 3 | Inboard ear, M3 into the deck (the far side is captured under the G409 roof) |
| `g409_sloped_deck_shed_shield_left` | 3 | M3 joints along its base rails into the deck |
| `g409_sloped_deck_shed_shield_right` | 3 | M3 joints along its base rails into the deck |
| `turret_outboard_double_shear_plate_left` | 2 | M3 joint to the side plate (plus the shared bearing) |
| `turret_outboard_double_shear_plate_right` | 2 | M3 joint to the side plate (plus the shared bearing) |
| `odom_pod_left_x_rail_cradle` | 4 | M4 to the left rail inner wall. Clamps both shaft ends, as above |
| `drive_motor_quad_block_mount_*` (×4, goBILDA 1201-0043-0002) | 4 each | M4 to the rail walls; 4× M4 into each drive motor's face |
| `gobilda_12v_battery_hold_down_strap` | 0 | Hook-and-loop strap through the tray (intentional, so the battery swaps fast) |

**Motors and pods (v12.4).** The drive motors had no mount and the pod cradles sat about 18 mm above the pods; both are fixed. The front axles moved out one hole (X 168) so the motors on each side have a 24 mm gap for their cables.

**Servos.** Before v12.3, S1 and S3 hung on their own output splines and S6 rested 0.2 mm above its cradle. All three now screw down through their ears. Each support keeps clear of the servo's cable-exit seal, and the S1 cage's lead slot lines up with it.

## 2. Must be verified on the real robot (pit crew checklist)

### 2.1 Build
- [ ] Print the J-chute (whole on an H2D, or the 4-piece kit). Bond the seams flush and sand any lip > 0.3 mm. **Roll a NECTAR and a POLLEN through the whole path by hand.**
- [ ] PETG-CF parts need a hardened nozzle. Print TPU wheels at 95A.
- [ ] Every belt is designed at its exact nominal length with no tensioner. **Check each belt for tooth skip under load.** If one skips, a printed 1–2 mm idler bump is the fix.
- [ ] Gecko 1: confirm both hub bearings are seated and the shaft has no visible wobble.
- [ ] Indexer gate: rotate S3 through +60° by hand. All 3 fingers must drop below the floor with no rubbing in the slits.
- [ ] Intake: lift the floating arms to the upper hard stop by hand. The wheels must not touch gecko 1 anywhere in that travel.
- [ ] Weigh the robot. The estimate is 12.4 kg + fasteners/wiring (~0.4 kg).

### 2.2 Electrical (inspection items)
- [ ] Legal battery (goBILDA 3100-0012-0020; its mount is profile-matched, so an old 45.4 mm-wide Matrix 14-0014 will NOT fit it), 20 A fuse intact, **3+ charged packs** for a match day.
- [ ] Main switch REV-31-1387 wired so it cuts **all** power. Reachable, and away from the shooter belts.
- [ ] 12 V main power ≥ 18 AWG; red/black (or the colours R610 allows) on the 12 V and 5 V buses.
- [ ] Servo injector powered only from the battery via XT30; ≤ 2 servos per port.
- [ ] No frame current paths (R605): check that the Control Hub and Expansion Hub are electrically isolated from the aluminium.
- [ ] Control Hub Wi-Fi password changed from the default; Bluetooth off (R711).
- [ ] Wire strain relief at the floating-intake pivot and on the gecko arm motor leads, which move.

### 2.3 Software / tuning
- [ ] Enter the Robot Configuration exactly as listed in §3 (names are case-sensitive).
- [ ] Run **HIVE-QUEEN Hardware Test** and check every motor direction, servo range and sensor reading.
- [ ] Set the servo end-points on the robot: S1 blocker stowed/deployed, S3 gate closed/open (+60°), S6 hood limits.
- [ ] Pinpoint: confirm the pod offsets and encoder directions (push the robot forward and left; X and Y must increase).
- [ ] Flywheel: tune kS / kV / kP until the at-speed window holds within ±30 RPM between shots.
- [ ] **Shot table:** fire 10 POLLEN and 10 NECTAR at D = 0.75, 1.0 and 1.25 m from the CELL lip; record RPM, hood and hit rate. Expect POLLEN and NECTAR to need different settings.
- [ ] HuskyLens: learn the HIVE AprilTags (IDs per the table in `RobotConstants`).
- [ ] Auto: run the full 30 s routine 10 times from both alliance sides. It **must park by 30 s every time**, even if a step fails.

### 2.4 Pre-match pit routine
- [ ] Battery > 13.0 V at rest.
- [ ] Correct alliance sign plates installed (red or blue).
- [ ] 3 POLLEN loaded in the magazine; 4th placed touching the front funnel on the field.
- [ ] Select the correct Auto OpMode (Red/Blue), 30 s timer enabled, then INIT. **Nothing moves during INIT.**

---

## 3. Robot Configuration (Control Hub) — names must match `RobotConstants`

| Device name | Type (Robot Configuration) | Hub / port |
|---|---|---|
| `fl` | goBILDA 5202/3/4 series motor | Expansion Hub motor 0 |
| `bl` | goBILDA 5202/3/4 series motor | Expansion Hub motor 1 |
| `shooterFront` | goBILDA 5202/3/4 (1150) | Expansion Hub motor 2 |
| `intake` | goBILDA 5202/3/4 (1150) | Expansion Hub motor 3 |
| `fr` | goBILDA 5202/3/4 series motor | Control Hub motor 0 |
| `br` | goBILDA 5202/3/4 series motor | Control Hub motor 1 |
| `shooterRear` | goBILDA 5202/3/4 (1150) | Control Hub motor 2 |
| `magazine` | goBILDA 5202/3/4 (1150) | Control Hub motor 3 |
| `lipGate` | Servo | Expansion Hub servo 0 (power via the 6 V injector) |
| `indexerGate` | Servo | Expansion Hub servo 1 (power via the 6 V injector) |
| `hoodPitch` | Servo | Expansion Hub servo 2 (power via the 6 V injector) |
| `pinpoint` | goBILDA Pinpoint Odometry Computer | Control Hub I2C 1 |
| `huskylens` | HuskyLens | Control Hub I2C 2 |
| `lipColor` | REV Color Sensor V3 | Expansion Hub I2C 0 |
| `throatColor` | REV Color Sensor V3 | Expansion Hub I2C 1 |
| `ballCount1` | REV 2m Distance Sensor | Expansion Hub I2C 2 |
| `ballCount2` | REV 2m Distance Sensor | Expansion Hub I2C 3 |
| `Webcam 1` | Webcam (optional, not modelled) | USB |

Ports follow the **side-by-side wiring plan** in [`06_WIRING_PLAN.md`](06_WIRING_PLAN.md): left-side devices go to the Expansion Hub, right-side to the Control Hub, which keeps every cable at or under 0.6 m.

The two Color V3s and the two 2m Distance sensors each have a fixed I2C address, so **each one must be on its own I2C bus** (as above). Names are case-sensitive.

**Offline logic self-test** (no robot needed): see the header of `TeamCode/src/test/java/org/firstinspires/ftc/teamcode/hivequeen/HiveQueenLogicSelfTest.java`. Result: **267 passed, 0 failed**. It covers match-clock lockouts, the 4-ball/opponent-NECTAR guard, shot-table clamping, wheel speeds, ball classification, the auto park deadline, heading wrap and the Blue 180° rotation. `assembleDebug` builds clean.

---

## 4. Known risks — test these first on the real robot
1. **Ball counting at speed.**
   * The entry counter (`ballCount1`, REV 2m Distance) can take ~20–30 ms per read. A ball passes it in ~40 ms, so a slow loop can miss one. The "4th slot full" sensor is now read only every 3rd loop to save time.
   * **Test:** intake 4 balls at full speed 20 times; the count must be exact.
   * If it misses, move the entry counter to a REV Color Sensor V3 (fast proximity read) or slow the magazine while intaking.
   * Touching balls with no gap between them can also merge into one count; the 4th-slot sensor corrects the count upward to 4.
2. **Long shots.** The hood's CAD range is about 62–72° elevation, and shots beyond ≈ 1.25 m need 56–60°. The driver warning therefore shows **OUT-OF-RANGE outside D = 0.6–1.3 m**. Shoot from the 0.75–1.25 m window.
3. **Tip tracking is an estimate.** It counts launched mass and doesn't see misses. Vision prefers the up CELL's tag cluster to avoid shooting a downward CELL (G417). Watch for this in practice.
4. **Lip colour UNKNOWN is let in.** In bad lighting an opponent NECTAR could be pulled in (G408). Tune the hue windows under the event lighting, and check the throat classifier rejects it.
5. **Field coordinates are from the manual's figures, not measured.** Every auto pose is a TUNE value; drive the routine on a real field before the event.
6. **TeleOp without a preceding auto:** odometry range is only valid if the drivers press BACK with the robot facing away from them. Vision is the main range source.
7. **Burst spacing.** At full feed power, queued balls reach the shooter about 35 ms apart, which may drain flywheel speed during a burst. If later shots fall short, lower `MAGAZINE_FEED_POWER` (never below 0.6) or pulse the feed.
8. **Belts have no tensioners** (exact nominal lengths). Watch for tooth skip on the shooter belts first, since they carry the highest load.
