# 06 — HIVE-QUEEN v12.6 Wiring Plan (CAD v111426m)

All positions come from the CAD (`cad/buildability_probe.py` → `cad/out/v12_buildability_probe.json`). Cable lengths are estimated as routed (Manhattan) length × 1.15 + an 80 mm service loop, then rounded up to the next standard cable.

**Rule of thumb used:** devices on the robot's left (+Y) go to the **Expansion Hub** (left bay), devices on the right go to the **Control Hub** (right bay). The code only uses device **names**, so ports can change freely. Just keep the Robot Configuration names exact.

## 1. Power (R601, R603, R605, R608, R609, R610)
| Run | From → to | Cable |
|---|---|---|
| Main | Battery (rear bay, goBILDA 12V NiMH Nested Battery 3100-0012-0020, XT30, 20 A fuse; plugs straight into the switch cable, no Tamiya adapter) → **REV-31-1387 main switch** (deck, rear-left cage) | REV switch cable, 500 mm, ≥ 18 AWG |
| Hub 1 | Switch → **Expansion Hub** XT30 #1 | XT30 50 cm (REV-31-1394-PK2) |
| Hub 2 | Expansion Hub XT30 #2 → **Control Hub** XT30 #1 | XT30 50 cm (REV-31-1394-PK2) |
| Servo power | Control Hub XT30 #2 → **goBILDA 3125 servo injector** XT30 | XT30 50 cm (REV-31-1394-PK2). The injector may only be powered from the battery via XT30 (R608); the hub's XT30 pass-through qualifies |
| Link | Control Hub ↔ Expansion Hub RS485 | REV RS485 50 cm (REV-31-1418-PK2) |

* Red = +12 V, black = ground on every 12 V run (R610). No connection to the aluminium frame anywhere (R605): mount both hubs on the printed plates, which insulate them, and check with a multimeter that frame-to-ground is open.
* The switch is the only path from the battery (R603). Keep the battery leads clear of the left shooter belt (lane `Y 72.6–84.6` since v12.6).

## 2. Motors (power + encoder; 8/8 ports used)
| Device | Name | Port | Est. run | Cable |
|---|---|---|---|---|
| M1 drive, front-left | `fl` | **Expansion Hub motor 0** | 380 mm | 500 mm |
| M2 drive, rear-left | `bl` | **Expansion Hub motor 1** | 243 mm | 300 mm |
| M5 shooter front (left) | `shooterFront` | **Expansion Hub motor 2** | 293 mm | 300 mm |
| M7 intake (left) | `intake` | **Expansion Hub motor 3** | 276 mm | 300 mm |
| M3 drive, front-right | `fr` | **Control Hub motor 0** | 380 mm | 500 mm |
| M4 drive, rear-right | `br` | **Control Hub motor 1** | 243 mm | 300 mm |
| M6 shooter counter (right) | `shooterRear` | **Control Hub motor 2** | 338 mm | 500 mm |
| M8 magazine (right) | `magazine` | **Control Hub motor 3** | 276 mm | 300 mm |

* **Encoder cables are required** on M5/M6 (flywheel velocity control) and on **M7/M8** (jam detection; it disables itself and shows `NO_ENCODER` if a cable is missing). Drive encoders are optional, since the Pinpoint does odometry.
* **Drive motor cables (M1–M4).** Each 5203's power cable and encoder port leave *radially* from the side of the rear (encoder) cap. Inside the Strafer rails they face **down, out of the rail's open bottom**.
  * Since v12.4 the front motors sit 24 mm forward of the rear ones. v12.6 moved every drive motor 2.4 mm away from its axle for the real 2315 miter pinion, so the gap between the caps is now **19.2 mm** at X ≈ 2.4–21.6 on each side. Both cable bundles (2× power + 2× encoder) drop out of the rail there; that is enough for the four leads side by side, but route each motor's leads out of its own side of the gap and don't add connectors inside the rail.
  * Zip-tie them up against the rail's lower lips, then run them inboard under the mid cross-channel and up to the hubs.
  * Keep them above Z ≈ 15 mm: the chassis has about 24 mm of ground clearance under the rails.
  * The drive runs `RUN_WITHOUT_ENCODER` with Pinpoint odometry, so the drive encoder cables can be left out to reduce bulk.
* **No motor sits on a moving part.** M7 and M8 are fixed and drive the floating intake and gecko arms through coaxial pivots, so there are no flexing cable runs.

* **Motor cable kit (each of M1–M8):** the 5203's 3.5 mm bullet leads → **3802-1213-0300** 300 mm bullet extension → **3801-0613-0100** bullet-to-JST-VH adaptor (100 mm) → hub motor port. Encoder: **3801-0709-0300** (JST-PH to XH, runs ≤ 300 mm) or **-0600** (M1, M3, M6). The fuse in the Nested battery is a 20 A **mini** blade (ATM), not ATO.

## 3. Servos (3/8; powered through the 6 V injector, ≤ 2 servos per port per R505)
| Device | Name | Signal from | Power via | Est. run | Cable |
|---|---|---|---|---|---|
| S1 G407 lip blocker (left upright) | `lipGate` | Expansion Hub servo 0 | injector out 1 | 474 mm | 600 mm extension (3802-1718-0600) |
| S3 indexer gate (under the bend) | `indexerGate` | Expansion Hub servo 1 | injector out 2 | 186 mm | 300 mm extension (3802-1718-0300) |
| S6 hood (top rear, right) | `hoodPitch` | Expansion Hub servo 2 | injector out 3 | 682 mm | **900 mm extension** (3802-1718-0900; goBILDA sells no 1 m) |
| Injector signal inputs | — | Expansion Hub servo 0–2 → injector in 1–3 | — | 234 mm | 300 mm (3802-1718-0300 x3) |

* S1, S3 and S6 are all fixed mounts. Only the arms they drive move, so there are no flexing servo leads.

## 4. Sensors (I2C; every sensor type on its own bus because the addresses are fixed)
| Device | Name | Port | Est. run | Cable |
|---|---|---|---|---|
| Pinpoint v2 | `pinpoint` | **Control Hub I2C 1** | 237 mm | 300 mm JST-PH 4-pin |
| HuskyLens (shooter, front-right) | `huskylens` | **Control Hub I2C 2** | 508 mm | 100 cm REV-31-1409 in a service loop; **re-pin it** - HuskyLens' Gravity 4-pin order is not REV's |
| Lip Color V3 (left, front) | `lipColor` | **Expansion Hub I2C 0** | 474 mm | 50 cm REV-31-1408 |
| Throat Color V3 (left shroud) | `throatColor` | **Expansion Hub I2C 1** | 176 mm | 30 cm REV-31-1407 |
| Ball count 1 = queue slot 2, 2m Distance (left shroud) | `ballCount1` | **Expansion Hub I2C 2** | 292 mm | 30 cm REV-31-1407 |
| Ball count 2 = queue slot 3, 2m Distance (left shroud) | `ballCount2` | **Expansion Hub I2C 3** | 400 mm | 50 cm REV-31-1408 |
| Odometry pod X (left) → Pinpoint X | — | Pinpoint | 492 mm | 600 mm JST-PH to XH (3801-0709-0600) |
| Odometry pod Y (rear centre) → Pinpoint Y | — | Pinpoint | 408 mm | 600 mm JST-PH to XH (3801-0709-0600) |

* All four ball sensors look through the **left** shroud windows, so they all go to the left Expansion Hub, which keeps the runs short.
* Control Hub I2C bus 0 holds the internal IMU. The Pinpoint goes on bus 1 to keep that bus clean.
* Keep I2C cables ≤ ~0.5–1 m and away from motor power leads. If a sensor drops out, shorten or re-route it before suspecting the code.
* No webcam is modelled. The code treats it as an optional fallback, and HuskyLens is the primary vision.

## 5. Routing rules
1. Every run is clipped to structure every ~80 mm. Nothing crosses a belt plane: `|Y|` 56–64 (intake stage 1, gecko b1/b3), 64.5–78.5 (gecko arms, b2), 72–80 (shooter belts), or the intake swing-arm sweep at `|Y|` 100–108.
2. Keep cables out of the 102 mm ball corridor (`|Y| < 51`) and off the deck top: the G409 shed roofs must stay clear.
3. Leave a service loop at each hub so it can be pulled out for inspection (R606).
4. Label both ends of every cable with its configuration name.
