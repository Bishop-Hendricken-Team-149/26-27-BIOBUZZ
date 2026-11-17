# 07 — HIVE-QUEEN v12.2 Game Scenario Review

Every situation the robot meets in a BIOBUZZ match, what the robot does about it, how that was verified, and what still has to be proven on a real field.

**Verification key**
* **CAD** = Fusion gates
* **SIM** = physics / ballistics simulation
* **TEST** = offline logic self-test (`HiveQueenLogicSelfTest`, 167/167)
* **CODE** = code review
* **FIELD** = must be tested on the robot

## A. Setup and autonomous
| # | Scenario | What the robot does | Verified | Still to test |
|---|---|---|---|---|
| A1 | Inspection sizing | Stowed **≤ 448 mm** wide (with the S1 guard) × 444 long × 440 tall. 3 POLLEN inside; the 4th waits on the tiles, so the floating intake isn't lifted | CAD (R102), SIM (4th ball → 456.1 mm) | Sizing box at inspection |
| A2 | INIT | Nothing moves (servos are only set at START, G403) | CODE, TEST | Watch it during INIT |
| A3 | Partner robot needs the same start spot | INIT menu: **slot A/B** (dpad L/R), **delay 0–10 s** (dpad U/D), **mode** FULL / PRELOADS_THEN_PARK / LEAVE_AND_PARK (X). Park targets split by slot (480 mm apart), so partners don't collide | CODE, build | Agree on slots with each partner at the event |
| A4 | 4th preload not picked up | Fires 3 anyway (198.7 g, tip uncertain), sweeps the spill area; finding balls there confirms the tip | CODE | Run it 10× |
| A5 | HuskyLens unplugged / no tags | Aims by Pinpoint odometry bearing. Telemetry shows HuskyLens NOT FOUND | CODE | Unplug-test |
| A6 | Running late | **Hard abort at 24 s** → park; drive cut at 29.5 s; G403 stop at 30 s regardless | TEST (park deadline), CODE | 10 full runs, both alliances |
| A7 | Blue alliance | Red path rotated 180° (the field is rotationally symmetric, not mirrored) | TEST | One run each side |
| A8 | Auto tip tracking wrong (missed shots) | The tracker estimates by launched mass; vision prefers the **up** CELL's tag cluster, so it won't fire into a down CELL | CODE | Watch for it |

## B. Collecting
| # | Scenario | What the robot does | Verified | Still to test |
|---|---|---|---|---|
| B1 | Intake at full drive speed | Rollers at **2.83 m/s = 1.8× drive speed** (M7 1150 RPM) | SIM | Drive through a ball line at full speed |
| B2 | Tile seams / uneven tiles | Lip **5 mm**, wings **6 mm** off the tiles (were 1.5 / 2 mm, which would snag) | CAD | Drive over seams |
| B3 | NECTAR (92 mm) in the mix | Floating intake rises on its arms; geckos lift on springs; NECTAR clears every station of the path | CAD (gate 5, 9) | Mixed feeds |
| B4 | **A lone ball** from the intake | Reaches the shooter: M8 1150 RPM; pinched ball at 2.17 m/s; coasts **239 mm across the 48.6 mm gap**. 435 RPM stalled a lone POLLEN | SIM | 10× single POLLEN, 10× single NECTAR |
| B5 | Opponent NECTAR at the mouth (G408) | Lip colour sensor → S1 blocker drops + roller reverses. **UNKNOWN colour is let in** | TEST, CODE | Tune hue windows under event light |
| B6 | 5th ball (G407) | At 4: blocker down + 250 ms reverse pulse; the 4th-slot sensor corrects the count up | TEST | Feed 6 balls |
| B7 | **Ball jam** in the intake or magazine | Encoder stall 350 ms → reverse pulse → retry. 3 jams in 5 s → 1.5 s cooldown; telemetry `jam:`. Needs M7/M8 encoder cables (auto-disables with `NO_ENCODER` if missing) | TEST | Jam a ball on purpose |
| B8 | **Dead ball-count sensor** (unplugged / 0 mm) | Treated as a FAULT, never as "ball present", so it can't fake "full" and lock the intake. Telemetry tells the gunner to count by hand (gamepad2 X = 0, B = 4) | TEST | Unplug-test |
| B9 | Balls touching each other through the entry sensor | May merge into one count; the 4th-slot sensor forces the count to 4 when full | CODE | 20× fast 4-ball intakes |
| B10 | Ball on the tiles under the robot | Can't happen: lowest structure is 20 mm, POLLEN is 71 mm, so the robot pushes it | CAD | — |

## C. Shooting and tipping
| # | Scenario | What the robot does | Verified | Still to test |
|---|---|---|---|---|
| C1 | Distance window | **Reliable 0.75–1.25 m from the CELL lip**; telemetry warns OUT-OF-RANGE outside 0.6–1.3 m. The hood's CAD range (62–72°) caps long shots | SIM, TEST | Shot table at 0.75 / 1.0 / 1.25 m |
| C2 | POLLEN vs NECTAR | Throat classifier picks the setpoint before the gate opens | CODE | Tune each ball type |
| C3 | Burst | Gate latches open while fire is held; full feed sends balls about 35 ms apart | CODE | If late shots drop, lower feed power (≥ 0.6) |
| C4 | Flywheel spin-up while driving | **Power ramp below half speed** (0 → 1 in ~0.4 s) to avoid brownouts | TEST (slew) | Watch battery voltage on telemetry |
| C5 | Low battery | Feedforward scales with 12.5 V / Vbatt | CODE | Swap packs under 12.5 V |
| C6 | HIVE tipping while still shooting | The manual says it may disrupt the tip. Auto waits 1.5 s after a tip; drivers should pause in TeleOp | CODE | Driver practice |
| C7 | Robot bumped while aiming | Heading lock re-aims on the tag cluster every loop | CODE | — |

## D. Contact and field hazards
| # | Scenario | What the robot does | Verified | Still to test |
|---|---|---|---|---|
| D1 | Side hit from another robot | **New S1 guard cage**: the servo sat flush with the wheel face, now 4 mm of printed guard takes the hit. M7/M8 motor ends are shielded by the wheels | CAD | — |
| D2 | Front into the wall | Funnel wings (X 228) touch first and protect the intake wheels (X 222.4) | CAD | — |
| D3 | Tipping into other robots / wheel over a ball | CG 116 mm high; tip angles 48–60°; it takes 1.11 g to tip | SIM | — |
| D4 | Balls raining from a tipped HIVE (G409) | 40° shed roofs on the deck, so nothing rests on top | CAD | — |
| D5 | Near or under the HIVE | 440 mm tall; the down CELL's mouth is far above (pivot 1116 mm). Light frame contact is legal (G417 example G) | Manual | Don't ram the frame |
| D6 | Main switch knocked | Upward-facing in a 4-sided cage on the deck | CAD | — |
| D7 | Expansion Hub link lost | Everything on the EH stops (left drive pair, M5, M7, sensors) | — | Strain-relieve the RS485 cable; check it before every match |

## E. End of match
| # | Scenario | What the robot does | Verified |
|---|---|---|---|
| E1 | 0:00 | All powered motion stops (G404); flywheels coast, never actively braked | TEST, CODE |
| E2 | TeleOp started late after auto | TeleOp gets its full 2:00 from its own START; the 8 s transition lockout is still enforced | TEST |
| E3 | Balls left in the robot | No penalty. Balls left in an upward CELL still score 2 each, so fire everything before the end | — |

## F. Known limits (accepted, not bugs)
* **Field coordinates are from the manual's figures.** Every auto pose is TUNE until driven on a real field.
* **Ball counting** can be fooled by touching balls or slow loops; the full-slot sensor and gamepad2 X/B are the backups.
* **No de-pollinator and no capper.** RPs come from tips, and ball supply recirculates through spills.
* **No webcam is mounted** in CAD; HuskyLens is primary vision and the code treats the webcam as optional.
