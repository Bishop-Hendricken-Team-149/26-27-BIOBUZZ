# 04 — Master Handoff Specification for Opus 5.5: Full Remediation of Project HIVE-QUEEN (`HIVE QUEEN v111326`) via Live Fusion 360 API

**Prepared for:** Autonomous Execution by Claude Opus 5.5 inside Hermes Agent Desktop  
**Repository Root:** `C:\Users\danu0\Downloads\26-27-BIOBUZZ`  
**Source Cloud CAD Document (DO NOT OVERWRITE IN-PLACE — SAVE AS NEW VERSION/FILE):**  
`BHHS FTC 149` (Hub) $\rightarrow$ `Default Project` $\rightarrow$ `FTC 26-27` $\rightarrow$ `HIVE QUEEN v111326` (`urn:adsk.wipprod:dm.lineage:J2r_l724TBCVFA8N0DO7Xg`)  
**Target Output Cloud Document Name:** `HIVE QUEEN v111426` ( inside `BHHS FTC 149/Default Project/FTC 26-27` ) + local export `C:\Users\danu0\Downloads\26-27-BIOBUZZ\cad\STEP\project_hive_queen_robot_v12.step`  
**Competition Manual Authority:** `C:\Users\danu0\Downloads\26-27-BIOBUZZ\BIOBUZZ_Competition_Manual_TU07.pdf` (Version `TU07`)  
**Team Shop Inventory Authority:** `C:\Users\danu0\Downloads\26-27-BIOBUZZ\cad\Inventory(Motion).csv`

---

## 0. Mission Brief for Opus 5.5

You are inheriting a complete, empirically verified mechanical, strategic, and rules audit of FTC Team 149's 2026–2027 BIOBUZZ robot (**Project HIVE-QUEEN**). Your job is to **autonomously execute all 5 remediation phases**:

1. **CAD Upgrade in Live Autodesk Fusion 360 (`HermesFusionBridge` on `http://127.0.0.1:18149`):** Fix all 7 showstopper mechanical bugs and 6 FTC rulebook vulnerabilities by upgrading `C:\Users\danu0\Downloads\26-27-BIOBUZZ\cad\run_full_robot_upgrade.py` (and `cad/src/printable_parts.py` where needed), running it on a clean baseline import (or cleaned copy of `HIVE QUEEN v111326`), verifying **100% pass on all quantitative B-Rep & kinematic gates**, visually inspecting multi-angle orthographic & cutaway renders, saving to the Fusion cloud folder `BHHS FTC 149/Default Project/FTC 26-27` as **`HIVE QUEEN v111426`** (preserving `HIVE QUEEN v111326` untouched!), and exporting `.step` + `.3mf` printables.
2. **BOM & Shop Inventory Reconciliation:** Regenerate `C:\Users\danu0\Downloads\26-27-BIOBUZZ\cad\out\HIVE_QUEEN_149_BOM_CHECKLIST.csv` (`utf-8-sig`) and `.xlsx` so 100% of COTS & custom parts match the upgraded CAD and `cad/Inventory(Motion).csv`.
3. **Strategy & Engineering Docs Sync:** Update `BIOBUZZ_Robot_Design_and_Strategy_Guide.md`, `docs/01_ARCHITECTURE_AND_GAME_STRATEGY.md`, and `docs/02_SUBSYSTEM_ENGINEERING_AND_DESIGN_CHOICES.md` to match the real v12 geometry, corrected HIVE tipping mass math (`>= 206 g`), and updated renders.
4. **FTC SDK v12.0 `TeamCode` Implementation:** Scaffold clean, compilable robot control code under `C:\Users\danu0\Downloads\26-27-BIOBUZZ\TeamCode\src\main\java\org\firstinspires\ftc\teamcode\` (and `TeamCode/build.gradle`) implementing `Pinpoint v2` 2-pod localization, `HuskyLens` / `AprilTagCluster` targeting, dual-ball voltage-compensated topspin flywheel control, `G407`/`G408` front-lip auto-rejection, `G403`/`G404` transition-motionless safety gates, `G410` 60s Flower capper lockout, and `R704.D` competition Wi-Fi telemetry compliance, verified with `./gradlew assembleDebug`.

> **CRITICAL SINGLE-THREADED CAD RULE:** Do **NOT** spawn parallel subagents that talk to `http://127.0.0.1:18149` simultaneously. Fusion 360's C++ kernel runs on a single main UI thread; all CAD operations must be executed sequentially by ONE agent. (If you delegate anything, only delegate independent non-CAD work like `TeamCode` Java scaffolding or BOM formatting after locking the part list.)

---

## 1. Live Fusion 360 Environment & Hard Operational Rules (Read First — Saves Hours!)

### 1.1 Exact Paths & Launching Fusion 360 Persistently on Windows
* **Fusion 360 Executable:**
  `C:\Users\danu0\AppData\Local\Autodesk\webdeploy\production\39e8a3d5e919c70b05ef3ebfe9c6bfc8d9ee001e\Fusion360.exe`
* **Bridge Add-In (auto-starts on Fusion launch):**
  `%APPDATA%\Autodesk\Autodesk Fusion 360\API\AddIns\HermesFusionBridge\HermesFusionBridge.py` (`http://127.0.0.1:18149` — endpoints: `GET /status`, `POST /exec`, `POST /screenshot`)
* **Repo CLI Helper:**
  `C:\Users\danu0\Downloads\26-27-BIOBUZZ\cad\fusion_cli.py`
* **Clean Baseline STEP (for fast 1.0s `BaseFeature` rebuilds):**
  `C:\Users\danu0\Downloads\26-27-BIOBUZZ\cad\STEP\project_hive_queen_robot.step`

#### Windows Process & Modal Pitfalls (Empirically Proven)
1. **Launch `Fusion360.exe` ONLY via `terminal(..., background=True)`:**
   - Never launch `Fusion360.exe` inside a foreground `terminal()` call (even via `powershell Start-Process`). On Windows, Hermes wraps foreground `terminal()` calls in a Job Object that kills all child processes as soon as the foreground tool call exits! Always start Fusion with:
     `terminal(command='"C:/Users/danu0/AppData/Local/Autodesk/webdeploy/production/39e8a3d5e919c70b05ef3ebfe9c6bfc8d9ee001e/Fusion360.exe"', background=True)`
2. **Wake Unfocused Qt Loop (`WM_NULL`) & Dismiss Startup Modals WITHOUT `WM_CLOSE` on the Main Window:**
   - When Fusion 360 is unfocused, Qt's Win32 dispatcher sleeps until `PostMessageW(hwnd, 0x0000, 0, 0)` (`WM_NULL`) is sent to windows with `"Fusion"` or `"Autodesk"` in the title (do **not** filter by `IsWindowVisible` for `WM_NULL`, as Qt's internal dispatcher window is hidden).
   - **CRITICAL:** Never send `WM_CLOSE` (`0x0010`) to `cls == "Qt683QWindowIcon" and title == "Fusion360"`! Once any modal closes, the main Fusion window matches that filter before a document title loads, causing `WM_CLOSE` to close Fusion or pop a blocking `"Unsaved changes"` modal that freezes `/status` and `/exec`! Only send `VK_ESCAPE` (`0x1B`) / `VK_RETURN` (`0x0D`) to startup/recovery modals:
   ```python
   if user32.IsWindowVisible(hwnd):
       if "Recovered" in title or (cls == "Qt683QWindowIcon" and title == "Fusion360"):
           user32.PostMessageW(hwnd, 0x0100, 0x1B, 0)  # VK_ESCAPE only — NEVER WM_CLOSE!
   ```
   - In git-bash on Windows, MSYS path conversion is disabled: if you ever need `taskkill`, pass single-slash `taskkill /F /IM Fusion360.exe` (never `//F`).
3. **Never Call `vp.refresh()` at the End of a Geometry `/exec` Call & Never Run `measureMinimumDistance` on 1,072-Face NURBS Channels:**
   - Calling `app.activeViewport.refresh()` at the end of `/exec` triggers an async Qt redraw that drops the next HTTP request's `fireCustomEvent`. Only call `vp.refresh()` inside dedicated screenshot calls, or omit it before returning.
   - Never call `app.measureManager.measureMinimumDistance(b1, b2)` where either body is an imported 1,000+-face COTS NURBS U-channel (`gobilda_1120_*`) or multi-body pod—it runs $O(F_1 \times F_2)$ NURBS distance synchronously and stalls for `>240 s`. Use `TemporaryBRepManager` (`tb.copy` + `tb.booleanOperation(..., IntersectionBooleanType)`) with pre-extracted float AABBs (`<1.0 s` across 1,200+ bodies!).
4. **Units & Strict 4-Step Parametric Execution Order (`1.0 s` vs. `>120 s` Hang):**
   - Fusion internal units are **`cm`** (`1 cm = 10 mm`) and **`cm³`** (`1 cm³ = 1,000 mm³`). Always divide `mm` by `10.0` (`pt_cm`, `vec_cm`) and multiply volumes by `1000.0` for `mm³`.
   - Follow the strict 4-step order in `ParametricDesignType` (documented in `docs/03_AI_AGENT_FUSION360_AND_CAD_GUIDE.md`):
     1. **Pre-Step 1:** Toggle `isLightBulbOn = False` on replaced COTS occurrences ONLY (never call `deleteMe()` or `transform2` before `bf.finishEdit()`). Build `scene_solids` analytic cutters using post-Step-3 analytical coordinates.
     2. **Step 1:** Create ONE `up_occ = root.occurrences.addNewComponent(...)` and ONE `bf = up_comp.features.baseFeatures.add()`. Build all custom B-Rep solids from **inner shafts/bearings/pulleys/belts OUTWARD to swing-arms/brackets/shrouds/plates/chutes** inside `bf.startEdit() ... bf.finishEdit()`.
     3. **Step 2:** Assign appearances via `up_comp.bRepBodies.itemByName(name)` immediately after `bf.finishEdit()`.
     4. **Step 3:** Spawn cloned COTS occurrences (`addExistingComponent`) and apply all `occ.transform2` updates (`occ.isGrounded = False; occ.transform2 = mat`—never set `occ.isGroundToParent = False` on nested proxies or cloned multi-body instances).
     5. **Step 4:** Run the unfiltered full-assembly `TemporaryBRepManager` intersection audit + `lumps.count == 1` check + `91.95 mm` Nectar & `71.12 mm` Pollen swept-sphere clearance checks.
5. **`vision_analyze` Batch Size & Format:**
   - Convert saved viewport `1600×1000` PNGs to `1280×800` RGB `.jpg` (`quality=88` via `PIL.Image`) and pass **at most 1–2 images per turn** to `vision_analyze` so remote multimodal calls never time out.

### 1.2 Opening `HIVE QUEEN v111326` vs. Saving `HIVE QUEEN v111426` via Fusion Data API
To locate the `FTC 26-27` cloud folder in `BHHS FTC 149 / Default Project` and save a clean new cloud document **`HIVE QUEEN v111426`** without overwriting `HIVE QUEEN v111326`:
```python
target_folder = None
source_df = None
for h_i in range(app.data.dataHubs.count):
    h = app.data.dataHubs.item(h_i)
    if h.name == "BHHS FTC 149":
        app.data.activeHub = h
        for p_i in range(h.dataProjects.count):
            p = h.dataProjects.item(p_i)
            if p.name == "Default Project":
                rf = p.rootFolder
                for f_i in range(rf.dataFolders.count):
                    fd = rf.dataFolders.item(f_i)
                    if fd.name == "FTC 26-27":
                        target_folder = fd
                        for df_i in range(fd.dataFiles.count):
                            df = fd.dataFiles.item(df_i)
                            if df.name == "HIVE QUEEN v111326":
                                source_df = df

# Best workflow for a 100% clean timeline without 443 legacy features or 116 hidden duplicate bodies:
# Import clean base C:/Users/danu0/Downloads/26-27-BIOBUZZ/cad/STEP/project_hive_queen_robot.step
# via import_manager.importToNewDocument(step_opts), run the upgraded v12 build + joints,
# and save directly to target_folder as "HIVE QUEEN v111426":
#   doc.saveAs("HIVE QUEEN v111426", target_folder, "v12 Full Mechanical, Rulebook & Dual-Ball Remediation", "")
```
*(Verify `HIVE QUEEN v111326` remains intact at v1 in `target_folder.dataFiles` and `HIVE QUEEN v111426` appears alongside it!)*

---

## 2. Exact Coordinate Frame & Verified Defect Ledger of `HIVE QUEEN v111326`

### 2.1 Robot Coordinate System
* **`+X`:** Front of robot (Asymmetrical Funnel, Floating Intake, Ramp Bullnose Lip)
* **`-X`:** Rear of robot (Battery Bay, Rear 10H Crossbar at `X = -192 mm`, Shooter Counter-Wheel & Hood Hinge)
* **`+Y`:** Left side of robot (`gobilda_1120_17hole_left:1` centered at `Y = +156 mm`, `Y ∈ [+132, +180 mm]`)
* **`-Y`:** Right side of robot (`gobilda_1120_17hole_left:2` centered at `Y = -156 mm`, `Y ∈ [-180, -132 mm]`)
* **`+Z`:** Vertical up from foam tiles (`Z = 0.0 mm` is the field floor; Mecanum wheel axles sit at `Z = 48.0 mm`; 17H & 10H chassis channels span `Z ∈ [24.0, 72.0 mm]` web-up `∩`)

### 2.2 Complete Audit Table — Every Defect You Must Fix

| ID | Subsystem | Exact Defect Measured in `HIVE QUEEN v111326` | Required Engineering Fix (v12 Spec) |
| :-: | :--- | :--- | :--- |
| **B1** | **Chute & Magazine Width** | `sprung_magazine_shroud_left/right` sit at `|Y| ∈ [42.0, 54.0 mm]` (**`84.00 mm`** clear width); `j_chute` curbs sit at `|Y| = 44.0 mm` (**`88.00 mm`** width); `st4_collar` cut is **`86.00 mm`**. Official `§9.8` NECTAR is **`91.95 mm`** (`3.62"`), causing `53.5–10,325.7 mm³` solid collisions. | Widen the **entire ball corridor** from front lip to shooter nip to **`102.0 mm` clear width (`Y ∈ [-51.0, +51.0 mm]`)**: shrouds at `|Y| = 53.0 mm` (inner face `|Y| = 51.0 mm`), J-chute floor `102 mm` wide with curbs at `|Y| ∈ [51.0, 55.0 mm]`, `st4_collar` slot `104 mm` wide, and shift `j_curve`/`j_elev` rearward `-8 mm` so a `91.95 mm` Nectar sphere has **`0.0 mm³` overlap** at every station. |
| **B2** | **Magazine Compliance & Belts** | `magazine_outboard_double_shear_bearing_plate_right` (`Y = -71.0 mm`) has **fixed `R = 4.1 mm` circular holes** at `(114,151)`, `(34,180)`, `(-12,240)`, pinning all 3 Gecko shafts at **`0.00 mm` travel**. Fixed belt from `M8 (104,98)` to `Gecko 1 (114,151)` would require `+37%` stretch to lift `+20 mm`. Zero springs exist on the magazine. | Replace linear slots with **Coaxial Belt-on-Pivot Swing-Arms** (`0.000 mm` belt stretch):<br>• **Gecko 1** pivots coaxially around `Motor M8`'s shaft `(104.0, 98.0)` (`R_arm1 ≈ 54–60 mm` matched to exact HTD-5M belt pitch).<br>• **Gecko 2 & Gecko 3** either pivot coaxially around fixed idler jackshafts driven from `M8` OR use curved **concentric circular arc slots** in *both* `sprung_magazine_shroud_left/right` AND `mag_ds_plate` centered on each driving pulley axis, with **seated `1611-0514-4008` bearings inside the swing-arm hubs** and **outboard torsion/extension springs + lower stop pins**. |
| **B3** | **Front Floating Intake Height & Stops** | Floating shaft at `(212.0, 117.0)` puts `R = 23.5 mm` compliant wheels at **`Z_min = 93.50 mm`**—above both `71.12 mm` Pollen (`+22.4 mm` air gap!) and `91.95 mm` Nectar (`+1.55 mm` air gap!) on the `Z = 0` floor. Swing-arm has **zero mechanical down/up stops** (`R102`/`R105.B`), and `j_front_lip` sits at **`Z = 24.08 mm`** (`~1"` cliff). | • Lengthen `floating_intake_swing_arm_left/right` to **`R_arm = 82.0 mm`** pivoting at `(156.0, 144.0)` down to **`(202.0, 76.5 mm)`** (`X_max` with wheel/star roller `<= +228.5 mm`, giving **`+12.7 mm` safety margin** inside `R102`'s `457.0 mm` starting cube!).<br>• At `Z_shaft = 76.5 mm` (`R = 23.5 mm`), **`Z_min = 53.0 mm`** (`18.1 mm` positive top compression/bite on `71.12 mm` Pollen on the floor, swinging up along the coaxial arc to `Z_shaft = 104 mm` for `91.95 mm` Nectar with `0.000 mm` belt stretch!).<br>• Add **bolted aluminum mechanical lower-stop & upper-stop pins** (`R102` & `R105.B`) on the front uprights/arms.<br>• Extend `j_ramp` forward/downward to a **low-profile floor-skimming wedge lip at `Z = 3.5 mm, X = +184.0 mm`** (`st1_bar` relocated behind/under the ramp at `X = +120 mm, Z = 28 mm` so nothing blocks the wedge). |
| **B4** | **Shooter Arc Slot Keyhole & Bearings** | `shooter_outboard_flanged_bearing_right_2` (`Ø17.0 mm`) was created before `ds_right` (`trim_prev=True`), carving a **`R = 8.5 mm` circular keyhole** at the top of the `9.2 mm`-wide (`slot_r = 4.6 mm`) concentric arc slot—trapping the bearing! Swing-arms have **zero ball bearings** (`Ø8.1 mm` bare aluminum on spinning steel), `M6` stub shaft has a `0.5 mm` gap, and horizontal return spring rods don't touch the arm (`1.0 mm` gap) and bind vertically. | • Seat **`1611-0514-4008` (`14 mm` OD, `8 mm` REX ID) flanged ball bearings directly inside the hubs of `shooter_coaxial_swing_arm_left/right`**.<br>• **Remove** any `Ø17 mm` bearing from inside the `ds_right` slot so **only the `Ø8 mm` REX shaft** passes cleanly through the `9.2 mm`-wide smooth concentric arc slots in `turret_side_plate_left/right` and `ds_right` (**verify `0` `R = 8.5 mm` keyhole faces at the counter-wheel coordinates!**).<br>• Bridge `Motor M6`'s output shaft continuously through a seated bearing at `(-108, 266)` and anchor the return extension spring between tangent pin bosses on the swing-arm and side plate with `0.0 mm` gap. |
| **B5** | **Shooter Backspin, Overspeed & Hood Hinge Crash** | Front `Ø96 mm` wheel + Rear `Ø72 mm` wheel at `1:1` equal `6,000 RPM` produces `26.4 m/s` (`3.6×` too fast, tiny `0.029 N·m` bare-motor torque) and **`-1,012 RPM` BACKSPIN** ($v_{\text{front}} > v_{\text{rear}}$ on a `+X` forward launch!). Rising `91.95 mm` Nectar ball (`X ∈ [-108, -16]`) crashes **`12,306.5 mm³`** into the hood hinge at `X = -96.0, Z = 382.0`. | • **True Topspin (`v_rear > v_front`):** Place the **larger/faster wheel (`Ø96 mm` Rhino + steel inertia discs, driven `1.2:1` overdrive = `24T->20T`) at the REAR (`-X`)** and the **smaller/slower wheel (`Ø72 mm` compliant wheel, `1:1` = `20T->20T`) at the FRONT (`+X`)**—OR keep `Ø96 mm` front / `Ø72 mm` rear and gear the Rear Counter-Wheel **`28T -> 16T` (`1.75:1` speedup)** while gearing the Front Main Wheel **`18T -> 24T` (`0.75:1` reduction)** so **$v_{\text{rear}} \approx 8.8\text{ m/s} > v_{\text{front}} \approx 6.5\text{ m/s}$** ($v_{\text{exit}} \approx 7.6\text{ m/s}$, **`+480 RPM` TOPSPIN**!).<br>• **Motor Spec Matched to Shop Inventory (`R501`):** Spec **`2× goBILDA 5203-2402-0005 (1,150 RPM, 5.2:1)`** (Team 149 already owns `1× 1,150 RPM` + `1× 6,000 RPM` bare motor + `10× 312 RPM` spare gearmotors to order/swap a `5.2:1` planetary cartridge per `R501`!), giving **`5.2×` higher torque** and dead-on `6.5–8.0 m/s` CELL ballistics.<br>• **Hood Hinge Clearance:** Shift the hood hinge & `servo_s6` coaxially rearward from `X = -96.0` to **`X = -116.0 mm, Z = 382.0 mm`** (clear behind the `X_min = -108 mm` Nectar ball path, `0.0 mm³` collision across the entire exit arc!). |
| **B6** | **Disconnected Drivetrain (`M1–M4`)** | `M1–M4` shafts sit at **`Z = 45.0 mm`** (`3.0 mm` below `Z = 48.0 mm` wheel axles, masked by carving notches into the wheel axles), with **`0` bevel gears** and **`0` wheel bearings** (`Ø7.84 mm` axles floating in `Ø14 mm` bores). | • Shift `M1–M4` vertically `+3.0 mm` so their output shafts lie on **`Z = 48.00 mm`** (exact intersection with `X = ±144.0 mm, Z = 48.00 mm` wheel axles!).<br>• Remove the fake `mot_cut` notch from `drivetrain_rex_wheel_axle_*`, seat **`8× 1611-0514-4008` flanged REX bearings** (`2` per wheel axle in the inner & outer walls of `gobilda_1120_17hole_left:1/:2`), and model **`8× goBILDA 2305 Series 1:1 Brass Miter Bevel Gears`** (`4` motor pinions + `4` axle gears—Team 149 has `18` bevel gears in `Inventory(Motion).csv`!) with `0.0 mm³` overlap.<br>• Verify Mecanum roller diagonals form a top-down **`X` pattern** (`3606-0000-0096` on `FL + RR`, `3606-0100-0096` on `FR + RL`). |
| **B7** | **Odometry Pods, Belt Pitch, Chute Overlap & Timeline Cleanup** | Rear Y-pod cradle erased to `0.008 cm³` by `trim_scene`; all 3 pods float at `Z_min = +3.83 mm`; 3rd pod exceeds `Pinpoint v2`'s 2 ports; belts off-grid; `0` Fusion joints; `116` hidden duplicate bodies. | • **2-Pod `Pinpoint v2` Layout:** Keep **Left Forward X-Pod** (`official_gobilda_4bar_odometry_pod:1`) and **Rear-Center Y-Strafe Pod** (`official_gobilda_4bar_odometry_pod:3`), dropping the redundant Right X-Pod (`saves ~250 g & $90`, matches `Pinpoint v2`'s exact **2 JST-PH encoder ports**!).<br>• Lower both pods `-4.0 mm` in `Z` (`Z_min ≈ -0.15 mm` touching the `Z = 0` floor tiles alongside the Mecanum wheels!).<br>• Build `odom_pod_center_y_strafe_rear_cradle` at `X ∈ [-170.0, -167.8 mm]` / flanking ears outside the pod's `scene_solids` box (`trim_scene=False` with explicit bolt clearance) so its solid volume is **`>= 20 cm³`**, rigidly bolting the Y-pod to `gobilda_1120_10hole_rear:1`.<br>• Size all pulley pitch diameters (`20T = 31.83 mm` or `24T = 38.20 mm` HTD-5M, or `24T = 22.92 mm` GT2-3M) and shaft C-C distances to **exact integer closed-loop COTS belt lengths** (`err < 0.25 mm`).<br>• Ground `gobilda_1120_17hole_left:1` (`isGrounded = True`) and add a `RigidGroup` (`root.rigidGroups.add(...)`) after all transforms so parts cannot be accidentally dragged in Fusion 360. |

---

## 3. Mandatory FTC Rulebook (`TU07`) & Strategic Upgrades to Build Into CAD

1. **`G407` (Max 4 Balls) + `G408` (No Opponent Nectar) Front-Lip Active Gate & Sensors (`Servo S1` + `Sens 2`):**
   - **Rule Requirement:** Controlling a 5th ball (`G407`) or opponent Nectar (`G408`) has **no momentary exception** if repeated; internal "intake-and-eject" sorters are explicitly penalized as **`STRATEGIC` (`MAJOR FOUL + YELLOW CARD`, `pp. 106–107`)**.
   - **CAD Addition:** Mount a compact **Front-Lip Color/Distance Sensor (`front_lip_color_sensor_v3` — `REV-31-1557`)** looking down at the entry scoop (`X = +188 mm`) plus **2× optical beam-break / ToF ball-count sensors** on the magazine shroud walls, and mount **Coaxial Front-Lip Hard-Stop Gate (`Servo S1`, `2000-0025-0003` — in stock!)** on the front-left 4H upright driving a lightweight aluminum/PETG **flip-down lip blocker bar** (`g407_front_lip_blocker_gate`). Normally stowed up clear of the `115 mm` throat; drops in `60 ms` to physically block the mouth the instant 4 balls are inside OR opponent Nectar approaches the lip.

2. **Pre-Shot Throat Ball-Size & Color Classifier (`Sens 1` — `REV-31-1557`):**
   - **CAD Addition:** Actually build `throat_ball_size_color_sensor` (missing in `v111326`!) + aluminum/PETG viewing bezel at `(-82.0, +53.0, 207.0 mm)` looking through a clean viewing window in `sprung_magazine_shroud_left` right at the queued ball (`Ball 3`) below `indexer_stopper_gate_paddle` (`Servo S3`).

3. **Lightweight 1-Servo Endgame FLOWER Nectar Capper (`Servo S7` — `<145 g`, Reclaims 60-Point Endgame Swing):**
   - **Strategic Math:** Untouched FLOWERS hold `4 POLLEN` each (`§10.3.1`). Dropping `1 NECTAR` into the `Ø4.0"` (`101.6 mm`) top ring at `z = 21.5"` (`546 mm`) in the final 60s (`G410`) scores `5 (Bottom Bonus) + 5×2 (Ownership) = 15 pts` per Flower (`60 pts` across 4 Flowers, or steals an opponent-stuffed Flower for `19–21 pts`).
   - **CAD Addition:** Using 1 of Team 149's 11 in-stock `2000-0025-0003` torque servos (`Servo S7`), mount a compact **folded single-link Carbon-Fiber / PETG Nectar Drop Arm (`endgame_flower_capper_arm`)** on the left mid tower (`X = -12, Y = +148, Z = 228 mm`):
     - **Stowed (`R102`):** Folds flat forward inside `X <= +210 mm, |Y| <= 178 mm, Z <= 380 mm` (100% inside the `18"` starting cube).
     - **Deployed (`R105.A` & `R105.B`):** Swings upward/forward against a **physical aluminum hard-stop block (`R105.B`—no software limit reliance!)** to position a C-cup (`inner Ø94 mm`) at **`Z = 585 mm` (`23.0" < 29.0"` max height)** and **`X = +280 mm` (`total length = 496 mm = 19.5" < 24.0"` max length)**, centered over a perimeter FLOWER (`9.6 mm` radial alignment aided by the front bumper V-notches per `G415` note!).

4. **Armor Both REV Hubs Inboard (`G419` & `R606`) + Add `6V` Servo Power Injector (`R502`/`R505`):**
   - Move `official_rev_control_hub_31_1595:1` and `official_rev_expansion_hub_31_1153:1` **inboard** onto protected rear-side mounting plates (`|Y| <= 168 mm` inside the `|Y| = 180 mm` chassis side plane, OR enclose `|Y| = 182–214 mm` inside bolted `4 mm` polycarbonate/aluminum side-impact armor shields with open diagnostic LED windows per `R606.B`!).
   - Model a **goBILDA 6V Servo Power Injector (`3125-0001-0001`)** (or **REV Servo Hub `REV-11-1855`**) next to the Expansion Hub so `S1`, `S3`, `S6`, and `S7` receive full regulated `6V` power (`R502` / `Table 12-3`).

5. **Sloped Anti-Catch Top Deck Shed Shields (`G409`):**
   - Add `38°–42°` sloped lightweight PETG deflection roofs over the exposed flat horizontal pockets of `cnc_turret_mounting_deck_plate_clean` (leaving the guarded upward-facing Main Power Switch bezel accessible per `R603.B`) so balls spilling from a tipping HIVE immediately roll off the robot instead of lodging on top (`G407`/`G409`).

6. **Inspection-Legal Quick-Swap Red/Blue Robot Signs (`R401–R403` & `G303.D`):**
   - Replace the undersized `46.25 mm`-numeral signs in `v111326` with **`174.0 mm × 72.0 mm`** (`6.85" × 2.83" > 6.5" × 2.5"` `R401.B/C`) multi-color AMS plates featuring **`57.5 mm` (`2.26"` `R403.A`) solid white Arabic numerals `"149"`** and **`7.25 mm` (`0.285" > 0.25"` `R403.B`) surrounding border**, held in tool-less slide-in rail channels on both left and right sides (`>= 90°` / `180°` apart) with both Red and Blue `.3mf` files exported.

---

## 4. Shop Inventory (`Inventory(Motion).csv`) Allocation Rules

Cross-check every actuator and hardware item against `C:\Users\danu0\Downloads\26-27-BIOBUZZ\cad\Inventory(Motion).csv`:

* **Motors (`8` Total — `R503` Legal):**
  - `M1–M4` (Drivetrain): Team 149 has **`10× 312 RPM`** and **`2× 435 RPM`** in stock. Spec **either** `4× 312 RPM` (`5203-2402-0019` — **100% In Stock now**, `0` to order; optionally overdrive `4:3` via bevel/belt to `416 RPM`) **or** `4× 435 RPM` (`2` In Stock + **`2` To Order** — mark clearly in BOM!).
  - `M5, M6` (Shooter Dual Wheels): Spec `2× 1,150 RPM` (`5203-2402-0005`) — `1×` In Stock + `1×` To Order (or swap `5.2:1` gearbox from one of the remaining `312 RPM` motors onto the in-stock `6,000 RPM` bare motor if planetary stages match, otherwise order `1× 5203-2402-0005`).
  - `M7` (Floating Intake): `1× 312 RPM` geared `2:1` (`624 RPM`) or `1× 1,150 RPM` (track exact cumulative count from `Inventory(Motion).csv`!).
  - `M8` (Sprung Gecko Magazine): `1× 312 RPM` (`5203-2402-0019` — In Stock, `6` left even if `M1–M4` use `312 RPM`!) geared `2:1` (`624 RPM`).
* **Servos (`4` Active of `8` Legal — `11× Torque Servos` In Stock!):**
  - `S1`: Front-Lip `G407`/`G408` Blocker Gate (`✅ In Stock — 1 of 11`)
  - `S3`: Upper-Throat Burst Stopper Gate (`✅ In Stock — 2 of 11`)
  - `S6`: Coaxial Basketball Exit Hood Pitch (`✅ In Stock — 3 of 11`)
  - `S7`: Endgame Flower Nectar Capper Arm (`✅ In Stock — 4 of 11`)
* **Bearings, Bevel Gears, Hubs & Shafts:**
  - `8mm REX Bearings`: `10×` in stock (`Inventory(Motion).csv` line 49) + `14×` non-REX (`line 50`) — flag exact shortfall `To Order` (`1611-0514-4008` packs) if total REX bearings exceed `10`! Do NOT falsely mark `22×` REX bearings "In Stock" when only `10×` REX bearings exist!
  - `Bevel Gears`: `10× big + 8× small` in stock (`lines 44, 47`) $\implies$ `8×` for `M1–M4` drivetrain is `✅ In Stock`.
  - `Sonic Hubs`: `12× small + 8× big` in stock (`lines 48, 54`) $\implies$ `✅ In Stock`.
  - `8mm REX Shafts`: `3×80, 1×104, 1×144, 1×264, 1×288, 3×312 mm` in stock (`lines 12–25`) — verify cut-list lengths sum within stock or list exact shafts `To Order`.

---

## 5. Mandatory Quantitative & Visual Acceptance Gates (Must ALL Pass Before Finishing!)

Do **not** declare the CAD remediation complete until a single verification script executed on the final Fusion 360 document returns **`PASS` (`True`) on all 9 gates**:

1. **`unfiltered_brep_overlaps == []` (`0.00 mm³`):** Full pairwise `TemporaryBRepManager` boolean intersection across **every** visible solid body in the assembly—**WITHOUT** skipping `gobilda_1120_10hole_rear:2/:3` or staged balls!
2. **`multi_lumps == []`:** Every custom B-Rep body has `b.lumps.count == 1` and `b.volume >= 0.25 cm³` (proving `trim_scene` / `trim_prev` never severed a part or shaved an odometry cradle into a `0.008 cm³` foil sliver!).
3. **`r102_stowed_pass == True`:** Stowed bounding box (`excluding preloaded balls`) satisfies `size_x <= 448.0 mm`, `size_y <= 448.0 mm`, `size_z <= 450.0 mm` (at least `7–9 mm` safety margin inside `R102`'s `457.0 mm` / `18.0"` cube!).
4. **`r105_expanded_pass == True`:** With Floating Intake at upper stop and Flower Capper Arm (`S7`) against its physical `R105.B` stop, envelope is strictly inside `609.6 mm (24") × 457.0 mm (18") × 736.6 mm (29")`.
5. **`nectar_92mm_swept_poses_overlaps == []`:** A `Ø91.95 mm` (`R = 45.975 mm`) test sphere placed at 10 stations along the path (Intake Mouth $\to$ Floor Wedge $\to$ Lower Ramp $\to$ Center Crossbar $\to$ J-Elbow $\to$ Deck Throat $\to$ Open Gate $\to$ Expanded Nip $\to$ Exiting past Hood Hinge at `Z = 382 mm` & `Z = 415 mm`) has **`0.0 mm³` collision** with all fixed walls, shrouds, curbs, deck plates, and hood hinges!
6. **`floor_contact_check == True`:** All 4 Mecanum wheels AND both `Pinpoint v2` dead-wheel odometry pods have `Z_min ∈ [-0.5, +0.2 mm]`, while the front intake rollers at their lower hard-stop have `Z_min <= 56.0 mm` (`>= 15 mm` positive bite on a `71.12 mm` floor Pollen ball).
7. **`no_keyhole_or_locked_slots == True`:** Zero `R > 4.7 mm` bearing pockets intersect the `9.2 mm`-wide concentric arc slots on `ds_right` or `mag_ds_plate`, and all moving shafts ride inside seated ball bearings inside their coaxial swing-arms.
8. **`topspin_verified == True`:** Rear shooter wheel surface speed exceeds front shooter wheel surface speed ($v_{\text{rear}} > v_{\text{front}}$, average $v_{\text{exit}} \in [6.8, 8.5\text{ m/s}]$) so forward (`+X`) shots carry positive topspin.
9. **Visual Sign-Off (`vision_analyze`):** Inspect 6 fresh `1280×800` RGB `.jpg` renders (`front-right iso`, `rear-left iso`, `underbelly iso`, `shooter right ortho`, `intake front close-up`, and `right-side internal cutaway`) at 1–2 images per turn to confirm zero floating parts, clean miter gear mesh on all 4 wheels, visible throat & front-lip sensors, armored hubs, legal `57.5 mm` signs, and smooth ball tangency throughout. Then save updated PNGs into `docs/images/01..08_*.png`.
