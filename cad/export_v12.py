"""
HIVE-QUEEN v12 exports (runs inside Fusion via cad/fusion_cli.py run-file, after a 100% gate pass):
  * full-assembly STEP  -> cad/STEP/project_hive_queen_robot_v12.step  (+ cad/out/HIVE_QUEEN_149_FUSION360.step)
  * one .3mf per printable custom body -> cad/3MF/v12/<body>.3mf
  * sign plate + numerals as .stl -> cad/3MF/v12/_sign_parts/ (merged into multi-colour red/blue .3mf by cad/build_sign_3mf.py)
"""
import json
import os
import adsk.core as core
import adsk.fusion as fusion

ROOT = "C:/Users/danu0/Downloads/26-27-BIOBUZZ/cad"
OUT3 = ROOT + "/3MF/v12"
SIGN = OUT3 + "/_sign_parts"
for d in (OUT3, SIGN, ROOT + "/STEP", ROOT + "/out"):
    os.makedirs(d, exist_ok=True)

em = design.exportManager
log = {"step": [], "3mf": [], "stl": [], "errors": []}
for path in (ROOT + "/STEP/project_hive_queen_robot_v12.step", ROOT + "/out/HIVE_QUEEN_149_FUSION360.step"):
    try:
        em.execute(em.createSTEPExportOptions(path, root))
        log["step"].append([path, os.path.getsize(path)])
    except Exception as e:
        log["errors"].append(["step", path, str(e)[:200]])

up = None
for i in range(root.occurrences.count):
    o = root.occurrences.item(i)
    if o.isLightBulbOn and o.component.name == "HIVE_QUEEN_v12_Remediation":
        up = o.component

PRINTABLE_PREFIXES = (
    "unified_continuous_j_chute", "sprung_magazine_shroud_", "asymmetrical_funnel_wing_", "g409_sloped_deck_shed_shield_",
    "floating_intake_swing_arm_", "floating_spoked_compliant_wheel_1", "gecko1_coaxial_swing_arm", "gecko2_coaxial_swing_arm_",
    "shooter_coaxial_swing_arm_", "variable_pitch_exit_hood_full", "g407_front_lip_blocker_gate", "indexer_stopper_gate_3prong_fingers",
    "turret_flywheel_motor_saddle_", "m7_intake_motor_clamp_saddle", "m8_magazine_motor_clamp_saddle", "odom_pod_", "pinpoint_v2_mount_plate",
    "gobilda_12v_battery_cradle_tray", "gobilda_12v_battery_hold_down_strap", "rev_control_hub_mount_plate", "rev_expansion_hub_mount_plate",
    "main_power_switch_impact_guard_cage", "rev_main_power_switch_housing", "turret_servo_cradle_s6_hood", "dfrobot_huskylens_mount_block",
    "s1_servo_guard_cage", "s3_indexer_servo_bracket", "magazine_outboard_double_shear_plate_right",
    "intake_upright_bearing_and_stop_plate_", "robot_sign_left_slide_rail_", "robot_sign_right_slide_rail_",
    "turret_ds_standoff_tube_", "turret_side_plate_foot_bracket_", "sign_rail_bolt_spacer_",   # v12.6: printed standoffs + side-plate feet (plates are flat CNC)
)
for b in up.bRepBodies:
    n = b.name
    if n.startswith(PRINTABLE_PREFIXES):
        p = f"{OUT3}/{n}.3mf"
        try:
            em.execute(em.createC3MFExportOptions(b, p))
            log["3mf"].append([n, os.path.getsize(p)])
        except Exception as e:
            log["errors"].append(["3mf", n, str(e)[:200]])
    if n.startswith("robot_sign_left_149_plate") or n.startswith("robot_sign_left_numeral_"):
        p = f"{SIGN}/{n}.stl"
        try:
            o = em.createSTLExportOptions(b, p)
            o.meshRefinement = adsk.fusion.MeshRefinementSettings.MeshRefinementHigh
            em.execute(o)
            log["stl"].append([n, os.path.getsize(p)])
        except Exception as e:
            log["errors"].append(["stl", n, str(e)[:200]])
open(ROOT + "/out/v12_export_log.json", "w").write(json.dumps(log, indent=1))
result = {"step": log["step"], "n3mf": len(log["3mf"]), "nstl": len(log["stl"]), "errors": log["errors"]}
