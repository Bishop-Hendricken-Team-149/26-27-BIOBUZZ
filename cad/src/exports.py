from pathlib import Path
from cad import export

from printable_parts import (
    floating_intake_swing_arm,
    flower_depollinator_fork,
    front_funnel_bumper_left,
    front_funnel_bumper_right,
    g407_lip_gate_and_sensor_mount,
    magazine_anti_snag_shroud,
    robot_sign_plate_149,
    robot_sign_plate_149_blue,
    shooter_cam_and_counter_arm,
    spoked_compliant_intake_wheel,
    star_roller_mold_half,
    star_roller_sleeve,
    stinger_nectar_cup,
)
from robot_assembly import project_hive_queen_expanded, project_hive_queen_robot

ROOT = Path(__file__).resolve().parent.parent

PRINTABLE_MODELS = [
    ("front_funnel_bumper_left", front_funnel_bumper_left),
    ("front_funnel_bumper_right", front_funnel_bumper_right),
    ("star_roller_sleeve", star_roller_sleeve),
    ("star_roller_mold_half", star_roller_mold_half),
    ("g407_lip_gate_and_sensor_mount", g407_lip_gate_and_sensor_mount),
    ("flower_depollinator_fork", flower_depollinator_fork),
    ("magazine_anti_snag_shroud", magazine_anti_snag_shroud),
    ("shooter_cam_and_counter_arm", shooter_cam_and_counter_arm),
    ("stinger_nectar_cup", stinger_nectar_cup),
    ("robot_sign_plate_149", robot_sign_plate_149),
    ("robot_sign_plate_149_blue", robot_sign_plate_149_blue),
    ("spoked_compliant_intake_wheel", spoked_compliant_intake_wheel),
    ("floating_intake_swing_arm", floating_intake_swing_arm),
]

if __name__ == "__main__":
    (ROOT / "STEP").mkdir(exist_ok=True)
    (ROOT / "GLB").mkdir(exist_ok=True)
    (ROOT / "STL").mkdir(exist_ok=True)
    (ROOT / "3MF").mkdir(exist_ok=True)

    # Full robot assembly exports (STEP AP242 with mechanism states + Animated GLB Showcase)
    robot_doc = project_hive_queen_robot()
    export(robot_doc, str(ROOT / "STEP" / "project_hive_queen_robot.step"))
    export(robot_doc, str(ROOT / "GLB" / "project_hive_queen_showcase.glb"))
    export(project_hive_queen_expanded(), str(ROOT / "GLB" / "project_hive_queen_expanded.glb"))

    # Individual Bambu AMS 2 Pro 3D-printable part exports (.stl + multi-color/material .3mf + .step)
    for name, fn in PRINTABLE_MODELS:
        doc = fn()
        export(doc, str(ROOT / "STL" / f"{name}.stl"))
        export(doc, str(ROOT / "3MF" / f"{name}.3mf"))
        export(doc, str(ROOT / "STEP" / f"{name}.step"))
