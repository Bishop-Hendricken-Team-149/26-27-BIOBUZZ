"""
HIVE-QUEEN v12 renders (runs inside Fusion). Writes:
  cad/out/renders/v12_<view>.jpg (1280x800 sign-off set)  and  docs/images/0N_*.png (doc figures)
"""
import os
import adsk.core as core

OUT = "C:/Users/danu0/Downloads/26-27-BIOBUZZ/cad/out/renders"
DOCS = "C:/Users/danu0/Downloads/26-27-BIOBUZZ/docs/images"
os.makedirs(OUT, exist_ok=True)
vp = app.activeViewport


def P(x, y, z):
    return core.Point3D.create(x / 10.0, y / 10.0, z / 10.0)


def all_bodies():
    res = []

    def walk(o):
        if not o.isLightBulbOn:
            return
        for i in range(o.bRepBodies.count):
            b = o.bRepBodies.item(i)
            if b.isLightBulbOn:
                res.append(b)
        for i in range(o.childOccurrences.count):
            walk(o.childOccurrences.item(i))
    for i in range(root.occurrences.count):
        walk(root.occurrences.item(i))
    return res


def shot(path, eye, target, w=1280, h=800, ortho=False, up=(0, 0, 1), extent=None):
    cam = vp.camera
    cam.isSmoothTransition = False
    cam.cameraType = core.CameraTypes.OrthographicCameraType if ortho else core.CameraTypes.PerspectiveCameraType
    cam.eye = P(*eye)
    cam.target = P(*target)
    cam.upVector = core.Vector3D.create(*up)
    if extent:
        cam.viewExtents = extent / 10.0
    vp.camera = cam
    vp.refresh()
    adsk.doEvents()
    vp.saveAsImageFile(path, w, h)      # first save forces the repaint of the new camera
    adsk.doEvents()
    return vp.saveAsImageFile(path, w, h)


def cut(pred):
    hidden = []
    for b in all_bodies():
        bb = b.boundingBox
        c = ((bb.minPoint.x + bb.maxPoint.x) * 5, (bb.minPoint.y + bb.maxPoint.y) * 5, (bb.minPoint.z + bb.maxPoint.z) * 5)
        if pred(b, c, bb):
            b.isLightBulbOn = False
            hidden.append(b)
    return hidden


def restore(hidden):
    for b in hidden:
        b.isLightBulbOn = True


VIEWS = [
    ("front_right_iso", (1150, -950, 820), (0, 0, 190), False, None, "01_full_robot_iso_front_right.png"),
    ("rear_left_iso", (-1150, 950, 820), (0, 0, 190), False, None, "02_full_robot_iso_rear_left.png"),
    ("underbelly_iso", (650, -750, -650), (0, 0, 40), False, None, "03_strafer_chassis_and_2pod_odometry_underbelly.png"),
    ("shooter_right_ortho", (-50, -1500, 330), (-50, 0, 330), True, 70.0 * 10, "07_shooter_right_concentric_arc_slot_and_huskylens.png"),
    ("intake_front_closeup", (720, -420, 300), (170, 0, 80), False, None, "04_floating_intake_and_asymmetrical_funnel.png"),
]
log = {}
for name, eye, tgt, ortho, ext, docname in VIEWS:
    shot(f"{DOCS}/{docname}", eye, tgt, w=1600, h=1000, ortho=ortho, extent=ext)
    log[name] = shot(f"{DOCS}/{docname}", eye, tgt, w=1600, h=1000, ortho=ortho, extent=ext)
# shooter front-left (doc 06)
shot(f"{DOCS}/06_shooter_front_left_coaxial_drive_and_vision.png", (760, 720, 520), (-40, 0, 320), w=1600, h=1000)
shot(f"{DOCS}/06_shooter_front_left_coaxial_drive_and_vision.png", (760, 720, 520), (-40, 0, 320), w=1600, h=1000)
# right-side internal cutaway: hide everything whose centre is right of the corridor (Y < -51 mm)
hid = cut(lambda b, c, bb: c[1] < -51.0 or (bb.maxPoint.y * 10 < -51.0))
shot(f"{DOCS}/05_continuous_j_chute_and_sprung_gecko_magazine.png", (-10, -1500, 230), (-10, 0, 230), w=1600, h=1000, ortho=True, extent=620.0)
log["right_side_cutaway"] = shot(f"{DOCS}/05_continuous_j_chute_and_sprung_gecko_magazine.png", (-10, -1500, 230), (-10, 0, 230), w=1600, h=1000, ortho=True, extent=620.0)
restore(hid)
# throat / indexer gate / power switch (doc 08): hide the left shroud + left side items for a view into the throat
hid = cut(lambda b, c, bb: c[1] > 51.0 and c[2] < 216.0 and -130 < c[0] < 60)
shot(f"{DOCS}/08_throat_indexer_gate_sensor_and_top_power_switch.png", (-420, 820, 520), (-70, 0, 170), w=1600, h=1000)
log["doc08"] = shot(f"{DOCS}/08_throat_indexer_gate_sensor_and_top_power_switch.png", (-420, 820, 520), (-70, 0, 170), w=1600, h=1000)
restore(hid)
# v12.3 buildability close-ups (cad/tmp only; not doc images)
TMPR = "C:/Users/danu0/Downloads/26-27-BIOBUZZ/cad/tmp"
log["v123_s1_cage"] = shot(f"{TMPR}/v123_s1_servo_cage_closeup.png", (520, 560, 300), (165, 190, 160), w=1400, h=1000)
hid = cut(lambda b, c, bb: c[1] > 95.0 or (c[1] > 51.0 and c[2] < 80.0))
log["v123_s3_bracket"] = shot(f"{TMPR}/v123_s3_bracket_closeup.png", (-260, 700, 260), (-68, 70, 95), w=1400, h=1000)
restore(hid)
log["v125_shroud_sensors"] = shot(f"{TMPR}/v125_shroud_sensors.png", (40, 330, 150), (40, 66, 110), w=1400, h=1000)
log["v125_lip_sensor"] = shot(f"{TMPR}/v125_lip_sensor.png", (260, 260, 120), (166, 66, 44), w=1400, h=1000)
log["v125_switch"] = shot(f"{TMPR}/v125_switch.png", (-40, 300, 380), (-114, 146, 240), w=1400, h=1000)
log["v125_injector"] = shot(f"{TMPR}/v125_injector.png", (60, 260, 120), (-10, 96, 206), w=1400, h=1000)
log["v125_huskylens"] = shot(f"{TMPR}/v125_huskylens.png", (260, -300, 420), (66, -86, 340), w=1400, h=1000)
log["v126_miter_underside"] = shot(f"{TMPR}/v126_miter_underside.png", (250, 300, -170), (160, 156, 48), w=1400, h=1000)
log["v126_battery"] = shot(f"{TMPR}/v126_battery.png", (-470, 220, 300), (-192, 12, 100), w=1400, h=1000)
log["v126_magazine_belts"] = shot(f"{TMPR}/v126_magazine_belts.png", (60, -40, 420), (60, -70, 160), w=1400, h=1000)
log["v126_shooter_belts"] = shot(f"{TMPR}/v126_shooter_belts.png", (-40, -380, 330), (-60, -80, 300), w=1400, h=1000)
log["v124_pods_underside"] = shot(f"{TMPR}/v124_pods_underside.png", (-420, 380, -260), (-140, 40, 25), w=1400, h=1000)
log["v124_motor_gap_left_rail"] = shot(f"{TMPR}/v124_motor_gap_left_rail.png", (60, 420, -220), (12, 156, 40), w=1400, h=1000)
log["v123_s6_posts"] = shot(f"{TMPR}/v123_s6_posts_closeup.png", (-420, -560, 640), (-106, -90, 380), w=1400, h=1000)
cam = vp.camera
cam.cameraType = core.CameraTypes.PerspectiveCameraType
vp.camera = cam
result = log
