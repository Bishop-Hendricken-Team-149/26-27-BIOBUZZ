"""
HIVE-QUEEN v12 finalize (runs inside Fusion AFTER a 100% gate pass):
  * ground gobilda_1120_17hole_left:1, add one RigidGroup over every visible top-level occurrence (B7)
  * save the document to BHHS FTC 149 / Default Project / FTC 26-27 as 'HIVE QUEEN v111426'
    (HIVE QUEEN v111326 is never opened for write and stays untouched)
"""
TARGET_NAME = "HIVE QUEEN v111426m"
out = {}
rbc = root.occurrences.itemByName("robot_base_core:1")
s1 = rbc.childOccurrences.itemByName("strafer_chassis_stage:1")
rail = s1.childOccurrences.itemByName("gobilda_1120_17hole_left:1")
try:
    rail.isGrounded = True
    out["grounded"] = rail.isGrounded
except Exception as e:
    out["ground_err"] = str(e)[:200]
try:
    if root.rigidGroups.itemByName("HIVE_QUEEN_v12_rigid") is None:
        oc = adsk.core.ObjectCollection.create()
        for i in range(root.occurrences.count):
            o = root.occurrences.item(i)
            if o.isLightBulbOn:
                oc.add(o)
        rg = root.rigidGroups.add(oc, True)
        rg.name = "HIVE_QUEEN_v12_rigid"
    out["rigid_groups"] = [root.rigidGroups.item(i).name for i in range(root.rigidGroups.count)]
except Exception as e:
    out["rigid_err"] = str(e)[:200]

folder = None
for h_i in range(app.data.dataHubs.count):
    h = app.data.dataHubs.item(h_i)
    if h.name == "BHHS FTC 149":
        app.data.activeHub = h
        for p_i in range(h.dataProjects.count):
            p = h.dataProjects.item(p_i)
            if p.name == "Default Project":
                rf = p.rootFolder
                for f_i in range(rf.dataFolders.count):
                    if rf.dataFolders.item(f_i).name == "FTC 26-27":
                        folder = rf.dataFolders.item(f_i)
existing = [folder.dataFiles.item(i).name for i in range(folder.dataFiles.count)] if folder else []
out["existing_files"] = existing
doc = app.activeDocument
if folder is None:
    out["save"] = "folder not found"
elif TARGET_NAME in existing and doc.isSaved and doc.name.startswith(TARGET_NAME):
    out["save"] = doc.save("v12 remediation re-save")
else:
    out["save"] = doc.saveAs(TARGET_NAME, folder, "HIVE-QUEEN v12.6 production pass (sign slide rails bolted to the mid towers; drill rejects crossing/tangent holes with their mates): every structural part screwed down (flywheel motor saddles, intake stop plates, M7 sleeve); servo torque paths via goBILDA 4001 servo-to-REX couplers on REX shafts (S1 lip blocker, S3 indexer, S6 hood); real flywheels (2x goBILDA 3628 82 mm plates per side on 1313 hubs, REX main shaft); CNC plates flat 2D 5052 6/5 mm with printed standoff tubes and foot brackets, no inserts in aluminium (DXF/STEP in cad/CNC); ball-count sensors re-sited to queue slots 2/3; sign 174x78 mm (7.25 mm visible border); goBILDA Nested battery in mount 1209; gates 1-11 pass, fastening + COTS mount audits clean", "")
out["doc_name"] = doc.name
result = out
