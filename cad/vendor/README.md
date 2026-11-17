# Vendor CAD (not in git)

The builder (`cad/run_full_robot_upgrade.py`) imports these vendor STEP files from this folder. They are third-party
downloads (~390 MB), so they are git-ignored. Re-download before rebuilding:

- goBILDA: `https://www.gobilda.com/content/step_files/<SKU>.zip` (unzip here; keep the file names)
- REV Robotics: `https://www.revrobotics.com/content/cad/<SKU>.STEP`
- DFRobot HuskyLens SEN0305: from the DFRobot product page

Files the builder expects (base names):

- `1120-0004-0120`
- `1120-0005-0144`
- `1120-0010-0264`
- `1120-0017-0432`
- `1121-0010-0264`
- `1201-0043-0002`
- `1209-0001-0001`
- `1209-0001-0001-REV0`
- `1313-1632-4008`
- `14-0014`
- `1613-0516-0008`
- `2000-0025-0003`
- `2315-4008-0030`
- `2315-4008-0030 assembly`
- `3100-0012-0020`
- `3110-0001-0001`
- `3110-0002-0002`
- `3125-0001-0001`
- `3213-3606-0001`
- `3213-3606-0002`
- `3417-4008-0016`
- `3606-0000-0100 assembly`
- `3606-0100-0100 assembly`
- `3628-0032-0082`
- `4001-0025-4008`
- `4001-0025-4008 assembly`
- `5203-2402-0019`
- `5203-2402-0019 assembly`
- `REV-31-1153`
- `REV-31-1302`
- `REV-31-1387`
- `REV-31-1505`
- `REV-31-1557`
- `REV-31-1595`
- `SEN0305`
- `mecanum_96mm_left`
- `mecanum_96mm_right`
