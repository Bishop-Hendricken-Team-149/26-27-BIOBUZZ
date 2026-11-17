# Shared numeric constants and goBILDA 8mm metric grid geometry for Project HIVE-QUEEN
# Coordinate frame:
#   +X = Front of robot (Intake, Flower V-Guide, De-Pollinator, Stinger deploy)
#   -X = Rear of robot (270° Turret center & primary HIVE launching direction)
#   +Y = Left of robot, -Y = Right of robot
#   +Z = Up from field tile surface (Z = 0.0 mm is the foam tile floor)

INCH = 25.4

# Rule R102 & R105.A Legal Envelopes (mm)
R102_CUBE_MM = 18.0 * INCH          # 457.2 mm starting cube limit
R105_MAX_LEN_MM = 24.0 * INCH       # 609.6 mm max expanded horizontal length
R105_MAX_WID_MM = 18.0 * INCH       # 457.2 mm max expanded horizontal width
R105_MAX_HGT_MM = 29.0 * INCH       # 736.6 mm max expanded vertical height

# BIOBUZZ Scoring Elements (mm)
POLLEN_DIA = 2.80 * INCH            # 71.12 mm
NECTAR_DIA = 3.62 * INCH            # 91.95 mm

# goBILDA 1120 Series Metric Grid Constants (mm)
GB_PITCH = 8.0                      # 8mm hole pitch grid
GB_CELL = 24.0                      # 24mm major pattern spacing
GB_BORE_DIA = 14.0                  # 14mm center hole diameter
GB_M4_CLEAR = 4.2                   # M4 screw clearance hole
GB_UCHAN_W = 48.0                   # 1120 U-Channel outer width
GB_UCHAN_H = 48.0                   # 1120 U-Channel outer height
GB_LCHAN_H = 26.0                   # 1121 Low-Side U-Channel height

# Standard goBILDA Strafer Chassis Dimensions (Modified Open-Front BIOBUZZ Layout)
WHEEL_RADIUS = 48.0                 # 96mm Mecanum wheel radius (axle at Z = 48.0 mm)
CHASSIS_Z = 48.0                    # Centerline Z of 17-hole side U-Channels
SIDE_CHAN_LEN = 432.0               # 17-Hole U-Channel (1120-0017-0432)
CROSS_CHAN_LEN = 264.0              # 10-Hole U-Channel (1120-0010-0264)
SIDE_CHAN_Y = 156.0                 # Centerline Y of left (+156) and right (-156) 17H channels
WHEEL_Y = 200.0                     # Centerline Y of 96mm Mecanum wheels (total width = 437.2 mm = 17.21")
WHEEL_X = 144.0                     # Front (+144) and Rear (-144) wheel axle X positions

# Lower Crossbar X Positions (Open-Front Bay for Intake & Magazine)
REAR_CROSS_X = -192.0               # Hole 16 (rear crossbar supporting Battery & Odometry)
MID_CROSS_X = 48.0                  # Low-Side 1121 Cross-Brace at X=+48mm, Z=28mm underneath the ramp!

# Vertical U-Channel Tower Stanchion X Positions (Bolted to top of 17H rails at Z = 72..216 mm)
REAR_TOWER_X = -144.0               # Rear 5-hole (144mm) vertical U-Channel stanchions
MID_TOWER_X = -12.0                 # Mid 5-hole (144mm) vertical U-Channel stanchions
FRONT_TOWER_X = 156.0               # Front 4-hole (120mm) vertical intake stanchions

# Subsystem Key Datums (mm)
INTAKE_ROLLER_X = 194.0             # Front compliant star-roller shaft X (ahead of ramp lip!)
INTAKE_ROLLER_Z = 94.0              # Front compliant star-roller shaft Z (bottom tips at Z=69mm to bite top of 71mm Pollen!)
INTAKE_SPAN_Y = 256.0               # Clear width between side channels

TURRET_CENTER_X = -72.0             # Rear-centered 270° turret pivot X (centered on Turret Bridge!)
TURRET_CENTER_Y = 0.0               # Centered on robot Y=0
TURRET_BASE_Z = 222.0               # Turret slewing ring base Z (flush on Turret Deck at Z=222mm!)

STINGER_PIVOT_X = -12.0             # Carbon-fiber Stinger arm pivot X (bolted inside Left Mid Tower!)
STINGER_PIVOT_Y = 156.0             # Mounted directly on top of Left Mid Vertical Stanchion
STINGER_PIVOT_Z = 244.0             # Pivot height Z (flush on top of Z=222mm CNC Deck Plate!)
