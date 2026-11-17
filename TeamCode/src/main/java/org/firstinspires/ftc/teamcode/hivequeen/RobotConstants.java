package org.firstinspires.ftc.teamcode.hivequeen;

/**
 * FTC Team 149 "HIVE-QUEEN" (BIOBUZZ 2026-27) -- every hardware-map name and every tunable
 * number lives here so the rest of the code never hard-codes a magic value.
 *
 * Robot frame convention used everywhere: +X = forward, +Y = left, +heading = CCW (degrees).
 * Lengths are millimetres unless the name says otherwise.
 *
 * Values tagged "TUNE" are first-pass estimates from the design/CAD and MUST be tuned on the
 * real robot before competition.
 */
public final class RobotConstants {
    private RobotConstants() { }

    // =========================================================================================
    // Hardware-map configuration names (must match the Robot Controller configuration exactly)
    // =========================================================================================
    // Drivetrain: 4x goBILDA 5203-2402-0019 (312 RPM) -> 1:1 miter bevels -> 96 mm mecanum (X)
    public static final String FRONT_LEFT   = "fl";
    public static final String FRONT_RIGHT  = "fr";
    public static final String BACK_LEFT    = "bl";
    public static final String BACK_RIGHT   = "br";

    // Localization: goBILDA Pinpoint v2 odometry computer on an I2C bus
    public static final String PINPOINT     = "pinpoint";

    // Vision: DFRobot HuskyLens on I2C (primary), optional UVC webcam (secondary)
    public static final String HUSKYLENS    = "huskylens";
    public static final String WEBCAM       = "Webcam 1";

    // Shooter
    public static final String SHOOTER_FRONT = "shooterFront"; // M5, 1150 RPM, front dia 96 mm wheel 1:1
    public static final String SHOOTER_REAR  = "shooterRear";  // M6, 1150 RPM, rear dia 72 mm wheel 2:1 overdrive
    public static final String HOOD_PITCH    = "hoodPitch";    // S6

    // Intake / magazine
    public static final String INTAKE        = "intake";       // M7, 1150 RPM (5203-2402-0005), 20T:20T x2 (1:1:1) -> 47 mm compliant wheels, 2.83 m/s
    public static final String MAGAZINE      = "magazine";     // M8, 1150 RPM (5203-2402-0005), HTD-3M 6 mm belts 33T:33T / 32T:32T (still 1:1) to the gecko arms

    // Servos (goBILDA 2000-0025-0002 Dual Mode torque servos, standard mode, on the 6 V servo power injector)
    public static final String LIP_GATE      = "lipGate";      // S1 front-lip flip-down blocker gate
    public static final String INDEXER_GATE  = "indexerGate";  // S3 upper-throat burst stopper

    // Sensors
    /**
     * REV Color Sensor V3 at the front lip, lens at (X 167, Z 43.5) looking across the mouth. It is
     * the ENTRY counter (game/BallCounter): +1 on a ball-present rising edge while the intake drives
     * forward and the ball is accepted, -1 if a reverse pulse pushes a counted ball back out past it.
     * The lip is crossed only once going in (rolled-back balls stop at gecko 1, ~X 133). It also sees
     * a ball parked in queue slot 4 (X 187.6, Z 46, under the compliant intake wheels).
     */
    public static final String LIP_COLOR     = "lipColor";
    /**
     * REV Color Sensor V3 at the shooter throat, looking at queue slot 1 (the ball held by the
     * closed S3 indexer gate, X -9.6, Z 127.8). EXIT counter: -1 on a falling edge while feeding and
     * NOT reversing; also the slot-1 occupancy sensor and the next-ball classifier.
     */
    public static final String THROAT_COLOR  = "throatColor";
    /**
     * Magazine OCCUPANCY sensors: REV 2m Distance Sensors (REV-31-1505) on the left corridor wall
     * looking across, read through the SDK {@code DistanceSensor} interface (CAD change: moved from
     * the old entry/4th-slot spots).
     *  - ballCount1: looks at the centre of queue slot 2 (X 56.4, Z 99.8).
     *  - ballCount2: looks at the centre of queue slot 3 (X 122.2, Z 71.9).
     * Slots are the packed POLLEN positions with the queue pressed against the closed S3 gate. They
     * are crossed repeatedly by balls rolling back down the ramp, so they are NEVER edge-counted:
     * they only give a LOWER BOUND on the count while the magazine holds the queue against the
     * gate at MAGAZINE_HOLD_POWER and the readings are stable for OCCUPANCY_STABLE_MS
     * (throat => >= 1, + ballCount1 => >= 2, + ballCount2 => >= 3). A dia 92 NECTAR can cover two
     * slot points -> overcount, which is the safe direction for the G407 4-ball limit.
     */
    public static final String BALL_COUNT_1  = "ballCount1";
    public static final String BALL_COUNT_2  = "ballCount2";

    // =========================================================================================
    // Game constants (BIOBUZZ Competition Manual TU07)
    // =========================================================================================
    /** Sec. 10.4 / Sec. 10.2: 30 s AUTO, 8 s transition, 2:00 TELEOP. */
    public static final double AUTO_DURATION_S        = 30.0;
    public static final double TRANSITION_DURATION_S  = 8.0;
    public static final double TELEOP_DURATION_S      = 120.0;
    /** G410: "ROBOTS may not enter NECTAR into the FLOWER scoring volume until the last 60 seconds of the MATCH." */
    public static final double FLOWER_NECTAR_WINDOW_S = 60.0;
    /** G407: "A ROBOT may not simultaneously CONTROL more than 4 SCORING ELEMENTS." */
    public static final int    MAX_CONTROLLED_ELEMENTS = 4;
    /** Sec. 10.3.4: "ROBOTS must start the MATCH contacting 4 pre-loaded POLLEN." */
    public static final int    PRELOADED_POLLEN       = 4;
    /**
     * START CONFIGURATION: only 3 preloads ride IN the magazine (a 4th inside lifts the floating
     * intake to 456.1 mm robot length vs the 457 mm R102 limit -- too marginal). The 4th preload
     * sits on the TILES touching the front funnel (legal, Sec. 10.3.4) and is ingested by the
     * first Autonomous motion (LEAVE_INTAKE).
     */
    public static final int    PRELOADS_IN_MAGAZINE   = 3;

    /** Sec. 9.8: POLLEN ~2.8 in (7.1 cm) yellow; NECTAR ~3.6 in (9.1 cm) red / blue. */
    public static final double POLLEN_DIAMETER_MM     = 71.12;
    public static final double NECTAR_DIAMETER_MM     = 91.95;

    /** Sec. 9.9: 36h11 AprilTags, 3.25 in (8.25 cm), four per cluster on the bottom of each CELL. */
    public static final double APRILTAG_SIZE_MM       = 82.55;
    /** Sec. 9.9: red CELL IDs 30-33 (far side) and 34-37 (audience side). */
    public static final int[]  RED_CELL_TAG_IDS       = {30, 31, 32, 33, 34, 35, 36, 37};
    /** Sec. 9.9: blue CELL IDs 38-41 (audience side) and 42-45 (far side). */
    public static final int[]  BLUE_CELL_TAG_IDS      = {38, 39, 40, 41, 42, 43, 44, 45};

    // =========================================================================================
    // Drivetrain
    // =========================================================================================
    public static final double DRIVE_MOTOR_TICKS_PER_REV = 537.7;   // 5203-2402-0019 312 RPM
    public static final double DRIVE_WHEEL_DIAMETER_MM   = 96.0;
    public static final double DRIVE_SLOW_MODE_SCALE     = 0.40;
    public static final double DRIVE_STICK_DEADBAND      = 0.05;

    // =========================================================================================
    // Pinpoint odometry (2 goBILDA 4-bar pods)
    // =========================================================================================
    /** Forward (X) pod position in the robot frame: (X=-95, Y=+110) mm -> LEFT side. */
    public static final double X_POD_ROBOT_X_MM = -95.0;
    public static final double X_POD_ROBOT_Y_MM = 110.0;
    /** Strafe (Y) pod position in the robot frame: (X=-146.2, Y=0) mm -> rear centre (CAD v12 pod:2 AABB centre). */
    public static final double Y_POD_ROBOT_X_MM = -146.2;
    public static final double Y_POD_ROBOT_Y_MM = 0.0;
    /** Flip if the pose moves the wrong way when pushing the robot forward/left. TUNE */
    public static final boolean X_POD_REVERSED  = false;
    public static final boolean Y_POD_REVERSED  = false;

    // =========================================================================================
    // Vision / heading lock
    // =========================================================================================
    public static final int    HUSKYLENS_IMAGE_WIDTH_PX  = 320;
    public static final int    HUSKYLENS_IMAGE_HEIGHT_PX = 240;
    public static final double HUSKYLENS_HFOV_DEG        = 60.0;  // TUNE: measure on the real unit
    public static final long   HUSKYLENS_READ_PERIOD_MS  = 50;    // ~20 Hz I2C polling
    /**
     * HuskyLens TAG_RECOGNITION reports the *learned* ID (1..n), not the AprilTag number.
     * Teach the HuskyLens the tags in this order so that learned ID k == index k-1 below.
     * (Learned IDs outside this table are ignored.)
     */
    public static final int[]  HUSKYLENS_LEARNED_ID_TO_APRILTAG = {
            30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45
    };
    /** Camera yaw offset vs. the shooter axis (+ = camera points left of the shooter). TUNE */
    public static final double CAMERA_YAW_OFFSET_DEG = 0.0;
    /**
     * HuskyLens lens position in the robot frame (CAD v12_design HUSKY_T: lens at Y -86, Z 354,
     * facing +X; i.e. 86 mm RIGHT of the shooter centreline). The lateral term is applied when the
     * camera yaw is converted to a shooter-axis yaw (ShotTable.cameraToShooterYawDeg). TUNE X.
     */
    public static final double HUSKYLENS_ROBOT_X_MM  = 76.0;
    public static final double HUSKYLENS_ROBOT_Y_MM  = -86.0;
    public static final double HEADING_LOCK_KP       = 0.018;  // turn power per degree. TUNE
    public static final double HEADING_LOCK_KS       = 0.04;   // static friction kick. TUNE
    public static final double HEADING_LOCK_MAX_TURN = 0.6;
    public static final double HEADING_LOCK_TOL_DEG  = 1.5;
    /** Vision target older than this is "lost"; lock then holds the last field heading via Pinpoint. */
    public static final long   VISION_STALE_MS       = 300;

    // =========================================================================================
    // Shooter (dual flywheel, vertical nip, topspin)
    // =========================================================================================
    public static final double SHOOTER_TICKS_PER_REV   = 145.1;   // 5203-2402-0005 1150 RPM output
    public static final double SHOOTER_MOTOR_FREE_RPM  = 1150.0;
    public static final double FRONT_WHEEL_DIAMETER_MM = 96.0;    // Rhino wheel, 20T->20T HTD-5M 1:1
    public static final double FRONT_WHEEL_RATIO       = 1.0;
    public static final double REAR_WHEEL_DIAMETER_MM  = 72.0;    // counter-wheel, 32T->16T = 2:1 overdrive
    public static final double REAR_WHEEL_RATIO        = 2.0;
    public static final double NOMINAL_VOLTAGE         = 12.0;
    /** Hub voltage reads are NOT bulk-cached (one LynxCommand per hub); sample at this period. */
    public static final long   VOLTAGE_SAMPLE_MS       = 250;
    /** Feedforward: power = (kS + kV*rpm) * 12/Vbatt. kV ~ 1/free-speed. TUNE */
    public static final double FLYWHEEL_KS             = 0.04;
    public static final double FLYWHEEL_KV             = 1.0 / SHOOTER_MOTOR_FREE_RPM;
    public static final double FLYWHEEL_KP             = 0.0012;  // power per RPM of error. TUNE
    /** At-speed window (motor RPM) and dwell time before feeding is allowed. */
    public static final double FLYWHEEL_TOLERANCE_RPM  = 30.0;
    public static final long   FLYWHEEL_SETTLE_MS      = 80;
    /** Highest RPM we ever command, leaving voltage headroom for recovery between shots. */
    public static final double FLYWHEEL_MAX_SETPOINT_RPM = 1050.0;
    /**
     * Missing-encoder detection (game/FlywheelWatchdog): power >= MIN_POWER for longer than FAULT_MS
     * while the encoder reads below MAX_RPM -> encoder FAULT. The controller then runs feedforward
     * only and "at speed" becomes time-based (FALLBACK_SPINUP_MS after the wheel was commanded) so
     * the robot can still shoot. TUNE the spin-up time on the robot (time 0 -> 1050 RPM, add margin).
     */
    public static final double FLYWHEEL_ENC_FAULT_MIN_POWER = 0.5;
    public static final double FLYWHEEL_ENC_FAULT_MAX_RPM   = 30.0;
    public static final long   FLYWHEEL_ENC_FAULT_MS        = 1000;
    public static final long   FLYWHEEL_FALLBACK_SPINUP_MS  = 2000;   // TUNE
    /** Spin-up power ramp below half speed (power units per second): 0 -> 1 in ~0.4 s. TUNE. */
    public static final double FLYWHEEL_SPINUP_SLEW_PER_S = 2.5;

    /**
     * Hood mechanical range (CAD v12 / docs/05): 62-72 deg exit elevation. Every shot-table
     * elevation is clamped into this range before it becomes a servo position.
     */
    public static final double HOOD_MIN_DEG = 62.0;
    public static final double HOOD_MAX_DEG = 72.0;
    /**
     * Two-point hood calibration: exit elevation (deg) -> S6 servo position. S6 drives the hood
     * COAXIALLY (hood_servo_shaft_coupler on the hinge, docs/02 Sec. 8; no linkage), so 1 deg of hood
     * = 1 deg of servo = 1/SERVO_RANGE_DEG of the position range: the 10 deg hood range is only
     * 0.033 of servo travel. HOOD_CAL_LOW_POS and the sign (HOOD_DIR) are TUNE on the bench with
     * HiveQueenHardwareTest + a protractor; keep the span = 10/300 unless a linkage is added.
     */
    public static final double HOOD_DIR          = 1.0;     // +1: position increases with elevation. TUNE
    public static final double HOOD_CAL_LOW_DEG  = HOOD_MIN_DEG;
    public static final double HOOD_CAL_LOW_POS  = 0.50;    // TUNE (bench, protractor)
    public static final double HOOD_CAL_HIGH_DEG = HOOD_MAX_DEG;
    public static final double HOOD_CAL_HIGH_POS = HOOD_CAL_LOW_POS
            + HOOD_DIR * (HOOD_CAL_HIGH_DEG - HOOD_CAL_LOW_DEG) / RobotConstants.SERVO_RANGE_DEG;   // 0.533
    /** S6 software stops = exactly the 62-72 deg calibration points (never drive into the hard stops). */
    public static final double HOOD_MIN_POS = Math.min(HOOD_CAL_LOW_POS, HOOD_CAL_HIGH_POS);
    public static final double HOOD_MAX_POS = Math.max(HOOD_CAL_LOW_POS, HOOD_CAL_HIGH_POS);
    /**
     * Reliable launch window D (shooter exit -> CELL lip). Re-checked with the hood clamped to 62 deg
     * (see ShotTable: RK4 drag model fitted to the parent 0.75/1.00 m rows): 1.25 m needs 5.28 m/s =
     * the 1050 RPM cap (no margin), so the window stays 0.75-1.25 m. NECTAR (+3 % RPM) is already at
     * the cap from ~1.2 m.
     */
    public static final double SHOT_RELIABLE_MIN_D_MM = 750.0;
    public static final double SHOT_RELIABLE_MAX_D_MM = 1250.0;
    /** Driver OUT-OF-RANGE warning band = the reliable window. */
    public static final double SHOT_WARN_MIN_D_MM = SHOT_RELIABLE_MIN_D_MM;
    public static final double SHOT_WARN_MAX_D_MM = SHOT_RELIABLE_MAX_D_MM;

    // =========================================================================================
    // Servo positions (TUNE all on the robot)
    // =========================================================================================
    /**
     * goBILDA 2000-0025-0002 Dual Mode servo in STANDARD mode: 300 deg over position 0..1. These are
     * torque servos: commanding past a printed part's hard stop stalls the servo against it (heat,
     * stripped prints; R103 warns about stalling against a hard stop while holding START config).
     * So each open/closed pair below is limited to the mechanism's CAD swing, and every command is
     * clamped to its pair (game/ServoLimits). Set the actual end-stops ON THE BENCH with
     * HiveQueenHardwareTest (servo moves only while A is held), then edit the *_OPEN/_CLOSED base
     * positions -- keep the spans unless the CAD changes.
     */
    public static final double SERVO_RANGE_DEG    = 300.0;
    /** S1 blocker bar swing stowed -> deployed (CAD v12_design BLOCKER_A_STOWED - BLOCKER_A_DEPLOYED = 121.8 deg, coaxial). */
    public static final double LIP_GATE_SWING_DEG = 122.0;
    /** S3 finger retract angle (CAD v12_design GATE_RETRACT_DEG). */
    public static final double INDEXER_TRAVEL_DEG = 60.0;
    public static final double LIP_GATE_DIR       = 1.0;   // +1: deploying increases the position. TUNE
    public static final double INDEXER_DIR        = 1.0;   // +1: opening increases the position. TUNE
    public static final double LIP_GATE_OPEN      = 0.15;  // stowed (flipped up): mouth open. TUNE (bench)
    public static final double LIP_GATE_BLOCKED   = LIP_GATE_OPEN
            + LIP_GATE_DIR * LIP_GATE_SWING_DEG / SERVO_RANGE_DEG;   // 0.557: flipped down, mouth blocked
    public static final double INDEXER_CLOSED     = 0.30;  // stops balls below the flywheels. TUNE (bench)
    public static final double INDEXER_OPEN       = INDEXER_CLOSED
            + INDEXER_DIR * INDEXER_TRAVEL_DEG / SERVO_RANGE_DEG;    // 0.50: fingers below the floor

    // =========================================================================================
    // Intake / magazine
    // =========================================================================================
    /**
     * M7 = goBILDA 5203-2402-0005 1150 RPM, 145.1 ticks/rev at the output, belts 1:1:1 to 47 mm
     * compliant wheels: 1150/60 * pi * 0.047 = 2.83 m/s surface at full power (~1.8x the 1.57 m/s
     * drive top speed, so balls are pulled in rather than bulldozed).
     */
    public static final double INTAKE_TICKS_PER_REV     = 145.1;
    /** Free-speed encoder rates for jam detection (1150 RPM output x 145.1 ticks/rev). */
    public static final double INTAKE_FREE_TPS   = 1150.0 / 60.0 * 145.1;
    public static final double MAGAZINE_FREE_TPS = 1150.0 / 60.0 * 145.1;
    public static final double INTAKE_WHEEL_DIAMETER_MM = 47.0;
    public static final double INTAKE_RATIO             = 1.0;
    public static final double INTAKE_POWER          = 0.9;    // TUNE 0.8-1.0 (~2.5 m/s surface)
    public static final double INTAKE_REVERSE_POWER  = -0.6;   // purge/reject pulse (~1.7 m/s). TUNE
    /**
     * Reject pulse: the roller reverses with the lip gate still OPEN for this long (so the ball is
     * pushed out, not into the closing bar); the gate drops only after the pulse and, for a colour
     * reject, only once the lip reads empty.
     */
    public static final long   REJECT_REVERSE_MS     = 250;
    /** Colour reject: keep reversing (gate open) at most this long while the ball is still at the lip. TUNE */
    public static final long   REJECT_MAX_REVERSE_MS = 750;
    /** Time the lip gate stays down after the rejected ball has left the lip (or after a 5th-ball pulse). */
    public static final long   REJECT_HOLD_MS        = 600;
    public static final double MAGAZINE_TICKS_PER_REV = 145.1;  // 5203-2402-0005 1150 RPM
    /**
     * A ball pinched between a gecko and the fixed chute floor/wall moves at HALF the gecko
     * surface speed. Path simulation (cad/v12_design.py geometry), lone ball, gate open:
     *  - staging a lone POLLEN from gecko 1 up to gecko 2 / the gate needs >= ~425 motor RPM
     *    (0.37 power at 1150 RPM); 0.6 gives ~2.2x energy margin. TUNE but keep >= 0.45.
     *  - a lone/last POLLEN coasting from gecko 2 across the 53 mm wall gap into the counter-wheel
     *    needs >= ~545 motor RPM (0.47 power). Full power gives 4.45x energy margin.
     * Lowering FEED power spaces shots out (flywheel recovery) but never go below ~0.6.
     */
    public static final double MAGAZINE_INDEX_POWER  = 0.6;
    public static final double MAGAZINE_FEED_POWER   = 1.0;
    /**
     * With balls inside and not feeding, the geckos push the queue gently against the closed S3
     * gate so unpowered balls do not roll back down the ramp (they would otherwise re-cross the
     * occupancy sensors and leave slot 1 empty at the throat). This stalls M8 at low power: keep it
     * small (heat). TUNE: lowest value that holds a full queue in place.
     */
    public static final double MAGAZINE_HOLD_POWER   = 0.15;

    // =========================================================================================
    // Sensor thresholds (TUNE)
    // =========================================================================================
    /** Ball present at the lip if the V3 proximity reads closer than this. */
    public static final double LIP_BALL_PRESENT_MM     = 60.0;
    /** Ball present at the throat if closer than this. */
    public static final double THROAT_BALL_PRESENT_MM  = 45.0;
    /**
     * Ball-size split at the throat: the throat sensor is mounted so a dia 91.95 NECTAR passes
     * ~10 mm closer to it than a dia 71.12 POLLEN (half the 20.8 mm diameter difference).
     */
    public static final double THROAT_NECTAR_MAX_MM    = 18.0;
    public static final float  COLOR_SENSOR_GAIN       = 8.0f;
    public static final double COLOR_MIN_SATURATION    = 0.30;
    /** Ball-count (occupancy) distance sensors: ball present when closer than this. TUNE */
    public static final double COUNT_BALL_PRESENT_MM   = 80.0;
    /** Occupancy lower bound only after the hold + all three readings have been steady this long. */
    public static final long   OCCUPANCY_STABLE_MS     = 200;
    /** Opponent NECTAR must be seen at the throat this long before firing is blocked (debounce). TUNE */
    public static final long   THROAT_OPPONENT_CONFIRM_MS = 100;
    /** Throat must read empty this long before an opponent-NECTAR fire block clears. */
    public static final long   THROAT_CLEAR_MS         = 150;

    // =========================================================================================
    // Autonomous drive-to-pose controller
    // =========================================================================================
    public static final double AUTO_TRANSLATION_KP     = 0.006;  // power per mm. TUNE
    public static final double AUTO_HEADING_KP         = 0.02;   // power per degree. TUNE
    public static final double AUTO_MAX_POWER          = 0.7;
    /** Slow, controlled speed for intake sweeps / creeping into the LOADING ZONE. TUNE */
    public static final double AUTO_COLLECT_POWER      = 0.40;
    /**
     * Minimum translation power while outside tolerance, so the pure-P controller does not stall
     * a few cm short of the target (mecanum strafing needs ~0.1 to break static friction). TUNE
     */
    public static final double AUTO_MIN_POWER          = 0.08;
    public static final double AUTO_POS_TOL_MM         = 25.0;
    public static final double AUTO_HEADING_TOL_DEG    = 2.0;

    // =========================================================================================
    // FIELD FRAME (Sec. 9.2-9.6, Figures 9-2, 9-3, 10-2) -- used by Autonomous + odometry aim
    // =========================================================================================
    // Origin = field centre. +X = toward the REAR wall (away from the audience). +Y = toward the
    // RED ALLIANCE wall (Sec. 9.5: red is on the left as seen from the audience). Heading 0 deg =
    // facing +X, CCW positive. The field is 180-degree ROTATIONALLY symmetric (Fig. 9-2/10-2:
    // red LOADING ZONE upper-left vs blue lower-right, gardens in opposite corners), so BLUE poses
    // are the RED poses rotated 180 deg about the origin (x,y,h) -> (-x,-y,h+180) -- NOT mirrored.
    /** Sec. 9.2: ~144 in nominal; inside-of-perimeter is ~141.4 in -> half = 1796 mm. TUNE (measure). */
    public static final double FIELD_HALF_MM          = 1796.0;
    public static final double TILE_MM                = 609.6;
    /** Robot footprint (CAD v12.2 stowed 444.0 x 443.7 mm). */
    public static final double ROBOT_HALF_LENGTH_MM   = 222.0;
    public static final double ROBOT_HALF_WIDTH_MM    = 222.0;
    /** Red HIVE pivot centre line is 12.75 in toward the red wall (team CAD of Fig. 9-8/9-9). TUNE */
    public static final double RED_HIVE_Y_MM          = 324.0;
    /**
     * Horizontal offset of an UPWARD CELL's mouth centroid from the pivot line, along X
     * (16.49 in, team CAD of Fig. 9-10/9-11). The red AUDIENCE CELL is at -X, the FAR CELL at +X.
     * The upward CELL's opening faces outward, so we always shoot from that CELL's own side. TUNE
     */
    public static final double CELL_MOUTH_OFFSET_X_MM = 419.0;
    /** Mouth LOWER LIP: 19.27 in from the pivot line, z = 53.5 in. ShotTable distance D is measured to it. */
    public static final double CELL_LIP_OFFSET_X_MM   = 489.5;
    /** Shooter exit point along robot X (CAD: ~52 mm behind robot centre). */
    public static final double SHOOTER_EXIT_X_MM      = -52.0;
    /**
     * Vision horizontal range (lens -> tag cluster) to launch distance D. The cluster sits under
     * the CELL ~250 mm beyond the lip; the HuskyLens lens (X ~76) is ~128 mm ahead of the shooter
     * exit (X -52), so D ~= range - 120 mm. Left at 0 until measured. TUNE by comparing with
     * tape-measured D.
     */
    public static final double VISION_RANGE_TO_D_OFFSET_MM = 0.0;
    /** Height of the AprilTag cluster (bottom face of the upward CELL) above the TILES. TUNE */
    public static final double CELL_TAG_HEIGHT_MM     = 1220.0;
    /** HuskyLens lens height above the TILES (CAD v12_design HUSKY_T: Z 354, front-right of the shooter). */
    public static final double CAMERA_HEIGHT_MM       = 354.0;
    /** Shooter exit direction relative to robot +X (0 = fires over the front). TUNE if not. */
    public static final double SHOOTER_YAW_OFFSET_DEG = 0.0;

    /** Sec. 9.3 / Fig. 9-3: LOADING ZONE ~23 in x 11 in, on the alliance wall, 2nd tile from the rear corner. */
    public static final double LOADING_ZONE_DEPTH_MM    = 279.4;
    public static final double RED_LOADING_ZONE_X_MM    = 1.5 * TILE_MM;   // tile centre, +914 mm

    // ---- RED Autonomous waypoints (field frame, mm / deg). BLUE = rotate 180 deg. ALL TUNE. ----
    /**
     * G304: touching the red wall (robot back on the wall, facing into the field), on the red half,
     * not in the LOADING ZONE (LZ starts at X = +609.6) and clear of the red-wall FLOWER (~X = -597).
     */
    public static final double AUTO_START_X_MM        = 300.0;
    public static final double AUTO_START_Y_MM        = FIELD_HALF_MM - ROBOT_HALF_LENGTH_MM;
    public static final double AUTO_START_HEADING_DEG = -90.0;
    /**
     * Alternate start slot (selected in INIT with gamepad1 dpad left/right) for when the partner
     * robot takes slot A. On the far side of the red-wall FLOWER (~X = -597), clear of it and of the
     * LOADING ZONE. TUNE on a real field.
     */
    public static final double AUTO_START_X_ALT_MM    = -1150.0;
    /** Max start delay selectable in INIT (gamepad1 dpad up/down, 1 s steps). */
    public static final int    AUTO_MAX_START_DELAY_S = 10;
    /** Park-target offset along the LOADING ZONE (23 in long) per start slot: centres 480 mm apart
     *  (> the 444 mm robot) and both robots still partially inside the zone. TUNE. */
    public static final double AUTO_PARK_SLOT_OFFSET_MM = 240.0;
    /**
     * Shot position on the AUDIENCE side of the red HIVE. Offset toward the red wall so the robot is
     * NOT under the spill path of a tipping CELL (G409). FAR-side shot = this mirrored in X.
     * Aim heading is computed at run time from the CELL position, not stored.
     */
    /*
     * Ballistics (parent analysis, with drag): reliable exit-speed window at the 1050 RPM cap is
     * launch distance D = 0.75-1.25 m (shooter exit -> CELL lower lip). Robot CENTRE therefore
     * 1.3-1.8 m from the HIVE pivot line (sweet spot ~1.55 m); -1450 keeps the back 120 mm off the
     * audience wall. At (-1450, 650) aimed at the CELL: D ~= 1.07 m. TUNE
     */
    public static final double AUTO_SHOT_X_MM         = -1450.0;
    public static final double AUTO_SHOT_Y_MM         = 650.0;
    /** Spill sweep waypoints on the AUDIENCE side after the audience CELL tips (x,y pairs). */
    public static final double[][] AUTO_SPILL_SWEEP_XY = {
            {-1000.0, 750.0}, {-1000.0, 430.0}, {-1500.0, 430.0}, {-1450.0, 800.0}
    };
    /** Via point that keeps the robot outside the HIVE frame (|X| < 495, |Y| < 628) and the red-wall FLOWER. */
    public static final double AUTO_VIA_X_MM          = 0.0;
    public static final double AUTO_VIA_Y_MM          = 1150.0;
    /** LOADING ZONE NECTAR pickup: approach facing the red wall, then creep in with the intake on. */
    public static final double AUTO_LZ_APPROACH_Y_MM  = 1250.0;
    public static final double AUTO_LZ_INTAKE_Y_MM    = FIELD_HALF_MM - ROBOT_HALF_LENGTH_MM - 40.0;
    /** PARK: robot front edge ~150 mm inside the LZ tape (Sec. 10.5.4 "at least partially"). */
    public static final double AUTO_PARK_Y_MM         = FIELD_HALF_MM - LOADING_ZONE_DEPTH_MM
            - ROBOT_HALF_LENGTH_MM + 150.0;

    // ---- Autonomous time budget (s since AUTO start). ----
    /** Hard deadline: whatever the state, start driving to PARK at this match time. TUNE */
    public static final double AUTO_PARK_ABORT_S          = 24.0;
    /** The PARK drive itself is cut here (leaves margin before 30 s / G403). */
    public static final double AUTO_PARK_DEADLINE_S       = 29.5;
    /** Planning estimates for "is there time for one more ...?" decisions. TUNE from logs. */
    public static final double AUTO_SHOT_CYCLE_EST_S      = 6.0;   // drive + aim + fire
    public static final double AUTO_SPILL_COLLECT_EST_S   = 6.5;   // settle + sweep
    public static final double AUTO_LZ_COLLECT_EST_S      = 5.0;   // drive + creep
    /** LEAVE_INTAKE: drive this far straight off the wall with the intake on to take the 4th preload. TUNE */
    public static final double AUTO_LEAVE_INTAKE_DIST_MM  = 300.0;
    public static final double AUTO_LEAVE_INTAKE_TIMEOUT_S = 1.5;
    /** Step timeouts (each step also gets clipped to the park abort time). */
    public static final double AUTO_DRIVE_TIMEOUT_S       = 3.5;
    public static final double AUTO_AIM_TIMEOUT_S         = 1.0;
    public static final double AUTO_FIRE_TIMEOUT_S        = 3.0;
    /** After the last ball passes the throat, keep feeding so it crosses the 53 mm unpowered gap. */
    public static final double AUTO_LAST_BALL_DWELL_S     = 0.35;
    /** TeleOp: same last-ball dwell -- the indexer stays open (magazine feeding) this long after the
     *  last counted shot even if the gunner already released fire. */
    public static final long   TELEOP_LAST_BALL_DWELL_MS  = 350;
    /** Indexer open at full feed this long with no count change -> assume the magazine is empty. */
    public static final double AUTO_ASSUME_EMPTY_S        = 2.0;
    public static final double AUTO_SPILL_SETTLE_S        = 1.5;   // let spilled balls hit the TILES (G409)
    public static final double AUTO_SWEEP_LEG_TIMEOUT_S   = 1.8;
    public static final double AUTO_LZ_INTAKE_TIMEOUT_S   = 1.5;
    public static final int    AUTO_MAX_SWEEPS_PER_SPILL  = 2;
    public static final int    AUTO_MAX_FIRE_FAILURES     = 1;

    // =========================================================================================
    // HIVE mass model (Sec. 9.8 + Event Field Setup Guide calibration, see docs/01)
    // =========================================================================================
    public static final double POLLEN_MASS_G           = 24.95;
    public static final double NECTAR_MASS_G           = 41.28;
    /** Sec. 10.3.1.B.i: 3 alliance NECTAR start in the upward CELL. */
    public static final int    STAGED_CELL_NECTAR      = 3;
    /** HIVE calibration: >= 199 g MUST tip (<= 174 g must not). Used to ASSUME a tip happened. */
    public static final double HIVE_TIP_MASS_G         = 199.0;
    /** Above this (but < 199 g) a tip is UNDEFINED, e.g. 3 NECTAR + 3 POLLEN = 198.7 g. */
    public static final double HIVE_NO_TIP_MASS_G      = 174.0;

    // =========================================================================================
    // R704.D telemetry compliance
    // =========================================================================================
    /** Telemetry is sent to the Driver Station no faster than every 250 ms (4 Hz). */
    public static final int TELEMETRY_INTERVAL_MS = 250;
}
