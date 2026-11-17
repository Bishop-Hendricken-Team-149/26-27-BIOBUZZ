package org.firstinspires.ftc.teamcode.hivequeen.subsystems;

import org.firstinspires.ftc.teamcode.hivequeen.RobotConstants;
import org.firstinspires.ftc.teamcode.hivequeen.game.BallType;
import org.firstinspires.ftc.teamcode.hivequeen.util.MathUtil;
import org.firstinspires.ftc.teamcode.hivequeen.util.Pose;

/**
 * Shooter setpoints vs LAUNCH DISTANCE D = horizontal distance from the shooter exit to the
 * upward CELL's mouth LOWER LIP (z = 53.5 in, 19.27 in from the HIVE pivot line).
 *
 * Exit speed / elevation per D. The parent ballistics analysis (with drag) gave
 *   D 0.75 m: 4.85 m/s @ 69 deg | 1.00 m: 5.05 @ 65 | 1.25 m: 5.30 @ 60 | 1.50 m: 5.50 @ 58 | 1.75 m: 5.70 @ 57
 * but 60/58/57 deg are below the hood's 62 deg mechanical minimum (HOOD_MIN_DEG, docs/05). Those
 * rows were RECOMPUTED at 62 deg with a point-mass model, RK4, a = -g z - k|v|v, k = rho*Cd*A/(2m)
 * (POLLEN 24.95 g, dia 71.12 mm, rho 1.20), criterion = ball centre at the lip height (53.5 in)
 * at horizontal distance D on the descending branch (as the parent rows). The two unknowns were
 * FITTED to the parent rows that are inside the hood range (0.75 m @ 69 = 4.85, 1.00 m @ 65 = 5.05):
 * launch height 0.400 m, Cd 0.79 (rms 0.003 m/s). Check: the fit reproduces the parent 1.25 m @ 60
 * row as 5.32 m/s (parent 5.30). Results at 62 deg: 1.25 m 5.28 | 1.50 m 5.54 | 1.75 m 5.83 m/s
 * (Cd +/-0.1 moves the 1.25 m value by about +/-0.04 m/s). Script: hq_ballistics.py (audit scratch).
 * Motor RPM is derived from the required exit speed:
 *   v_exit = eta * (v_front + v_rear) / 2,  v_front = pi*0.096*rpm/60 (1:1),  v_rear = pi*0.072*2*rpm/60 (32T:16T)
 * so both motors run the SAME RPM (the 2:1 overdrive already gives v_rear = 1.5 v_front topspin).
 * At the 1050 RPM cap (eta 0.80) the exit speed is 5.28 m/s = exactly the 1.25 m requirement at
 * 62 deg, so the RELIABLE window stays D = 0.75-1.25 m (no RPM margin at 1.25 m; NECTAR is capped
 * from ~1.2 m). Entries beyond are clamped at the cap and flagged OUT-OF-RANGE to the driver.
 * Elevations are clamped to the hood range [62, 72] deg before they become servo positions.
 * The domain is clamped to [0.75, 1.75] m. TUNE eta, the NECTAR scale and the hood calibration.
 */
public final class ShotTable {
    private ShotTable() { }

    /** One interpolated shooter setpoint. */
    public static final class Setpoint {
        public final double frontRpm, rearRpm, hood;
        /** Exit elevation (deg) after clamping to the hood range. */
        public final double elevationDeg;

        public Setpoint(double frontRpm, double rearRpm, double hood) {
            this(frontRpm, rearRpm, hood, Double.NaN);
        }

        public Setpoint(double frontRpm, double rearRpm, double hood, double elevationDeg) {
            this.frontRpm = frontRpm;
            this.rearRpm = rearRpm;
            this.hood = hood;
            this.elevationDeg = elevationDeg;
        }
    }

    /** Launch distance D (mm): shooter exit -> CELL lower lip, horizontal. */
    static final double[] RANGE_MM         = { 750, 1000, 1250, 1500, 1750 };
    /** Required exit speed (m/s) and elevation (deg) at each D (0.75/1.00: parent; 1.25+: drag model at 62 deg). */
    static final double[] EXIT_SPEED_MPS   = { 4.85, 5.05, 5.28, 5.54, 5.83 };
    static final double[] ELEVATION_DEG    = { 69.0, 65.0, 62.0, 62.0, 62.0 };

    /** Wheel-to-ball speed transfer (exit speed / mean wheel surface speed). TUNE (0.75-0.85). */
    public static final double TRANSFER_EFFICIENCY = 0.80;
    /** NECTAR (91.95 mm, 41.28 g, more nip compression) needs a little more wheel speed. TUNE */
    public static final double NECTAR_RPM_SCALE    = 1.03;

    static final double[] POLLEN_RPM = new double[RANGE_MM.length];
    static final double[] NECTAR_RPM = new double[RANGE_MM.length];
    static final double[] HOOD_POS   = new double[RANGE_MM.length];

    static {
        for (int i = 0; i < RANGE_MM.length; i++) {
            double rpm = rpmForExitSpeed(EXIT_SPEED_MPS[i]);
            POLLEN_RPM[i] = Math.min(rpm, RobotConstants.FLYWHEEL_MAX_SETPOINT_RPM);
            NECTAR_RPM[i] = Math.min(rpm * NECTAR_RPM_SCALE, RobotConstants.FLYWHEEL_MAX_SETPOINT_RPM);
            HOOD_POS[i] = hoodPositionForDeg(ELEVATION_DEG[i]);   // clamps to 62-72 deg
        }
    }

    /** Default D used when neither vision nor odometry can provide one (auto shot pose ~1.07 m). */
    public static final double DEFAULT_RANGE_MM = 1000;

    public static Setpoint lookup(BallType ball, double rangeMm) {
        double r = Double.isNaN(rangeMm) ? DEFAULT_RANGE_MM : rangeMm;   // interpolate() clamps the ends
        double[] rpm = (ball != null && ball.isNectar()) ? NECTAR_RPM : POLLEN_RPM;
        double v = MathUtil.interpolate(RANGE_MM, rpm, r);
        double elev = clampElevationDeg(MathUtil.interpolate(RANGE_MM, ELEVATION_DEG, r));
        return new Setpoint(v, v, hoodPositionForDeg(elev), elev);
    }

    /** Clamp an exit elevation to the hood's mechanical range (62-72 deg). */
    public static double clampElevationDeg(double deg) {
        return MathUtil.clamp(deg, RobotConstants.HOOD_MIN_DEG, RobotConstants.HOOD_MAX_DEG);
    }

    /** Equal motor RPM on M5 and M6 that produces {@code exitMps} (before the RPM cap). */
    public static double rpmForExitSpeed(double exitMps) {
        double perRpm = TRANSFER_EFFICIENCY * 0.5 * (
                surfaceSpeedMps(1.0, RobotConstants.FRONT_WHEEL_DIAMETER_MM, RobotConstants.FRONT_WHEEL_RATIO)
              + surfaceSpeedMps(1.0, RobotConstants.REAR_WHEEL_DIAMETER_MM, RobotConstants.REAR_WHEEL_RATIO));
        return exitMps / perRpm;
    }

    /**
     * Linear two-point hood calibration (deg -> servo position). The elevation is first clamped to
     * the hood range (62-72 deg), the position to the S6 software stops.
     */
    public static double hoodPositionForDeg(double deg) {
        deg = clampElevationDeg(deg);
        double t = (deg - RobotConstants.HOOD_CAL_LOW_DEG)
                / (RobotConstants.HOOD_CAL_HIGH_DEG - RobotConstants.HOOD_CAL_LOW_DEG);
        double pos = RobotConstants.HOOD_CAL_LOW_POS
                + t * (RobotConstants.HOOD_CAL_HIGH_POS - RobotConstants.HOOD_CAL_LOW_POS);
        return MathUtil.clamp(pos, RobotConstants.HOOD_MIN_POS, RobotConstants.HOOD_MAX_POS);
    }

    /** True if D is outside the driver-warning band (= reliable window 0.75-1.25 m): show OUT-OF-RANGE. */
    public static boolean outOfRange(double rangeMm) {
        return Double.isNaN(rangeMm) || rangeMm < RobotConstants.SHOT_WARN_MIN_D_MM
                || rangeMm > RobotConstants.SHOT_WARN_MAX_D_MM;
    }

    /**
     * Odometry launch distance D: from the shooter exit point (SHOOTER_EXIT_X_MM along the robot
     * heading) to the CELL lip point, both in the field frame.
     */
    public static double launchDistanceMm(Pose robot, Pose lipPoint) {
        double h = Math.toRadians(robot.headingDeg);
        double ex = robot.xMm + RobotConstants.SHOOTER_EXIT_X_MM * Math.cos(h);
        double ey = robot.yMm + RobotConstants.SHOOTER_EXIT_X_MM * Math.sin(h);
        return Math.hypot(lipPoint.xMm - ex, lipPoint.yMm - ey);
    }

    /**
     * Motor RPM -> wheel surface speed (m/s): pi * d * (motorRpm * ratio) / 60, where ratio is the
     * wheel turns per motor turn (front 1.0 for 20T:20T, counter-wheel 2.0 for 32T:16T).
     * 1150 RPM: front dia 96 @1:1 = 5.78 m/s, rear dia 72 @2:1 = 8.67 m/s.
     */
    public static double surfaceSpeedMps(double motorRpm, double wheelDiameterMm, double ratio) {
        return Math.PI * wheelDiameterMm / 1000.0 * motorRpm * ratio / 60.0;
    }

    /**
     * Camera bearing -> bearing seen from the SHOOTER exit (robot frame, + = left). The camera sits
     * at (camX, camY) in the robot frame (HuskyLens: 86 mm right of the shooter centreline), the
     * shooter exit at (SHOOTER_EXIT_X_MM, 0); without this lateral term the heading lock aims the
     * camera, not the shooter, at the cluster (~4 deg error at 1.2 m). {@code horizRangeMm} is the
     * horizontal camera -> target range; NaN uses DEFAULT_RANGE_MM.
     */
    public static double cameraToShooterYawDeg(double camYawDeg, double horizRangeMm, double camXmm, double camYmm) {
        double r = (Double.isNaN(horizRangeMm) || horizRangeMm <= 0) ? DEFAULT_RANGE_MM : horizRangeMm;
        double a = Math.toRadians(camYawDeg);
        double tx = camXmm + r * Math.cos(a) - RobotConstants.SHOOTER_EXIT_X_MM;
        double ty = camYmm + r * Math.sin(a);
        return Math.toDegrees(Math.atan2(ty, tx));
    }

    /**
     * Vision gives a SLANT range to the tag cluster (well above the lens); convert to horizontal.
     */
    public static double slantToHorizontalMm(double slantMm, double heightDiffMm) {
        if (Double.isNaN(slantMm)) return Double.NaN;
        double h2 = slantMm * slantMm - heightDiffMm * heightDiffMm;
        return h2 > 0 ? Math.sqrt(h2) : 0.0;
    }
}
