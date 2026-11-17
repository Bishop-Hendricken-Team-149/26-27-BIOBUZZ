package org.firstinspires.ftc.teamcode.hivequeen;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hivequeen.game.Alliance;
import org.firstinspires.ftc.teamcode.hivequeen.game.BurstPolicy;
import org.firstinspires.ftc.teamcode.hivequeen.game.HiveTracker;
import org.firstinspires.ftc.teamcode.hivequeen.game.MatchClock;
import org.firstinspires.ftc.teamcode.hivequeen.game.ThroatGuard;
import org.firstinspires.ftc.teamcode.hivequeen.subsystems.BallSensors;
import org.firstinspires.ftc.teamcode.hivequeen.subsystems.HeadingLock;
import org.firstinspires.ftc.teamcode.hivequeen.subsystems.HuskyLensTargeting;
import org.firstinspires.ftc.teamcode.hivequeen.subsystems.Intake;
import org.firstinspires.ftc.teamcode.hivequeen.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.hivequeen.subsystems.PinpointLocalizer;
import org.firstinspires.ftc.teamcode.hivequeen.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.hivequeen.subsystems.ShotTable;
import org.firstinspires.ftc.teamcode.hivequeen.subsystems.TargetObservation;
import org.firstinspires.ftc.teamcode.hivequeen.subsystems.WebcamAprilTagTargeting;
import org.firstinspires.ftc.teamcode.hivequeen.util.MathUtil;
import org.firstinspires.ftc.teamcode.hivequeen.util.Pose;

import java.util.Locale;

/**
 * Wires every subsystem together and enforces the match-timing rules in ONE place:
 *  - G403 / G404 transition-motionless gate ({@link MatchClock#poweredMotionAllowed()}): outside
 *    AUTO (for Autonomous) or TELEOP (for TeleOp) every motor is zeroed and no servo is
 *    re-commanded. Flywheels coast and servos just hold -- "Movement due to inertia, gravity,
 *    or de-energizing of actuators, etc. is not considered powered movement."
 *  - Servo start positions (S1 stowed, S3 closed, S6 default hood): pre-positioned in INIT via
 *    {@link #prePositionServos()} where that is legal (R103.B: "initializing an OpMode that
 *    pre-positions servos and motors to a desired stationary position"; Autonomous INIT, and
 *    TeleOp INIT only when it is NOT the AUTO->TELEOP transition, G403 note), then re-commanded
 *    (no motion) on the first loop where motion is legal.
 *  - G407/G408 via {@link Intake} -> IntakeGuard (lip colour gate, max 4) and {@link ThroatGuard}
 *    (opponent NECTAR at the throat is never fired). (No FLOWER capper: the robot never puts
 *    NECTAR in a FLOWER, so G410 cannot be violated.)
 */
public class HiveQueenRobot {
    public final RobotHardware hw;
    public final MatchClock clock;
    public final MecanumDrive drive;
    public final PinpointLocalizer localizer;
    public final HuskyLensTargeting husky;
    public final WebcamAprilTagTargeting webcamTags;
    public final HeadingLock headingLock;
    public final Shooter shooter;
    public final BallSensors balls;
    public final Intake intake;
    private final ThroatGuard throatGuard = new ThroatGuard();
    private boolean throatBlocked = false;

    private Alliance alliance;
    /** Which of our CELLS is (believed to be) facing up: drives the odometry aim/range fallback. */
    private HiveTracker.Side targetSide;
    private boolean startPositionsApplied = false;
    private boolean motionAllowed = false;
    private double batteryVoltage = RobotConstants.NOMINAL_VOLTAGE;
    private long lastVoltageSampleMs = Long.MIN_VALUE / 2;

    /*
     * Hand-over from Autonomous to TeleOp (same RC app process). The Pinpoint keeps integrating
     * between OpModes, so TeleOp keeps the field pose instead of re-zeroing it.
     */
    public static volatile boolean carryValid = false;
    public static volatile int carryBallCount = 0;
    public static volatile HiveTracker.Side carryUpSide = HiveTracker.Side.AUDIENCE;

    public HiveQueenRobot(HardwareMap hardwareMap, Alliance alliance, boolean autonomous, boolean useWebcam) {
        this.alliance = alliance;
        this.targetSide = alliance.initialUpSide();
        hw = new RobotHardware(hardwareMap);
        clock = autonomous ? MatchClock.forAutonomous() : MatchClock.forTeleOp();
        drive = new MecanumDrive(hw);
        localizer = new PinpointLocalizer(hw.pinpoint);
        husky = new HuskyLensTargeting(hw.huskyLens, alliance);
        webcamTags = new WebcamAprilTagTargeting(useWebcam ? hw.webcam : null, alliance);
        headingLock = new HeadingLock();
        shooter = new Shooter(hw);
        balls = new BallSensors(hw);
        intake = new Intake(hw, alliance);
    }

    /** Monotonic milliseconds (System.currentTimeMillis can jump when the DS syncs the clock). */
    public static long nowMs() { return System.nanoTime() / 1_000_000L; }

    /**
     * R103.B INIT pre-position of the servos (S1 lip gate stowed/open, S3 indexer closed, S6 hood
     * at the default setpoint); motors stay off. Call only where INIT motion is legal: Autonomous
     * INIT, or TeleOp INIT that is not during the AUTO->TELEOP transition (G403 note). Spans are
     * limited to the CAD travel (ServoLimits), so this cannot stall a servo on a hard stop.
     */
    public void prePositionServos() {
        intake.applyStartPositions();
        shooter.applyStartPositions();
    }

    /**
     * INIT-phase setup. Reads/configures sensors only -- commands NO actuator (servo pre-positioning
     * is the separate {@link #prePositionServos()}).
     * @param resetPose true to zero the Pinpoint pose + recalibrate its IMU (robot must be still).
     *                  TeleOp passes false after an Autonomous so the field pose survives.
     */
    public void init(boolean resetPose) {
        husky.init();
        if (resetPose) localizer.resetAndCalibrate();
    }

    public void setAlliance(Alliance a) {
        if (a != alliance) targetSide = a.initialUpSide();
        alliance = a;
        husky.setAlliance(a);
        webcamTags.setAlliance(a);
        intake.setAlliance(a);
        Alliance.lastUsed = a;
    }

    public Alliance getAlliance() { return alliance; }

    public HiveTracker.Side getTargetSide() { return targetSide; }

    public void setTargetSide(HiveTracker.Side side) { targetSide = side; }

    /** Restrict vision to the target CELL's cluster (Autonomous) or use the closest cluster (-1). */
    public void setVisionPreferTargetCell(boolean prefer) {
        int c = prefer ? alliance.cellClusterIndex(targetSide) : -1;
        husky.setPreferredCluster(c);
        webcamTags.setPreferredCluster(c);
    }

    /**
     * Sensor + gate update; call first in every loop. Returns whether powered motion is legal.
     */
    public boolean updateSensorsAndGates() {
        hw.clearBulkCache();
        long now = nowMs();
        localizer.update();
        husky.update();
        webcamTags.update();
        if (now - lastVoltageSampleMs >= RobotConstants.VOLTAGE_SAMPLE_MS) {
            batteryVoltage = hw.batteryVoltage();   // 1 un-cached Lynx command per hub -> only 4 Hz
            lastVoltageSampleMs = now;
        }
        balls.read(now);
        intake.setLipSensorFault(balls.isLipFault());          // lip FAULT -> colour gate auto-OFF
        balls.updateCount(now, shooter.isIndexerOpen(), intake.isMagazineReversing(),
                intake.isIntakeForward(), intake.isIntakeReversing(),
                intake.isRejecting(balls.getLipBall()), intake.isHolding());

        motionAllowed = clock.poweredMotionAllowed();
        drive.setEnabled(motionAllowed);
        shooter.setEnabled(motionAllowed);
        intake.setEnabled(motionAllowed);
        if (!motionAllowed) {
            hw.stopAllMotors();
        } else if (!startPositionsApplied) {
            intake.applyStartPositions();
            shooter.applyStartPositions();
            startPositionsApplied = true;
        }
        return motionAllowed;
    }

    public boolean isMotionAllowed() { return motionAllowed; }

    /** Best available target observation (HuskyLens preferred, webcam fallback), or null. */
    public TargetObservation bestTarget() {
        long now = System.nanoTime();
        TargetObservation h = husky.getLatest();
        if (h != null && h.ageMs(now) < RobotConstants.VISION_STALE_MS) return h;
        TargetObservation w = webcamTags.getLatest();
        if (w != null && w.ageMs(now) < RobotConstants.VISION_STALE_MS) return w;
        return null;
    }

    /**
     * Launch distance D (mm, shooter exit -> CELL lower lip, horizontal) for the ShotTable.
     * Vision ranges are SLANT ranges to the tag cluster (~0.8 m above the lens) and are converted
     * to horizontal first -- the old code fed slant range from vision but horizontal range from
     * odometry into the same table.
     */
    public double rangeToTargetMm() {
        TargetObservation t = bestTarget();
        if (t != null && t.hasRange()) {
            double horiz = ShotTable.slantToHorizontalMm(t.rangeMm,
                    RobotConstants.CELL_TAG_HEIGHT_MM - RobotConstants.CAMERA_HEIGHT_MM);
            return horiz + RobotConstants.VISION_RANGE_TO_D_OFFSET_MM;
        }
        return ShotTable.launchDistanceMm(localizer.getPose(), alliance.cellLipPoint(targetSide));
    }

    /** Field heading that points the shooter at the target CELL from {@code from} (odometry aim). */
    public double aimHeadingDeg(Pose from) {
        return MathUtil.wrapDeg(from.bearingTo(alliance.cellAimPoint(targetSide))
                - RobotConstants.SHOOTER_YAW_OFFSET_DEG);
    }

    /**
     * Mechanism update; call after the drive command each loop.
     * @param requestedIntake roller request (-1..1)
     * @param spinUp          flywheels on
     * @param fire            open burst gate when at speed (stays latched while held)
     */
    public void updateMechanisms(double requestedIntake, boolean spinUp, boolean fire) {
        long now = nowMs();
        // G408 downstream: opponent NECTAR at the throat -> S3 stays closed until it is reversed out.
        throatBlocked = throatGuard.update(now, balls.getThroatBall(), alliance,
                !intake.isColourGateOverride() && !balls.isThroatFault());
        shooter.setFireInhibit(throatBlocked);
        shooter.setNextBall(balls.getThroatBallForShot());
        shooter.setQueueHasNectar(balls.queueHasNectar());
        shooter.setRangeMm(rangeToTargetMm());
        shooter.setSpinning(spinUp);
        // Last-ball dwell (TeleOp gunner may release fire right after the last throat exit).
        boolean dwell = BurstPolicy.lastBallDwellActive(shooter.isIndexerOpen(), balls.getCount(), now,
                balls.getLastShotMs(), RobotConstants.TELEOP_LAST_BALL_DWELL_MS);
        shooter.update(batteryVoltage, now, fire || dwell);
        intake.update(now, balls.getCount(), balls.getLipBall(), requestedIntake, shooter.isIndexerOpen());
        if (!motionAllowed) hw.stopAllMotors(); // belt and braces for G403/G404
    }

    public double getBatteryVoltage() { return batteryVoltage; }

    /** Driver override of the G408 lip colour gate (and the throat fire block): true = OFF. */
    public void setColourGateOverride(boolean off) { intake.setColourGateOverride(off); }

    public boolean isColourGateOverride() { return intake.isColourGateOverride(); }

    /** True while an opponent NECTAR at the throat blocks firing. */
    public boolean isThroatBlocked() { return throatBlocked; }

    private String colourGateStatus() {
        if (intake.isColourGateOverride()) return "OFF (driver override)";
        if (balls.isLipFault()) return "OFF (lipColor FAULT)";
        return "ON";
    }

    /** Compact telemetry (R704.D: a few lines, DS only, 4 Hz). */
    public void addTelemetry(Telemetry t) {
        long now = nowMs();
        t.addLine(String.format(Locale.US, "%s | %s | %.0fs left | motion=%s",
                alliance, clock.phase(), clock.matchRemainingS(), motionAllowed ? "OK" : "LOCKED"));
        t.addLine("Pose " + localizer.getPose() + "  pinpoint=" + localizer.getStatus());
        TargetObservation tgt = bestTarget();
        double d = shooter.getRangeMm();
        t.addLine(String.format(Locale.US, "Target %s CELL: %s | D %.2f m %s",
                targetSide, tgt == null ? "odometry" : String.format(Locale.US, "yaw %.1f, %d tags",
                        tgt.yawErrorDeg, tgt.tagCount),
                d / 1000.0, ShotTable.outOfRange(d) ? String.format(Locale.US, "!! OUT-OF-RANGE (%.2f-%.2f m) !!",
                        RobotConstants.SHOT_WARN_MIN_D_MM / 1000.0, RobotConstants.SHOT_WARN_MAX_D_MM / 1000.0) : "ok"));
        t.addLine(String.format(Locale.US, "Shooter F %.0f/%.0f R %.0f/%.0f rpm (%.1f/%.1f m/s) %s ball=%s V=%.1f",
                shooter.front().getMeasuredRpm(), shooter.front().getTargetRpm(),
                shooter.rear().getMeasuredRpm(), shooter.rear().getTargetRpm(),
                shooter.frontSurfaceMps(), shooter.rearSurfaceMps(),
                shooter.atSpeed(now) ? "READY" : "...", shooter.getBallForSetpoint(), batteryVoltage));
        t.addLine(String.format(Locale.US, "Mag %d/4  lip=%s  block=%s  colour gate %s  jam: %s",
                balls.getCount(), balls.getLipBall(), intake.getBlockReason(), colourGateStatus(),
                intake.getJamStatus()));
        if (throatBlocked) t.addLine("!! OPPONENT NECTAR AT THROAT: firing blocked -> REVERSE it out (gamepad2 LT) !!");
        String sf = balls.getFaults();
        if (!sf.isEmpty()) t.addLine("!! SENSOR FAULT " + sf + " -> count by hand: gamepad2 X = 0, B = 4 !!");
        String ef = shooter.getEncoderFaults();
        if (!ef.isEmpty()) t.addLine("!! FLYWHEEL ENCODER FAULT " + ef + " -> timed spin-up fallback, check cable !!");
    }

    public void shutdown() {
        hw.stopAllMotors();
        webcamTags.close();
    }
}
