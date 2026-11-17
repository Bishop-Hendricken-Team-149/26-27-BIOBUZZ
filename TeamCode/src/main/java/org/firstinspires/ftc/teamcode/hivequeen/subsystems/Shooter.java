package org.firstinspires.ftc.teamcode.hivequeen.subsystems;

import org.firstinspires.ftc.teamcode.hivequeen.RobotConstants;
import org.firstinspires.ftc.teamcode.hivequeen.RobotHardware;
import org.firstinspires.ftc.teamcode.hivequeen.game.BallType;
import org.firstinspires.ftc.teamcode.hivequeen.game.BurstPolicy;
import org.firstinspires.ftc.teamcode.hivequeen.game.ServoLimits;

/**
 * Fixed-yaw, dual-flywheel, vertical-nip topspin shooter.
 *  - M5 shooterFront: front dia 96 mm Rhino wheel, 1:1 (20T->20T HTD-5M)
 *  - M6 shooterRear : rear dia 72 mm counter-wheel, 2:1 overdrive (32T->16T)
 *  - S6 hoodPitch   : exit-angle hood
 *  - S3 indexerGate : upper-throat burst stopper (opened only when both wheels are at speed)
 *
 * Setpoints come from {@link ShotTable} using the ball class and the range to the target. Because
 * the hardware cannot re-check "at speed" between the packed balls of a burst (see
 * {@link BurstPolicy}), the setpoint is the MAX over the next ball (throat) and every ball in the
 * queue, and it is LATCHED while the indexer is open.
 *
 * Fire inhibit: an opponent NECTAR at the throat (game/ThroatGuard) keeps S3 closed (G408).
 */
public class Shooter {
    private final RobotHardware hw;
    private final FlywheelController front, rear;
    private boolean spinning = false;
    private boolean enabled = true;
    private BallType ballForSetpoint = BallType.POLLEN;
    private double rangeMm = ShotTable.DEFAULT_RANGE_MM;
    private ShotTable.Setpoint setpoint = ShotTable.lookup(BallType.POLLEN, ShotTable.DEFAULT_RANGE_MM);
    private boolean indexerOpen = false;
    private boolean fireInhibit = false;
    private boolean queueHasNectar = false;
    private BallType nextBall = BallType.NONE;

    public Shooter(RobotHardware hw) {
        this.hw = hw;
        front = new FlywheelController(hw.shooterFront);
        rear = new FlywheelController(hw.shooterRear);
    }

    /**
     * S3 closed + S6 hood at the default setpoint. Called in INIT (R103.B: an OpMode may pre-position
     * servos in INIT, robot then stays still -- never during the AUTO->TELEOP transition, G403) and
     * again at the first legal loop.
     */
    public void applyStartPositions() {
        hw.indexerGate.setPosition(ServoLimits.indexer(RobotConstants.INDEXER_CLOSED));
        hw.hoodPitch.setPosition(ServoLimits.hood(setpoint.hood));
        indexerOpen = false;
    }

    /** G403/G404 gate. Disabling zeroes both flywheels (they coast -- inertia is allowed). */
    public void setEnabled(boolean enabled) { this.enabled = enabled; }

    public void setSpinning(boolean on) { spinning = on; }

    /** Front (M5, dia 96 mm, 1:1) wheel surface speed at the measured motor RPM, m/s. */
    public double frontSurfaceMps() {
        return ShotTable.surfaceSpeedMps(front.getMeasuredRpm(), RobotConstants.FRONT_WHEEL_DIAMETER_MM,
                RobotConstants.FRONT_WHEEL_RATIO);
    }

    /** Counter-wheel (M6, dia 72 mm, 32T:16T = wheel turns 2x motor) surface speed, m/s. */
    public double rearSurfaceMps() {
        return ShotTable.surfaceSpeedMps(rear.getMeasuredRpm(), RobotConstants.REAR_WHEEL_DIAMETER_MM,
                RobotConstants.REAR_WHEEL_RATIO);
    }

    public boolean isSpinning() { return spinning; }

    /** Ball classification from the throat sensor (size-aware, see BallSensors.getThroatBallForShot). */
    public void setNextBall(BallType ball) { nextBall = ball == null ? BallType.NONE : ball; }

    /** True if any ball recorded in the magazine queue is NECTAR (burst setpoint = max). */
    public void setQueueHasNectar(boolean nectar) { queueHasNectar = nectar; }

    /** True = never open S3 (opponent NECTAR at the throat); closes it if open. */
    public void setFireInhibit(boolean inhibit) { fireInhibit = inhibit; }

    public boolean isFireInhibited() { return fireInhibit; }

    public void setRangeMm(double rangeMm) {
        if (!Double.isNaN(rangeMm)) this.rangeMm = rangeMm;
    }

    /** Request the burst gate; it only actually opens when at speed. */
    public void update(double batteryVoltage, long nowMs, boolean wantFire) {
        // Burst setpoint = max of the queue; latched (not re-picked) while the gate is open.
        if (!indexerOpen && (nextBall.isBall() || queueHasNectar)) {
            ballForSetpoint = BurstPolicy.burstBall(nextBall, queueHasNectar);
        }
        setpoint = ShotTable.lookup(ballForSetpoint, rangeMm);
        boolean run = enabled && spinning;
        front.setTargetRpm(run ? setpoint.frontRpm : 0.0);
        rear.setTargetRpm(run ? setpoint.rearRpm : 0.0);
        front.update(batteryVoltage, nowMs);
        rear.update(batteryVoltage, nowMs);

        if (!enabled) { indexerOpen = false; return; } // G403/G404: don't command servos while motion is illegal
        hw.hoodPitch.setPosition(ServoLimits.hood(setpoint.hood));
        // Burst fire: the gate OPENS only when both wheels are at speed, then stays LATCHED open
        // while fire is held. Each shot dips the wheel speed out of tolerance; re-closing then
        // (old behaviour) stopped the magazine mid-burst and could pinch a ball in the fingers,
        // and starved the last ball of the full-power push it needs across the 53 mm gap.
        indexerOpen = wantFire && run && !fireInhibit && (indexerOpen || atSpeed(nowMs));
        hw.indexerGate.setPosition(ServoLimits.indexer(indexerOpen ? RobotConstants.INDEXER_OPEN : RobotConstants.INDEXER_CLOSED));
    }

    public boolean atSpeed(long nowMs) { return front.atSpeed(nowMs) && rear.atSpeed(nowMs); }

    /** Telemetry: "" when healthy, else which flywheel encoder is in FAULT (time-based fallback active). */
    public String getEncoderFaults() {
        return ((front.isEncoderFault() ? "front " : "") + (rear.isEncoderFault() ? "rear" : "")).trim();
    }

    public boolean isIndexerOpen() { return indexerOpen; }

    public ShotTable.Setpoint getSetpoint() { return setpoint; }

    public BallType getBallForSetpoint() { return ballForSetpoint; }

    public double getRangeMm() { return rangeMm; }

    public FlywheelController front() { return front; }

    public FlywheelController rear() { return rear; }
}
