package org.firstinspires.ftc.teamcode.hivequeen.game;

import org.firstinspires.ftc.teamcode.hivequeen.RobotConstants;

/**
 * G407 / G408 front-lip auto-rejection logic (pure decision logic, no hardware).
 *
 * G407 "No more than 4 at a time. A ROBOT may not simultaneously CONTROL more than 4 SCORING
 * ELEMENTS." ... "REPEATED instances of a ROBOT CONTROLLING 5 or more SCORING ELEMENTS are
 * likely to be perceived as STRATEGIC, even if the ROBOT has a 'keep 4 and discard the rest'
 * sorting mechanism." Teams are encouraged to build "systems to prevent active pickup/intaking
 * of more than 4 SCORING ELEMENTS."
 *
 * G408 "Don't CONTROL opponent NECTAR. A ROBOT may not CONTROL the opponent's NECTAR."
 * "REPEATED instances of a ROBOT CONTROLLING an opponent's NECTAR are likely to be perceived as
 * STRATEGIC, even if the ROBOT has a sorting mechanism that automatically discards opponent
 * NECTAR by returning it to approximately its original state."
 *
 * Therefore HIVE-QUEEN never intakes-then-ejects. The decision is made at the mouth:
 *  - COLOUR GATE (G408): only POLLEN or OUR alliance NECTAR is accepted. Opponent NECTAR and an
 *    UNKNOWN colour are rejected: the roller reverses with the S1 lip gate still OPEN for
 *    REJECT_REVERSE_MS (the ball is pushed out, not into the closing bar), keeps reversing (gate
 *    open, max REJECT_MAX_REVERSE_MS) while the ball is still at the lip, then waits with the gate
 *    open until the lip reads empty, and only then drops the gate for REJECT_HOLD_MS.
 *    The colour gate can be disabled (driver override when the lip sensor misreads; automatically
 *    when the lip sensor is in FAULT) -- then the DRIVER is responsible for G408.
 *  - magazine count >= 4 (G407): drop the lip gate and refuse forward intake. When the 4th ball
 *    was just counted while intaking, a 5th may already be under the roller: a REJECT_REVERSE_MS
 *    reverse pulse runs with the gate still OPEN, then the gate drops and stays down while full.
 *    (The lip cannot "read empty" here: it sees our own 4th ball parked in queue slot 4.)
 */
public class IntakeGuard {

    public enum Reason { NONE, MAGAZINE_FULL, OPPONENT_NECTAR, UNKNOWN_BALL }

    /** Output of one decision step. */
    public static final class Command {
        public final boolean lipGateBlocked;
        public final double intakePower;
        public final Reason reason;

        Command(boolean lipGateBlocked, double intakePower, Reason reason) {
            this.lipGateBlocked = lipGateBlocked;
            this.intakePower = intakePower;
            this.reason = reason;
        }
    }

    private Alliance alliance;
    private boolean colourGateOverride = false;   // driver: colour gate OFF
    private boolean lipSensorFault = false;       // automatic: colour gate OFF

    // colour-reject sequence
    private long rejectStartMs = -1;
    private long gateDropMs = -1;
    private Reason rejectReason = Reason.NONE;
    // 5th-ball pulse
    private int lastCount = 0;
    private long fullPulseStartMs = -1;

    public IntakeGuard(Alliance alliance) { this.alliance = alliance; }

    public void setAlliance(Alliance alliance) { this.alliance = alliance; }

    /** Driver override: true = colour gate OFF (accept every colour; driver handles G408). */
    public void setColourGateOverride(boolean off) { colourGateOverride = off; }

    public boolean isColourGateOverride() { return colourGateOverride; }

    /** Lip colour sensor FAULT (reads &lt; 2 mm / NaN / missing) auto-disables the colour gate. */
    public void setLipSensorFault(boolean fault) { lipSensorFault = fault; }

    /** True when colour-based rejection is active (no override, lip sensor healthy). */
    public boolean isColourGateEnabled() { return !colourGateOverride && !lipSensorFault; }

    /** G408 colour gate decision for a ball at the lip (true = must not be taken in). */
    public boolean wouldReject(BallType lipBall) {
        return isColourGateEnabled() && rejectReasonFor(lipBall) != Reason.NONE;
    }

    /** True while a colour-reject sequence (reverse / wait / gate hold) is running. */
    public boolean isRejecting() { return rejectStartMs >= 0; }

    private Reason rejectReasonFor(BallType lipBall) {
        if (lipBall == alliance.opponentNectar()) return Reason.OPPONENT_NECTAR;
        if (lipBall == BallType.UNKNOWN) return Reason.UNKNOWN_BALL;
        return Reason.NONE;
    }

    /**
     * @param nowMs              monotonic time in ms
     * @param magazineCount      balls currently held (from BallSensors)
     * @param lipBall            classification of whatever is at the front scoop
     * @param requestedIntake    driver/auto requested roller power (-1..1, + = intake)
     */
    public Command update(long nowMs, int magazineCount, BallType lipBall, double requestedIntake) {
        boolean full = magazineCount >= RobotConstants.MAX_CONTROLLED_ELEMENTS;
        // The 4th ball was just counted while we were intaking: a 5th ball may already be under the
        // roller. A short reverse pulse returns it (G407 example C: MOMENTARY control of a 5th ball
        // that is "reversed quickly" is not STRATEGIC).
        if (full && lastCount < RobotConstants.MAX_CONTROLLED_ELEMENTS && requestedIntake > 0) {
            fullPulseStartMs = nowMs;
        }
        lastCount = magazineCount;
        long sincePulse = fullPulseStartMs < 0 ? Long.MAX_VALUE : nowMs - fullPulseStartMs;
        if (sincePulse >= RobotConstants.REJECT_REVERSE_MS + RobotConstants.REJECT_HOLD_MS) fullPulseStartMs = -1;

        // 5th-ball pulse: gate OPEN while reversing, then down. The post-pulse hold also applies if
        // the pulse pushed our own 4th ball back out (count drops to 3), so it cannot oscillate fast.
        if (fullPulseStartMs >= 0) {
            if (sincePulse < RobotConstants.REJECT_REVERSE_MS) {
                return new Command(false, RobotConstants.INTAKE_REVERSE_POWER, Reason.MAGAZINE_FULL);
            }
            return new Command(true, Math.min(0.0, requestedIntake), Reason.MAGAZINE_FULL);
        }

        // Magazine full (G407): gate down, forward intake forbidden; driver may still outtake.
        // No colour rejects while full: the lip then sees our own 4th ball in slot 4.
        if (full) {
            rejectStartMs = -1;
            gateDropMs = -1;
            return new Command(true, Math.min(0.0, requestedIntake), Reason.MAGAZINE_FULL);
        }

        // Colour gate (G408): opponent NECTAR or UNKNOWN at the lip.
        if (wouldReject(lipBall) && rejectStartMs < 0) {
            rejectStartMs = nowMs;
            gateDropMs = -1;
            rejectReason = rejectReasonFor(lipBall);
        }
        if (rejectStartMs >= 0) {
            long dt = nowMs - rejectStartMs;
            boolean lipEmpty = lipBall == BallType.NONE;
            if (gateDropMs < 0) {
                if (dt < RobotConstants.REJECT_REVERSE_MS) {
                    // gate OPEN: push the ball out of the mouth before anything closes on it
                    return new Command(false, RobotConstants.INTAKE_REVERSE_POWER, rejectReason);
                }
                if (!lipEmpty) {
                    // still at the lip: keep pushing for a while, then just wait (gate open, roller
                    // never pulling) -- never drop the bar onto the ball
                    double p = dt < RobotConstants.REJECT_MAX_REVERSE_MS
                            ? RobotConstants.INTAKE_REVERSE_POWER : Math.min(0.0, requestedIntake);
                    return new Command(false, p, rejectReason);
                }
                gateDropMs = nowMs;   // lip empty: now the gate may drop
            }
            if (nowMs - gateDropMs < RobotConstants.REJECT_HOLD_MS) {
                return new Command(true, Math.min(0.0, requestedIntake), rejectReason);
            }
            rejectStartMs = -1;   // clear, fall through
            gateDropMs = -1;
            rejectReason = Reason.NONE;
        }

        return new Command(false, requestedIntake, Reason.NONE);
    }
}
