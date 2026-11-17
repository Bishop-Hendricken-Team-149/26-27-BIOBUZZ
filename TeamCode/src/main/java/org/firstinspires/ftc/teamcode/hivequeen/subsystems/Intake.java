package org.firstinspires.ftc.teamcode.hivequeen.subsystems;

import org.firstinspires.ftc.teamcode.hivequeen.RobotConstants;
import org.firstinspires.ftc.teamcode.hivequeen.RobotHardware;
import org.firstinspires.ftc.teamcode.hivequeen.game.Alliance;
import org.firstinspires.ftc.teamcode.hivequeen.game.BallType;
import org.firstinspires.ftc.teamcode.hivequeen.game.IntakeGuard;
import org.firstinspires.ftc.teamcode.hivequeen.game.JamDetector;
import org.firstinspires.ftc.teamcode.hivequeen.game.ServoLimits;

/**
 * M7 floating compliant roller intake + S1 front-lip blocker gate + M8 gecko-wheel magazine.
 * All intake commands pass through {@link IntakeGuard} (G407 / G408) before reaching hardware:
 * the robot refuses a 5th ball, an opponent NECTAR or an UNKNOWN colour at the mouth instead of
 * sorting inside (colour gate can be overridden by the driver / auto-disabled on a lip FAULT).
 *
 * Magazine modes: FEED (indexer open) | INDEX (intaking) | driver outtake (reverse) |
 * HOLD (balls inside, otherwise idle: MAGAZINE_HOLD_POWER keeps the queue pressed on the closed
 * S3 gate so balls do not roll back; BallCounter reads occupancy only in this mode) | off.
 */
public class Intake {
    private final RobotHardware hw;
    private final IntakeGuard guard;
    private boolean enabled = true;
    private IntakeGuard.Command lastCommand;
    private boolean magazineReversing = false;
    private boolean holding = false;
    private double appliedIntakePower = 0.0;
    /** Stall/jam handling. Needs the intake and magazine ENCODER cables plugged in (auto-disables if not). */
    private final JamDetector intakeJam = new JamDetector(RobotConstants.INTAKE_FREE_TPS);
    private final JamDetector magazineJam = new JamDetector(RobotConstants.MAGAZINE_FREE_TPS);

    public Intake(RobotHardware hw, Alliance alliance) {
        this.hw = hw;
        this.guard = new IntakeGuard(alliance);
    }

    public void setAlliance(Alliance a) { guard.setAlliance(a); }

    /** Driver override of the G408 colour gate (true = OFF). */
    public void setColourGateOverride(boolean off) { guard.setColourGateOverride(off); }

    public boolean isColourGateOverride() { return guard.isColourGateOverride(); }

    /** Lip colour sensor FAULT -> colour gate automatically OFF. */
    public void setLipSensorFault(boolean fault) { guard.setLipSensorFault(fault); }

    public boolean isColourGateEnabled() { return guard.isColourGateEnabled(); }

    /** True if the ball now at the lip is (or will be) rejected -- BallCounter must not count it. */
    public boolean isRejecting(BallType lipBall) { return guard.isRejecting() || guard.wouldReject(lipBall); }

    /** G403/G404 gate. */
    public void setEnabled(boolean enabled) {
        this.enabled = enabled;
        if (!enabled) {
            hw.intake.setPower(0);
            hw.magazine.setPower(0);
            magazineReversing = false;
            holding = false;
            appliedIntakePower = 0.0;
        }
    }

    /** Lip gate stowed (mouth open). Called in INIT (R103.B pre-position) and again at START. */
    public void applyStartPositions() {
        hw.lipGate.setPosition(ServoLimits.lipGate(RobotConstants.LIP_GATE_OPEN));
    }

    /**
     * @param requestedIntake driver/auto roller request (-1..1, + = in)
     * @param feedShooter     true when the shooter's indexer is open (magazine pushes balls up)
     */
    public void update(long nowMs, int magazineCount, BallType lipBall, double requestedIntake, boolean feedShooter) {
        lastCommand = guard.update(nowMs, magazineCount, lipBall, requestedIntake);
        if (!enabled) return;
        hw.lipGate.setPosition(ServoLimits.lipGate(
                lastCommand.lipGateBlocked ? RobotConstants.LIP_GATE_BLOCKED : RobotConstants.LIP_GATE_OPEN));
        appliedIntakePower = intakeJam.update(nowMs, lastCommand.intakePower, hw.intake.getVelocity());
        hw.intake.setPower(appliedIntakePower);

        double mag;
        boolean hold = false;
        if (feedShooter) mag = RobotConstants.MAGAZINE_FEED_POWER;
        else if (lastCommand.intakePower > 0) mag = RobotConstants.MAGAZINE_INDEX_POWER; // stage balls
        // Only a DRIVER outtake backs balls out of the magazine. A guard-generated reverse pulse
        // (rejected ball at the lip, 5th-ball pulse) must not push OUR balls back out toward it.
        else if (lastCommand.intakePower < 0 && requestedIntake < 0) mag = -RobotConstants.MAGAZINE_INDEX_POWER;
        // 5th-ball pulse: the roller reverses next to our 4th ball (slot 4) -- push the queue in. TUNE
        else if (lastCommand.intakePower < 0 && lastCommand.reason == IntakeGuard.Reason.MAGAZINE_FULL
                && magazineCount > 0) mag = RobotConstants.MAGAZINE_INDEX_POWER;
        else if (magazineCount > 0) { mag = RobotConstants.MAGAZINE_HOLD_POWER; hold = true; }
        else mag = 0.0;
        // Jam handling on the magazine only while FEEDING (gate open). While staging/holding, a full
        // queue pressed against the closed gate legitimately stalls the geckos and is not a jam.
        double magOut;
        if (feedShooter) {
            magOut = magazineJam.update(nowMs, mag, hw.magazine.getVelocity());
        } else {
            magazineJam.update(nowMs, 0.0, hw.magazine.getVelocity());   // keeps its clock/encoder check current
            magOut = mag;
        }
        hw.magazine.setPower(magOut);
        magazineReversing = magOut < 0;
        holding = hold;
    }

    /** True while the magazine runs backwards (driver outtake or an unjam pulse while feeding). */
    public boolean isMagazineReversing() { return magazineReversing; }

    /** True while the magazine holds the queue against the closed gate at MAGAZINE_HOLD_POWER. */
    public boolean isHolding() { return holding; }

    /** Applied roller power (after the guard and the jam detector), for the entry counter. */
    public boolean isIntakeForward() { return appliedIntakePower > 0.05; }

    public boolean isIntakeReversing() { return appliedIntakePower < -0.05; }

    /** Telemetry: jam detector states (OK / UNJAM / COOLDOWN / NO_ENCODER). */
    public String getJamStatus() { return "intake " + intakeJam.getState() + ", mag " + magazineJam.getState(); }

    public IntakeGuard.Reason getBlockReason() {
        return lastCommand == null ? IntakeGuard.Reason.NONE : lastCommand.reason;
    }
}
