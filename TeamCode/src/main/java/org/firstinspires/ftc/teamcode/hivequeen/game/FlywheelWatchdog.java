package org.firstinspires.ftc.teamcode.hivequeen.game;

import org.firstinspires.ftc.teamcode.hivequeen.RobotConstants;

/**
 * Missing/unplugged flywheel ENCODER detection (pure logic). If the motor is driven with
 * >= FLYWHEEL_ENC_FAULT_MIN_POWER for more than FLYWHEEL_ENC_FAULT_MS while the encoder reads
 * below FLYWHEEL_ENC_FAULT_MAX_RPM, the encoder is declared FAULT. (A stalled wheel looks the same;
 * a 1150 RPM flywheel cannot stay stalled at that power for a second in normal use.) The fault
 * clears if the encoder later reports real motion (cable re-seated).
 *
 * In FAULT the controller runs feedforward only, and "at speed" is time based: the wheel has been
 * commanded continuously for FLYWHEEL_FALLBACK_SPINUP_MS (TUNE) -- so the robot can still shoot.
 */
public final class FlywheelWatchdog {
    private boolean fault = false;
    private long highPowerSinceMs = -1;
    private long commandedSinceMs = -1;
    private double lastTargetRpm = 0.0;

    public void update(long nowMs, double appliedPower, double measuredRpm, double targetRpm) {
        boolean moving = Math.abs(measuredRpm) > RobotConstants.FLYWHEEL_ENC_FAULT_MAX_RPM;
        if (appliedPower >= RobotConstants.FLYWHEEL_ENC_FAULT_MIN_POWER && !moving) {
            if (highPowerSinceMs < 0) highPowerSinceMs = nowMs;
            if (nowMs - highPowerSinceMs > RobotConstants.FLYWHEEL_ENC_FAULT_MS) fault = true;
        } else {
            highPowerSinceMs = -1;
        }
        if (moving && Math.abs(measuredRpm) > 2.0 * RobotConstants.FLYWHEEL_ENC_FAULT_MAX_RPM) fault = false;

        // time-based spin-up clock: restarts whenever the wheel is (re)commanded or the target rises
        if (targetRpm <= 0.0) commandedSinceMs = -1;
        else if (commandedSinceMs < 0 || targetRpm > lastTargetRpm + RobotConstants.FLYWHEEL_TOLERANCE_RPM) commandedSinceMs = nowMs;
        lastTargetRpm = targetRpm;
    }

    public boolean isFault() { return fault; }

    /** Time-based "at speed" used while the encoder is in FAULT. */
    public boolean fallbackAtSpeed(long nowMs) {
        return commandedSinceMs >= 0 && nowMs - commandedSinceMs >= RobotConstants.FLYWHEEL_FALLBACK_SPINUP_MS;
    }
}
