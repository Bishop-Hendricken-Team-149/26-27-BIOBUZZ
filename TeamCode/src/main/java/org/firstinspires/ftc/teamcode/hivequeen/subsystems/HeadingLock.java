package org.firstinspires.ftc.teamcode.hivequeen.subsystems;

import org.firstinspires.ftc.teamcode.hivequeen.RobotConstants;
import org.firstinspires.ftc.teamcode.hivequeen.util.MathUtil;

/**
 * Driver heading-lock assist for the fixed-yaw shooter.
 *
 * While the driver holds "aim", the turn stick is replaced by a P(+kS) controller on the
 * vision yaw error to the alliance CELL AprilTag cluster. Every fresh vision frame is converted
 * to an absolute FIELD heading target (Pinpoint heading + yaw error); between frames, or if
 * vision drops out briefly, the lock keeps holding that field heading using Pinpoint, so the
 * loop runs at full rate even though the HuskyLens only updates at ~20 Hz.
 */
public class HeadingLock {
    private double targetFieldHeadingDeg = Double.NaN;
    private double lastErrorDeg = Double.NaN;

    /** Clears the target (call when the driver releases the aim button). */
    public void reset() {
        targetFieldHeadingDeg = Double.NaN;
        lastErrorDeg = Double.NaN;
    }

    /**
     * @param obs             latest vision observation (may be null)
     * @param robotHeadingDeg current Pinpoint heading
     * @param nowNanos        System.nanoTime()
     * @return turn command (+ = CCW), or NaN if there is no target at all (driver keeps control)
     */
    public double update(TargetObservation obs, double robotHeadingDeg, long nowNanos) {
        if (obs != null && obs.ageMs(nowNanos) < RobotConstants.VISION_STALE_MS) {
            // NOTE: the observation is up to ~50 ms old; the heading at capture time is close
            // enough at aiming speeds. Refine with a heading history buffer if needed.
            targetFieldHeadingDeg = robotHeadingDeg + obs.yawErrorDeg;
        }
        if (Double.isNaN(targetFieldHeadingDeg)) {
            lastErrorDeg = Double.NaN;
            return Double.NaN;
        }
        double err = MathUtil.wrapDeg(targetFieldHeadingDeg - robotHeadingDeg);
        lastErrorDeg = err;
        if (Math.abs(err) < RobotConstants.HEADING_LOCK_TOL_DEG) return 0.0;
        double turn = RobotConstants.HEADING_LOCK_KP * err + Math.signum(err) * RobotConstants.HEADING_LOCK_KS;
        return MathUtil.clamp(turn, -RobotConstants.HEADING_LOCK_MAX_TURN, RobotConstants.HEADING_LOCK_MAX_TURN);
    }

    public boolean isOnTarget() {
        return !Double.isNaN(lastErrorDeg) && Math.abs(lastErrorDeg) < RobotConstants.HEADING_LOCK_TOL_DEG;
    }

    public double getLastErrorDeg() { return lastErrorDeg; }
}
