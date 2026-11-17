package org.firstinspires.ftc.teamcode.hivequeen.game;

import org.firstinspires.ftc.teamcode.hivequeen.RobotConstants;

/**
 * Servo travel limits (pure logic). goBILDA 2000-0025-0002 Dual Mode servo in standard mode spans
 * SERVO_RANGE_DEG (300 deg) over position 0..1. Every mechanism command is clamped to the two
 * calibrated end positions of that mechanism, so a bad setpoint can never drive a printed part
 * into its hard stop. The end positions themselves are set on the bench (HiveQueenHardwareTest).
 */
public final class ServoLimits {
    private ServoLimits() { }

    /** Mechanism angle between two servo positions (deg). */
    public static double spanDeg(double posA, double posB) {
        return Math.abs(posA - posB) * RobotConstants.SERVO_RANGE_DEG;
    }

    /** Position {@code deg} away from {@code basePos} in direction {@code dir} (+1 / -1). */
    public static double positionAt(double basePos, double deg, double dir) {
        return basePos + Math.signum(dir) * deg / RobotConstants.SERVO_RANGE_DEG;
    }

    /** Clamp {@code pos} between the two end positions (any order) and to 0..1. */
    public static double clampBetween(double pos, double endA, double endB) {
        double lo = Math.max(0.0, Math.min(endA, endB));
        double hi = Math.min(1.0, Math.max(endA, endB));
        if (Double.isNaN(pos)) return lo;
        return Math.max(lo, Math.min(hi, pos));
    }

    public static double lipGate(double pos) {
        return clampBetween(pos, RobotConstants.LIP_GATE_OPEN, RobotConstants.LIP_GATE_BLOCKED);
    }

    public static double indexer(double pos) {
        return clampBetween(pos, RobotConstants.INDEXER_CLOSED, RobotConstants.INDEXER_OPEN);
    }

    public static double hood(double pos) {
        return clampBetween(pos, RobotConstants.HOOD_MIN_POS, RobotConstants.HOOD_MAX_POS);
    }
}
