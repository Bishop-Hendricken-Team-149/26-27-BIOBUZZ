package org.firstinspires.ftc.teamcode.hivequeen.subsystems;

/** A vision measurement of the alliance CELL AprilTag cluster. Immutable. */
public final class TargetObservation {
    /** Yaw error of the cluster centroid relative to the shooter axis, deg (+ = target is LEFT). */
    public final double yawErrorDeg;
    /** Estimated range to the cluster, mm (NaN when unknown). */
    public final double rangeMm;
    /** Number of tags of the chosen cluster that were visible (1..4). */
    public final int tagCount;
    /** System.nanoTime() of the measurement. */
    public final long timestampNanos;

    public TargetObservation(double yawErrorDeg, double rangeMm, int tagCount, long timestampNanos) {
        this.yawErrorDeg = yawErrorDeg;
        this.rangeMm = rangeMm;
        this.tagCount = tagCount;
        this.timestampNanos = timestampNanos;
    }

    public double ageMs(long nowNanos) { return (nowNanos - timestampNanos) / 1e6; }

    public boolean hasRange() { return !Double.isNaN(rangeMm); }
}
