package org.firstinspires.ftc.teamcode.hivequeen.util;

import java.util.Locale;

/** Immutable 2-D pose: x/y in millimetres, heading in degrees (CCW +), field frame. */
public final class Pose {
    public final double xMm;
    public final double yMm;
    public final double headingDeg;

    public Pose(double xMm, double yMm, double headingDeg) {
        this.xMm = xMm;
        this.yMm = yMm;
        this.headingDeg = headingDeg;
    }

    public double distanceTo(Pose o) { return Math.hypot(o.xMm - xMm, o.yMm - yMm); }

    /** Field-frame bearing (deg) from this pose to {@code o}. */
    public double bearingTo(Pose o) { return Math.toDegrees(Math.atan2(o.yMm - yMm, o.xMm - xMm)); }

    /** 180-degree rotation about the field centre (RED -> BLUE on the rotationally symmetric field). */
    public Pose rotated180() { return new Pose(-xMm, -yMm, MathUtil.wrapDeg(headingDeg + 180.0)); }

    /** Mirror across the field Y axis (X -> -X): audience side <-> rear side of the same HIVE. */
    public Pose mirroredX() { return new Pose(-xMm, yMm, MathUtil.wrapDeg(180.0 - headingDeg)); }

    public Pose withHeading(double h) { return new Pose(xMm, yMm, h); }

    @Override
    public String toString() {
        return String.format(Locale.US, "(%.0f mm, %.0f mm, %.1f deg)", xMm, yMm, headingDeg);
    }
}
