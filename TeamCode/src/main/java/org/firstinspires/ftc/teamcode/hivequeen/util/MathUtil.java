package org.firstinspires.ftc.teamcode.hivequeen.util;

/** Small math helpers (Java 8 compatible, no allocations). */
public final class MathUtil {
    private MathUtil() { }

    public static double clamp(double v, double lo, double hi) {
        return v < lo ? lo : (v > hi ? hi : v);
    }

    /** Wrap an angle in degrees to (-180, 180]. */
    public static double wrapDeg(double deg) {
        double a = deg % 360.0;
        if (a <= -180.0) a += 360.0;
        if (a > 180.0) a -= 360.0;
        return a;
    }

    public static double deadband(double v, double band) {
        return Math.abs(v) < band ? 0.0 : v;
    }

    /** Linear interpolation of y over sorted xs (clamped at the ends). */
    public static double interpolate(double[] xs, double[] ys, double x) {
        if (x <= xs[0]) return ys[0];
        int n = xs.length;
        if (x >= xs[n - 1]) return ys[n - 1];
        for (int i = 1; i < n; i++) {
            if (x <= xs[i]) {
                double t = (x - xs[i - 1]) / (xs[i] - xs[i - 1]);
                return ys[i - 1] + t * (ys[i] - ys[i - 1]);
            }
        }
        return ys[n - 1];
    }

    /** Limit how fast a value may RISE per second (falling is never limited). */
    public static double slewUp(double prev, double want, long dtMs, double ratePerS) {
        if (want <= prev) return want;
        return Math.min(want, prev + ratePerS * Math.max(0, dtMs) / 1000.0);
    }
}
