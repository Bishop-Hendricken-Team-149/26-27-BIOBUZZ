package org.firstinspires.ftc.teamcode.hivequeen.game;

import org.firstinspires.ftc.teamcode.hivequeen.RobotConstants;

/**
 * Pure colour/size classifier for SCORING ELEMENTS (Sec. 9.8: POLLEN yellow, NECTAR red / blue).
 * Kept free of FTC SDK types so it can be unit-tested offline; BallSensors feeds it the
 * NormalizedRGBA channels.
 */
public final class BallClassifier {
    private BallClassifier() { }

    /**
     * Hue windows: red ~0-25 / 335-360, yellow (POLLEN) ~35-75, blue ~190-250 degrees.
     * Low saturation -> UNKNOWN. Distance invalid (NaN / &lt; MIN_VALID_MM = sensor FAULT) or not
     * closer than {@code presentMm} -> NONE (a dead sensor reading 0 mm is never a ball). TUNE on real balls.
     */
    public static BallType classify(float r, float g, float b, double distMm, double presentMm) {
        if (!present(distMm, presentMm)) return BallType.NONE;
        float[] hsv = rgbToHsv(r, g, b);
        float hue = hsv[0], sat = hsv[1];
        if (sat < RobotConstants.COLOR_MIN_SATURATION) return BallType.UNKNOWN;
        if (hue < 25f || hue > 335f) return BallType.RED_NECTAR;
        if (hue >= 35f && hue <= 75f) return BallType.POLLEN;
        if (hue >= 190f && hue <= 250f) return BallType.BLUE_NECTAR;
        return BallType.UNKNOWN;
    }

    /** RGB (any non-negative scale) -> {hue 0-360, saturation 0-1, value}; pure Java (no android.graphics). */
    public static float[] rgbToHsv(float r, float g, float b) {
        float max = Math.max(r, Math.max(g, b));
        float min = Math.min(r, Math.min(g, b));
        float d = max - min;
        float h;
        if (d <= 1e-6f) h = 0f;
        else if (max == r) h = 60f * (((g - b) / d) % 6f);
        else if (max == g) h = 60f * (((b - r) / d) + 2f);
        else h = 60f * (((r - g) / d) + 4f);
        if (h < 0) h += 360f;
        float s = max <= 1e-6f ? 0f : d / max;
        return new float[]{h, s, max};
    }

    /**
     * Ball class used ONLY to pick the shooter SETPOINT when colour is ambiguous: size decides (an
     * ambiguous large ball maps onto a NECTAR value; only isNectar() is used by the ShotTable).
     * Never feed this into the HIVE mass model -- use {@link #forTracker}.
     */
    public static BallType forShot(BallType colourClass, boolean isLarge) {
        if (colourClass == BallType.UNKNOWN) return isLarge ? BallType.RED_NECTAR : BallType.POLLEN;
        return colourClass;
    }

    /**
     * Ball class for the HIVE mass model (HiveTracker): an unclassified ball counts as POLLEN (the
     * lighter mass, so a tip is never assumed too early), whatever its apparent size.
     */
    public static BallType forTracker(BallType colourClass) {
        if (colourClass == null || colourClass == BallType.UNKNOWN || colourClass == BallType.NONE) return BallType.POLLEN;
        return colourClass;
    }

    /**
     * Distance readings below this are physically impossible for our mounts (the sensors look
     * across the corridor through shroud windows), so they mean a fault: an unplugged/failed I2C
     * device typically reads 0. A fault must NEVER be treated as "ball present", or a dead
     * occupancy sensor would raise the count and could lock the intake for the whole match.
     */
    public static final double MIN_VALID_MM = 2.0;

    public static boolean validMm(double mm) { return !Double.isNaN(mm) && !Double.isInfinite(mm) && mm >= MIN_VALID_MM; }

    /** True only for a VALID reading closer than the threshold. */
    public static boolean present(double mm, double thresholdMm) { return validMm(mm) && mm < thresholdMm; }
}
