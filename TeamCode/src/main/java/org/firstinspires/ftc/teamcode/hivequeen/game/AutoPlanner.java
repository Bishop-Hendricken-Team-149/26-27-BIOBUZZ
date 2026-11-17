package org.firstinspires.ftc.teamcode.hivequeen.game;

import org.firstinspires.ftc.teamcode.hivequeen.RobotConstants;

/**
 * Pure time-budget logic for the Autonomous state machine (unit-tested offline).
 *
 * Rules:
 *  - At or after {@link RobotConstants#AUTO_PARK_ABORT_S} the only legal action is PARK.
 *  - A new action is started only if its estimated duration still ends before the abort time.
 *  - Every step's timeout is clipped so it can never run past the abort time; the PARK step's
 *    timeout is clipped to {@link RobotConstants#AUTO_PARK_DEADLINE_S} (< 30 s AUTO end).
 */
public final class AutoPlanner {
    private AutoPlanner() { }

    public enum Action { SHOOT, COLLECT_SPILL, COLLECT_LOADING_ZONE, PARK }

    /**
     * True if the straight segment between two field points brings the robot body into the HIVE
     * frame footprint (49.46 x 38.95 in at field centre, Sec. 9.6.1) inflated by the robot
     * half-width -> the route needs a via point (also avoids G417 contact with the frame).
     */
    public static boolean segmentHitsHiveFrame(double x1, double y1, double x2, double y2) {
        double hx = 0.5 * 38.95 * 25.4 + RobotConstants.ROBOT_HALF_WIDTH_MM;   // along X (audience<->rear)
        double hy = 0.5 * 49.46 * 25.4 + RobotConstants.ROBOT_HALF_WIDTH_MM;   // along Y (red<->blue)
        for (int i = 0; i <= 50; i++) {
            double t = i / 50.0;
            double x = x1 + t * (x2 - x1), y = y1 + t * (y2 - y1);
            if (Math.abs(x) <= hx && Math.abs(y) <= hy) return true;
        }
        return false;
    }

    public static boolean mustPark(double elapsedS) {
        return elapsedS >= RobotConstants.AUTO_PARK_ABORT_S;
    }

    /** True if an action estimated at {@code estimateS} started now still ends before the abort time. */
    public static boolean fits(double elapsedS, double estimateS) {
        return elapsedS + estimateS <= RobotConstants.AUTO_PARK_ABORT_S;
    }

    /** Timeout for a non-PARK step started at {@code elapsedS}: min(nominal, time left before abort), >= 0. */
    public static double stepTimeoutS(double elapsedS, double nominalS) {
        double left = RobotConstants.AUTO_PARK_ABORT_S - elapsedS;
        return Math.max(0.0, Math.min(nominalS, left));
    }

    /** Timeout for the PARK drive started at {@code elapsedS}: until the park deadline, >= 0. */
    public static double parkTimeoutS(double elapsedS) {
        return Math.max(0.0, RobotConstants.AUTO_PARK_DEADLINE_S - elapsedS);
    }

    /**
     * Decide the next high-level action.
     * @param elapsedS        seconds since AUTO start
     * @param ballsHeld       magazine count
     * @param shootingAllowed false after repeated FIRE failures (jam / sensor fault)
     * @param spillBalls      balls believed still lying in the last tip's spill area (0 = none / swept)
     * @param lzNectar        LOADING ZONE NECTAR believed available (1 per tip, G426.A) and not yet tried
     */
    public static Action next(double elapsedS, int ballsHeld, boolean shootingAllowed,
                              int spillBalls, int lzNectar) {
        if (mustPark(elapsedS)) return Action.PARK;
        double shot = RobotConstants.AUTO_SHOT_CYCLE_EST_S;
        boolean canShoot = shootingAllowed && ballsHeld > 0;
        if (canShoot && ballsHeld >= RobotConstants.MAX_CONTROLLED_ELEMENTS) {
            return fits(elapsedS, shot) ? Action.SHOOT : Action.PARK;
        }
        if (shootingAllowed && spillBalls > 0
                && fits(elapsedS, RobotConstants.AUTO_SPILL_COLLECT_EST_S + shot)) {
            return Action.COLLECT_SPILL;
        }
        if (shootingAllowed && lzNectar > 0
                && fits(elapsedS, RobotConstants.AUTO_LZ_COLLECT_EST_S + shot)) {
            return Action.COLLECT_LOADING_ZONE;
        }
        if (canShoot && fits(elapsedS, shot)) return Action.SHOOT;
        return Action.PARK;
    }
}
