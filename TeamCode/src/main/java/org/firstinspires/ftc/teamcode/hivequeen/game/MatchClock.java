package org.firstinspires.ftc.teamcode.hivequeen.game;

import org.firstinspires.ftc.teamcode.hivequeen.RobotConstants;

/**
 * Software model of the BIOBUZZ MATCH timeline (Sec. 10.1 / Sec. 10.4):
 * "MATCHES consist of pre-MATCH setup, a 30-second AUTO period, an 8-second transition period
 * between AUTO and TELEOP, and a 2-minute TELEOP period, followed by the post-MATCH reset."
 *
 * Anchor strategy:
 *  - Autonomous calls {@link #markAutoStart()} at START. That timestamp is kept in a static so
 *    TeleOp (same RC app process) knows the real MATCH time, including the 8 s transition.
 *  - TeleOp calls {@link #startTeleOp(long)}. If a recent AUTO anchor exists it is used,
 *    otherwise the TeleOp START press is treated as the start of TELEOP.
 *
 * All methods take explicit nanosecond timestamps so the logic can be unit-tested without a
 * robot. Uses only {@link System#nanoTime()} in the convenience overloads.
 */
public class MatchClock {

    public enum Phase {
        /** OpMode initialized, not started. No powered motion allowed (G403 INIT note). */
        PRE_START,
        AUTO,
        /** G403: "Any powered movement ... is not allowed during the transition period." */
        TRANSITION,
        TELEOP,
        /** Last 60 s of the MATCH (still TELEOP) -- G410 FLOWER NECTAR window is open. */
        ENDGAME,
        /** G404: "ROBOTS must no longer have powered movement after the end of TELEOP". */
        POST_MATCH
    }

    private static final double NS = 1e9;
    /** An AUTO anchor older than this at TeleOp START is considered a different match. */
    private static final double MAX_AUTO_ANCHOR_AGE_S =
            RobotConstants.AUTO_DURATION_S + RobotConstants.TRANSITION_DURATION_S + 60.0;

    /** Shared between OpModes in the same app process. 0 = no anchor. */
    private static volatile long autoStartNanos = 0;

    private final boolean isAutonomous;
    /** MATCH time 0 (start of AUTO) in nanoTime units; 0 until started. */
    private long matchZeroNanos = 0;
    private boolean started = false;
    private boolean usedAutoAnchor = false;

    private MatchClock(boolean isAutonomous) { this.isAutonomous = isAutonomous; }

    public static MatchClock forAutonomous() { return new MatchClock(true); }

    public static MatchClock forTeleOp() { return new MatchClock(false); }

    /** Call from Autonomous START. */
    public void markAutoStart() { markAutoStart(System.nanoTime()); }

    public void markAutoStart(long nowNanos) {
        matchZeroNanos = nowNanos;
        autoStartNanos = nowNanos;
        started = true;
    }

    /** Call from TeleOp START. */
    public void startTeleOp() { startTeleOp(System.nanoTime()); }

    public void startTeleOp(long nowNanos) {
        long anchor = autoStartNanos;
        double anchorAgeS = (nowNanos - anchor) / NS;
        if (anchor != 0 && anchorAgeS > 0 && anchorAgeS < MAX_AUTO_ANCHOR_AGE_S) {
            // START pressed during the transition: hold motion until AUTO-anchor + 38 s (G403).
            // START pressed after that: treat the press as the true start of TELEOP, so a late
            // START (or a late AUTO start) never cuts TELEOP short at the end (the old code
            // anchored the END to the AUTO press and could lock the robot out early).
            long teleNominalZero = nowNanos - secondsToNanos(
                    RobotConstants.AUTO_DURATION_S + RobotConstants.TRANSITION_DURATION_S);
            matchZeroNanos = Math.max(anchor, teleNominalZero);
            usedAutoAnchor = true;
        } else {
            // No AUTO this match: assume START was pressed at the beginning of TELEOP.
            matchZeroNanos = nowNanos - secondsToNanos(
                    RobotConstants.AUTO_DURATION_S + RobotConstants.TRANSITION_DURATION_S);
            usedAutoAnchor = false;
        }
        started = true;
    }

    /** True if an Autonomous START happened recently enough to belong to this MATCH. */
    public static boolean hasRecentAutoAnchor(long nowNanos) {
        long anchor = autoStartNanos;
        double ageS = (nowNanos - anchor) / NS;
        return anchor != 0 && ageS > 0 && ageS < MAX_AUTO_ANCHOR_AGE_S;
    }

    /** Forget the AUTO anchor (e.g. after a practice run). */
    public static void clearAutoAnchor() { autoStartNanos = 0; }

    public boolean isStarted() { return started; }

    public boolean usedAutoAnchor() { return usedAutoAnchor; }

    /** Seconds since MATCH start (start of AUTO). */
    public double matchElapsedS(long nowNanos) {
        return started ? (nowNanos - matchZeroNanos) / NS : 0.0;
    }

    public double matchElapsedS() { return matchElapsedS(System.nanoTime()); }

    /** Seconds remaining in the whole MATCH (the FIELD timer value during TELEOP). */
    public double matchRemainingS(long nowNanos) {
        return Math.max(0.0, teleopEndS() - matchElapsedS(nowNanos));
    }

    public double matchRemainingS() { return matchRemainingS(System.nanoTime()); }

    public Phase phase(long nowNanos) {
        if (!started) return Phase.PRE_START;
        double t = matchElapsedS(nowNanos);
        double autoEnd = RobotConstants.AUTO_DURATION_S;
        double teleStart = autoEnd + RobotConstants.TRANSITION_DURATION_S;
        double teleEnd = teleopEndS();
        if (t < 0) return Phase.PRE_START;
        if (t < autoEnd) return isAutonomous ? Phase.AUTO : Phase.TRANSITION;
        if (isAutonomous) {
            // An Autonomous OpMode is never allowed to drive past the end of AUTO.
            return t < teleStart ? Phase.TRANSITION : Phase.POST_MATCH;
        }
        if (t < teleStart) return Phase.TRANSITION;
        if (t < teleEnd - RobotConstants.FLOWER_NECTAR_WINDOW_S) return Phase.TELEOP;
        if (t < teleEnd) return Phase.ENDGAME;
        return Phase.POST_MATCH;
    }

    public Phase phase() { return phase(System.nanoTime()); }

    /**
     * G403 / G404 transition-motionless gate: true only while powered movement is legal.
     * AUTO OpMode: only during AUTO. TeleOp OpMode: only during TELEOP (incl. ENDGAME).
     */
    public boolean poweredMotionAllowed(long nowNanos) {
        Phase p = phase(nowNanos);
        return p == Phase.AUTO || p == Phase.TELEOP || p == Phase.ENDGAME;
    }

    public boolean poweredMotionAllowed() { return poweredMotionAllowed(System.nanoTime()); }

    private static double teleopEndS() {
        return RobotConstants.AUTO_DURATION_S + RobotConstants.TRANSITION_DURATION_S
                + RobotConstants.TELEOP_DURATION_S;
    }

    private static long secondsToNanos(double s) { return (long) (s * NS); }
}
