package org.firstinspires.ftc.teamcode.hivequeen.game;

import org.firstinspires.ftc.teamcode.hivequeen.RobotConstants;

/**
 * Pure-logic ESTIMATE of our alliance HIVE state (no sensor can see the HIVE mass directly).
 *
 * Sec. 9.6 / 10.3.1: the HIVE is bi-stable; at match start our upward CELL holds 3 alliance
 * NECTAR (123.84 g). Every ball we LAUNCH is assumed to land in the upward CELL; once the
 * modelled mass reaches {@link RobotConstants#HIVE_TIP_MASS_G} (199 g = "must tip" calibration)
 * we assume a TIP: the CELL that was up spills its balls onto the field on its own side and the
 * other (empty) CELL becomes the upward one.
 *
 * Mass per ball: POLLEN 24.95 g, NECTAR 41.28 g; an unclassified ball counts as POLLEN (the
 * lighter choice, so a tip is never assumed too early).
 */
public final class HiveTracker {

    /** Which CELL of our HIVE: the one nearer the audience wall (-X) or the rear wall (+X). */
    public enum Side {
        AUDIENCE, FAR;

        public Side opposite() { return this == AUDIENCE ? FAR : AUDIENCE; }
    }

    private Side upSide;
    private double cellMassG;
    private int ballsInCell;
    private int tips = 0;
    private Side lastSpillSide = null;
    private int lastSpillBalls = 0;

    /** Match-start state: {@code initialUp} CELL holds the 3 staged NECTAR. */
    public HiveTracker(Side initialUp) {
        this(initialUp, RobotConstants.STAGED_CELL_NECTAR * RobotConstants.NECTAR_MASS_G,
                RobotConstants.STAGED_CELL_NECTAR);
    }

    public HiveTracker(Side upSide, double cellMassG, int ballsInCell) {
        this.upSide = upSide;
        this.cellMassG = cellMassG;
        this.ballsInCell = ballsInCell;
    }

    public static double massOf(BallType b) {
        return b != null && b.isNectar() ? RobotConstants.NECTAR_MASS_G : RobotConstants.POLLEN_MASS_G;
    }

    /** Record one LAUNCHED ball. @return true if this launch is assumed to have TIPPED the HIVE. */
    public boolean recordLaunch(BallType ball) {
        cellMassG += massOf(ball);
        ballsInCell++;
        if (cellMassG >= RobotConstants.HIVE_TIP_MASS_G) {
            flip();
            return true;
        }
        return false;
    }

    /** A TIP happened (assumed or confirmed by the driver): spill the up CELL, raise the other. */
    public void flip() {
        tips++;
        lastSpillSide = upSide;
        lastSpillBalls = ballsInCell;
        upSide = upSide.opposite();
        cellMassG = 0.0;
        ballsInCell = 0;
    }

    /**
     * True when the modelled up-CELL mass is in the HIVE's undefined band (174-199 g), e.g. after
     * firing only 3 POLLEN onto the 3 staged NECTAR (198.7 g): it may or may not have tipped.
     */
    public boolean isTipUncertain() {
        return cellMassG > RobotConstants.HIVE_NO_TIP_MASS_G && cellMassG < RobotConstants.HIVE_TIP_MASS_G;
    }

    public Side getUpSide() { return upSide; }

    public double getCellMassG() { return cellMassG; }

    public int getBallsInCell() { return ballsInCell; }

    public int getTips() { return tips; }

    /** Side whose CELL spilled at the last tip, or null if no tip yet. */
    public Side getLastSpillSide() { return lastSpillSide; }

    /** Number of balls that spilled at the last tip (0 if none). */
    public int getLastSpillBalls() { return lastSpillBalls; }
}
