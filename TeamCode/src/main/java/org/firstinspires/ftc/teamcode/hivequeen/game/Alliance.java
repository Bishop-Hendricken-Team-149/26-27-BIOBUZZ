package org.firstinspires.ftc.teamcode.hivequeen.game;

import org.firstinspires.ftc.teamcode.hivequeen.RobotConstants;
import org.firstinspires.ftc.teamcode.hivequeen.util.Pose;

/**
 * Alliance colour, its CELL AprilTag IDs (Sec. 9.9), the colour of its own NECTAR (Sec. 9.8) and
 * its relation to the field frame defined in {@link RobotConstants} (+X = rear wall, +Y = red wall).
 *
 * The BIOBUZZ field is 180-degree ROTATIONALLY symmetric (Fig. 9-2 / 10-2), so a RED path is
 * converted to BLUE by rotating it 180 deg about the field centre -- not by mirroring Y.
 */
public enum Alliance {
    RED(RobotConstants.RED_CELL_TAG_IDS, BallType.RED_NECTAR),
    BLUE(RobotConstants.BLUE_CELL_TAG_IDS, BallType.BLUE_NECTAR);

    private final int[] cellTagIds;
    private final BallType ownNectar;

    Alliance(int[] cellTagIds, BallType ownNectar) {
        this.cellTagIds = cellTagIds;
        this.ownNectar = ownNectar;
    }

    public boolean isOwnCellTag(int aprilTagId) {
        for (int id : cellTagIds) {
            if (id == aprilTagId) return true;
        }
        return false;
    }

    public BallType ownNectar() { return ownNectar; }

    public BallType opponentNectar() {
        return this == RED ? BallType.BLUE_NECTAR : BallType.RED_NECTAR;
    }

    public Alliance opposite() { return this == RED ? BLUE : RED; }

    /** Convert a pose authored for RED into this alliance's pose (identity for RED, 180-deg rotation for BLUE). */
    public Pose fromRed(Pose redPose) { return this == RED ? redPose : redPose.rotated180(); }

    /**
     * Cluster index used by the targeting classes ((tagId - 30) / 4):
     * 0 = red FAR (30-33), 1 = red AUDIENCE (34-37), 2 = blue AUDIENCE (38-41), 3 = blue FAR (42-45).
     */
    public int cellClusterIndex(HiveTracker.Side side) {
        if (this == RED) return side == HiveTracker.Side.FAR ? 0 : 1;
        return side == HiveTracker.Side.FAR ? 3 : 2;
    }

    /**
     * Sec. 10.3.1 / Fig. 10-2: at match start the red HIVE's AUDIENCE CELL and the blue HIVE's
     * FAR CELL face up (each holding 3 alliance NECTAR).
     */
    public HiveTracker.Side initialUpSide() {
        return this == RED ? HiveTracker.Side.AUDIENCE : HiveTracker.Side.FAR;
    }

    /**
     * Field-frame heading the drivers look along from their ALLIANCE AREA: red drivers stand
     * outside the +Y wall looking toward -Y, blue drivers outside the -Y wall looking toward +Y.
     */
    public double driverFacingDeg() { return this == RED ? -90.0 : 90.0; }

    /**
     * Field-frame aim point (mouth of this alliance's CELL on {@code side}), in mm.
     * Red HIVE at +Y; red AUDIENCE CELL at -X, red FAR CELL at +X. BLUE = 180-deg rotation,
     * which maps red-AUDIENCE onto blue-FAR -- hence the side swap for BLUE.
     */
    public Pose cellAimPoint(HiveTracker.Side side) {
        return cellPoint(side, RobotConstants.CELL_MOUTH_OFFSET_X_MM);
    }

    /** Field-frame point at the middle of the CELL mouth's LOWER LIP (ShotTable distance D is measured to it). */
    public Pose cellLipPoint(HiveTracker.Side side) {
        return cellPoint(side, RobotConstants.CELL_LIP_OFFSET_X_MM);
    }

    private Pose cellPoint(HiveTracker.Side side, double offsetFromPivotMm) {
        HiveTracker.Side redSide = this == RED ? side : side.opposite();
        double x = redSide == HiveTracker.Side.FAR ? offsetFromPivotMm : -offsetFromPivotMm;
        return fromRed(new Pose(x, RobotConstants.RED_HIVE_Y_MM, 0.0));
    }

    /**
     * Remembered across OpModes (same app process) so TeleOp defaults to the alliance the
     * last Autonomous ran for. The driver can still change it during TeleOp INIT.
     */
    public static volatile Alliance lastUsed = RED;
}
