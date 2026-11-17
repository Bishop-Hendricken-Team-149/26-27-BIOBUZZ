package org.firstinspires.ftc.teamcode.hivequeen.subsystems;

import com.qualcomm.hardware.dfrobot.HuskyLens;

import org.firstinspires.ftc.teamcode.hivequeen.RobotConstants;
import org.firstinspires.ftc.teamcode.hivequeen.game.Alliance;

/**
 * Primary targeting: DFRobot HuskyLens (SDK class com.qualcomm.hardware.dfrobot.HuskyLens) in
 * TAG_RECOGNITION mode. HuskyLens is legal as a "configurable but not programmable" vision
 * coprocessor (R702 Example 5) and talks to the hub over wired I2C (no Wi-Fi, R704.A).
 *
 * BIOBUZZ Sec. 9.9: four 3.25 in 36h11 tags per cluster on the bottom of each CELL; red IDs 30-37,
 * blue IDs 38-45. We group the visible own-alliance tags by cluster (IDs 30-33, 34-37, ...),
 * pick the cluster with the most visible tags (ties -> largest apparent size = closest) and
 * use the centroid of those tags as the aim point.
 */
public class HuskyLensTargeting {
    private final HuskyLens lens;
    private Alliance alliance;
    private long lastReadMs = 0;
    private TargetObservation latest = null;
    private boolean connected = false;
    /** Cluster index (0..3) to prefer, or -1 for "closest own cluster". */
    private int preferredCluster = -1;

    public HuskyLensTargeting(HuskyLens lens, Alliance alliance) {
        this.lens = lens;
        this.alliance = alliance;
    }

    /** Call during INIT. Returns false if the HuskyLens does not answer on I2C. */
    public boolean init() {
        connected = lens != null && lens.knock();   // lens is null when not in the configuration
        if (connected) lens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);
        return connected;
    }

    public void setAlliance(Alliance alliance) { this.alliance = alliance; }

    /**
     * Aim only at this CELL's cluster (Autonomous knows which CELL is up). If that cluster is NOT
     * visible, no new observation is produced (the aim falls back to the odometry bearing) -- never
     * the closest other cluster, which may be the DOWNWARD CELL (shots at its underside, G417.C).
     * -1 = pick the CLOSEST own cluster: the upward CELL's mouth leans toward the robot's side,
     * so when shooting from the correct side the up CELL is the nearer one.
     */
    public void setPreferredCluster(int clusterIndex) {
        if (clusterIndex != preferredCluster) latest = null;   // never aim at the previous CELL's cluster
        preferredCluster = clusterIndex;
    }

    public boolean isConnected() { return connected; }

    /** Polls the HuskyLens at most every HUSKYLENS_READ_PERIOD_MS. */
    public void update() {
        if (!connected) return;
        long nowMs = System.nanoTime() / 1_000_000L;   // monotonic (wall clock can jump on DS time sync)
        if (nowMs - lastReadMs < RobotConstants.HUSKYLENS_READ_PERIOD_MS) return;
        lastReadMs = nowMs;

        HuskyLens.Block[] blocks = lens.blocks();
        if (blocks == null || blocks.length == 0) return;

        // Accumulate per-cluster (cluster index = (tagId - 30) / 4, 0..3).
        int[] count = new int[4];
        double[] sumX = new double[4];
        double[] sumW = new double[4];
        for (HuskyLens.Block b : blocks) {
            int tagId = learnedIdToAprilTag(b.id);
            if (tagId < 0 || !alliance.isOwnCellTag(tagId)) continue;
            int c = (tagId - 30) / 4;
            if (c < 0 || c > 3) continue;
            count[c]++;
            sumX[c] += b.x;
            sumW[c] += b.width;
        }
        int best = -1;
        if (preferredCluster >= 0 && preferredCluster < 4) {
            if (count[preferredCluster] > 0) best = preferredCluster;   // else: no target (odometry aim)
        } else {
            for (int c = 0; c < 4; c++) {   // closest = largest mean apparent tag width
                if (count[c] == 0) continue;
                if (best < 0 || sumW[c] / count[c] > sumW[best] / count[best]) best = c;
            }
        }
        if (best < 0) return;

        double cx = sumX[best] / count[best];
        double meanWidthPx = sumW[best] / count[best];
        double slantMm = widthToRangeMm(meanWidthPx);
        double camYaw = pixelToYawDeg(cx) + RobotConstants.CAMERA_YAW_OFFSET_DEG;
        // The lens is 86 mm right of the shooter centreline: convert to the shooter-axis bearing.
        double horiz = ShotTable.slantToHorizontalMm(slantMm,
                RobotConstants.CELL_TAG_HEIGHT_MM - RobotConstants.CAMERA_HEIGHT_MM);
        double yaw = ShotTable.cameraToShooterYawDeg(camYaw, horiz,
                RobotConstants.HUSKYLENS_ROBOT_X_MM, RobotConstants.HUSKYLENS_ROBOT_Y_MM);
        latest = new TargetObservation(yaw, slantMm, count[best], System.nanoTime());
    }

    /** Latest observation, or null if nothing seen yet. Check its age before trusting it. */
    public TargetObservation getLatest() { return latest; }

    static int learnedIdToAprilTag(int learnedId) {
        int idx = learnedId - 1;
        int[] table = RobotConstants.HUSKYLENS_LEARNED_ID_TO_APRILTAG;
        return (idx >= 0 && idx < table.length) ? table[idx] : -1;
    }

    /** Horizontal pixel -> yaw (deg, + = left of centre) using a pinhole model. */
    static double pixelToYawDeg(double xPx) {
        double half = RobotConstants.HUSKYLENS_IMAGE_WIDTH_PX / 2.0;
        double focalPx = half / Math.tan(Math.toRadians(RobotConstants.HUSKYLENS_HFOV_DEG / 2.0));
        return Math.toDegrees(Math.atan((half - xPx) / focalPx));
    }

    /** Apparent tag width -> range (mm). Rough: ignores tag tilt (tags face the floor, Sec. 9.9). */
    static double widthToRangeMm(double widthPx) {
        if (widthPx <= 1) return Double.NaN;
        double half = RobotConstants.HUSKYLENS_IMAGE_WIDTH_PX / 2.0;
        double focalPx = half / Math.tan(Math.toRadians(RobotConstants.HUSKYLENS_HFOV_DEG / 2.0));
        return RobotConstants.APRILTAG_SIZE_MM * focalPx / widthPx;
    }
}
