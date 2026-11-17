package org.firstinspires.ftc.teamcode.hivequeen.subsystems;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.hivequeen.RobotConstants;
import org.firstinspires.ftc.teamcode.hivequeen.game.Alliance;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;

import java.util.List;

/**
 * Optional secondary targeting path: SDK VisionPortal + AprilTagProcessor on "Webcam 1" using
 * the SDK 12.0 BIOBUZZ tag library ({@link AprilTagGameDatabase#getBioBuzzTagLibrary()}).
 *
 * R704.D: "No continuous video stream is allowed." -> live view is disabled and the portal is
 * never streamed anywhere; only the processed bearing/range numbers are used on the robot.
 */
public class WebcamAprilTagTargeting {
    private final VisionPortal portal;
    private final AprilTagProcessor processor;
    private Alliance alliance;
    private TargetObservation latest = null;
    /** Cluster index (0..3) to prefer (not visible -> no target, odometry aim), or -1 for "closest
     *  own cluster" (see HuskyLensTargeting, G417.C). */
    private int preferredCluster = -1;

    /** @param webcam may be null, in which case this class is inert. */
    public WebcamAprilTagTargeting(WebcamName webcam, Alliance alliance) {
        this.alliance = alliance;
        if (webcam == null) {
            portal = null;
            processor = null;
            return;
        }
        processor = new AprilTagProcessor.Builder()
                .setTagLibrary(AprilTagGameDatabase.getBioBuzzTagLibrary())
                .setOutputUnits(DistanceUnit.MM, AngleUnit.DEGREES)
                .setDrawAxes(false)
                .setDrawCubeProjection(false)
                .setDrawTagOutline(false)
                .build();
        portal = new VisionPortal.Builder()
                .setCamera(webcam)
                .addProcessor(processor)
                .enableLiveView(false)       // R704.D: no video stream
                .build();
    }

    public boolean isAvailable() { return portal != null; }

    public void setAlliance(Alliance alliance) { this.alliance = alliance; }

    public void setPreferredCluster(int clusterIndex) {
        if (clusterIndex != preferredCluster) latest = null;   // never aim at the previous CELL's cluster
        preferredCluster = clusterIndex;
    }

    public void update() {
        if (processor == null) return;
        List<AprilTagDetection> detections = processor.getFreshDetections();
        if (detections == null || detections.isEmpty()) return;

        int[] count = new int[4];
        double[] sumBearing = new double[4];
        double[] sumRange = new double[4];
        for (AprilTagDetection d : detections) {
            if (!(d instanceof AprilTagSingleDetection)) continue;
            AprilTagSingleDetection s = (AprilTagSingleDetection) d;
            if (!alliance.isOwnCellTag(s.id) || s.ftcPose == null) continue;
            int c = (s.id - 30) / 4;
            if (c < 0 || c > 3) continue;
            count[c]++;
            sumBearing[c] += s.ftcPose.bearing;   // + = tag is to the left (FTC convention)
            sumRange[c] += s.ftcPose.range;
        }
        int best = -1;
        if (preferredCluster >= 0 && preferredCluster < 4) {
            if (count[preferredCluster] > 0) best = preferredCluster;   // else: no target (odometry aim)
        } else {
            for (int c = 0; c < 4; c++) {   // closest cluster (smallest mean range)
                if (count[c] > 0 && (best < 0 || sumRange[c] / count[c] < sumRange[best] / count[best])) best = c;
            }
        }
        if (best < 0) return;
        latest = new TargetObservation(
                sumBearing[best] / count[best] + RobotConstants.CAMERA_YAW_OFFSET_DEG,
                sumRange[best] / count[best],
                count[best],
                System.nanoTime());
    }

    public TargetObservation getLatest() { return latest; }

    public void close() {
        if (portal != null) portal.close();
    }
}
