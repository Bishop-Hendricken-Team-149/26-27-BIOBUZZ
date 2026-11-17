package org.firstinspires.ftc.teamcode.hivequeen.subsystems;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.hivequeen.RobotConstants;
import org.firstinspires.ftc.teamcode.hivequeen.util.Pose;

/**
 * goBILDA Pinpoint v2 odometry computer with exactly two goBILDA 4-bar dead-wheel pods, using the
 * SDK's built-in {@link GoBildaPinpointDriver} (com.qualcomm.hardware.gobilda, FTC SDK 12.0).
 *
 * Pod geometry (robot frame, +X forward, +Y left):
 *  - forward (X) pod: (X=-95, Y=+110) mm  -> left side
 *  - strafe  (Y) pod: (X=-146.2, Y=0) mm  -> rear centre
 *
 * Pinpoint offset convention (driver Javadoc): xOffset = how far LEFT of the tracking centre the
 * X (forward) pod is; yOffset = how far FORWARD of the tracking centre the Y (strafe) pod is.
 * Hence setOffsets(+110, -146.2, MM) = setOffsets(X_POD_ROBOT_Y_MM, Y_POD_ROBOT_X_MM).
 */
public class PinpointLocalizer {
    private final GoBildaPinpointDriver pinpoint;
    private Pose pose = new Pose(0, 0, 0);
    private double velXMmS = 0, velYMmS = 0;

    public PinpointLocalizer(GoBildaPinpointDriver pinpoint) {
        this.pinpoint = pinpoint;
        pinpoint.setOffsets(RobotConstants.X_POD_ROBOT_Y_MM, RobotConstants.Y_POD_ROBOT_X_MM, DistanceUnit.MM);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(
                RobotConstants.X_POD_REVERSED ? GoBildaPinpointDriver.EncoderDirection.REVERSED
                        : GoBildaPinpointDriver.EncoderDirection.FORWARD,
                RobotConstants.Y_POD_REVERSED ? GoBildaPinpointDriver.EncoderDirection.REVERSED
                        : GoBildaPinpointDriver.EncoderDirection.FORWARD);
    }

    /** Zero the pose and recalibrate the Pinpoint IMU. Robot MUST be stationary (~0.25 s). */
    public void resetAndCalibrate() {
        pinpoint.resetPosAndIMU();
        pose = new Pose(0, 0, 0);
    }

    /** Overwrite the current pose (e.g. the Autonomous starting pose). */
    public void setPose(Pose p) {
        pinpoint.setPosition(new Pose2D(DistanceUnit.MM, p.xMm, p.yMm, AngleUnit.DEGREES, p.headingDeg));
        pose = p;
    }

    /** Call exactly once per loop. Reads position + velocity in one bulk I2C transaction. */
    public void update() {
        pinpoint.update();
        Pose2D p = pinpoint.getPosition();
        pose = new Pose(p.getX(DistanceUnit.MM), p.getY(DistanceUnit.MM), p.getHeading(AngleUnit.DEGREES));
        velXMmS = pinpoint.getVelX(DistanceUnit.MM);
        velYMmS = pinpoint.getVelY(DistanceUnit.MM);
    }

    public Pose getPose() { return pose; }

    public double getHeadingDeg() { return pose.headingDeg; }

    public double getSpeedMmS() { return Math.hypot(velXMmS, velYMmS); }

    public GoBildaPinpointDriver.DeviceStatus getStatus() { return pinpoint.getDeviceStatus(); }

    public boolean isReady() {
        return pinpoint.getDeviceStatus() == GoBildaPinpointDriver.DeviceStatus.READY;
    }

    public int rawEncoderX() { return pinpoint.getEncoderX(); }

    public int rawEncoderY() { return pinpoint.getEncoderY(); }
}
