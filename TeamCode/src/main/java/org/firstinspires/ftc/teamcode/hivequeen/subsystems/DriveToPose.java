package org.firstinspires.ftc.teamcode.hivequeen.subsystems;

import org.firstinspires.ftc.teamcode.hivequeen.RobotConstants;
import org.firstinspires.ftc.teamcode.hivequeen.util.MathUtil;
import org.firstinspires.ftc.teamcode.hivequeen.util.Pose;

/**
 * Minimal holonomic drive-to-pose P controller for Autonomous, fed by Pinpoint.
 * Field error is rotated into the robot frame so the mecanum drive translates and turns at once.
 */
public class DriveToPose {
    private final MecanumDrive drive;
    private final PinpointLocalizer loc;

    public DriveToPose(MecanumDrive drive, PinpointLocalizer loc) {
        this.drive = drive;
        this.loc = loc;
    }

    /** One control step toward {@code target}. Returns true when within tolerance (drive stopped). */
    public boolean step(Pose target, double maxPower) {
        return step(target, maxPower, RobotConstants.AUTO_POS_TOL_MM, RobotConstants.AUTO_HEADING_TOL_DEG, true);
    }

    /**
     * @param stopAtTarget false for pass-through waypoints: report "reached" without braking.
     */
    public boolean step(Pose target, double maxPower, double posTolMm, double headingTolDeg, boolean stopAtTarget) {
        Pose p = loc.getPose();
        double ex = target.xMm - p.xMm;
        double ey = target.yMm - p.yMm;
        double eh = MathUtil.wrapDeg(target.headingDeg - p.headingDeg);
        if (Math.hypot(ex, ey) < posTolMm && Math.abs(eh) < headingTolDeg) {
            if (stopAtTarget) drive.stop();
            return true;
        }
        double h = Math.toRadians(p.headingDeg);
        double fwd = ex * Math.cos(h) + ey * Math.sin(h);     // field -> robot frame
        double lft = -ex * Math.sin(h) + ey * Math.cos(h);
        double vx = RobotConstants.AUTO_TRANSLATION_KP * fwd;
        double vy = RobotConstants.AUTO_TRANSLATION_KP * lft;
        double mag = Math.hypot(vx, vy);
        if (mag > maxPower) { vx *= maxPower / mag; vy *= maxPower / mag; }
        else if (mag > 1e-9 && mag < RobotConstants.AUTO_MIN_POWER
                && Math.hypot(ex, ey) >= posTolMm) {
            // Pure P stalls just outside tolerance (static friction); keep a minimum push.
            double s = Math.min(RobotConstants.AUTO_MIN_POWER, maxPower) / mag;
            vx *= s; vy *= s;
        }
        double turn = MathUtil.clamp(RobotConstants.AUTO_HEADING_KP * eh, -maxPower, maxPower);
        drive.driveRobot(vx, vy, turn);
        return false;
    }
}
