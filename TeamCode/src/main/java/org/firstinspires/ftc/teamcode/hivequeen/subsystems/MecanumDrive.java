package org.firstinspires.ftc.teamcode.hivequeen.subsystems;

import org.firstinspires.ftc.teamcode.hivequeen.RobotHardware;

/**
 * 4-motor mecanum drive: goBILDA 312 RPM motors -> 1:1 miter bevels -> 96 mm mecanum wheels,
 * rollers in the standard "X" pattern (seen from above), left side reversed in RobotHardware.
 *
 * Robot frame: +X forward, +Y left, +turn = CCW.
 */
public class MecanumDrive {
    private final RobotHardware hw;
    private boolean enabled = true;

    public MecanumDrive(RobotHardware hw) { this.hw = hw; }

    /** G403/G404 gate: when disabled every call results in zero power. */
    public void setEnabled(boolean enabled) {
        this.enabled = enabled;
        if (!enabled) stop();
    }

    /**
     * Robot-centric drive.
     * @param forward +1 = full forward
     * @param left    +1 = full strafe left
     * @param turn    +1 = full CCW rotation
     */
    public void driveRobot(double forward, double left, double turn) {
        if (!enabled) { stop(); return; }
        double flP = forward - left - turn;
        double frP = forward + left + turn;
        double blP = forward + left - turn;
        double brP = forward - left + turn;
        double max = Math.max(1.0, Math.max(Math.max(Math.abs(flP), Math.abs(frP)),
                Math.max(Math.abs(blP), Math.abs(brP))));
        hw.fl.setPower(flP / max);
        hw.fr.setPower(frP / max);
        hw.bl.setPower(blP / max);
        hw.br.setPower(brP / max);
    }

    /**
     * Field-centric drive: inputs are in the field frame; robot heading from Pinpoint.
     */
    public void driveField(double fieldForward, double fieldLeft, double turn, double robotHeadingDeg) {
        double h = Math.toRadians(robotHeadingDeg);
        double cos = Math.cos(-h), sin = Math.sin(-h);
        double fwd = fieldForward * cos - fieldLeft * sin;
        double lft = fieldForward * sin + fieldLeft * cos;
        driveRobot(fwd, lft, turn);
    }

    public void stop() {
        hw.fl.setPower(0);
        hw.fr.setPower(0);
        hw.bl.setPower(0);
        hw.br.setPower(0);
    }
}
