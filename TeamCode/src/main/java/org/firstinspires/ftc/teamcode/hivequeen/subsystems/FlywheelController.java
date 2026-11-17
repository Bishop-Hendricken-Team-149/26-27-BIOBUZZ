package org.firstinspires.ftc.teamcode.hivequeen.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.hivequeen.RobotConstants;
import org.firstinspires.ftc.teamcode.hivequeen.game.FlywheelWatchdog;
import org.firstinspires.ftc.teamcode.hivequeen.util.MathUtil;

/**
 * Voltage-compensated velocity controller for ONE flywheel motor (goBILDA 5203-2402-0005,
 * 1150 RPM, 145.1 ticks/rev at the output shaft).
 *
 *   power = (kS*sign(target) + kV*targetRpm) * (12.0 / Vbattery)  +  kP*(targetRpm - measuredRpm)
 *
 * Velocity is measured with {@link DcMotorEx#getVelocity()} (ticks/s, from the bulk cache) and
 * converted to motor-output RPM. The feedforward term is scaled by nominal/actual voltage so the
 * same setpoint gives the same wheel speed on a fresh (13.5 V) or tired (12 V) battery.
 *
 * Missing encoder ({@link FlywheelWatchdog}): if the velocity stays ~0 while the power is high for
 * > FLYWHEEL_ENC_FAULT_MS, the encoder is flagged FAULT; the loop then runs feedforward only (no kP
 * on a fake 0 RPM, which would drive full power) and {@link #atSpeed} becomes time based
 * (FLYWHEEL_FALLBACK_SPINUP_MS, TUNE) so the robot can still fire.
 */
public class FlywheelController {
    private final DcMotorEx motor;
    private final double kS, kV, kP;
    private double targetRpm = 0.0;
    private double measuredRpm = 0.0;
    private double lastPower = 0.0;
    private long lastUpdateMs = -1;
    private long inToleranceSinceMs = -1;
    private final FlywheelWatchdog watchdog = new FlywheelWatchdog();

    public FlywheelController(DcMotorEx motor) {
        this(motor, RobotConstants.FLYWHEEL_KS, RobotConstants.FLYWHEEL_KV, RobotConstants.FLYWHEEL_KP);
    }

    public FlywheelController(DcMotorEx motor, double kS, double kV, double kP) {
        this.motor = motor;
        this.kS = kS;
        this.kV = kV;
        this.kP = kP;
    }

    public void setTargetRpm(double rpm) {
        targetRpm = MathUtil.clamp(rpm, 0.0, RobotConstants.FLYWHEEL_MAX_SETPOINT_RPM);
    }

    public double getTargetRpm() { return targetRpm; }

    public double getMeasuredRpm() { return measuredRpm; }

    public double getLastPower() { return lastPower; }

    /** Run one control step. */
    public void update(double batteryVoltage, long nowMs) {
        measuredRpm = motor.getVelocity() / RobotConstants.SHOOTER_TICKS_PER_REV * 60.0;
        watchdog.update(nowMs, lastPower, measuredRpm, targetRpm);
        boolean encFault = watchdog.isFault();
        // Encoder FAULT: feedforward only (measured RPM is meaningless).
        double want = computePower(targetRpm, encFault ? targetRpm : measuredRpm, batteryVoltage, kS, kV, encFault ? 0.0 : kP);
        // Spin-up current limit: two 1150 RPM motors starting at full power while the robot drives
        // can sag the battery into a hub brownout. Below half speed, ramp the power instead.
        long dt = lastUpdateMs < 0 ? 0 : nowMs - lastUpdateMs;
        lastUpdateMs = nowMs;
        if (encFault || measuredRpm < 0.5 * targetRpm) want = MathUtil.slewUp(lastPower, want, dt, RobotConstants.FLYWHEEL_SPINUP_SLEW_PER_S);
        lastPower = want;
        motor.setPower(lastPower);

        if (targetRpm > 0 && Math.abs(targetRpm - measuredRpm) <= RobotConstants.FLYWHEEL_TOLERANCE_RPM) {
            if (inToleranceSinceMs < 0) inToleranceSinceMs = nowMs;
        } else {
            inToleranceSinceMs = -1;
        }
    }

    /** Pure control law (unit-testable). */
    public static double computePower(double targetRpm, double measuredRpm, double batteryVoltage,
                                      double kS, double kV, double kP) {
        if (targetRpm <= 0.0) return 0.0;   // coast down, never actively brake/reverse
        double v = batteryVoltage > 1.0 ? batteryVoltage : RobotConstants.NOMINAL_VOLTAGE;
        double ff = (kS + kV * targetRpm) * (RobotConstants.NOMINAL_VOLTAGE / v);
        double fb = kP * (targetRpm - measuredRpm);
        return MathUtil.clamp(ff + fb, 0.0, 1.0);
    }

    /**
     * True once within tolerance continuously for FLYWHEEL_SETTLE_MS. With the encoder in FAULT:
     * true FLYWHEEL_FALLBACK_SPINUP_MS after the wheel was commanded (time-based fallback).
     */
    public boolean atSpeed(long nowMs) {
        if (watchdog.isFault()) return watchdog.fallbackAtSpeed(nowMs);
        return inToleranceSinceMs >= 0 && nowMs - inToleranceSinceMs >= RobotConstants.FLYWHEEL_SETTLE_MS;
    }

    /** True if the encoder is believed missing (velocity ~0 at high power for > 1 s). */
    public boolean isEncoderFault() { return watchdog.isFault(); }

    /** Motor RPM -> wheel surface speed (m/s) for a given wheel diameter and overdrive ratio. */
    public static double surfaceSpeedMps(double motorRpm, double wheelDiameterMm, double overdriveRatio) {
        return ShotTable.surfaceSpeedMps(motorRpm, wheelDiameterMm, overdriveRatio);
    }
}
