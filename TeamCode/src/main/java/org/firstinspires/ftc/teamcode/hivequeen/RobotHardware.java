package org.firstinspires.ftc.teamcode.hivequeen;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import java.util.List;

/**
 * Single place where every HIVE-QUEEN device is pulled from the hardware map, using the names in
 * {@link RobotConstants}. Construction is side-effect free with respect to motion: no motor
 * power and no servo position is commanded here, so INIT never twitches an actuator (G403 note:
 * "If the INIT portion of the OpMode causes the ROBOT to violate this rule (powering actuators
 * such that they move or twitch in any way), then the team should wait until TELEOP begins").
 */
public class RobotHardware {
    // Drivetrain
    public final DcMotorEx fl, fr, bl, br;
    // Shooter / intake / magazine
    public final DcMotorEx shooterFront, shooterRear, intake, magazine;
    // Servos
    public final Servo lipGate, indexerGate, hoodPitch;
    // Sensors
    public final NormalizedColorSensor lipColor, throatColor;
    public final DistanceSensor ballCount1, ballCount2;
    public final GoBildaPinpointDriver pinpoint;
    /** Null when "huskylens" is not in the active configuration. */
    public final HuskyLens huskyLens;
    /** Optional secondary camera; null when "Webcam 1" is not in the active configuration. */
    public final WebcamName webcam;

    private final List<LynxModule> hubs;
    private final HardwareMap hardwareMap;

    public RobotHardware(HardwareMap hw) {
        this.hardwareMap = hw;

        // Bulk reads: one I2C/serial transaction per hub per loop.
        hubs = hw.getAll(LynxModule.class);
        for (LynxModule hub : hubs) hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        fl = hw.get(DcMotorEx.class, RobotConstants.FRONT_LEFT);
        fr = hw.get(DcMotorEx.class, RobotConstants.FRONT_RIGHT);
        bl = hw.get(DcMotorEx.class, RobotConstants.BACK_LEFT);
        br = hw.get(DcMotorEx.class, RobotConstants.BACK_RIGHT);
        // Left side reversed (motors mirror each other through the 1:1 miter bevels).
        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);
        fr.setDirection(DcMotorSimple.Direction.FORWARD);
        br.setDirection(DcMotorSimple.Direction.FORWARD);
        for (DcMotorEx m : new DcMotorEx[]{fl, fr, bl, br}) {
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        shooterFront = hw.get(DcMotorEx.class, RobotConstants.SHOOTER_FRONT);
        shooterRear  = hw.get(DcMotorEx.class, RobotConstants.SHOOTER_REAR);
        // Directions: both wheels must drive the ball UP the vertical nip. TUNE on the robot.
        shooterFront.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterRear.setDirection(DcMotorSimple.Direction.REVERSE);
        for (DcMotorEx m : new DcMotorEx[]{shooterFront, shooterRear}) {
            // Our own voltage-compensated loop runs on top of raw power; encoders only measured.
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT); // flywheels coast down
        }

        intake   = hw.get(DcMotorEx.class, RobotConstants.INTAKE);
        magazine = hw.get(DcMotorEx.class, RobotConstants.MAGAZINE);
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        magazine.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        magazine.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        magazine.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lipGate     = hw.get(Servo.class, RobotConstants.LIP_GATE);
        indexerGate = hw.get(Servo.class, RobotConstants.INDEXER_GATE);
        hoodPitch   = hw.get(Servo.class, RobotConstants.HOOD_PITCH);

        lipColor    = hw.get(NormalizedColorSensor.class, RobotConstants.LIP_COLOR);
        throatColor = hw.get(NormalizedColorSensor.class, RobotConstants.THROAT_COLOR);
        lipColor.setGain(RobotConstants.COLOR_SENSOR_GAIN);
        throatColor.setGain(RobotConstants.COLOR_SENSOR_GAIN);
        ballCount1  = hw.get(DistanceSensor.class, RobotConstants.BALL_COUNT_1);
        ballCount2  = hw.get(DistanceSensor.class, RobotConstants.BALL_COUNT_2);

        pinpoint  = hw.get(GoBildaPinpointDriver.class, RobotConstants.PINPOINT);
        // Optional: a missing/unplugged HuskyLens must not crash Autonomous INIT (targeting then
        // falls back to the webcam and Pinpoint odometry aim).
        huskyLens = hw.tryGet(HuskyLens.class, RobotConstants.HUSKYLENS);
        webcam    = hw.tryGet(WebcamName.class, RobotConstants.WEBCAM);
    }

    /** Call once at the top of every loop before reading any motor/sensor on the hubs. */
    public void clearBulkCache() {
        for (LynxModule hub : hubs) hub.clearBulkCache();
    }

    /** Lowest valid battery voltage across all hubs (used for flywheel feedforward scaling). */
    public double batteryVoltage() {
        double v = Double.POSITIVE_INFINITY;
        for (VoltageSensor s : hardwareMap.voltageSensor) {
            double reading = s.getVoltage();
            if (reading > 1.0) v = Math.min(v, reading);
        }
        return Double.isInfinite(v) ? RobotConstants.NOMINAL_VOLTAGE : v;
    }

    /** Zero every motor (G404 / end of AUTO). Servos simply keep their last position. */
    public void stopAllMotors() {
        for (DcMotorEx m : new DcMotorEx[]{fl, fr, bl, br, shooterFront, shooterRear, intake, magazine}) {
            m.setPower(0.0);
        }
    }
}
