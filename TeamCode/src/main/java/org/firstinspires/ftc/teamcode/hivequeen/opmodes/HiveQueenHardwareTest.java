package org.firstinspires.ftc.teamcode.hivequeen.opmodes;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.hivequeen.RobotConstants;
import org.firstinspires.ftc.teamcode.hivequeen.RobotHardware;
import org.firstinspires.ftc.teamcode.hivequeen.subsystems.PinpointLocalizer;
import org.firstinspires.ftc.teamcode.hivequeen.util.CompetitionTelemetry;
import org.firstinspires.ftc.teamcode.hivequeen.util.EdgeDetector;

import java.util.Locale;

/**
 * Pit hardware test. Verifies every configured device name and lets you exercise ONE actuator
 * at a time. Not for MATCH use.
 *
 * gamepad1 dpad up/down = select device; left stick Y = motor power (hold) or servo position
 * (servo moves only while A is held, so nothing moves by accident); telemetry shows every sensor.
 *
 * SERVO END-STOPS are set HERE, on the bench, with the mechanism free: creep toward each end with
 * A held, stop just before the printed part touches its hard stop, and write the base position
 * into RobotConstants (LIP_GATE_OPEN, INDEXER_CLOSED, HOOD_CAL_LOW_POS + directions). The match
 * code then limits each servo to its CAD span (S1 122 deg, S3 60 deg, S6 10 deg of 300 deg).
 * This screen is NOT span-limited -- watch the mechanism.
 */
@TeleOp(name = "HIVE-QUEEN Hardware Test", group = "HIVE-QUEEN Test")
public class HiveQueenHardwareTest extends OpMode {
    private RobotHardware hw;
    private PinpointLocalizer loc;
    private DcMotorEx[] motors;
    private String[] motorNames;
    private Servo[] servos;
    private String[] servoNames;
    private int selected = 0;
    private final EdgeDetector up = new EdgeDetector();
    private final EdgeDetector down = new EdgeDetector();
    private boolean huskyOk = false;

    @Override
    public void init() {
        CompetitionTelemetry.configure(telemetry);
        hw = new RobotHardware(hardwareMap);
        loc = new PinpointLocalizer(hw.pinpoint);
        loc.resetAndCalibrate();
        huskyOk = hw.huskyLens != null && hw.huskyLens.knock();
        if (huskyOk) hw.huskyLens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);
        motors = new DcMotorEx[]{hw.fl, hw.fr, hw.bl, hw.br, hw.shooterFront, hw.shooterRear, hw.intake, hw.magazine};
        motorNames = new String[]{RobotConstants.FRONT_LEFT, RobotConstants.FRONT_RIGHT, RobotConstants.BACK_LEFT,
                RobotConstants.BACK_RIGHT, RobotConstants.SHOOTER_FRONT, RobotConstants.SHOOTER_REAR,
                RobotConstants.INTAKE, RobotConstants.MAGAZINE};
        servos = new Servo[]{hw.lipGate, hw.indexerGate, hw.hoodPitch};
        servoNames = new String[]{RobotConstants.LIP_GATE, RobotConstants.INDEXER_GATE,
                RobotConstants.HOOD_PITCH};
        telemetry.addLine("All devices found. Webcam: " + (hw.webcam != null ? "yes" : "no")
                + "  HuskyLens: " + (huskyOk ? "OK" : "NO ANSWER"));
    }

    @Override
    public void loop() {
        hw.clearBulkCache();
        loc.update();
        int total = motors.length + servos.length;
        if (up.rose(gamepad1.dpad_up)) selected = (selected + total - 1) % total;
        if (down.rose(gamepad1.dpad_down)) selected = (selected + 1) % total;

        double stick = -gamepad1.left_stick_y;
        for (int i = 0; i < motors.length; i++) motors[i].setPower(i == selected ? stick : 0.0);
        String sel;
        if (selected < motors.length) {
            DcMotorEx m = motors[selected];
            double tpr = selected < 4 ? RobotConstants.DRIVE_MOTOR_TICKS_PER_REV
                    : selected < 6 ? RobotConstants.SHOOTER_TICKS_PER_REV
                    : selected == 6 ? RobotConstants.INTAKE_TICKS_PER_REV       // M7 1150 RPM
                    : RobotConstants.MAGAZINE_TICKS_PER_REV;                     // M8 1150 RPM
            sel = String.format(Locale.US, "MOTOR %s power %.2f  vel %.0f t/s = %.0f rpm  pos %d",
                    motorNames[selected], stick, m.getVelocity(), m.getVelocity() / tpr * 60.0,
                    m.getCurrentPosition());
        } else {
            int s = selected - motors.length;
            double pos = 0.5 + 0.5 * stick;
            if (gamepad1.a) servos[s].setPosition(pos);
            double[][] ends = {
                    {RobotConstants.LIP_GATE_OPEN, RobotConstants.LIP_GATE_BLOCKED},
                    {RobotConstants.INDEXER_CLOSED, RobotConstants.INDEXER_OPEN},
                    {RobotConstants.HOOD_MIN_POS, RobotConstants.HOOD_MAX_POS}};
            sel = String.format(Locale.US, "SERVO %s target %.3f (hold A to move) | match limits %.3f..%.3f",
                    servoNames[s], pos, ends[s][0], ends[s][1]);
        }

        telemetry.addLine(sel);
        telemetry.addLine("Pinpoint " + loc.getStatus() + " " + loc.getPose()
                + " encX=" + loc.rawEncoderX() + " encY=" + loc.rawEncoderY());
        NormalizedRGBA lip = hw.lipColor.getNormalizedColors();
        NormalizedRGBA thr = hw.throatColor.getNormalizedColors();
        telemetry.addLine(String.format(Locale.US, "lipColor r%.3f g%.3f b%.3f | throatColor r%.3f g%.3f b%.3f",
                lip.red, lip.green, lip.blue, thr.red, thr.green, thr.blue));
        telemetry.addLine(String.format(Locale.US, "ballCount1 (slot 2) %.0f mm  ballCount2 (slot 3) %.0f mm  V=%.2f",
                hw.ballCount1.getDistance(DistanceUnit.MM), hw.ballCount2.getDistance(DistanceUnit.MM),
                hw.batteryVoltage()));
        if (huskyOk) {
            HuskyLens.Block[] blocks = hw.huskyLens.blocks();
            telemetry.addLine("HuskyLens tags: " + (blocks == null ? 0 : blocks.length));
            if (blocks != null) {
                for (int i = 0; i < Math.min(4, blocks.length); i++) {
                    telemetry.addLine("  id=" + blocks[i].id + " x=" + blocks[i].x + " w=" + blocks[i].width);
                }
            }
        }
    }

    @Override
    public void stop() {
        if (hw != null) hw.stopAllMotors();
    }
}
