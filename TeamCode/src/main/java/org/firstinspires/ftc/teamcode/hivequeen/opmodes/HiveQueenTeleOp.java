package org.firstinspires.ftc.teamcode.hivequeen.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hivequeen.RobotConstants;
import org.firstinspires.ftc.teamcode.hivequeen.HiveQueenRobot;
import org.firstinspires.ftc.teamcode.hivequeen.game.Alliance;
import org.firstinspires.ftc.teamcode.hivequeen.game.BallType;
import org.firstinspires.ftc.teamcode.hivequeen.game.MatchClock;
import org.firstinspires.ftc.teamcode.hivequeen.util.CompetitionTelemetry;
import org.firstinspires.ftc.teamcode.hivequeen.util.EdgeDetector;
import org.firstinspires.ftc.teamcode.hivequeen.util.MathUtil;
import org.firstinspires.ftc.teamcode.hivequeen.util.Pose;

/**
 * HIVE-QUEEN TeleOp (single OpMode, alliance chosen during INIT).
 *
 * INIT: gamepad1 X = BLUE, B = RED (defaults to the last Autonomous alliance). Motors never run
 * in INIT. Servos (S1 stowed, S3 closed, S6 hood) are pre-positioned in INIT (R103.B) ONLY when no
 * AUTO ran in the last ~98 s; after our AUTO (= during the AUTO->TELEOP transition) nothing moves
 * in INIT (G403 note). If you INIT this OpMode during the transition WITHOUT having run our AUTO,
 * wait until TELEOP begins (the servos would twitch). Motion is enabled only while the MATCH clock
 * is in TELEOP; if START is pressed during the transition the robot waits (G403), and all outputs
 * are cut 2:00 after TELEOP starts (G404).
 *
 * After an Autonomous in the same match the Pinpoint field pose, magazine count and the
 * "which CELL is up" estimate are carried over (the pose is NOT re-zeroed). Without an AUTO the
 * count starts at PRELOADS_IN_MAGAZINE (3: G304 requires 4 preloads, the 4th is on the TILES and
 * is counted at the lip when ingested); place the robot facing AWAY from the drivers at INIT (or
 * press BACK once so facing).
 *
 * gamepad1 (driver):  left stick = translate, right stick X = turn, LB = slow,
 *                     RB (hold) = heading-lock on the alliance CELL AprilTag cluster,
 *                     dpad_up = toggle field-centric, back = "robot now faces away from us".
 * gamepad2 (gunner):  RT = intake, LT = outtake, A = toggle flywheels, RB (hold) = fire,
 *                     X = set magazine count 0, B = set magazine count 4,
 *                     Y = HIVE tipped -> switch target CELL (odometry aim/range fallback),
 *                     BACK = toggle the G408 COLOUR GATE override (OFF = accept every colour at
 *                     the lip and do not block an opponent NECTAR at the throat; use only if the
 *                     lip/throat colour sensor misreads -- the drivers are then responsible for
 *                     G408). The gate also turns OFF automatically while lipColor is in FAULT.
 */
@TeleOp(name = "HIVE-QUEEN TeleOp", group = "HIVE-QUEEN")
public class HiveQueenTeleOp extends OpMode {
    private HiveQueenRobot robot;
    private Alliance alliance = Alliance.lastUsed;
    private boolean fromAuto = false;

    private boolean fieldCentric = false;
    private boolean flywheelsOn = false;
    private final EdgeDetector fieldCentricBtn = new EdgeDetector();
    private final EdgeDetector flywheelBtn = new EdgeDetector();
    private final EdgeDetector headingResetBtn = new EdgeDetector();
    private final EdgeDetector targetSideBtn = new EdgeDetector();
    private final EdgeDetector colourGateBtn = new EdgeDetector();
    private boolean colourGateOverride = false;
    private boolean servosPrePositioned = false;

    @Override
    public void init() {
        CompetitionTelemetry.configure(telemetry);   // R704.D
        fromAuto = HiveQueenRobot.carryValid && MatchClock.hasRecentAutoAnchor(System.nanoTime());
        robot = new HiveQueenRobot(hardwareMap, alliance, false, true);
        robot.init(!fromAuto);                         // keep the AUTO field pose
        if (fromAuto) robot.setTargetSide(HiveQueenRobot.carryUpSide);
        robot.setVisionPreferTargetCell(false);        // TeleOp: closest own cluster
        // R103.B pre-position, but never during the AUTO->TELEOP transition (G403 note).
        if (!MatchClock.hasRecentAutoAnchor(System.nanoTime())) {
            robot.prePositionServos();
            servosPrePositioned = true;
        }
        telemetry.addLine("HIVE-QUEEN TeleOp initialized. X = BLUE, B = RED");
    }

    @Override
    public void init_loop() {
        if (gamepad1.x) alliance = Alliance.BLUE;
        if (gamepad1.b) alliance = Alliance.RED;
        robot.setAlliance(alliance);
        robot.localizer.update();
        robot.husky.update();
        telemetry.addLine("Alliance: " + alliance + "   (gamepad1 X = BLUE, B = RED)");
        telemetry.addLine("HuskyLens: " + (robot.husky.isConnected() ? "OK" : "NOT FOUND")
                + "  Webcam: " + (robot.webcamTags.isAvailable() ? "OK" : "none"));
        telemetry.addLine("Pinpoint: " + robot.localizer.getStatus()
                + (fromAuto ? "  (pose kept from AUTO " + robot.localizer.getPose() + ")" : "  (pose reset)"));
        telemetry.addLine(servosPrePositioned ? "Servos pre-positioned in INIT (R103.B) - keep the robot still"
                : "Servos held from AUTO (no INIT motion, G403)");
    }

    @Override
    public void start() {
        robot.clock.startTeleOp();
        if (fromAuto && robot.clock.usedAutoAnchor()) {
            robot.balls.setCount(HiveQueenRobot.carryBallCount);
        } else {
            // G304: 4 preloads, 3 ride in the magazine, the 4th is on the TILES (counted at the lip).
            robot.balls.setCount(RobotConstants.PRELOADS_IN_MAGAZINE, BallType.POLLEN);
            // No AUTO: robot was set facing away from the drivers -> field heading = driver facing.
            Pose p = robot.localizer.getPose();
            robot.localizer.setPose(new Pose(p.xMm, p.yMm, alliance.driverFacingDeg()));
        }
        HiveQueenRobot.carryValid = false;   // consume the hand-over
    }

    @Override
    public void loop() {
        boolean motion = robot.updateSensorsAndGates();

        // ---------------- driver ----------------
        if (fieldCentricBtn.rose(gamepad1.dpad_up)) fieldCentric = !fieldCentric;
        if (headingResetBtn.rose(gamepad1.back)) {
            // "The robot now faces away from us": keep the field frame consistent with AUTO.
            Pose p = robot.localizer.getPose();
            robot.localizer.setPose(new Pose(p.xMm, p.yMm, alliance.driverFacingDeg()));
        }
        double scale = gamepad1.left_bumper ? RobotConstants.DRIVE_SLOW_MODE_SCALE : 1.0;
        double fwd = -MathUtil.deadband(gamepad1.left_stick_y, RobotConstants.DRIVE_STICK_DEADBAND) * scale;
        double left = -MathUtil.deadband(gamepad1.left_stick_x, RobotConstants.DRIVE_STICK_DEADBAND) * scale;
        double turn = -MathUtil.deadband(gamepad1.right_stick_x, RobotConstants.DRIVE_STICK_DEADBAND) * scale;

        if (gamepad1.right_bumper) {
            double lockTurn = robot.headingLock.update(robot.bestTarget(),
                    robot.localizer.getHeadingDeg(), System.nanoTime());
            if (!Double.isNaN(lockTurn)) turn = lockTurn;   // no target ever seen -> driver keeps turn
        } else {
            robot.headingLock.reset();
        }

        if (motion) {
            if (fieldCentric) {
                // Stick "up" = away from the drivers = field heading driverFacingDeg (red -90, blue +90).
                double h = robot.localizer.getHeadingDeg() - alliance.driverFacingDeg();
                robot.drive.driveField(fwd, left, turn, h);
            } else {
                robot.drive.driveRobot(fwd, left, turn);
            }
        }

        // ---------------- gunner ----------------
        if (flywheelBtn.rose(gamepad2.a)) flywheelsOn = !flywheelsOn;
        if (targetSideBtn.rose(gamepad2.y)) robot.setTargetSide(robot.getTargetSide().opposite());
        if (colourGateBtn.rose(gamepad2.back)) {
            colourGateOverride = !colourGateOverride;
            robot.setColourGateOverride(colourGateOverride);
        }
        if (gamepad2.x) robot.balls.setCount(0);
        if (gamepad2.b) robot.balls.setCount(RobotConstants.MAX_CONTROLLED_ELEMENTS);
        double intakeReq = gamepad2.right_trigger > 0.1 ? RobotConstants.INTAKE_POWER * gamepad2.right_trigger
                : (gamepad2.left_trigger > 0.1 ? -gamepad2.left_trigger : 0.0);

        robot.updateMechanisms(intakeReq, flywheelsOn, gamepad2.right_bumper);

        robot.addTelemetry(telemetry);
        telemetry.addLine((fieldCentric ? "FIELD" : "ROBOT") + "-centric"
                + (gamepad1.right_bumper ? (robot.headingLock.isOnTarget() ? "  AIM: LOCKED" : "  AIM: turning") : "")
                + "  | gp2 BACK = colour gate " + (colourGateOverride ? "override ON (G408 = drivers!)" : "override off"));
    }

    @Override
    public void stop() {
        if (robot != null) robot.shutdown();
    }
}
