package org.firstinspires.ftc.teamcode.hivequeen.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hivequeen.HiveQueenRobot;
import org.firstinspires.ftc.teamcode.hivequeen.RobotConstants;
import org.firstinspires.ftc.teamcode.hivequeen.game.Alliance;
import org.firstinspires.ftc.teamcode.hivequeen.game.AutoPlanner;
import org.firstinspires.ftc.teamcode.hivequeen.game.BallType;
import org.firstinspires.ftc.teamcode.hivequeen.game.HiveTracker;
import org.firstinspires.ftc.teamcode.hivequeen.subsystems.DriveToPose;
import org.firstinspires.ftc.teamcode.hivequeen.subsystems.TargetObservation;
import org.firstinspires.ftc.teamcode.hivequeen.util.CompetitionTelemetry;
import org.firstinspires.ftc.teamcode.hivequeen.util.EdgeDetector;
import org.firstinspires.ftc.teamcode.hivequeen.util.Pose;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/**
 * HIVE-QUEEN Autonomous: a time-budgeted state machine shared by both alliances. Every path is
 * authored for RED in the field frame of {@link RobotConstants} and rotated 180 deg for BLUE
 * (the field is rotationally symmetric). No FLOWER interaction anywhere.
 *
 * Start (G304): back touching our alliance wall between the FLOWER and the LOADING ZONE, facing
 * into the field, 3 POLLEN in the magazine + the 4th preload on the TILES touching the front
 * funnel (R102: a 4th inside lifts the floating intake to 456.1 / 457 mm).
 *
 *  1. LEAVE_INTAKE  drive ~300 mm straight off the wall with the intake on (LEAVE, 3 pts) until the
 *                   ball sensors count the 4th preload or 1.5 s.
 *  2. DRIVE_TO_SHOT spin up while driving to the shot pose on the up-CELL's side (D ~1.07 m).
 *  3. AIM           heading-lock on the up CELL's AprilTag cluster (odometry bearing fallback).
 *  4. FIRE          burst the magazine. 4 POLLEN + 3 staged NECTAR = 223.6 g -> TIP #1 (20 pts).
 *                   If the 4th preload was NOT ingested only 3 are fired: 198.7 g is under the
 *                   199 g must-tip line, so tip #1 is UNCERTAIN (see step 5).
 *  5. DECIDE        {@link AutoPlanner#next}: sweep the tip spill (intake guard active: max 4,
 *                   never opponent NECTAR), grab the LOADING-ZONE NECTAR the HP may enter per tip
 *                   (G426.A), shoot again into the newly-up CELL (aiming at tip #2), or PARK.
 *                   Uncertain tip #1: sweep the spill area anyway; picking up any ball proves the
 *                   CELL spilled (tip confirmed), finding nothing means it did not tip.
 *  6. PARK          at the latest at {@link RobotConstants#AUTO_PARK_ABORT_S} (24 s) whatever the
 *                   state; the PARK drive is cut at 29.5 s. Then everything stops (G403).
 *
 * Every step has a timeout, clipped so no step can run past the park-abort time.
 */
public abstract class HiveQueenAutoBase extends LinearOpMode {

    protected abstract Alliance alliance();

    private enum State {
        START_DELAY, LEAVE_INTAKE, DRIVE_TO_SHOT, AIM, FIRE, DECIDE, SPILL_SETTLE, SWEEP, TO_LZ, LZ_INTAKE, PARK, DONE
    }

    /** INIT-selectable auto mode (gamepad1 X cycles). */
    private enum Mode { FULL, PRELOADS_THEN_PARK, LEAVE_AND_PARK }
    private Mode mode = Mode.FULL;
    private boolean altStart = false;
    private int startDelayS = 0;

    private HiveQueenRobot robot;
    private Alliance a;
    private DriveToPose driveTo;
    private HiveTracker tracker;

    private State state;
    private double stateStartS;
    private double stateTimeoutS;
    /** Route being followed (last pose = final target, the rest are pass-through). */
    private final List<Pose> route = new ArrayList<>();
    private int routeIndex;

    private Pose shotPose;
    private Pose leavePose;
    private int countAtStepStart;
    // FIRE bookkeeping
    private int shotsSeen;
    private int shotsThisFire;
    private int tipsAtFireStart;
    private double indexerOpenSinceS, lastShotS, emptySinceS;
    // planner inputs
    private int spillBalls = 0;
    private HiveTracker.Side spillSide = null;
    private boolean spillUncertain = false;
    private int sweepsOfSpill = 0;
    private int lzNectar = 0;
    private int fireFailures = 0;
    private double lastTipS = -100.0;
    private String lastDecision = "-";

    @Override
    public void runOpMode() {
        CompetitionTelemetry.configure(telemetry);   // R704.D
        a = alliance();
        Alliance.lastUsed = a;
        HiveQueenRobot.carryValid = false;
        robot = new HiveQueenRobot(hardwareMap, a, true, true);
        robot.init(true);                             // zero Pinpoint + IMU while still (motors never run in INIT)
        robot.prePositionServos();                    // R103.B: S1 stowed, S3 closed, S6 hood -- then stay still
        driveTo = new DriveToPose(robot.drive, robot.localizer);
        tracker = new HiveTracker(a.initialUpSide());
        // INIT menu (reading gamepads is allowed in INIT; nothing moves - G403):
        //   dpad left/right = start slot A/B, dpad up/down = start delay, X = mode.
        EdgeDetector slotL = new EdgeDetector(), slotR = new EdgeDetector(), dUp = new EdgeDetector(),
                dDn = new EdgeDetector(), modeBtn = new EdgeDetector();
        Pose start = a.fromRed(new Pose(RobotConstants.AUTO_START_X_MM, RobotConstants.AUTO_START_Y_MM,
                RobotConstants.AUTO_START_HEADING_DEG));   // default if START is pressed before the menu runs

        while (opModeInInit()) {
            if (slotL.rose(gamepad1.dpad_left) || slotR.rose(gamepad1.dpad_right)) altStart = !altStart;
            if (dUp.rose(gamepad1.dpad_up)) startDelayS = Math.min(RobotConstants.AUTO_MAX_START_DELAY_S, startDelayS + 1);
            if (dDn.rose(gamepad1.dpad_down)) startDelayS = Math.max(0, startDelayS - 1);
            if (modeBtn.rose(gamepad1.x)) mode = Mode.values()[(mode.ordinal() + 1) % Mode.values().length];
            start = a.fromRed(new Pose(altStart ? RobotConstants.AUTO_START_X_ALT_MM : RobotConstants.AUTO_START_X_MM,
                    RobotConstants.AUTO_START_Y_MM, RobotConstants.AUTO_START_HEADING_DEG));
            robot.localizer.update();
            telemetry.addLine("HIVE-QUEEN Auto " + a + " ready: 3 POLLEN in magazine + 1 on the tiles at the funnel");
            telemetry.addLine(String.format(Locale.US, "Slot %s (dpad L/R) | delay %d s (dpad U/D) | mode %s (X)",
                    altStart ? "B" : "A", startDelayS, mode));
            telemetry.addLine("Pinpoint: " + robot.localizer.getStatus()
                    + "  HuskyLens: " + (robot.husky.isConnected() ? "OK" : "NOT FOUND")
                    + "  Webcam: " + (robot.webcamTags.isAvailable() ? "OK" : "none"));
            telemetry.addLine("Start pose " + start + "  up CELL: " + tracker.getUpSide()
                    + "  | servos pre-positioned (R103.B): keep still");
            telemetry.update();
            sleep(50);
        }
        if (isStopRequested()) return;
        // LEAVE: straight ahead (into the field) by AUTO_LEAVE_INTAKE_DIST_MM, same heading.
        double h0 = Math.toRadians(start.headingDeg);
        leavePose = new Pose(start.xMm + RobotConstants.AUTO_LEAVE_INTAKE_DIST_MM * Math.cos(h0),
                start.yMm + RobotConstants.AUTO_LEAVE_INTAKE_DIST_MM * Math.sin(h0), start.headingDeg);

        robot.clock.markAutoStart();
        robot.localizer.setPose(start);
        robot.balls.setCount(RobotConstants.PRELOADS_IN_MAGAZINE, BallType.POLLEN);
        robot.setTargetSide(tracker.getUpSide());
        robot.setVisionPreferTargetCell(true);
        if (startDelayS > 0) enter(State.START_DELAY, startDelayS);
        else enter(State.LEAVE_INTAKE, RobotConstants.AUTO_LEAVE_INTAKE_TIMEOUT_S);

        while (opModeIsActive()) {
            if (!robot.updateSensorsAndGates()) break;      // end of AUTO -> G403 motionless
            double t = robot.clock.matchElapsedS();
            double inState = t - stateStartS;
            boolean timedOut = inState >= stateTimeoutS;

            // Hard park deadline: abort whatever we are doing.
            if (AutoPlanner.mustPark(t) && state != State.PARK && state != State.DONE) beginPark(t);

            double intakeReq = 0.0;
            boolean spin = false, fire = false;
            switch (state) {
                case START_DELAY:
                    robot.drive.stop();
                    if (timedOut) enter(State.LEAVE_INTAKE, AutoPlanner.stepTimeoutS(t, RobotConstants.AUTO_LEAVE_INTAKE_TIMEOUT_S));
                    break;
                case LEAVE_INTAKE: {
                    intakeReq = RobotConstants.INTAKE_POWER;
                    boolean got4th = robot.balls.getCount() >= RobotConstants.PRELOADED_POLLEN;
                    driveTo.step(leavePose, RobotConstants.AUTO_COLLECT_POWER);   // holds there once reached
                    if (got4th || timedOut) {
                        // Not detected -> fire the 3 anyway; tip #1 becomes "uncertain" (198.7 g).
                        if (mode == Mode.LEAVE_AND_PARK) beginPark(t);
                        else startShot(t);
                    }
                    break;
                }
                case DRIVE_TO_SHOT:
                    spin = true;
                    if (followRoute(RobotConstants.AUTO_MAX_POWER) || timedOut) {
                        robot.headingLock.reset();
                        enter(State.AIM, AutoPlanner.stepTimeoutS(t, RobotConstants.AUTO_AIM_TIMEOUT_S));
                    }
                    break;
                case AIM: {
                    spin = true;
                    TargetObservation obs = robot.bestTarget();
                    boolean onTarget;
                    if (obs != null) {
                        double turn = robot.headingLock.update(obs, robot.localizer.getHeadingDeg(), System.nanoTime());
                        robot.drive.driveRobot(0, 0, Double.isNaN(turn) ? 0 : turn);
                        onTarget = robot.headingLock.isOnTarget();
                    } else {
                        Pose p = robot.localizer.getPose();
                        onTarget = driveTo.step(shotPose.withHeading(robot.aimHeadingDeg(p)), RobotConstants.AUTO_MAX_POWER);
                    }
                    if (onTarget || timedOut) {
                        robot.drive.stop();
                        startFire(t);
                    }
                    break;
                }
                case FIRE:
                    spin = true;
                    fire = true;
                    robot.drive.stop();
                    updateFire(t, timedOut);
                    break;
                case DECIDE:
                    robot.drive.stop();
                    decide(t);
                    break;
                case SPILL_SETTLE:
                    robot.drive.stop();                        // G409: let spilled balls hit the TILES first
                    if (timedOut) startSweep(t);
                    break;
                case SWEEP:
                    intakeReq = RobotConstants.INTAKE_POWER;
                    if (followRoute(RobotConstants.AUTO_COLLECT_POWER) || timedOut
                            || robot.balls.getCount() >= RobotConstants.MAX_CONTROLLED_ELEMENTS) {
                        finishSweep(t);
                    }
                    break;
                case TO_LZ:
                    if (followRoute(RobotConstants.AUTO_MAX_POWER) || timedOut) {
                        countAtStepStart = robot.balls.getCount();
                        setRoute(lzPose(RobotConstants.AUTO_LZ_INTAKE_Y_MM));
                        enter(State.LZ_INTAKE, AutoPlanner.stepTimeoutS(t, RobotConstants.AUTO_LZ_INTAKE_TIMEOUT_S));
                    }
                    break;
                case LZ_INTAKE:
                    intakeReq = RobotConstants.INTAKE_POWER;
                    if (followRoute(RobotConstants.AUTO_COLLECT_POWER) || timedOut
                            || robot.balls.getCount() > countAtStepStart
                            || robot.balls.getCount() >= RobotConstants.MAX_CONTROLLED_ELEMENTS) {
                        lzNectar = Math.max(0, lzNectar - 1);   // one attempt per available NECTAR
                        enter(State.DECIDE, 1.0);
                    }
                    break;
                case PARK:
                    if (followRoute(RobotConstants.AUTO_MAX_POWER) || timedOut
                            || t >= RobotConstants.AUTO_PARK_DEADLINE_S) {
                        robot.drive.stop();
                        enter(State.DONE, 1e9);
                    }
                    break;
                case DONE:
                default:
                    robot.drive.stop();
                    break;
            }
            robot.updateMechanisms(intakeReq, spin, fire);

            HiveQueenRobot.carryBallCount = robot.balls.getCount();   // hand-over to TeleOp
            HiveQueenRobot.carryUpSide = tracker.getUpSide();
            HiveQueenRobot.carryValid = true;

            robot.addTelemetry(telemetry);
            telemetry.addLine(String.format(Locale.US,
                    "AUTO %s t=%.1fs | tips(est)=%d up=%s cell=%.0fg%s | spill=%d lz=%d fails=%d | %s",
                    state, t, tracker.getTips(), tracker.getUpSide(), tracker.getCellMassG(),
                    tracker.isTipUncertain() ? "?" : "", spillBalls, lzNectar, fireFailures, lastDecision));
            telemetry.update();
        }
        robot.shutdown();
    }

    // ------------------------------------------------------------------------------------------
    // State helpers
    // ------------------------------------------------------------------------------------------
    private void enter(State s, double timeoutS) {
        state = s;
        stateStartS = robot.clock.matchElapsedS();
        stateTimeoutS = timeoutS;
    }

    private void startShot(double t) {
        robot.setTargetSide(tracker.getUpSide());
        robot.setVisionPreferTargetCell(true);
        Pose base = sidePose(RobotConstants.AUTO_SHOT_X_MM, RobotConstants.AUTO_SHOT_Y_MM, 0.0, tracker.getUpSide());
        shotPose = base.withHeading(robot.aimHeadingDeg(base));
        setRoute(shotPose);
        enter(State.DRIVE_TO_SHOT, AutoPlanner.stepTimeoutS(t, RobotConstants.AUTO_DRIVE_TIMEOUT_S));
    }

    private void startFire(double t) {
        shotsSeen = robot.balls.getShotsCounted();
        shotsThisFire = 0;
        tipsAtFireStart = tracker.getTips();
        countAtStepStart = robot.balls.getCount();
        indexerOpenSinceS = -1;
        lastShotS = -1;
        emptySinceS = -1;
        enter(State.FIRE, AutoPlanner.stepTimeoutS(t, RobotConstants.AUTO_FIRE_TIMEOUT_S));
    }

    private void updateFire(double t, boolean timedOut) {
        int shots = robot.balls.getShotsCounted();
        while (shotsSeen < shots) {                 // feed every counted launch into the HIVE model
            shotsSeen++;
            shotsThisFire++;
            lastShotS = t;
            tracker.recordLaunch(robot.balls.getLastShotBall());
        }
        if (robot.shooter.isIndexerOpen() && indexerOpenSinceS < 0) indexerOpenSinceS = t;

        int count = robot.balls.getCount();
        if (count == 0) {
            if (emptySinceS < 0) emptySinceS = t;
            // keep feeding so the last ball coasts the 53 mm unpowered gap into the counter-wheel
            if (t - emptySinceS >= RobotConstants.AUTO_LAST_BALL_DWELL_S) finishFire(t, false);
            return;
        }
        if (indexerOpenSinceS >= 0
                && t - Math.max(indexerOpenSinceS, lastShotS) >= RobotConstants.AUTO_ASSUME_EMPTY_S) {
            // Gate open at full feed for 2 s with no throat event: the throat counter is missing
            // shots. A 4-ball burst takes < 1.5 s, so assume the rest were launched (unknown type
            // -> POLLEN mass, the conservative choice for the tip estimate).
            for (int i = 0; i < count; i++) tracker.recordLaunch(BallType.POLLEN);
            shotsThisFire += count;
            robot.balls.setCount(0);
            finishFire(t, false);
            return;
        }
        if (timedOut) finishFire(t, true);
    }

    private void finishFire(double t, boolean timedOut) {
        if (timedOut && shotsThisFire == 0) fireFailures++;
        int newTips = tracker.getTips() - tipsAtFireStart;
        if (newTips > 0) {
            lastTipS = t;
            spillSide = tracker.getLastSpillSide();
            spillBalls = tracker.getLastSpillBalls();
            spillUncertain = false;
            sweepsOfSpill = 0;
            lzNectar += newTips;                      // G426.A: HP may enter 1 NECTAR per tip
        } else if (tracker.isTipUncertain() && shotsThisFire > 0) {
            // e.g. only 3 preloads fired: 198.7 g. Sweep the up CELL's spill area to find out.
            lastTipS = t;
            spillSide = tracker.getUpSide();
            spillBalls = tracker.getBallsInCell();
            spillUncertain = true;
            sweepsOfSpill = 0;
        }
        robot.setTargetSide(tracker.getUpSide());
        robot.setVisionPreferTargetCell(true);
        enter(State.DECIDE, 1.0);
    }

    private void decide(double t) {
        if (mode != Mode.FULL) { beginPark(t); return; }   // PRELOADS_THEN_PARK: park after the first volley
        boolean shootingAllowed = fireFailures <= RobotConstants.AUTO_MAX_FIRE_FAILURES;
        AutoPlanner.Action act = AutoPlanner.next(t, robot.balls.getCount(), shootingAllowed, spillBalls, lzNectar);
        lastDecision = act.name();
        switch (act) {
            case SHOOT:
                startShot(t);
                break;
            case COLLECT_SPILL: {
                double settleLeft = lastTipS + RobotConstants.AUTO_SPILL_SETTLE_S - t;
                if (settleLeft > 0) enter(State.SPILL_SETTLE, AutoPlanner.stepTimeoutS(t, settleLeft));
                else startSweep(t);
                break;
            }
            case COLLECT_LOADING_ZONE:
                setRoute(lzPose(RobotConstants.AUTO_LZ_APPROACH_Y_MM));
                enter(State.TO_LZ, AutoPlanner.stepTimeoutS(t, RobotConstants.AUTO_DRIVE_TIMEOUT_S));
                break;
            case PARK:
            default:
                beginPark(t);
                break;
        }
    }

    private void startSweep(double t) {
        countAtStepStart = robot.balls.getCount();
        route.clear();
        Pose prev = robot.localizer.getPose();
        for (double[] xy : RobotConstants.AUTO_SPILL_SWEEP_XY) {
            Pose p = sidePose(xy[0], xy[1], 0.0, spillSide);
            Pose withHeading = p.withHeading(prev.bearingTo(p));   // intake (front) leads the sweep
            addLeg(prev, withHeading);
            prev = withHeading;
        }
        routeIndex = 0;
        double nominal = RobotConstants.AUTO_SWEEP_LEG_TIMEOUT_S * RobotConstants.AUTO_SPILL_SWEEP_XY.length;
        enter(State.SWEEP, AutoPlanner.stepTimeoutS(t, nominal));
    }

    private void finishSweep(double t) {
        int collected = Math.max(0, robot.balls.getCount() - countAtStepStart);
        sweepsOfSpill++;
        if (spillUncertain) {
            if (collected > 0) {
                // Balls were lying where the up CELL would spill: the HIVE did tip.
                tracker.flip();
                spillBalls = tracker.getLastSpillBalls() - collected;
                spillUncertain = false;
                lzNectar += 1;
                robot.setTargetSide(tracker.getUpSide());
                robot.setVisionPreferTargetCell(true);
            } else {
                spillBalls = 0;                           // nothing there: assume no tip
            }
        } else {
            spillBalls -= collected;
        }
        if (spillBalls <= 0 || sweepsOfSpill >= RobotConstants.AUTO_MAX_SWEEPS_PER_SPILL) spillBalls = 0;
        enter(State.DECIDE, 1.0);
    }

    private void beginPark(double t) {
        // Slot A parks on the HIVE-side half of the LOADING ZONE, slot B on the other half, so partners
        // never aim at the same spot; both still overlap the zone (only "partially in" is required).
        double parkX = RobotConstants.RED_LOADING_ZONE_X_MM + (altStart ? 1 : -1) * RobotConstants.AUTO_PARK_SLOT_OFFSET_MM;
        setRoute(a.fromRed(new Pose(parkX, RobotConstants.AUTO_PARK_Y_MM, 90.0)));
        lastDecision = "PARK";
        enter(State.PARK, AutoPlanner.parkTimeoutS(t));
    }

    // ------------------------------------------------------------------------------------------
    // Geometry helpers
    // ------------------------------------------------------------------------------------------
    /** RED-authored AUDIENCE-side point, mirrored to the rear side for {@code ourSide == FAR}, then to our alliance. */
    private Pose sidePose(double redX, double redY, double redH, HiveTracker.Side ourSide) {
        HiveTracker.Side redSide = a == Alliance.RED ? ourSide : ourSide.opposite();
        Pose p = new Pose(redX, redY, redH);
        if (redSide == HiveTracker.Side.FAR) p = p.mirroredX();
        return a.fromRed(p);
    }

    /** LOADING ZONE column, facing our wall (red +90 deg). */
    private Pose lzPose(double redY) {
        return a.fromRed(new Pose(RobotConstants.RED_LOADING_ZONE_X_MM, redY, 90.0));
    }

    /** New route from the current pose to {@code target}, with a via point around the HIVE frame if needed. */
    private void setRoute(Pose target) {
        route.clear();
        addLeg(robot.localizer.getPose(), target);
        routeIndex = 0;
    }

    private void addLeg(Pose from, Pose to) {
        if (AutoPlanner.segmentHitsHiveFrame(from.xMm, from.yMm, to.xMm, to.yMm)) {
            Pose via = a.fromRed(new Pose(RobotConstants.AUTO_VIA_X_MM, RobotConstants.AUTO_VIA_Y_MM, 0.0));
            route.add(via.withHeading(to.headingDeg));
        }
        route.add(to);
    }

    /** Step along the route. Intermediate points are pass-through (loose tolerance). @return true at the end. */
    private boolean followRoute(double maxPower) {
        if (route.isEmpty()) { robot.drive.stop(); return true; }
        while (routeIndex < route.size() - 1) {
            boolean reached = driveTo.step(route.get(routeIndex), maxPower, 120.0, 30.0, false);
            if (!reached) return false;
            routeIndex++;
        }
        return driveTo.step(route.get(route.size() - 1), maxPower);
    }
}
