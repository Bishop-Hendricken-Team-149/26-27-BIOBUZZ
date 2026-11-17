package org.firstinspires.ftc.teamcode.hivequeen;

import org.firstinspires.ftc.teamcode.hivequeen.game.JamDetector;

import org.firstinspires.ftc.teamcode.hivequeen.game.Alliance;
import org.firstinspires.ftc.teamcode.hivequeen.game.AutoPlanner;
import org.firstinspires.ftc.teamcode.hivequeen.game.BallClassifier;
import org.firstinspires.ftc.teamcode.hivequeen.game.BallCounter;
import org.firstinspires.ftc.teamcode.hivequeen.game.BallType;
import org.firstinspires.ftc.teamcode.hivequeen.game.BurstPolicy;
import org.firstinspires.ftc.teamcode.hivequeen.game.FlywheelWatchdog;
import org.firstinspires.ftc.teamcode.hivequeen.game.HiveTracker;
import org.firstinspires.ftc.teamcode.hivequeen.game.IntakeGuard;
import org.firstinspires.ftc.teamcode.hivequeen.game.MatchClock;
import org.firstinspires.ftc.teamcode.hivequeen.game.ServoLimits;
import org.firstinspires.ftc.teamcode.hivequeen.game.ThroatGuard;
import org.firstinspires.ftc.teamcode.hivequeen.subsystems.HeadingLock;
import org.firstinspires.ftc.teamcode.hivequeen.subsystems.ShotTable;
import org.firstinspires.ftc.teamcode.hivequeen.subsystems.TargetObservation;
import org.firstinspires.ftc.teamcode.hivequeen.util.EdgeDetector;
import org.firstinspires.ftc.teamcode.hivequeen.util.MathUtil;
import org.firstinspires.ftc.teamcode.hivequeen.util.Pose;

/**
 * Offline logic self-test for the pure (no FTC SDK / Android) HIVE-QUEEN classes.
 * No JUnit and no robot needed -- plain javac + java (JDK 8+), fully offline:
 *
 *   cd C:/Users/danu0/Downloads/26-27-BIOBUZZ
 *   J="C:/Program Files/Android/Android Studio/jbr/bin"
 *   M=TeamCode/src/main/java/org/firstinspires/ftc/teamcode/hivequeen
 *   "$J/javac" -d TeamCode/build/hq-selftest $M/RobotConstants.java $M/game/*.java \
 *       $M/util/MathUtil.java $M/util/Pose.java $M/util/EdgeDetector.java \
 *       $M/subsystems/ShotTable.java $M/subsystems/HeadingLock.java $M/subsystems/TargetObservation.java \
 *       TeamCode/src/test/java/org/firstinspires/ftc/teamcode/hivequeen/HiveQueenLogicSelfTest.java
 *   "$J/java" -cp TeamCode/build/hq-selftest org.firstinspires.ftc.teamcode.hivequeen.HiveQueenLogicSelfTest
 *
 * Exit code 0 = all checks passed, 1 = at least one failure (each failure is printed).
 */
public final class HiveQueenLogicSelfTest {
    private static int pass = 0, fail = 0;
    private static String group = "";

    private static void check(boolean ok, String what) {
        if (ok) pass++;
        else { fail++; System.out.println("  FAIL [" + group + "] " + what); }
    }

    private static void near(double actual, double expected, double tol, String what) {
        check(Math.abs(actual - expected) <= tol, what + " (got " + actual + ", want " + expected + " +/- " + tol + ")");
    }

    private static long s(double sec) { return (long) (sec * 1e9); }

    public static void main(String[] args) {
        matchClock();
        intakeGuard();
        shotTable();
        ballClassifier();
        hiveTracker();
        autoPlanner();
        allianceGeometry();
        headingWrap();
        ballCounter();
        colourGate();
        throatGuard();
        servoLimits();
        flywheelWatchdog();
        burstPolicy();
        cameraOffset();
        // ---- JamDetector (v12.2 readiness) ----
        {
            double free = 1150.0 / 60.0 * 145.1;
            JamDetector j = new JamDetector(free);
            long t0 = 0; double out = 0;
            for (long ms = 0; ms <= 2000; ms += 20) out = j.update(ms, 0.9, 0.9 * free * 0.8);
            check(j.getState() == JamDetector.State.OK && out == 0.9, "jam: normal running is never overridden");
            long ms = 2000; out = 0.9;
            for (; ms <= 2600 && out > 0; ms += 20) out = j.update(ms, 0.9, 0.0);
            check(j.getState() == JamDetector.State.UNJAM && out < 0, "jam: stalled 350 ms -> reverse pulse");
            int unjams = 1; boolean cooled = false; JamDetector.State prev = j.getState();
            for (long e = ms + 6000; ms < e && !cooled; ms += 20) {
                out = j.update(ms, 0.9, 0.0);
                if (j.getState() == JamDetector.State.UNJAM && prev != JamDetector.State.UNJAM) unjams++;
                if (j.getState() == JamDetector.State.COOLDOWN) cooled = (out == 0.0);
                prev = j.getState();
            }
            check(cooled && unjams == 2, "jam: 3rd jam inside 5 s -> cooldown with power 0 (after 2 reverse pulses)");
            JamDetector n = new JamDetector(free);
            for (long e = 0; e <= 3500; e += 20) out = n.update(e, 0.9, 0.0);
            check(n.getState() == JamDetector.State.NO_ENCODER && out == 0.9, "jam: encoder never moves -> detection disabled, command passes");
            JamDetector g = new JamDetector(free);
            g.update(0, 0.9, 500); out = 0.9;
            for (long e = 20; e <= 280; e += 20) out = g.update(e, 0.9, 0.0);
            check(out == 0.9, "jam: 300 ms spin-up grace after a command starts");
        }
        // ---- sensor fault handling ----
        check(!BallClassifier.present(0.0, 80) && !BallClassifier.present(Double.NaN, 80) && BallClassifier.present(40, 80)
                && !BallClassifier.present(120, 80) && !BallClassifier.validMm(0.0), "dead distance sensor (0 mm / NaN) is a fault, never 'ball present'");
        // ---- flywheel spin-up slew ----
        check(Math.abs(MathUtil.slewUp(0.0, 1.0, 100, 2.5) - 0.25) < 1e-9 && MathUtil.slewUp(0.8, 0.2, 100, 2.5) == 0.2,
                "slew: rise limited to 2.5/s, fall unlimited");

        System.out.println(String.format("HiveQueenLogicSelfTest: %d passed, %d failed", pass, fail));
        System.exit(fail == 0 ? 0 : 1);
    }

    // ------------------------------------------------------------------------------------------
    static void matchClock() {
        group = "MatchClock";
        final long base = s(1000);
        MatchClock.clearAutoAnchor();
        MatchClock auto = MatchClock.forAutonomous();
        check(auto.phase(base) == MatchClock.Phase.PRE_START, "auto INIT = PRE_START");
        check(!auto.poweredMotionAllowed(base), "no motion during INIT (G403 note)");
        auto.markAutoStart(base);
        check(auto.phase(base + s(0.01)) == MatchClock.Phase.AUTO, "AUTO right after START");
        check(auto.poweredMotionAllowed(base + s(29.9)), "motion at 29.9 s");
        check(!auto.poweredMotionAllowed(base + s(30.0)), "no motion at 30.0 s (transition)");
        check(auto.phase(base + s(35)) == MatchClock.Phase.TRANSITION, "auto opmode in transition at 35 s");
        check(auto.phase(base + s(40)) == MatchClock.Phase.POST_MATCH, "auto opmode never re-enables after 38 s");
        check(!auto.poweredMotionAllowed(base + s(60)), "auto opmode locked at 60 s");

        // TeleOp START pressed DURING the transition (33 s): stay locked until 38 s.
        MatchClock tele = MatchClock.forTeleOp();
        check(MatchClock.hasRecentAutoAnchor(base + s(33)), "recent AUTO anchor visible to TeleOp");
        tele.startTeleOp(base + s(33));
        check(tele.usedAutoAnchor(), "TeleOp used the AUTO anchor");
        check(tele.phase(base + s(35)) == MatchClock.Phase.TRANSITION, "early START -> still TRANSITION");
        check(!tele.poweredMotionAllowed(base + s(37.9)), "no motion at 37.9 s (G403)");
        check(tele.poweredMotionAllowed(base + s(38.1)), "motion at 38.1 s (TELEOP)");
        check(tele.phase(base + s(99)) == MatchClock.Phase.ENDGAME, "ENDGAME in the last 60 s");
        check(tele.poweredMotionAllowed(base + s(157.9)), "motion at 157.9 s");
        check(!tele.poweredMotionAllowed(base + s(158.0)), "no motion after TELEOP end (G404)");
        near(tele.matchRemainingS(base + s(98)), 60.0, 1e-6, "60 s remaining at 98 s");

        // TeleOp START pressed LATE (45 s): TELEOP is counted from the press, not cut short.
        MatchClock late = MatchClock.forTeleOp();
        late.startTeleOp(base + s(45));
        check(late.poweredMotionAllowed(base + s(45.01)), "late START -> motion immediately");
        check(late.poweredMotionAllowed(base + s(45 + 119.9)), "late START still gets the full 2:00");
        check(!late.poweredMotionAllowed(base + s(45 + 120.01)), "late START locks 2:00 after the press");

        // No AUTO at all.
        MatchClock.clearAutoAnchor();
        check(!MatchClock.hasRecentAutoAnchor(base + s(50)), "anchor cleared");
        MatchClock solo = MatchClock.forTeleOp();
        solo.startTeleOp(base + s(500));
        check(!solo.usedAutoAnchor(), "no anchor -> START = start of TELEOP");
        check(solo.phase(base + s(500.1)) == MatchClock.Phase.TELEOP, "TELEOP right after START");
        check(solo.phase(base + s(561)) == MatchClock.Phase.ENDGAME, "ENDGAME after 60 s");
        check(!solo.poweredMotionAllowed(base + s(620.01)), "locked after 120 s");

        // A stale anchor (practice AUTO long ago) is ignored.
        MatchClock a2 = MatchClock.forAutonomous();
        a2.markAutoStart(base);
        MatchClock stale = MatchClock.forTeleOp();
        stale.startTeleOp(base + s(200));
        check(!stale.usedAutoAnchor(), "AUTO anchor older than 98 s ignored");
        MatchClock.clearAutoAnchor();
    }

    // ------------------------------------------------------------------------------------------
    static void intakeGuard() {
        group = "IntakeGuard";
        long REV = RobotConstants.REJECT_REVERSE_MS, HOLD = RobotConstants.REJECT_HOLD_MS;
        IntakeGuard g = new IntakeGuard(Alliance.RED);
        IntakeGuard.Command c = g.update(0, 3, BallType.NONE, 1.0);
        check(!c.lipGateBlocked && c.intakePower == 1.0 && c.reason == IntakeGuard.Reason.NONE, "3 balls: intake allowed");
        c = g.update(10, 3, BallType.POLLEN, 1.0);
        check(!c.lipGateBlocked && c.intakePower > 0, "POLLEN at lip accepted");
        c = g.update(20, 3, BallType.RED_NECTAR, 1.0);
        check(!c.lipGateBlocked && c.intakePower > 0, "own (red) NECTAR accepted");
        // 4th ball counted while intaking -> reverse pulse with the gate still OPEN, then gate down.
        c = g.update(100, 4, BallType.NONE, 1.0);
        check(c.reason == IntakeGuard.Reason.MAGAZINE_FULL && c.intakePower < 0, "4th ball: reverse pulse");
        check(!c.lipGateBlocked, "4th ball: gate stays OPEN during the reverse pulse (no pinch)");
        c = g.update(100 + REV + 10, 4, BallType.POLLEN, 1.0);
        check(c.lipGateBlocked && c.intakePower == 0.0, "full: gate down + forward intake refused after pulse (G407)");
        c = g.update(100 + REV + 20, 3, BallType.NONE, 1.0);
        check(c.lipGateBlocked && c.intakePower <= 0, "pulse expelled our 4th (count 3): post-pulse hold, no fast oscillation");
        c = g.update(2000, 4, BallType.NONE, -0.5);
        check(c.intakePower == -0.5 && c.lipGateBlocked, "full: driver outtake still allowed");
        boolean everForward = false;
        for (BallType b : BallType.values()) {
            for (int n = 4; n <= 6; n++) if (g.update(3000 + n, n, b, 1.0).intakePower > 0) everForward = true;
        }
        check(!everForward, "never forward intake with >= 4 balls, any lip ball");
        IntakeGuard f2 = new IntakeGuard(Alliance.RED);
        f2.update(0, 4, BallType.NONE, 0.0);
        c = f2.update(500, 4, BallType.UNKNOWN, 0.0);
        check(c.lipGateBlocked && c.reason == IntakeGuard.Reason.MAGAZINE_FULL && c.intakePower == 0.0,
                "full: own 4th ball (UNKNOWN in slot 4) never triggers a colour reject");

        IntakeGuard pre = new IntakeGuard(Alliance.RED);
        c = pre.update(0, 4, BallType.NONE, 0.0);
        check(c.lipGateBlocked && c.intakePower == 0.0, "preloaded 4 with no intake request: no pulse, gate down");

        // G408 opponent NECTAR: gate OPEN while reversing; drops only once the lip reads empty.
        IntakeGuard r = new IntakeGuard(Alliance.RED);
        c = r.update(0, 1, BallType.BLUE_NECTAR, 1.0);
        check(!c.lipGateBlocked && c.intakePower < 0 && c.reason == IntakeGuard.Reason.OPPONENT_NECTAR,
                "RED: blue NECTAR -> reverse with the lip gate OPEN (G408, no pinch)");
        c = r.update(REV + 20, 1, BallType.BLUE_NECTAR, 1.0);
        check(!c.lipGateBlocked && c.intakePower < 0, "ball still at the lip after the pulse: keep pushing, gate open");
        c = r.update(RobotConstants.REJECT_MAX_REVERSE_MS + 50, 1, BallType.BLUE_NECTAR, 1.0);
        check(!c.lipGateBlocked && c.intakePower == 0.0, "still there after max reverse: wait, gate open, roller never pulls");
        long tEmpty = RobotConstants.REJECT_MAX_REVERSE_MS + 100;
        c = r.update(tEmpty, 1, BallType.NONE, 1.0);
        check(c.lipGateBlocked && c.intakePower <= 0, "lip empty -> gate drops, hold");
        c = r.update(tEmpty + HOLD - 10, 1, BallType.NONE, 1.0);
        check(c.lipGateBlocked && c.intakePower <= 0, "gate hold lasts REJECT_HOLD_MS after the lip emptied");
        c = r.update(tEmpty + HOLD + 10, 1, BallType.NONE, 1.0);
        check(!c.lipGateBlocked && c.intakePower == 1.0 && c.reason == IntakeGuard.Reason.NONE, "hold expires -> normal");
        IntakeGuard q = new IntakeGuard(Alliance.RED);
        q.update(0, 1, BallType.BLUE_NECTAR, 1.0);
        c = q.update(REV + 5, 1, BallType.NONE, 1.0);
        check(c.lipGateBlocked && c.intakePower <= 0, "ball gone right after the pulse -> gate drops immediately");
        boolean gateEverDownWithBall = false;
        IntakeGuard w = new IntakeGuard(Alliance.RED);
        for (long ms = 0; ms <= 5000; ms += 20) {
            IntakeGuard.Command k = w.update(ms, 2, BallType.BLUE_NECTAR, 1.0);
            if (k.lipGateBlocked || k.intakePower > 0) gateEverDownWithBall = true;
        }
        check(!gateEverDownWithBall, "opponent NECTAR sitting at the lip: gate never closes on it, never pulled in");

        IntakeGuard b = new IntakeGuard(Alliance.BLUE);
        check(b.update(0, 0, BallType.RED_NECTAR, 1.0).reason == IntakeGuard.Reason.OPPONENT_NECTAR, "BLUE: red NECTAR rejected");
        IntakeGuard b2 = new IntakeGuard(Alliance.BLUE);
        check(b2.update(0, 0, BallType.BLUE_NECTAR, 1.0).intakePower > 0, "BLUE: own NECTAR accepted");
    }

    // ------------------------------------------------------------------------------------------
    static void shotTable() {
        group = "ShotTable";
        ShotTable.Setpoint lo = ShotTable.lookup(BallType.POLLEN, 750);
        ShotTable.Setpoint hi = ShotTable.lookup(BallType.POLLEN, 1750);
        ShotTable.Setpoint below = ShotTable.lookup(BallType.POLLEN, 200);
        ShotTable.Setpoint above = ShotTable.lookup(BallType.POLLEN, 9000);
        check(below.frontRpm == lo.frontRpm && below.hood == lo.hood, "clamped below 0.75 m");
        check(above.frontRpm == hi.frontRpm && above.hood == hi.hood, "clamped above 1.75 m");
        ShotTable.Setpoint nan = ShotTable.lookup(BallType.POLLEN, Double.NaN);
        ShotTable.Setpoint def = ShotTable.lookup(BallType.POLLEN, ShotTable.DEFAULT_RANGE_MM);
        check(nan.frontRpm == def.frontRpm, "NaN range -> default range");
        ShotTable.Setpoint a = ShotTable.lookup(BallType.POLLEN, 750), m = ShotTable.lookup(BallType.POLLEN, 875),
                z = ShotTable.lookup(BallType.POLLEN, 1000);
        near(m.frontRpm, 0.5 * (a.frontRpm + z.frontRpm), 1e-9, "linear interpolation midpoint (rpm)");
        near(m.hood, 0.5 * (a.hood + z.hood), 1e-9, "linear interpolation midpoint (hood)");
        double prevRpm = -1, prevHood = 2;
        boolean mono = true, capOk = true, nectarMore = true, equalWheels = true;
        for (double d = 600; d <= 1900; d += 25) {
            ShotTable.Setpoint p = ShotTable.lookup(BallType.POLLEN, d);
            ShotTable.Setpoint n = ShotTable.lookup(BallType.BLUE_NECTAR, d);
            if (p.frontRpm < prevRpm - 1e-9 || p.hood > prevHood + 1e-9) mono = false;
            prevRpm = p.frontRpm; prevHood = p.hood;
            if (p.frontRpm > RobotConstants.FLYWHEEL_MAX_SETPOINT_RPM || n.rearRpm > RobotConstants.FLYWHEEL_MAX_SETPOINT_RPM)
                capOk = false;
            if (n.frontRpm < p.frontRpm) nectarMore = false;
            if (p.frontRpm != p.rearRpm) equalWheels = false;
            if (p.hood < RobotConstants.HOOD_MIN_POS || p.hood > RobotConstants.HOOD_MAX_POS) capOk = false;
        }
        check(mono, "rpm non-decreasing and hood (elevation) non-increasing with distance");
        check(capOk, "never above FLYWHEEL_MAX_SETPOINT_RPM, hood within stops");
        check(RobotConstants.FLYWHEEL_MAX_SETPOINT_RPM <= 1100, "RPM cap not raised above 1100");
        check(nectarMore, "NECTAR setpoint >= POLLEN setpoint");
        check(equalWheels, "M5/M6 equal motor RPM (2:1 overdrive gives the topspin)");
        near(ShotTable.surfaceSpeedMps(1150, RobotConstants.FRONT_WHEEL_DIAMETER_MM, RobotConstants.FRONT_WHEEL_RATIO),
                5.78, 0.01, "front 96 mm @1:1, 1150 rpm = 5.78 m/s");
        near(ShotTable.surfaceSpeedMps(1150, RobotConstants.REAR_WHEEL_DIAMETER_MM, RobotConstants.REAR_WHEEL_RATIO),
                8.67, 0.01, "counter-wheel 72 mm @2:1, 1150 rpm = 8.67 m/s");
        double rpm = ShotTable.rpmForExitSpeed(5.05);
        double vExit = ShotTable.TRANSFER_EFFICIENCY * 0.5 * (
                ShotTable.surfaceSpeedMps(rpm, 96, 1) + ShotTable.surfaceSpeedMps(rpm, 72, 2));
        near(vExit, 5.05, 1e-9, "rpmForExitSpeed inverts the wheel-speed model");
        check(ShotTable.outOfRange(749) && !ShotTable.outOfRange(750) && !ShotTable.outOfRange(1250)
                && ShotTable.outOfRange(1251) && ShotTable.outOfRange(Double.NaN), "OUT-OF-RANGE band = reliable window 0.75-1.25 m");
        // Hood clamp: every table elevation inside the 62-72 deg mechanical range.
        boolean elevOk = true;
        for (double d = 0; d <= 2500; d += 10) {
            double e = ShotTable.lookup(BallType.POLLEN, d).elevationDeg;
            if (!(e >= RobotConstants.HOOD_MIN_DEG - 1e-9 && e <= RobotConstants.HOOD_MAX_DEG + 1e-9)) elevOk = false;
        }
        check(elevOk, "shot-table elevation always within the hood range 62-72 deg");
        near(ShotTable.lookup(BallType.POLLEN, 1500).elevationDeg, 62.0, 1e-9, "long shots clamped to the 62 deg hood minimum");
        check(ShotTable.hoodPositionForDeg(50) == ShotTable.hoodPositionForDeg(62)
                && ShotTable.hoodPositionForDeg(85) == ShotTable.hoodPositionForDeg(72), "hood position clamps elevation to 62-72");
        near(ShotTable.hoodPositionForDeg(62), RobotConstants.HOOD_CAL_LOW_POS, 1e-12, "62 deg -> low calibration point");
        near(ShotTable.clampElevationDeg(57), 62, 1e-12, "57 deg (old table) -> 62");
        // window edge: 1.25 m at 62 deg needs ~the 1050 RPM cap (drag-model row)
        near(ShotTable.lookup(BallType.POLLEN, 1250).frontRpm, RobotConstants.FLYWHEEL_MAX_SETPOINT_RPM, 1.0,
                "1.25 m (window edge) runs at the RPM cap");
        check(ShotTable.lookup(BallType.POLLEN, 1200).frontRpm < RobotConstants.FLYWHEEL_MAX_SETPOINT_RPM,
                "inside the window POLLEN is below the cap");
        near(ShotTable.slantToHorizontalMm(1000, 600), 800, 1e-9, "slant 1000 / dz 600 -> 800 horizontal");
        check(ShotTable.slantToHorizontalMm(500, 600) == 0.0, "slant shorter than dz -> 0");
        // Auto launch pose is inside the reliable window D = 0.75-1.25 m and 1.3-1.8 m from the pivot line.
        for (Alliance al : Alliance.values()) {
            HiveTracker.Side side = al.initialUpSide();
            Pose shot = al == Alliance.RED
                    ? new Pose(RobotConstants.AUTO_SHOT_X_MM, RobotConstants.AUTO_SHOT_Y_MM, 0)
                    : new Pose(RobotConstants.AUTO_SHOT_X_MM, RobotConstants.AUTO_SHOT_Y_MM, 0).rotated180();
            Pose aimed = shot.withHeading(shot.bearingTo(al.cellAimPoint(side)));
            double d = ShotTable.launchDistanceMm(aimed, al.cellLipPoint(side));
            check(d >= 750 && d <= 1250, al + " auto shot D = " + Math.round(d) + " mm within 0.75-1.25 m");
            double fromPivot = Math.abs(aimed.xMm);
            check(fromPivot >= 1300 && fromPivot <= 1800, al + " auto shot robot centre 1.3-1.8 m from pivot line");
            check(Math.abs(aimed.xMm) + RobotConstants.ROBOT_HALF_LENGTH_MM < RobotConstants.FIELD_HALF_MM,
                    al + " auto shot pose inside the perimeter");
        }
    }

    // ------------------------------------------------------------------------------------------
    static void ballClassifier() {
        group = "BallClassifier";
        double P = RobotConstants.LIP_BALL_PRESENT_MM;
        check(BallClassifier.classify(1f, 0.9f, 0.1f, 20, P) == BallType.POLLEN, "yellow -> POLLEN");
        check(BallClassifier.classify(1f, 0.05f, 0.05f, 20, P) == BallType.RED_NECTAR, "red -> RED_NECTAR");
        check(BallClassifier.classify(1f, 0.0f, 0.2f, 20, P) == BallType.RED_NECTAR, "magenta-red (hue 348) -> RED_NECTAR");
        check(BallClassifier.classify(0.1f, 0.3f, 1f, 20, P) == BallType.BLUE_NECTAR, "blue -> BLUE_NECTAR");
        check(BallClassifier.classify(0.5f, 0.5f, 0.5f, 20, P) == BallType.UNKNOWN, "grey (no saturation) -> UNKNOWN");
        check(BallClassifier.classify(0.1f, 1f, 0.1f, 20, P) == BallType.UNKNOWN, "green -> UNKNOWN");
        check(BallClassifier.classify(1f, 0.9f, 0.1f, P + 1, P) == BallType.NONE, "too far -> NONE");
        check(BallClassifier.classify(1f, 0.9f, 0.1f, Double.NaN, P) == BallType.NONE, "no distance -> NONE");
        check(BallClassifier.classify(1f, 0.9f, 0.1f, 1.0, P) == BallType.NONE, "< 2 mm (sensor FAULT) -> NONE");
        near(BallClassifier.rgbToHsv(1f, 1f, 0f)[0], 60, 1e-4, "hue(yellow) = 60");
        near(BallClassifier.rgbToHsv(0f, 0f, 1f)[0], 240, 1e-4, "hue(blue) = 240");
        check(BallClassifier.forShot(BallType.UNKNOWN, true).isNectar(), "ambiguous large ball -> NECTAR setpoint");
        check(BallClassifier.forShot(BallType.UNKNOWN, false) == BallType.POLLEN, "ambiguous small ball -> POLLEN");
        check(BallClassifier.forShot(BallType.BLUE_NECTAR, false) == BallType.BLUE_NECTAR, "colour wins when known");
        check(BallClassifier.forTracker(BallType.UNKNOWN) == BallType.POLLEN && BallClassifier.forTracker(BallType.NONE) == BallType.POLLEN
                && BallClassifier.forTracker(BallType.RED_NECTAR) == BallType.RED_NECTAR,
                "HIVE tracker: unknown (even large) counts as POLLEN mass");
        check(BallType.RED_NECTAR.isNectar() && BallType.BLUE_NECTAR.isNectar() && !BallType.POLLEN.isNectar()
                && !BallType.NONE.isBall() && BallType.UNKNOWN.isBall(), "BallType predicates");
    }

    // ------------------------------------------------------------------------------------------
    static void hiveTracker() {
        group = "HiveTracker";
        HiveTracker t = new HiveTracker(Alliance.RED.initialUpSide());
        near(t.getCellMassG(), 123.84, 0.01, "3 staged NECTAR = 123.84 g");
        for (int i = 0; i < 3; i++) check(!t.recordLaunch(BallType.POLLEN), "preload " + (i + 1) + " no tip yet");
        near(t.getCellMassG(), 198.69, 0.01, "3N + 3P = 198.69 g");
        check(t.isTipUncertain(), "198.69 g is in the undefined 174-199 g band (tip #1 uncertain)");
        check(t.recordLaunch(BallType.POLLEN), "4th POLLEN (223.6 g) tips the HIVE");
        check(t.getTips() == 1 && t.getUpSide() == HiveTracker.Side.FAR, "after tip: FAR CELL up");
        check(t.getLastSpillSide() == HiveTracker.Side.AUDIENCE && t.getLastSpillBalls() == 7, "7 balls spill on the AUDIENCE side");
        check(t.getCellMassG() == 0.0 && !t.isTipUncertain(), "new up CELL empty");
        for (int i = 0; i < 4; i++) check(!t.recordLaunch(BallType.RED_NECTAR), "4 NECTAR (165 g) do not tip an empty CELL");
        check(t.recordLaunch(BallType.RED_NECTAR), "5th NECTAR (206.4 g) tips");
        check(t.getUpSide() == HiveTracker.Side.AUDIENCE && t.getTips() == 2, "tip #2 -> AUDIENCE up again");
        near(HiveTracker.massOf(BallType.UNKNOWN), RobotConstants.POLLEN_MASS_G, 1e-9, "unknown ball counted as POLLEN mass");
        HiveTracker f = new HiveTracker(HiveTracker.Side.AUDIENCE, 0, 0);
        f.flip();
        check(f.getUpSide() == HiveTracker.Side.FAR && f.getTips() == 1, "manual flip");
    }

    // ------------------------------------------------------------------------------------------
    static void autoPlanner() {
        group = "AutoPlanner";
        double abort = RobotConstants.AUTO_PARK_ABORT_S;
        check(AutoPlanner.next(0.0, 4, true, 0, 0) == AutoPlanner.Action.SHOOT, "t=0, 4 balls -> SHOOT");
        check(AutoPlanner.next(abort, 4, true, 7, 1) == AutoPlanner.Action.PARK, "at abort time -> PARK regardless");
        check(AutoPlanner.next(abort + 3, 0, true, 0, 0) == AutoPlanner.Action.PARK, "after abort -> PARK");
        check(AutoPlanner.next(8.0, 0, true, 7, 1) == AutoPlanner.Action.COLLECT_SPILL, "spill available -> sweep");
        check(AutoPlanner.next(8.0, 0, true, 0, 1) == AutoPlanner.Action.COLLECT_LOADING_ZONE, "LZ NECTAR -> collect");
        check(AutoPlanner.next(8.0, 2, true, 0, 0) == AutoPlanner.Action.SHOOT, "partial load, no sources -> SHOOT");
        check(AutoPlanner.next(8.0, 0, true, 0, 0) == AutoPlanner.Action.PARK, "nothing to do -> PARK early");
        check(AutoPlanner.next(abort - RobotConstants.AUTO_SHOT_CYCLE_EST_S + 0.1, 4, true, 0, 0)
                == AutoPlanner.Action.PARK, "no time for another shot -> PARK");
        check(AutoPlanner.next(abort - 8.0, 1, true, 5, 0) == AutoPlanner.Action.SHOOT,
                "no time for sweep+shot but time for a shot -> SHOOT");
        check(AutoPlanner.next(5.0, 4, false, 7, 1) == AutoPlanner.Action.PARK, "shooting disabled (jam) -> PARK");
        near(AutoPlanner.stepTimeoutS(0.0, 3.5), 3.5, 1e-9, "timeout = nominal early");
        near(AutoPlanner.stepTimeoutS(abort - 1.0, 3.5), 1.0, 1e-9, "timeout clipped to the abort time");
        near(AutoPlanner.stepTimeoutS(abort + 2.0, 3.5), 0.0, 1e-9, "timeout 0 after abort");
        near(AutoPlanner.parkTimeoutS(abort), RobotConstants.AUTO_PARK_DEADLINE_S - abort, 1e-9, "park timeout to deadline");
        near(AutoPlanner.parkTimeoutS(31.0), 0.0, 1e-9, "park timeout never negative");
        check(RobotConstants.AUTO_PARK_DEADLINE_S < RobotConstants.AUTO_DURATION_S, "park deadline before AUTO end");

        // Worst-case simulation: every step runs to its full (clipped) timeout and every action
        // the planner picks takes its estimate. The robot must START parking by the abort time
        // and have the park drive cut before 30 s, whatever happens.
        int[][] scenarios = { {4, 7, 1}, {4, 0, 0}, {3, 6, 0}, {4, 7, 3} };
        for (int[] sc : scenarios) {
            double t = 0.0;
            int balls = sc[0], spill = sc[1], lz = sc[2];
            AutoPlanner.Action act;
            int guard = 0;
            while ((act = AutoPlanner.next(t, balls, true, spill, lz)) != AutoPlanner.Action.PARK && guard++ < 50) {
                switch (act) {
                    case SHOOT: t += AutoPlanner.stepTimeoutS(t, RobotConstants.AUTO_SHOT_CYCLE_EST_S); balls = 0; break;
                    case COLLECT_SPILL: t += AutoPlanner.stepTimeoutS(t, RobotConstants.AUTO_SPILL_COLLECT_EST_S);
                        balls = Math.min(4, balls + 2); spill = Math.max(0, spill - 2); break;
                    default: t += AutoPlanner.stepTimeoutS(t, RobotConstants.AUTO_LZ_COLLECT_EST_S);
                        balls = Math.min(4, balls + 1); lz--; break;
                }
            }
            check(t <= abort + 1e-9, "scenario " + java.util.Arrays.toString(sc) + ": park starts by " + abort + " s (t=" + t + ")");
            check(t + AutoPlanner.parkTimeoutS(t) <= RobotConstants.AUTO_PARK_DEADLINE_S + 1e-9, "park drive cut by deadline");
        }

        // Route via-point logic around the HIVE frame.
        check(AutoPlanner.segmentHitsHiveFrame(-1450, 650, 1450, 650), "audience->far straight line hits the HIVE frame");
        check(!AutoPlanner.segmentHitsHiveFrame(-1450, 1150, 1450, 1150), "via line along Y=1150 is clear");
        check(!AutoPlanner.segmentHitsHiveFrame(RobotConstants.AUTO_START_X_MM, RobotConstants.AUTO_START_Y_MM,
                RobotConstants.AUTO_SHOT_X_MM, RobotConstants.AUTO_SHOT_Y_MM), "start -> audience shot is clear");
        check(!AutoPlanner.segmentHitsHiveFrame(RobotConstants.AUTO_VIA_X_MM, RobotConstants.AUTO_VIA_Y_MM,
                RobotConstants.AUTO_VIA_X_MM, RobotConstants.AUTO_VIA_Y_MM), "via point itself is clear");
        // Start pose legality (G304): touching the wall, not in the LOADING ZONE.
        check(RobotConstants.AUTO_START_X_MM + RobotConstants.ROBOT_HALF_WIDTH_MM < RobotConstants.TILE_MM,
                "start pose not in the LOADING ZONE tile");
        check(RobotConstants.AUTO_START_Y_MM + RobotConstants.ROBOT_HALF_LENGTH_MM >= RobotConstants.FIELD_HALF_MM - 1,
                "start pose touches the alliance wall");
        double parkFront = RobotConstants.AUTO_PARK_Y_MM + RobotConstants.ROBOT_HALF_LENGTH_MM;
        check(parkFront > RobotConstants.FIELD_HALF_MM - RobotConstants.LOADING_ZONE_DEPTH_MM
                && parkFront < RobotConstants.FIELD_HALF_MM, "park pose is partially in the LOADING ZONE");
    }

    // ------------------------------------------------------------------------------------------
    static void allianceGeometry() {
        group = "Alliance";
        Pose r = new Pose(300, 1574, -90);
        Pose b = Alliance.BLUE.fromRed(r);
        check(b.xMm == -300 && b.yMm == -1574 && Math.abs(b.headingDeg - 90) < 1e-9, "BLUE = RED rotated 180 deg");
        check(Alliance.RED.fromRed(r) == r, "RED identity");
        Pose ra = Alliance.RED.cellAimPoint(HiveTracker.Side.AUDIENCE);
        check(ra.xMm < 0 && ra.yMm > 0, "red AUDIENCE CELL at -X, +Y (red side)");
        Pose bf = Alliance.BLUE.cellAimPoint(HiveTracker.Side.FAR);
        check(bf.xMm > 0 && bf.yMm < 0, "blue FAR CELL at +X, -Y (blue side)");
        Pose ba = Alliance.BLUE.cellAimPoint(HiveTracker.Side.AUDIENCE);
        check(ba.xMm < 0 && ba.yMm < 0, "blue AUDIENCE CELL at -X, -Y");
        check(Alliance.RED.initialUpSide() == HiveTracker.Side.AUDIENCE
                && Alliance.BLUE.initialUpSide() == HiveTracker.Side.FAR, "initial up CELLs (Fig. 10-2)");
        // cluster index <-> tag IDs (Sec. 9.9): 30-33 red far, 34-37 red audience, 38-41 blue audience, 42-45 blue far
        check(Alliance.RED.cellClusterIndex(HiveTracker.Side.FAR) == (30 - 30) / 4, "red FAR -> tags 30-33");
        check(Alliance.RED.cellClusterIndex(HiveTracker.Side.AUDIENCE) == (34 - 30) / 4, "red AUDIENCE -> tags 34-37");
        check(Alliance.BLUE.cellClusterIndex(HiveTracker.Side.AUDIENCE) == (38 - 30) / 4, "blue AUDIENCE -> tags 38-41");
        check(Alliance.BLUE.cellClusterIndex(HiveTracker.Side.FAR) == (42 - 30) / 4, "blue FAR -> tags 42-45");
        check(Alliance.RED.isOwnCellTag(33) && !Alliance.RED.isOwnCellTag(38) && Alliance.BLUE.isOwnCellTag(45), "tag ownership");
        check(Alliance.RED.opponentNectar() == BallType.BLUE_NECTAR && Alliance.BLUE.opponentNectar() == BallType.RED_NECTAR,
                "opponent NECTAR colours");
        check(Alliance.RED.driverFacingDeg() == -90 && Alliance.BLUE.driverFacingDeg() == 90, "driver facing");
        Pose lzRed = new Pose(RobotConstants.RED_LOADING_ZONE_X_MM, RobotConstants.AUTO_PARK_Y_MM, 90);
        Pose lzBlue = Alliance.BLUE.fromRed(lzRed);
        check(lzBlue.xMm < 0 && lzBlue.yMm < 0, "blue LOADING ZONE lower-right of Fig. 9-2 (audience side, blue wall)");
    }

    // ------------------------------------------------------------------------------------------
    static void headingWrap() {
        group = "HeadingWrap";
        near(MathUtil.wrapDeg(190), -170, 1e-9, "wrap 190");
        near(MathUtil.wrapDeg(-190), 170, 1e-9, "wrap -190");
        near(MathUtil.wrapDeg(180), 180, 1e-9, "wrap 180 stays 180");
        near(MathUtil.wrapDeg(-180), 180, 1e-9, "wrap -180 -> 180");
        near(MathUtil.wrapDeg(540), 180, 1e-9, "wrap 540");
        near(MathUtil.wrapDeg(720), 0, 1e-9, "wrap 720");
        near(MathUtil.wrapDeg(-725), -5, 1e-9, "wrap -725");
        near(MathUtil.wrapDeg(359.5), -0.5, 1e-9, "wrap 359.5");
        near(new Pose(0, 0, 170).rotated180().headingDeg, -10, 1e-9, "rotated180 wraps heading");
        near(new Pose(0, 0, 0).mirroredX().headingDeg, 180, 1e-9, "mirroredX: 0 -> 180");
        near(new Pose(0, 0, -30).mirroredX().headingDeg, -150, 1e-9, "mirroredX: -30 -> -150");
        near(new Pose(0, 0, 0).bearingTo(new Pose(0, 100, 0)), 90, 1e-9, "bearing +Y = 90");

        // HeadingLock across the +/-180 seam: robot at 175, target 10 deg to the left -> field 185 = -175.
        long now = 1_000_000_000L;
        HeadingLock lock = new HeadingLock();
        double turn = lock.update(new TargetObservation(10, Double.NaN, 4, now), 175, now);
        check(turn > 0, "target left -> CCW (+) turn across the seam");
        near(lock.getLastErrorDeg(), 10, 1e-9, "error 10 deg, not -350");
        double t2 = lock.update(null, -176, now + 100_000_000L);   // robot turned to -176 (= 184)
        near(lock.getLastErrorDeg(), 1, 1e-9, "held field heading via odometry, wrapped error 1 deg");
        check(t2 == 0.0 && lock.isOnTarget(), "inside tolerance -> on target, zero turn");
        double t3 = lock.update(null, -170, now + 200_000_000L);
        check(t3 < 0, "overshoot -> CW (-) correction");
        lock.reset();
        check(Double.isNaN(lock.update(null, 0, now)), "no target ever -> NaN (driver keeps control)");
        TargetObservation stale = new TargetObservation(5, 1000, 4, now);
        check(Double.isNaN(new HeadingLock().update(stale, 0, now + (RobotConstants.VISION_STALE_MS + 1) * 1_000_000L)),
                "stale observation ignored");

        group = "EdgeDetector";
        EdgeDetector e = new EdgeDetector();
        check(!e.rose(false) && e.rose(true) && !e.rose(true) && !e.rose(false) && e.rose(true), "rising edges only");
    }

    // ------------------------------------------------------------------------------------------
    /** Fresh healthy inputs at time {@code ms} (nothing seen, motors idle). */
    static BallCounter.Inputs in(long ms) {
        BallCounter.Inputs i = new BallCounter.Inputs();
        i.nowMs = ms;
        return i;
    }

    static void ballCounter() {
        group = "BallCounter";
        // ---- (a) lip entry ----
        BallCounter bc = new BallCounter();
        bc.setCount(RobotConstants.PRELOADS_IN_MAGAZINE, BallType.POLLEN);
        check(bc.getCount() == 3, "start: 3 preloads in the magazine");
        BallCounter.Inputs i = in(0); i.intakeForward = true; bc.update(i);            // prime
        i = in(20); i.intakeForward = true; i.lipBall = BallType.POLLEN; bc.update(i);
        check(bc.getCount() == 4, "lip rising edge while intaking -> +1 (4th preload)");
        i = in(40); i.intakeForward = true; i.lipBall = BallType.POLLEN; bc.update(i);
        check(bc.getCount() == 4, "ball held in front of the lip is counted once");
        i = in(60); i.intakeForward = true; bc.update(i);
        check(bc.getCount() == 4, "falling edge while intaking (ball went in) -> no change");

        BallCounter e = new BallCounter();
        e.setCount(1, BallType.POLLEN);
        e.update(in(0));
        i = in(20); i.intakeForward = true; i.lipBall = BallType.POLLEN; e.update(i);
        check(e.getCount() == 2, "entry +1");
        i = in(40); i.intakeReversing = true; i.lipBall = BallType.POLLEN; e.update(i);
        i = in(60); i.intakeReversing = true; e.update(i);
        check(e.getCount() == 1, "reverse/unjam pulse expels the counted ball back out past the lip -> -1");
        // rejected ball never counted, and its expulsion does not decrement
        i = in(80); i.intakeForward = true; i.rejecting = true; i.lipBall = BallType.BLUE_NECTAR; e.update(i);
        check(e.getCount() == 1, "rejected ball at the lip is not counted");
        i = in(100); i.intakeReversing = true; i.rejecting = true; i.lipBall = BallType.BLUE_NECTAR; e.update(i);
        i = in(120); i.intakeReversing = true; i.rejecting = true; e.update(i);
        check(e.getCount() == 1, "rejected ball pushed back out: no -1");
        // ball seen while idle (not driven) is not counted
        i = in(140); i.lipBall = BallType.POLLEN; e.update(i);
        i = in(160); e.update(i);
        check(e.getCount() == 1, "ball rolling past the lip while the intake is off: not counted");
        // late accept: UNKNOWN at the rising edge (rejected), then pulled in once classified
        i = in(180); i.intakeForward = true; i.rejecting = true; i.lipBall = BallType.UNKNOWN; e.update(i);
        i = in(200); i.intakeForward = true; i.lipBall = BallType.POLLEN; e.update(i);
        i = in(220); i.intakeForward = true; e.update(i);
        check(e.getCount() == 2, "not counted at the rising edge but pulled in -> counted on the falling edge");
        // driver outtake: a ball coming back out from inside crosses the lip outward -> -1
        i = in(240); i.intakeReversing = true; i.magazineReversing = true; i.lipBall = BallType.POLLEN; e.update(i);
        i = in(260); i.intakeReversing = true; i.magazineReversing = true; e.update(i);
        check(e.getCount() == 1, "outtake: ball leaving outward past the lip -> -1");
        // lip FAULT: no edges at all
        BallCounter lf = new BallCounter();
        lf.setCount(2, BallType.POLLEN);
        lf.update(in(0));
        for (long ms = 20; ms <= 200; ms += 20) {
            i = in(ms); i.intakeForward = true; i.lipFault = true; i.lipBall = (ms / 20) % 2 == 0 ? BallType.POLLEN : BallType.NONE;
            lf.update(i);
        }
        check(lf.getCount() == 2, "lip sensor FAULT: no entry edges counted");

        // ---- (b) throat exit ----
        BallCounter t = new BallCounter();
        t.setCount(3, BallType.POLLEN);
        i = in(0); i.throatBall = BallType.POLLEN; t.update(i);
        i = in(20); i.throatBall = BallType.POLLEN; i.feeding = true; t.update(i);
        i = in(40); i.feeding = true; t.update(i);
        check(t.getCount() == 2 && t.getShots() == 1, "throat falling edge while feeding -> -1 (shot)");
        check(t.getLastShotBall() == BallType.POLLEN && t.getLastShotMs() == 40, "shot class + time recorded");
        i = in(60); i.throatBall = BallType.POLLEN; i.feeding = true; t.update(i);
        i = in(80); i.feeding = true; i.magazineReversing = true; t.update(i);
        check(t.getCount() == 2 && t.getShots() == 1, "throat falling edge while feeding AND reversing (unjam) -> no shot");
        i = in(100); i.throatBall = BallType.POLLEN; i.feeding = true; t.update(i);
        i = in(120); i.feeding = true; t.update(i);
        check(t.getCount() == 1 && t.getShots() == 2, "after the unjam the same ball is counted once when it leaves");
        i = in(140); i.throatBall = BallType.POLLEN; t.update(i);
        i = in(160); t.update(i);
        check(t.getCount() == 1, "throat falling edge while NOT feeding (roll-back) -> no change");
        BallCounter u = new BallCounter();
        u.setCount(1, BallType.UNKNOWN);
        i = in(0); i.throatBall = BallType.UNKNOWN; u.update(i);
        i = in(20); i.feeding = true; u.update(i);
        check(u.getLastShotBall() == BallType.POLLEN, "UNKNOWN launched -> POLLEN mass for the HIVE tracker");

        // ---- roll-back not double counted ----
        BallCounter rb = new BallCounter();
        rb.setCount(3, BallType.POLLEN);
        rb.update(in(0));
        for (long ms = 20; ms <= 2000; ms += 20) {        // magazine stopped: balls roll back and forth
            i = in(ms);
            boolean on = (ms / 60) % 2 == 0;
            i.throatBall = on ? BallType.POLLEN : BallType.NONE;
            i.count1Present = !on;
            i.count2Present = on;
            rb.update(i);
        }
        check(rb.getCount() == 3, "balls rolling back across throat/ballCount1/ballCount2 never change the count");

        // ---- (c) occupancy lower bound ----
        check(BallCounter.occupancyBound(true, false, true, false, true, false) == 3, "throat+c1+c2 -> >= 3");
        check(BallCounter.occupancyBound(true, false, true, false, false, false) == 2, "throat+c1 -> >= 2");
        check(BallCounter.occupancyBound(false, false, true, false, true, false) == 0, "no throat ball -> no bound (chain)");
        check(BallCounter.occupancyBound(true, false, true, true, true, false) == 1, "ballCount1 FAULT ends the chain at 1");
        BallCounter oc = new BallCounter();
        oc.setCount(0, BallType.POLLEN);
        for (long ms = 0; ms <= 180; ms += 20) {
            i = in(ms); i.holding = true; i.throatBall = BallType.POLLEN; i.count1Present = true; i.count2Present = true;
            oc.update(i);
        }
        check(oc.getCount() == 0, "occupancy not applied before OCCUPANCY_STABLE_MS of steady hold");
        i = in(220); i.holding = true; i.throatBall = BallType.POLLEN; i.count1Present = true; i.count2Present = true;
        oc.update(i);
        check(oc.getCount() == 3, "steady hold 200 ms: throat+c1+c2 -> count raised to 3");
        BallCounter nh = new BallCounter();
        for (long ms = 0; ms <= 1000; ms += 20) {
            i = in(ms); i.throatBall = BallType.POLLEN; i.count1Present = true; nh.update(i);   // NOT holding
        }
        check(nh.getCount() == 0, "occupancy ignored while the magazine is not holding the queue");
        BallCounter nd = new BallCounter();
        nd.setCount(4, BallType.POLLEN);
        for (long ms = 0; ms <= 1000; ms += 20) {
            i = in(ms); i.holding = true; i.throatBall = BallType.POLLEN; nd.update(i);          // bound 1
        }
        check(nd.getCount() == 4, "occupancy never DECREASES the count (lower bound only)");
        BallCounter fl = new BallCounter();
        for (long ms = 0; ms <= 1000; ms += 20) {
            i = in(ms); i.holding = true; i.throatFault = true; i.count1Present = true; i.count2Present = true; fl.update(i);
        }
        check(fl.getCount() == 0, "throat FAULT: no occupancy bound from the other sensors");
        BallCounter fl2 = new BallCounter();
        for (long ms = 0; ms <= 1000; ms += 20) {
            i = in(ms); i.holding = true; i.throatBall = BallType.POLLEN; i.count1Fault = true; i.count2Present = true; fl2.update(i);
        }
        check(fl2.getCount() == 1, "ballCount1 FAULT: bound stops at 1");
        BallCounter fl3 = new BallCounter();
        for (long ms = 0; ms <= 1000; ms += 20) {
            i = in(ms); i.holding = true; i.throatBall = BallType.POLLEN; i.count1Present = true; i.count2Fault = true; fl3.update(i);
        }
        check(fl3.getCount() == 2, "ballCount2 FAULT: bound stops at 2");
        BallCounter fd = new BallCounter();
        for (long ms = 0; ms <= 1000; ms += 20) {
            i = in(ms); i.holding = true; i.throatBall = BallType.POLLEN; i.count1Present = true; i.count2Present = true;
            i.feeding = true; fd.update(i);
        }
        check(fd.getCount() == 0, "no occupancy bound while feeding");

        // ---- (d) clamp 0..4 ----
        BallCounter cl = new BallCounter();
        cl.setCount(9, BallType.POLLEN);
        check(cl.getCount() == 4, "setCount clamps to 4");
        cl.setCount(-3, BallType.POLLEN);
        check(cl.getCount() == 0, "setCount clamps to 0");
        cl.setCount(4, BallType.POLLEN);
        cl.update(in(0));
        for (long ms = 20; ms <= 400; ms += 40) {
            i = in(ms); i.intakeForward = true; i.lipBall = BallType.POLLEN; cl.update(i);
            i = in(ms + 20); i.intakeForward = true; cl.update(i);
        }
        check(cl.getCount() == 4 && cl.getQueue().size() == 4, "entries beyond 4 are clamped (count and queue)");
        BallCounter z = new BallCounter();
        z.setCount(0, BallType.POLLEN);
        i = in(0); i.throatBall = BallType.POLLEN; z.update(i);
        i = in(20); i.feeding = true; z.update(i);
        i = in(40); i.lipBall = BallType.POLLEN; i.intakeReversing = true; z.update(i);
        i = in(60); i.intakeReversing = true; z.update(i);
        check(z.getCount() == 0, "count never below 0");

        // ---- queue classes (burst setpoint) ----
        BallCounter qc = new BallCounter();
        qc.setCount(2, BallType.POLLEN);
        qc.update(in(0));
        check(!qc.queueHasNectar(), "POLLEN-only queue");
        i = in(20); i.intakeForward = true; i.lipBall = BallType.RED_NECTAR; qc.update(i);
        i = in(40); i.intakeForward = true; qc.update(i);
        check(qc.queueHasNectar() && qc.getQueue().get(2) == BallType.RED_NECTAR, "own NECTAR recorded at the back of the queue");
    }

    // ------------------------------------------------------------------------------------------
    static void colourGate() {
        group = "ColourGate";
        IntakeGuard g = new IntakeGuard(Alliance.RED);
        check(g.isColourGateEnabled(), "colour gate ON by default");
        IntakeGuard.Command c = g.update(0, 1, BallType.UNKNOWN, 1.0);
        check(c.reason == IntakeGuard.Reason.UNKNOWN_BALL && c.intakePower < 0 && !c.lipGateBlocked,
                "UNKNOWN colour at the lip rejected like an opponent ball (G408)");
        check(g.wouldReject(BallType.UNKNOWN) && g.wouldReject(BallType.BLUE_NECTAR)
                && !g.wouldReject(BallType.POLLEN) && !g.wouldReject(BallType.RED_NECTAR), "accept only POLLEN / own NECTAR");
        IntakeGuard o = new IntakeGuard(Alliance.RED);
        o.setColourGateOverride(true);
        check(!o.isColourGateEnabled(), "driver override -> colour gate OFF");
        c = o.update(0, 1, BallType.UNKNOWN, 1.0);
        check(c.intakePower == 1.0 && !c.lipGateBlocked, "override: UNKNOWN accepted");
        c = o.update(20, 1, BallType.BLUE_NECTAR, 1.0);
        check(c.intakePower == 1.0, "override: colour gate fully off (drivers handle G408)");
        IntakeGuard f = new IntakeGuard(Alliance.RED);
        f.setLipSensorFault(true);
        check(!f.isColourGateEnabled() && !f.wouldReject(BallType.UNKNOWN), "lip sensor FAULT auto-disables the colour gate");
        c = f.update(0, 4, BallType.NONE, 1.0);
        check(c.reason == IntakeGuard.Reason.MAGAZINE_FULL, "G407 limit still enforced with the colour gate off");
        f.setLipSensorFault(false);
        check(f.isColourGateEnabled(), "fault cleared -> colour gate back ON");
        // classifier: a dead colour sensor (0 mm) is never a ball
        check(BallClassifier.classify(1f, 0.9f, 0.1f, 0.0, RobotConstants.LIP_BALL_PRESENT_MM) == BallType.NONE,
                "0 mm (FAULT) reading classifies as NONE, not a ball");
    }

    // ------------------------------------------------------------------------------------------
    static void throatGuard() {
        group = "ThroatGuard";
        ThroatGuard tg = new ThroatGuard();
        check(!tg.update(0, BallType.BLUE_NECTAR, Alliance.RED, true), "one frame of opponent colour does not block");
        check(tg.update(RobotConstants.THROAT_OPPONENT_CONFIRM_MS, BallType.BLUE_NECTAR, Alliance.RED, true),
                "opponent NECTAR at the throat (confirmed) blocks firing");
        check(tg.update(500, BallType.POLLEN, Alliance.RED, true), "stays blocked while something sits at the throat");
        check(tg.update(600, BallType.NONE, Alliance.RED, true), "still blocked right after the throat empties");
        check(!tg.update(600 + RobotConstants.THROAT_CLEAR_MS, BallType.NONE, Alliance.RED, true),
                "cleared once reversed out (throat empty THROAT_CLEAR_MS)");
        ThroatGuard own = new ThroatGuard();
        boolean blocked = false;
        for (long ms = 0; ms < 1000; ms += 20) {
            blocked |= own.update(ms, BallType.RED_NECTAR, Alliance.RED, true);
            blocked |= own.update(ms + 10, BallType.UNKNOWN, Alliance.RED, true);
        }
        check(!blocked, "own NECTAR / UNKNOWN at the throat never block firing");
        ThroatGuard ov = new ThroatGuard();
        check(!ov.update(0, BallType.BLUE_NECTAR, Alliance.RED, false) && !ov.update(500, BallType.BLUE_NECTAR, Alliance.RED, false),
                "override / throat FAULT -> no block");
        ThroatGuard bl = new ThroatGuard();
        bl.update(0, BallType.RED_NECTAR, Alliance.BLUE, true);
        check(bl.update(200, BallType.RED_NECTAR, Alliance.BLUE, true), "BLUE: red NECTAR at the throat blocks");
    }

    // ------------------------------------------------------------------------------------------
    static void servoLimits() {
        group = "ServoLimits";
        double tol = 0.5;   // deg
        check(ServoLimits.spanDeg(RobotConstants.LIP_GATE_OPEN, RobotConstants.LIP_GATE_BLOCKED)
                <= RobotConstants.LIP_GATE_SWING_DEG + tol, "S1 lip gate span <= 122 deg blocker swing");
        check(ServoLimits.spanDeg(RobotConstants.INDEXER_CLOSED, RobotConstants.INDEXER_OPEN)
                <= RobotConstants.INDEXER_TRAVEL_DEG + tol, "S3 indexer span <= 60 deg finger travel");
        check(ServoLimits.spanDeg(RobotConstants.HOOD_MIN_POS, RobotConstants.HOOD_MAX_POS)
                <= RobotConstants.HOOD_MAX_DEG - RobotConstants.HOOD_MIN_DEG + tol, "S6 hood span <= 10 deg (62-72, coaxial)");
        near(ServoLimits.spanDeg(RobotConstants.INDEXER_CLOSED, RobotConstants.INDEXER_OPEN), 60.0, 1e-6, "S3 opens the full 60 deg");
        double[] all = {RobotConstants.LIP_GATE_OPEN, RobotConstants.LIP_GATE_BLOCKED, RobotConstants.INDEXER_CLOSED,
                RobotConstants.INDEXER_OPEN, RobotConstants.HOOD_MIN_POS, RobotConstants.HOOD_MAX_POS};
        boolean inRange = true;
        for (double v : all) if (v < 0 || v > 1) inRange = false;
        check(inRange, "all end positions inside 0..1");
        check(ServoLimits.lipGate(1.0) == Math.max(RobotConstants.LIP_GATE_OPEN, RobotConstants.LIP_GATE_BLOCKED)
                && ServoLimits.lipGate(0.0) == Math.min(RobotConstants.LIP_GATE_OPEN, RobotConstants.LIP_GATE_BLOCKED),
                "lip gate command clamped to its span");
        check(ServoLimits.indexer(0.95) <= Math.max(RobotConstants.INDEXER_CLOSED, RobotConstants.INDEXER_OPEN) + 1e-12
                && ServoLimits.hood(0.0) == RobotConstants.HOOD_MIN_POS && ServoLimits.hood(1.0) == RobotConstants.HOOD_MAX_POS
                && ServoLimits.hood(Double.NaN) == RobotConstants.HOOD_MIN_POS, "indexer / hood commands clamped");
        near(ServoLimits.positionAt(0.2, 30, -1), 0.1, 1e-12, "positionAt: 30 deg = 0.1 of travel");
    }

    // ------------------------------------------------------------------------------------------
    static void flywheelWatchdog() {
        group = "FlywheelWatchdog";
        FlywheelWatchdog ok = new FlywheelWatchdog();
        for (long ms = 0; ms <= 3000; ms += 20) ok.update(ms, 0.9, Math.min(1000, ms), 1000);
        check(!ok.isFault(), "healthy encoder (speed rising) -> no fault");
        FlywheelWatchdog w = new FlywheelWatchdog();
        for (long ms = 0; ms <= 900; ms += 20) w.update(ms, 0.9, 0.0, 1000);
        check(!w.isFault(), "not yet (< 1 s)");
        for (long ms = 920; ms <= 1100; ms += 20) w.update(ms, 0.9, 0.0, 1000);
        check(w.isFault(), "power high, velocity 0 for > 1 s -> encoder FAULT");
        check(!w.fallbackAtSpeed(1100), "fallback not ready before the spin-up time");
        check(w.fallbackAtSpeed(RobotConstants.FLYWHEEL_FALLBACK_SPINUP_MS), "time-based at-speed after FLYWHEEL_FALLBACK_SPINUP_MS");
        w.update(5000, 0.0, 0.0, 0.0);
        check(!w.fallbackAtSpeed(5000), "target 0 resets the spin-up clock");
        w.update(5020, 0.5, 400.0, 1000);
        check(!w.isFault(), "encoder reports motion again -> fault clears");
        FlywheelWatchdog lo = new FlywheelWatchdog();
        for (long ms = 0; ms <= 3000; ms += 20) lo.update(ms, 0.2, 0.0, 200);
        check(!lo.isFault(), "low power (slow spin) never flags a fault");
    }

    // ------------------------------------------------------------------------------------------
    static void burstPolicy() {
        group = "BurstPolicy";
        check(BurstPolicy.burstBall(BallType.POLLEN, false) == BallType.POLLEN, "all POLLEN -> POLLEN setpoint");
        check(BurstPolicy.burstBall(BallType.POLLEN, true).isNectar(), "NECTAR anywhere in the queue -> NECTAR (max) setpoint");
        check(BurstPolicy.burstBall(BallType.BLUE_NECTAR, false) == BallType.BLUE_NECTAR, "NECTAR at the throat -> NECTAR");
        ShotTable.Setpoint p = ShotTable.lookup(BurstPolicy.burstBall(BallType.POLLEN, true), 1000);
        check(p.frontRpm >= ShotTable.lookup(BallType.POLLEN, 1000).frontRpm, "burst setpoint >= every ball's setpoint");
        long D = RobotConstants.TELEOP_LAST_BALL_DWELL_MS;
        check(BurstPolicy.lastBallDwellActive(true, 0, 1100, 1000, D), "last ball: indexer stays open inside the dwell");
        check(!BurstPolicy.lastBallDwellActive(true, 0, 1000 + D, 1000, D), "dwell ends after TELEOP_LAST_BALL_DWELL_MS");
        check(!BurstPolicy.lastBallDwellActive(true, 2, 1100, 1000, D), "no dwell while balls remain (gunner controls)");
        check(!BurstPolicy.lastBallDwellActive(false, 0, 1100, 1000, D), "dwell never re-opens a closed indexer");
        near(D / 1000.0, RobotConstants.AUTO_LAST_BALL_DWELL_S, 1e-9, "TeleOp dwell = Auto dwell");
    }

    // ------------------------------------------------------------------------------------------
    static void cameraOffset() {
        group = "CameraOffset";
        // Target dead ahead of the SHOOTER at 1.2 m horizontal: the camera, 86 mm to the right, sees it
        // slightly LEFT; converting back must give ~0 deg.
        double camX = RobotConstants.HUSKYLENS_ROBOT_X_MM, camY = RobotConstants.HUSKYLENS_ROBOT_Y_MM;
        double tx = RobotConstants.SHOOTER_EXIT_X_MM + 1200, ty = 0;
        double camYaw = Math.toDegrees(Math.atan2(ty - camY, tx - camX));
        double horiz = Math.hypot(tx - camX, ty - camY);
        check(camYaw > 3.0, "camera right of the shooter sees a straight-ahead target to the left");
        near(ShotTable.cameraToShooterYawDeg(camYaw, horiz, camX, camY), 0.0, 1e-9, "lateral term removes the camera offset");
        near(ShotTable.cameraToShooterYawDeg(10.0, 1000, RobotConstants.SHOOTER_EXIT_X_MM, 0.0), 10.0, 1e-9,
                "camera at the shooter exit: no correction");
        check(!Double.isNaN(ShotTable.cameraToShooterYawDeg(0.0, Double.NaN, camX, camY)), "NaN range -> default range");
        near(RobotConstants.CAMERA_HEIGHT_MM, 354.0, 1e-9, "HuskyLens lens height 354 mm (CAD)");
    }
}
