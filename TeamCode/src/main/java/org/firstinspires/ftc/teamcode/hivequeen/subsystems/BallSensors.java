package org.firstinspires.ftc.teamcode.hivequeen.subsystems;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.hivequeen.RobotConstants;
import org.firstinspires.ftc.teamcode.hivequeen.RobotHardware;
import org.firstinspires.ftc.teamcode.hivequeen.game.BallClassifier;
import org.firstinspires.ftc.teamcode.hivequeen.game.BallCounter;
import org.firstinspires.ftc.teamcode.hivequeen.game.BallType;

import java.util.List;

/**
 * All ball sensing (hardware side). The counting model itself is the pure, unit-tested
 * {@link BallCounter}; this class only reads the sensors, classifies, flags faults and feeds it.
 *  - lipColor (REV Color Sensor V3, lens X 167 across the mouth): ENTRY counter + G408 colour gate.
 *  - throatColor (REV Color Sensor V3 at queue slot 1, the ball held by S3): EXIT counter, slot-1
 *    occupancy, next-ball class; size via proximity (NECTAR dia 91.95 sits closer than POLLEN dia
 *    71.12) cross-checked with colour.
 *  - ballCount1 / ballCount2 (REV 2m Distance, DistanceSensor interface): slot 2 / slot 3
 *    OCCUPANCY (lower bound only, while the queue is held against the gate; see RobotConstants).
 * Every sensor reading &lt; 2 mm or NaN (or a missing distance interface) is a FAULT, never a ball.
 *
 * Colour classification (Sec. 9.8): POLLEN yellow, NECTAR red or blue -- see {@link BallClassifier}.
 *
 * Call order per loop: {@link #read} (sensors), then {@link #updateCount} (needs the intake /
 * shooter state, which in turn needs the classified lip ball).
 */
public class BallSensors {
    private final RobotHardware hw;
    private final DistanceSensor lipDist, throatDist;
    private final BallCounter counter = new BallCounter();
    private final BallCounter.Inputs in = new BallCounter.Inputs();

    private BallType lipBall = BallType.NONE;
    private BallType throatBall = BallType.NONE;
    private double lipMm = Double.NaN, throatMm = Double.NaN, c1Mm = Double.NaN, c2Mm = Double.NaN;
    private boolean lipFault = false, throatFault = false, count1Fault = false, count2Fault = false;
    private boolean count1Present = false, count2Present = false;

    /** REV 2m Distance reads can block the loop for ~20-30 ms each. The occupancy sensors are
     *  static (only used after OCCUPANCY_STABLE_MS of steady readings), so read ONE of them per
     *  loop, alternating. */
    private int loopIdx = 0;

    public BallSensors(RobotHardware hw) {
        this.hw = hw;
        // The REV Color Sensor V3 driver implements DistanceSensor as well.
        lipDist = hw.lipColor instanceof DistanceSensor ? (DistanceSensor) hw.lipColor : null;
        throatDist = hw.throatColor instanceof DistanceSensor ? (DistanceSensor) hw.throatColor : null;
    }

    /** Set the count (start of match: PRELOADS_IN_MAGAZINE POLLEN; gunner override 0 / 4). Clamped 0..4. */
    public void setCount(int n) { setCount(n, BallType.UNKNOWN); }

    public void setCount(int n, BallType fill) { counter.setCount(n, fill); }

    /** Read + classify every ball sensor and refresh the fault flags. */
    public void read(long nowMs) {
        // --- lip -------------------------------------------------------------------------
        lipMm = lipDist != null ? lipDist.getDistance(DistanceUnit.MM) : Double.NaN;
        lipFault = !BallClassifier.validMm(lipMm);
        lipBall = classify(hw.lipColor.getNormalizedColors(), lipMm, RobotConstants.LIP_BALL_PRESENT_MM);

        // --- throat ----------------------------------------------------------------------
        throatMm = throatDist != null ? throatDist.getDistance(DistanceUnit.MM) : Double.NaN;
        throatFault = !BallClassifier.validMm(throatMm);
        throatBall = classify(hw.throatColor.getNormalizedColors(), throatMm, RobotConstants.THROAT_BALL_PRESENT_MM);

        // --- occupancy (one 2m read per loop) ----------------------------------------------
        if ((loopIdx++ & 1) == 0) {
            c1Mm = hw.ballCount1.getDistance(DistanceUnit.MM);
            count1Fault = !BallClassifier.validMm(c1Mm);
            count1Present = BallClassifier.present(c1Mm, RobotConstants.COUNT_BALL_PRESENT_MM);
        } else {
            c2Mm = hw.ballCount2.getDistance(DistanceUnit.MM);
            count2Fault = !BallClassifier.validMm(c2Mm);
            count2Present = BallClassifier.present(c2Mm, RobotConstants.COUNT_BALL_PRESENT_MM);
        }
    }

    /**
     * Run the counting model for this loop.
     * @param feeding          indexer open and magazine feeding (throat exit = shot)
     * @param magReversing     magazine running backwards (outtake / unjam pulse)
     * @param intakeForward    roller actually driving in
     * @param intakeReversing  roller actually driving out (outtake / reject / unjam pulse)
     * @param rejecting        the lip ball is being rejected by the G407/G408 guard
     * @param holding          magazine at hold power with the indexer closed
     */
    public void updateCount(long nowMs, boolean feeding, boolean magReversing, boolean intakeForward,
                            boolean intakeReversing, boolean rejecting, boolean holding) {
        in.nowMs = nowMs;
        in.lipBall = lipBall;
        in.lipFault = lipFault;
        in.throatBall = throatBall;
        in.throatFault = throatFault;
        in.count1Present = count1Present;
        in.count1Fault = count1Fault;
        in.count2Present = count2Present;
        in.count2Fault = count2Fault;
        in.feeding = feeding;
        in.magazineReversing = magReversing;
        in.intakeForward = intakeForward;
        in.intakeReversing = intakeReversing;
        in.rejecting = rejecting;
        in.holding = holding;
        counter.update(in);
    }

    /** Colour + distance classifier (delegates to the pure, unit-tested {@link BallClassifier}). */
    public static BallType classify(NormalizedRGBA c, double distMm, double presentMm) {
        return BallClassifier.classify(c.red, c.green, c.blue, distMm, presentMm);
    }

    /** Size-only classification at the throat: true if the ball is the larger NECTAR. */
    public boolean throatBallIsLarge() {
        return !throatFault && throatMm <= RobotConstants.THROAT_NECTAR_MAX_MM;
    }

    public BallType getLipBall() { return lipBall; }

    /** Raw colour classification of the ball at the throat. */
    public BallType getThroatBall() { return throatBall; }

    /**
     * Ball class used to pick the shooter setpoint table (dual-ball shooter). Colour decides
     * POLLEN vs NECTAR; if colour is ambiguous, proximity decides size (only the SIZE matters to
     * the shooter, so an ambiguous large ball maps onto a NECTAR value). NOT for the HIVE model.
     */
    public BallType getThroatBallForShot() {
        return BallClassifier.forShot(throatBall, throatBallIsLarge());
    }

    /** True if any ball recorded in the magazine queue is NECTAR (burst setpoint = max). */
    public boolean queueHasNectar() { return counter.queueHasNectar(); }

    public List<BallType> getQueue() { return counter.getQueue(); }

    /** Class of the most recently LAUNCHED ball for the HIVE mass model (unknown -> POLLEN). */
    public BallType getLastShotBall() { return counter.getLastShotBall(); }

    public long getLastShotMs() { return counter.getLastShotMs(); }

    public int getCount() { return counter.getCount(); }

    public int getShotsCounted() { return counter.getShots(); }

    public boolean isLipFault() { return lipFault; }

    public boolean isThroatFault() { return throatFault; }

    public boolean isCount1Fault() { return count1Fault; }

    public boolean isCount2Fault() { return count2Fault; }

    /** Telemetry: "" when healthy, otherwise which ball sensor reads an impossible value. */
    public String getFaults() {
        return ((lipFault ? "lipColor " : "") + (throatFault ? "throatColor " : "")
                + (count1Fault ? "ballCount1 " : "") + (count2Fault ? "ballCount2 " : "")).trim();
    }

    public double getLipMm() { return lipMm; }

    public double getThroatMm() { return throatMm; }

    public double getCount1Mm() { return c1Mm; }

    public double getCount2Mm() { return c2Mm; }
}
