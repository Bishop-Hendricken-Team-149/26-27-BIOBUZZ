package org.firstinspires.ftc.teamcode.hivequeen.game;

import org.firstinspires.ftc.teamcode.hivequeen.RobotConstants;

import java.util.ArrayList;
import java.util.List;

/**
 * Magazine ball count (pure logic, no hardware), G407 "max 4 CONTROLLED".
 *
 * Geometry (robot frame, mm): the ball path runs from the mouth (X ~210, Z ~35) UP a 23 deg ramp
 * to the closed S3 indexer gate. Packed POLLEN queue slots against the gate: slot1 (held ball,
 * throat sensor) X -9.6 Z 127.8 | slot2 (ballCount1) X 56.4 Z 99.8 | slot3 (ballCount2) X 122.2
 * Z 71.9 | slot4 X 187.6 Z 46 (at the mouth, seen by the lip colour sensor at X 167). When the
 * magazine stops, unpowered balls roll BACK down until gecko 1 stops them (~X 133), so every sensor
 * between the gate and X 133 is crossed repeatedly; the lip (X 167) is crossed only once, going in.
 *
 * Counting model:
 *  (a) ENTRY at the lip: +1 on a ball-present rising edge while the intake drives forward and the
 *      ball is not being rejected. If the ball was not counted on the way in (e.g. it was still
 *      UNKNOWN at the rising edge) but is then pulled in (falling edge while driving forward), it is
 *      counted late. A counted ball pushed back out past the lip by a reverse pulse / outtake
 *      (falling edge while the intake reverses) is -1.
 *  (b) EXIT at the throat: -1 on a falling edge while feeding AND NOT reversing (an unjam reverse
 *      pulse during a burst moves the ball off the sensor without launching it).
 *  (c) OCCUPANCY lower bound, read ONLY while the magazine holds the queue against the closed gate
 *      at MAGAZINE_HOLD_POWER and all readings have been steady >= OCCUPANCY_STABLE_MS:
 *      throat => >= 1, + ballCount1 => >= 2, + ballCount2 => >= 3. Never decreases the count
 *      (a NECTAR covering two slot points overcounts, the safe direction for G407).
 *  (d) count is clamped to 0..MAX_CONTROLLED_ELEMENTS everywhere.
 * Faulted sensors (reading < 2 mm / NaN, see BallClassifier.validMm) are ignored: no edges, no
 * occupancy contribution.
 *
 * The counter also keeps the ORDER of ball classes (single-file chute: entries at the back, shots
 * from the front) so the shooter can pick a burst setpoint for the heaviest ball in the queue.
 */
public final class BallCounter {

    /** One loop of inputs (mutable struct: fill and pass to {@link #update}). */
    public static final class Inputs {
        public long nowMs;
        /** Lip colour sensor: ball class at the mouth (NONE = nothing) and FAULT flag. */
        public BallType lipBall = BallType.NONE;
        public boolean lipFault;
        /** Throat colour sensor (slot 1): colour class and FAULT flag. */
        public BallType throatBall = BallType.NONE;
        public boolean throatFault;
        /** Occupancy distance sensors (slot 2 / slot 3) and FAULT flags. */
        public boolean count1Present, count1Fault, count2Present, count2Fault;
        /** Intake roller actually driving in / out (applied power, incl. guard + jam pulses). */
        public boolean intakeForward, intakeReversing;
        /** The ball at the lip is being rejected (G407/G408 guard): never counted on entry. */
        public boolean rejecting;
        /** Indexer open and magazine feeding the flywheels. */
        public boolean feeding;
        /** Magazine running backwards (driver outtake or jam-detector unjam pulse). */
        public boolean magazineReversing;
        /** Magazine at MAGAZINE_HOLD_POWER with the indexer closed (queue pressed on the gate). */
        public boolean holding;
    }

    private int count = 0;
    private final List<BallType> queue = new ArrayList<>();
    private boolean primed = false;

    private boolean lipWas = false, lipBallCounted = false;
    private BallType lipBallSeen = BallType.NONE;
    private boolean throatWas = false;
    private BallType throatColourWhileBlocked = BallType.NONE;

    private long holdSinceMs = -1, occupancyChangedMs = -1;
    private int lastOccupancy = -1;

    private int shots = 0;
    private BallType lastShotBall = BallType.NONE;
    private long lastShotMs = Long.MIN_VALUE / 2;

    /** Set the count (start of match, gunner override); the queue is filled with {@code fill}. */
    public void setCount(int n, BallType fill) {
        count = clamp(n);
        queue.clear();
        for (int i = 0; i < count; i++) queue.add(fill);
    }

    public void update(Inputs in) {
        // ---------------- (a) lip entry --------------------------------------------------------
        boolean lipNow = !in.lipFault && in.lipBall != BallType.NONE;
        if (in.lipFault) {
            lipWas = false;                         // no edges while faulted
            lipBallCounted = false;
        } else if (primed) {
            if (lipNow && !lipWas) {                // rising edge
                lipBallSeen = in.lipBall;
                if (in.intakeForward && !in.rejecting) {
                    add(in.lipBall);
                    lipBallCounted = true;
                } else {
                    // Coming back OUT from inside (outtake) = a ball we hold; entering while
                    // rejected / not driven = not ours (yet).
                    lipBallCounted = in.intakeReversing && count > 0;
                }
            } else if (lipNow) {
                if (in.lipBall != BallType.UNKNOWN) lipBallSeen = in.lipBall;   // refine the class
            } else if (lipWas) {                    // falling edge
                if (in.intakeReversing) {
                    if (lipBallCounted) removeLast();          // counted ball expelled outward
                } else if (in.intakeForward && !lipBallCounted && !in.rejecting) {
                    add(lipBallSeen);                           // pulled in after a late accept
                }
                lipBallCounted = false;
            }
            lipWas = lipNow;
        } else {
            // First loop: only latch. A ball already at the lip is treated as NOT yet counted, so if
            // it is then pulled in it is counted (worst case +1 overcount, the safe G407 direction).
            lipWas = lipNow;
            lipBallSeen = in.lipBall;
            lipBallCounted = false;
        }

        // ---------------- (b) throat exit ------------------------------------------------------
        boolean throatNow = !in.throatFault && in.throatBall != BallType.NONE;
        if (throatNow) throatColourWhileBlocked = in.throatBall;
        if (in.throatFault) {
            throatWas = false;
        } else {
            if (primed && throatWas && !throatNow && in.feeding && !in.magazineReversing) {
                removeFirst();
                shots++;
                lastShotMs = in.nowMs;
                lastShotBall = BallClassifier.forTracker(throatColourWhileBlocked);
            }
            throatWas = throatNow;
        }

        // ---------------- (c) occupancy lower bound -------------------------------------------
        int occ = occupancyBound(throatNow, in.throatFault, in.count1Present, in.count1Fault,
                in.count2Present, in.count2Fault);
        if (occ != lastOccupancy) { lastOccupancy = occ; occupancyChangedMs = in.nowMs; }
        if (in.holding && !in.feeding && !in.magazineReversing) {
            if (holdSinceMs < 0) holdSinceMs = in.nowMs;
        } else {
            holdSinceMs = -1;
        }
        if (holdSinceMs >= 0 && primed
                && in.nowMs - Math.max(holdSinceMs, occupancyChangedMs) >= RobotConstants.OCCUPANCY_STABLE_MS) {
            while (count < occ) add(BallType.UNKNOWN);   // lower bound only: never decreases
        }
        primed = true;
    }

    /**
     * Lower bound from the slot sensors (queue pressed on the closed gate): throat => 1,
     * + ballCount1 => 2, + ballCount2 => 3. A faulted sensor ends the chain.
     */
    public static int occupancyBound(boolean throat, boolean throatFault, boolean c1, boolean c1Fault,
                                     boolean c2, boolean c2Fault) {
        if (throatFault || !throat) return 0;
        if (c1Fault || !c1) return 1;
        if (c2Fault || !c2) return 2;
        return 3;
    }

    private void add(BallType b) {
        if (count >= RobotConstants.MAX_CONTROLLED_ELEMENTS) return;   // (d) clamp
        count++;
        queue.add(b == null || b == BallType.NONE ? BallType.UNKNOWN : b);
    }

    private void removeLast() {
        if (count <= 0) return;
        count--;
        if (!queue.isEmpty()) queue.remove(queue.size() - 1);
    }

    private void removeFirst() {
        if (count <= 0) return;
        count--;
        if (!queue.isEmpty()) queue.remove(0);
    }

    private static int clamp(int n) { return Math.max(0, Math.min(n, RobotConstants.MAX_CONTROLLED_ELEMENTS)); }

    public int getCount() { return count; }

    /** True if any ball in the queue was classified as NECTAR (either colour) on entry. */
    public boolean queueHasNectar() {
        for (BallType b : queue) if (b.isNectar()) return true;
        return false;
    }

    /** Snapshot of the queue, front (next to fire) first. */
    public List<BallType> getQueue() { return new ArrayList<>(queue); }

    public int getShots() { return shots; }

    /** Class of the most recently LAUNCHED ball for the HIVE mass model (UNKNOWN -> POLLEN). */
    public BallType getLastShotBall() { return lastShotBall; }

    public long getLastShotMs() { return lastShotMs; }
}
