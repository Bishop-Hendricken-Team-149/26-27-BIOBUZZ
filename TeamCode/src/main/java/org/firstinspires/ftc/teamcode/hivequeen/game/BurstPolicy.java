package org.firstinspires.ftc.teamcode.hivequeen.game;

/**
 * Multi-ball burst rules (pure logic).
 *
 * Re-checking "at speed" before EACH ball of a burst is NOT feasible with this hardware: the balls
 * are packed against each other, S3 can only stop the ball in slot 1 (re-closing its fingers under
 * a moving ball pinches it), and pausing the magazine would also stop the push the ball in flight
 * needs to cross the 53 mm unpowered gap into the counter-wheel. So the whole burst runs at ONE
 * setpoint: the MAX setpoint of the balls in the queue (NECTAR >= POLLEN at every range), latched
 * while the indexer is open. Limitation: shots 2..4 leave with whatever wheel speed has recovered
 * (the flywheel inertia discs keep the dip small); POLLEN fired in a NECTAR burst gets ~3 % more RPM.
 */
public final class BurstPolicy {
    private BurstPolicy() { }

    /**
     * Ball class whose setpoint the burst uses: NECTAR if the next ball (throat, size-aware) or any
     * ball recorded in the queue is NECTAR, else POLLEN. NONE/UNKNOWN never lower it.
     */
    public static BallType burstBall(BallType nextBallForShot, boolean queueHasNectar) {
        if ((nextBallForShot != null && nextBallForShot.isNectar()) || queueHasNectar) {
            return nextBallForShot != null && nextBallForShot.isNectar() ? nextBallForShot : BallType.RED_NECTAR;
        }
        return BallType.POLLEN;
    }

    /**
     * TeleOp last-ball dwell: keep the indexer open (magazine feeding) for {@code dwellMs} after the
     * last counted shot, even if fire was released, so the last ball crosses the unpowered gap.
     */
    public static boolean lastBallDwellActive(boolean indexerOpen, int count, long nowMs, long lastShotMs, long dwellMs) {
        return indexerOpen && count == 0 && nowMs - lastShotMs >= 0 && nowMs - lastShotMs < dwellMs;
    }
}
