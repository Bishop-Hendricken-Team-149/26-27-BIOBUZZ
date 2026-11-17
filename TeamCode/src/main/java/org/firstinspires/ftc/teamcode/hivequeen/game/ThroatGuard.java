package org.firstinspires.ftc.teamcode.hivequeen.game;

import org.firstinspires.ftc.teamcode.hivequeen.RobotConstants;

/**
 * Downstream G408 check (pure logic). If an opponent NECTAR somehow reaches the throat (colour gate
 * overridden, misread at the lip, ball knocked in), the robot must NOT launch it into our HIVE:
 * firing is blocked (S3 indexer held closed) and the driver is warned to clear it by reversing.
 *
 * Opponent colour must be seen for THROAT_OPPONENT_CONFIRM_MS (one-frame misreads do not block);
 * once blocked, the block holds until the throat reads EMPTY for THROAT_CLEAR_MS (the ball was
 * reversed out). An UNKNOWN colour at the throat does not block (it fires; HIVE mass = POLLEN).
 * Disabled (never blocks) when the colour gate is overridden by the driver or the throat sensor
 * is in FAULT.
 */
public final class ThroatGuard {
    private boolean blocked = false;
    private long opponentSinceMs = -1, emptySinceMs = -1;

    /** @return true while firing must be blocked. */
    public boolean update(long nowMs, BallType throatBall, Alliance alliance, boolean enabled) {
        if (!enabled) {
            blocked = false;
            opponentSinceMs = -1;
            emptySinceMs = -1;
            return false;
        }
        if (throatBall == alliance.opponentNectar()) {
            if (opponentSinceMs < 0) opponentSinceMs = nowMs;
            if (nowMs - opponentSinceMs >= RobotConstants.THROAT_OPPONENT_CONFIRM_MS) blocked = true;
            emptySinceMs = -1;
        } else {
            opponentSinceMs = -1;
            if (blocked) {
                if (throatBall == BallType.NONE) {
                    if (emptySinceMs < 0) emptySinceMs = nowMs;
                    if (nowMs - emptySinceMs >= RobotConstants.THROAT_CLEAR_MS) blocked = false;
                } else {
                    emptySinceMs = -1;   // something still sits at the throat
                }
            }
        }
        return blocked;
    }

    public boolean isBlocked() { return blocked; }
}
