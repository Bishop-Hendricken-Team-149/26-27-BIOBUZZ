package org.firstinspires.ftc.teamcode.hivequeen.game;

/**
 * Pure-logic stall/jam detector for one roller motor (intake M7 or magazine M8).
 *
 * If the motor is commanded (|power| >= MIN_POWER) but its encoder velocity stays below
 * STALL_FRACTION of the expected speed for STALL_MS, the detector overrides the power with a
 * short reverse pulse (UNJAM_POWER for UNJAM_MS) and then lets the command resume. Three jams
 * inside RETRY_WINDOW_MS put it in COOLDOWN (power 0 for COOLDOWN_MS) so a hard jam cannot cook
 * the motor; the driver sees it on telemetry.
 *
 * Encoder safety: if the motor has been commanded for more than NO_ENCODER_MS in total and the
 * encoder has never shown any motion, the encoder cable is assumed unplugged and detection is
 * disabled permanently (state NO_ENCODER) instead of reversing the roller forever.
 */
public final class JamDetector {
    public enum State { OK, UNJAM, COOLDOWN, NO_ENCODER }

    public static final double MIN_POWER = 0.3;
    public static final double STALL_FRACTION = 0.15;
    public static final long STALL_MS = 350;
    public static final long UNJAM_MS = 250;
    public static final double UNJAM_POWER = 0.6;
    public static final long GRACE_MS = 300;          // spin-up grace after a command starts / unjam ends
    public static final long RETRY_WINDOW_MS = 5000;
    public static final int MAX_RETRIES = 3;
    public static final long COOLDOWN_MS = 1500;
    public static final long NO_ENCODER_MS = 3000;
    public static final double MOTION_TPS = 50.0;

    private final double freeTicksPerSec;
    private State state = State.OK;
    private boolean encoderSeen = false;
    private long poweredMs = 0, lastMs = -1;
    private long commandSinceMs = -1, stallSinceMs = -1, stateUntilMs = -1;
    private double unjamSign = -1;
    private final long[] jamTimes = new long[MAX_RETRIES];
    private int jamIdx = 0, jams = 0;

    public JamDetector(double freeTicksPerSec) { this.freeTicksPerSec = freeTicksPerSec; }

    /**
     * @param power     the power the subsystem wants to apply
     * @param ticksPerS measured encoder velocity
     * @return the power to actually apply
     */
    public double update(long nowMs, double power, double ticksPerS) {
        long dt = lastMs < 0 ? 0 : Math.max(0, nowMs - lastMs);
        lastMs = nowMs;
        if (Math.abs(ticksPerS) > MOTION_TPS) encoderSeen = true;
        boolean commanded = Math.abs(power) >= MIN_POWER;
        if (commanded) poweredMs += dt;
        if (state == State.NO_ENCODER) return power;
        if (!encoderSeen && poweredMs > NO_ENCODER_MS) { state = State.NO_ENCODER; return power; }

        if (state == State.UNJAM) {
            if (nowMs < stateUntilMs) return unjamSign * UNJAM_POWER;
            state = State.OK;
            commandSinceMs = nowMs;          // grace again after the pulse
            stallSinceMs = -1;
        }
        if (state == State.COOLDOWN) {
            if (nowMs < stateUntilMs) return 0.0;
            state = State.OK;
            jams = 0;
            commandSinceMs = -1;
            stallSinceMs = -1;
        }
        if (!commanded) { commandSinceMs = -1; stallSinceMs = -1; return power; }
        if (commandSinceMs < 0) commandSinceMs = nowMs;
        if (!encoderSeen || nowMs - commandSinceMs < GRACE_MS) return power;

        boolean stalled = Math.abs(ticksPerS) < STALL_FRACTION * freeTicksPerSec * Math.abs(power);
        if (!stalled) { stallSinceMs = -1; return power; }
        if (stallSinceMs < 0) stallSinceMs = nowMs;
        if (nowMs - stallSinceMs < STALL_MS) return power;

        // jam confirmed
        jamTimes[jamIdx] = nowMs;
        jamIdx = (jamIdx + 1) % MAX_RETRIES;
        jams = Math.min(MAX_RETRIES, jams + 1);
        stallSinceMs = -1;
        long oldest = jamTimes[jamIdx];  // the slot about to be overwritten = oldest of the last MAX_RETRIES
        if (jams >= MAX_RETRIES && nowMs - oldest <= RETRY_WINDOW_MS) {
            state = State.COOLDOWN;
            stateUntilMs = nowMs + COOLDOWN_MS;
            return 0.0;
        }
        state = State.UNJAM;
        unjamSign = -Math.signum(power);
        stateUntilMs = nowMs + UNJAM_MS;
        return unjamSign * UNJAM_POWER;
    }

    public State getState() { return state; }

    public boolean isEncoderSeen() { return encoderSeen; }
}
