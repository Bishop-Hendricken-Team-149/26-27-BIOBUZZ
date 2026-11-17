package org.firstinspires.ftc.teamcode.hivequeen.util;

/** Rising-edge detector for gamepad buttons. */
public final class EdgeDetector {
    private boolean last = false;

    /** @return true only on the loop where {@code now} goes false -> true */
    public boolean rose(boolean now) {
        boolean r = now && !last;
        last = now;
        return r;
    }
}
