package org.firstinspires.ftc.teamcode.hivequeen.game;

/**
 * SCORING ELEMENTS (Sec. 9.8): POLLEN = ~2.8 in (7.1 cm) yellow ball; NECTAR = ~3.6 in (9.1 cm)
 * ball in red or blue. "POLLEN and NECTAR are not perfectly spherical and may vary in size."
 */
public enum BallType {
    NONE,
    POLLEN,
    RED_NECTAR,
    BLUE_NECTAR,
    /** Something is there but colour/size could not be classified confidently. */
    UNKNOWN;

    public boolean isNectar() { return this == RED_NECTAR || this == BLUE_NECTAR; }

    public boolean isBall() { return this != NONE; }
}
