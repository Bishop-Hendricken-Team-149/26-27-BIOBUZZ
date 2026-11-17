package org.firstinspires.ftc.teamcode.hivequeen.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.hivequeen.game.Alliance;

/** RED Autonomous. preselectTeleOp makes the DS queue the TeleOp after AUTO. */
@Autonomous(name = "HIVE-QUEEN Auto RED", group = "HIVE-QUEEN", preselectTeleOp = "HIVE-QUEEN TeleOp")
public class HiveQueenAutoRed extends HiveQueenAutoBase {
    @Override
    protected Alliance alliance() { return Alliance.RED; }
}
