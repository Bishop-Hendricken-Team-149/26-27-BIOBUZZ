package org.firstinspires.ftc.teamcode.hivequeen.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.hivequeen.game.Alliance;

/** BLUE Autonomous (RED path rotated 180 deg about the field centre; the field is rotationally symmetric). */
@Autonomous(name = "HIVE-QUEEN Auto BLUE", group = "HIVE-QUEEN", preselectTeleOp = "HIVE-QUEEN TeleOp")
public class HiveQueenAutoBlue extends HiveQueenAutoBase {
    @Override
    protected Alliance alliance() { return Alliance.BLUE; }
}
