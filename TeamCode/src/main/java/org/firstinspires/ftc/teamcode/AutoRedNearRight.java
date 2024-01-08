package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Auto Red Near Right", group = "Concept")
public class AutoRedNearRight extends XBotRedNear implements AutoOpMode {
    @Override
    public void runOpMode() {
        autoRedNear(Parking.RIGHT);
    }
}
