package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Auto Red Near Left", group = "Concept")
public class AutoRedNearLeft extends XBotRedNear implements AutoOpMode {
    @Override
    public void runOpMode() {
        autoRedNear(Parking.LEFT);
    }
}
