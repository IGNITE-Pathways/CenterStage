package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Auto Red Far Right", group = "Concept")
public class AutoRedFarRight extends XBotRedFar implements AutoOpMode {
    @Override
    public void runOpMode() {
        autoRedFar(Parking.RIGHT);
    }
}
