package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Auto Red Far Left", group = "Concept")
public class AutoRedFarLeft extends XBotRedFar implements AutoOpMode {
    @Override
    public void runOpMode() {
        autoRedFar(Parking.LEFT);
    }
}
