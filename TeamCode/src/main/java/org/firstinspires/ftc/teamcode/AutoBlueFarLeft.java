package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Auto Blue Far Left", group = "Concept")
public class AutoBlueFarLeft extends XBotBlueFar implements AutoOpMode {
    @Override
    public void runOpMode() {
        // Initialize hardware
        autoBlueFar(Parking.LEFT);
    }
}
