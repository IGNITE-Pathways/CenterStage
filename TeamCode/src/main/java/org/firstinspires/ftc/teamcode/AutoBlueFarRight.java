package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Auto Blue Far Right", group = "Concept")
public class AutoBlueFarRight extends XBotBlueFar implements AutoOpMode {
    @Override
    public void runOpMode() {
        autoBlueFar(Parking.RIGHT);
    }
}
