package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Auto Blue Near Right", group = "Concept")
public class AutoBlueNearRight extends XBotBlueNear implements AutoOpMode {
    @Override
    public void runOpMode() {
        autoBlueNear(Parking.RIGHT);
    }
}
