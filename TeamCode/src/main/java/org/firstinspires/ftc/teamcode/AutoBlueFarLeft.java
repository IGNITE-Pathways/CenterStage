package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Auto Blue Far Left", group = "Concept")
public class AutoBlueFarLeft extends XBotAutoOpMode implements AutoOpMode {
    @Override
    public void runOpMode() {
        // Initialize hardware
        initializeAuto();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                autonomousPlay(Alliance.BLUE, DistanceFromBackdrop.FAR, Parking.LEFT);
                sleep(10);
            }
        }
        stopRobot();
    }
}
