package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Auto Red Near Left", group = "Concept")
public class AutoRedNearLeft extends XBotAutoOpMode implements AutoOpMode {
    @Override
    public void runOpMode() {
        // Initialize hardware
        initializeAuto();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                autonomousPlay(Alliance.RED, DistanceFromBackdrop.NEAR, Parking.LEFT);
                sleep(10);
            }
        }
        stopRobot();
    }
}
