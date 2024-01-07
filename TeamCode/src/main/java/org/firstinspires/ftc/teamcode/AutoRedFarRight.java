package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Auto Red Far Right", group = "Concept")
public class AutoRedFarRight extends XBotAutoOpMode implements AutoOpMode {
    @Override
    public void runOpMode() {
        Double DROP_LINE_X = 43.0;
        Double WHITE_STACK_Y = 9.0;
        Double WHITE_STACK_X = -50.0;

        // Initialize hardware
        initializeAuto();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

//        autonomousPlay(Alliance.RED, DistanceFromBackdrop.FAR, Parking.RIGHT);

        stopRobot();
    }
}
