package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous(name = "Auto Blue Far Right", group = "Concept")
public class AutoBlueFarRight extends XBotAutoOpMode implements AutoOpMode {
    @Override
    public void runOpMode() {
        // Initialize hardware
        initializeAuto();
        mecanumDrive.setPoseEstimate(new Pose2d(-15, 63.5, Math.toRadians(90)));

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                autonomousPlay(Alliance.BLUE, DistanceFromBackdrop.FAR, Parking.RIGHT);
                sleep(10);
            }
        }
        stopRobot();
    }
}
