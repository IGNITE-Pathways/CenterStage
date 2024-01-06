package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Auto Blue Near Right", group = "Concept")
public class AutoBlueNearRight extends XBotAutoOpMode implements AutoOpMode {
    @Override
    public void runOpMode() {
        // Initialize hardware
        initializeAuto();
        xDrive.setPoseEstimate(new Pose2d(15, 63.5, Math.toRadians(90)));

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        if (opModeIsActive()) {
            autonomousPlay(Alliance.BLUE, DistanceFromBackdrop.NEAR, Parking.RIGHT);
        }
        stopRobot();
    }
}
