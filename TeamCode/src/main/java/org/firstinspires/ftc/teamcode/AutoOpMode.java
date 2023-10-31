package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Autonomous Drive", group = "Concept")
public class AutoOpMode extends XBotOpMode {

    // Constants for autonomous movement
    private static final double AUTONOMOUS_SPEED = 0.5;  // Adjust as needed
    private static final int DISTANCE_TO_DRIVE = 1000;  // Adjust as needed

    private final ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        // Initialize hardware
        initialize();
        initDriveMotorsToUseEncoders();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // Autonomous driving
        driveForwardForDistance(DISTANCE_TO_DRIVE);

        // Stop the robot
        stopRobot();
    }

    private void driveForwardForDistance(int distance) {
        // Reset encoders
        resetDriveEncoders();

        // Set target position for the motors
        leftFront.setTargetPosition(distance);
        rightBack.setTargetPosition(distance);
        leftBack.setTargetPosition(distance);
        rightBack.setTargetPosition(distance);

        // Set motors to run to position
        setDriveRunToPosition();

        // Set motors power
        setDriveMotorsPower(AUTONOMOUS_SPEED);

        // Wait for motors to reach target position
        while (opModeIsActive() && areDriveMotorsBusy()) {
            // Additional actions or checks can be added here
            telemetry.addData("Status", "Driving forward...");
            telemetry.update();
            idle();
        }

        // Stop the motors
        stopDriveMotors();

        // Set motors back to normal mode
        stopDriveRunUsingEncoder();
    }

    // Other autonomous actions and methods can be added here

    private void stopRobot() {
        stopDriveMotors();
        // Additional actions to stop the robot can be added here
    }


}
