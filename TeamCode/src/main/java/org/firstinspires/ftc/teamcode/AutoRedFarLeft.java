package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.XBot.ARM_POSITION_HIGH;
import static org.firstinspires.ftc.teamcode.XBot.ARM_POSITION_UP;
import static org.firstinspires.ftc.teamcode.XBot.MAX_ARM_POSITION;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

import java.util.List;

@Autonomous(name = "Auto Red Far Left", group = "Concept")
public class AutoRedFarLeft extends XBotOpMode implements AutoOpMode {

    // Constants for autonomous movement
    private static final double AUTONOMOUS_SPEED = 0.5;  // Adjust as needed
    private static final int DISTANCE_TO_DRIVE = 500;  // Adjust as needed
    private final ElapsedTime runtime = new ElapsedTime();
    SpikeMark spikeMark = SpikeMark.RIGHT;
    Alliance alliance = Alliance.RED;
    DistanceFromBackdrop distanceFromBackdrop = DistanceFromBackdrop.FAR;
    Parking parking = Parking.LEFT;
    float detectionConfidence = 0;
    boolean teamPropDetectionCompleted = false;
    boolean spikeMarkPixelDropped = false;
    boolean aTagPixelDropped = false;

    @Override
    public void runOpMode() {
        // Initialize hardware
        initialize();
        initDriveMotorsToUseEncoders();
        detectTeamPropMultipleTries();
        gameMode = GameMode.AUTO_OP_MODE;

        closeBothClaws();
        telemetry.addData("Status", "Initialized");
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.addData("SpikeMark", spikeMark + ", confidence" + detectionConfidence);
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                if (!teamPropDetectionCompleted) {
                    teamPropDetectionCompleted = detectTeamProp();
                    if ((runtime.milliseconds() > 4000) && (!teamPropDetectionCompleted)) {
                        //Give up -- assume RIGHT
                        teamPropDetectionCompleted = true;
                        spikeMark = SpikeMark.RIGHT;
                    }
                    if (teamPropDetectionCompleted) {
                        switchToAprilTagCamera();
                        // Save CPU resources; can resume streaming when needed.
                        visionPortal.stopStreaming(); //Stop until we are ready
                    }
                } else {
                    telemetry.addData("SpikeMark", spikeMark + ", confidence" + detectionConfidence);
                    // Push telemetry to the Driver Station.
                    telemetry.update();

                    if (!spikeMarkPixelDropped) {
                        driveBackwardForDistance(400);
                        moveArmToPosition(ARM_POSITION_UP);

                        switch (spikeMark) {
                            case LEFT:
                                driveForwardForDistance(350);
                                strafeRightForDistance(660);
                                moveArmToPosition(1800);
                                driveBackwardForDistance(150);
                                openLeftClaw();
                                break;
                            case RIGHT:
                                driveForwardForDistance(350);
                                strafeLeftForDistance(600);
                                moveArmToPosition(1800);
                                driveBackwardForDistance(150);
                                openLeftClaw();
                                break;
                            case CENTER:
                                strafeRightForDistance(300);
                                openLeftClaw();
                        }
                        spikeMarkPixelDropped = true;
                        moveArmToPosition(ARM_POSITION_UP);
                    }
                }
                // Share the CPU.
                sleep(20);
            }
        }
        // Stop the robot
        stopRobot();
    }

    private void detectTeamPropMultipleTries() {
        int tries = 400;
        while(!detectTeamProp() && (tries > 0) ) {
            sleep(10);
            tries -= 1;
        }
    }
    private boolean detectTeamProp() {

        boolean foundX = false;
        List<Recognition> currentRecognitions = tfod.getRecognitions();
//        telemetry.addData("# Objects Detected", currentRecognitions.size());

        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {

            if (recognition.getLeft() < 100) {
                spikeMark = SpikeMark.LEFT;
            } else {
                spikeMark = SpikeMark.CENTER;
            }
            foundX = true;
            detectionConfidence = recognition.getConfidence();

            double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
            double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;

//            telemetry.addData(":=========="," ");
            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Position", "%.0f / %.0f", x, y);
            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
            telemetry.addData("SpikeMark", spikeMark);
        }   // end for() loop
        return foundX;
    }   // end method telemetryTfod()

    private void driveForwardForDistance(int distance) {
        // Reset encoders
        resetDriveEncoders();

        // Set target position for the motors
        leftFront.setTargetPosition(distance);
        rightFront.setTargetPosition(distance);
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

    private void driveBackwardForDistance(int distance) {
        // Reset encoders
        resetDriveEncoders();

        // Set target position for the motors
        leftFront.setTargetPosition(-distance);
        rightFront.setTargetPosition(-distance);
        leftBack.setTargetPosition(-distance);
        rightBack.setTargetPosition(-distance);

        // Set motors to run to position
        setDriveRunToPosition();

        // Set motors power
        setDriveMotorsPower(AUTONOMOUS_SPEED);

        // Wait for motors to reach target position
        while (opModeIsActive() && areDriveMotorsBusy()) {
            // Additional actions or checks can be added here
            telemetry.addData("Status", "Driving backward...");
            telemetry.update();
            idle();
        }

        // Stop the motors
        stopDriveMotors();

        // Set motors back to normal mode
        stopDriveRunUsingEncoder();
    }

    private void strafeLeftForDistance(int distance) {
        // Reset encoders
        resetDriveEncoders();

        // Set target position for the motors
        leftFront.setTargetPosition(-distance);
        rightFront.setTargetPosition(distance);
        leftBack.setTargetPosition(distance);
        rightBack.setTargetPosition(-distance);

        // Set motors to run to position
        setDriveRunToPosition();

        // Set motors power
        setDriveMotorsPower(AUTONOMOUS_SPEED);

        // Wait for motors to reach target position
        while (opModeIsActive() && areDriveMotorsBusy()) {
            // Additional actions or checks can be added here
            telemetry.addData("Status", "Driving backward...");
            telemetry.update();
            idle();
        }

        // Stop the motors
        stopDriveMotors();

        // Set motors back to normal mode
        stopDriveRunUsingEncoder();
    }

    private void strafeRightForDistance(int distance) {
        // Reset encoders
        resetDriveEncoders();

        // Set target position for the motors
        leftFront.setTargetPosition(distance);
        rightFront.setTargetPosition(-distance);
        leftBack.setTargetPosition(-distance);
        rightBack.setTargetPosition(distance);

        // Set motors to run to position
        setDriveRunToPosition();

        // Set motors power
        setDriveMotorsPower(AUTONOMOUS_SPEED);

        // Wait for motors to reach target position
        while (opModeIsActive() && areDriveMotorsBusy()) {
            // Additional actions or checks can be added here
            telemetry.addData("Status", "Driving backward...");
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
        // Save more CPU resources when camera is no longer needed.
        visionPortal.close();
    }


}
