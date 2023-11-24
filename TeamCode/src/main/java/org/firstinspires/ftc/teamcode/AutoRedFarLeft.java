package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.MoveRobot.BACKWARD;
import static org.firstinspires.ftc.teamcode.MoveRobot.FORWARD;
import static org.firstinspires.ftc.teamcode.MoveRobot.STRAFE_LEFT;
import static org.firstinspires.ftc.teamcode.MoveRobot.STRAFE_RIGHT;
import static org.firstinspires.ftc.teamcode.MoveRobot.TANK_TURN_LEFT;
import static org.firstinspires.ftc.teamcode.MoveRobot.TANK_TURN_RIGHT;
import static org.firstinspires.ftc.teamcode.XBot.ARM_POSITION_HIGH;
import static org.firstinspires.ftc.teamcode.XBot.ARM_POSITION_UP;
import static org.firstinspires.ftc.teamcode.XBot.DESIRED_DISTANCE;
import static org.firstinspires.ftc.teamcode.XBot.MAX_AUTO_SPEED;
import static org.firstinspires.ftc.teamcode.XBot.MAX_AUTO_STRAFE;
import static org.firstinspires.ftc.teamcode.XBot.MAX_AUTO_TURN;
import static org.firstinspires.ftc.teamcode.XBot.SPEED_GAIN;
import static org.firstinspires.ftc.teamcode.XBot.STRAFE_GAIN;
import static org.firstinspires.ftc.teamcode.XBot.TURN_GAIN;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

import java.util.List;

@Autonomous(name = "Auto Red Far Left", group = "Concept")
public class AutoRedFarLeft extends XBotOpMode implements AutoOpMode {
    static final double DRIVE_SPEED = 0.4;     // Max driving speed for better distance accuracy.
    private final ElapsedTime runtime = new ElapsedTime();
    SpikeMark spikeMark = SpikeMark.RIGHT;
    Alliance alliance = Alliance.RED;
    DistanceFromBackdrop distanceFromBackdrop = DistanceFromBackdrop.FAR;
    Parking parking = Parking.LEFT;
    float detectionConfidence = 0;
    boolean teamPropDetectionCompleted = false;
    boolean spikeMarkPixelDropped = false;
    boolean aTagPixelDropped = false;
    static final double TURN_SPEED = 0.2;     // Max Turn speed to limit turn rate
    static final double HEADING_THRESHOLD = 1.0;    // How close must the heading get to the target before moving to next step.
    // Requiring more accuracy (a smaller number) will often make the turn take longer to get into the final position.
    // Define the Proportional control coefficient (or GAIN) for "heading control".
    // We define one value when Turning (larger errors), and the other is used when Driving straight (smaller errors).
    // Increase these numbers if the heading does not corrects strongly enough (eg: a heavy robot or using tracks)
    // Decrease these numbers if the heading does not settle on the correct value (eg: very agile robot with omni wheels)
    static final double P_TURN_GAIN = 0.02;     // Larger is more responsive, but also less stable
    static final double P_DRIVE_GAIN = 0.03;     // Larger is more responsive, but also less stable
    // Constants for autonomous movement
    private static final double AUTONOMOUS_SPEED = 0.6;  // Adjust as needed
    boolean DEBUG = false;
    @Override
    public void runOpMode() {
        // Initialize hardware
        initialize();
        initializeIMU();
        initDriveMotorsToUseEncoders();
        if (!DEBUG) {
            detectTeamPropMultipleTries();
        }
        gameMode = GameMode.AUTO_OP_MODE;

        closeBothClaws();
        telemetry.addData("Status", "Initialized");
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.addData("SpikeMark", spikeMark + ", confidence" + detectionConfidence);
        telemetry.addData(">", "Robot Heading = %4.0f", getHeading());
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                if (DEBUG) {
                    //Debug
                    moveArmToPosition(ARM_POSITION_HIGH);
                    moveRobot(1025, TANK_TURN_LEFT);
//                    moveRobot(2000, STRAFE_LEFT);
//                    moveRobot(3000, BACKWARD);
//                    moveRobot(2000, STRAFE_RIGHT);
                    moveRobot(1025, TANK_TURN_RIGHT);
                    fixRobotYaw(0);
                    continue;
                }

                if (!teamPropDetectionCompleted) {
                    teamPropDetectionCompleted = detectTeamProp();
                    if ((runtime.milliseconds() > 3000) && (!teamPropDetectionCompleted)) {
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
                    telemetry.addData(">", "Robot Heading = %4.0f", getHeading());
                    // Push telemetry to the Driver Station.
                    telemetry.update();

                    if (!spikeMarkPixelDropped) {
                        moveRobot(400, BACKWARD);
                        moveArmToPosition(ARM_POSITION_UP);

                        switch (spikeMark) {
                            case LEFT:
                                moveRobot(350, FORWARD);
                                moveRobot(660, STRAFE_RIGHT);
                                moveArmToPosition(1800);
                                moveRobot(150, BACKWARD);
                                openLeftClaw();
                                moveArmToPosition(ARM_POSITION_UP);
                                moveRobot(1050, TANK_TURN_RIGHT);
                                break;
                            case RIGHT:
                                moveRobot(350, FORWARD);
                                moveRobot(600, STRAFE_LEFT);
                                moveArmToPosition(1800);
                                moveRobot(150, BACKWARD);
                                openLeftClaw();
                                moveArmToPosition(ARM_POSITION_UP);
                                moveRobot(1050, TANK_TURN_RIGHT);
                                break;
                            case CENTER:
                                moveRobot(300, STRAFE_RIGHT);
                                moveArmToPosition(1800);
                                moveRobot(250, BACKWARD);
                                openLeftClaw();
                                moveArmToPosition(ARM_POSITION_UP);
                                moveRobot(1050, TANK_TURN_RIGHT);
                        }
                        spikeMarkPixelDropped = true;
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
            //If multiple detections for any reason -- use the one with highest
            if (recognition.getConfidence() > detectionConfidence) {
                if (recognition.getLeft() < 100) {
                    spikeMark = SpikeMark.LEFT;
                } else {
                    spikeMark = SpikeMark.CENTER;
                }
                foundX = true;
                detectionConfidence = recognition.getConfidence();

                double x = (recognition.getLeft() + recognition.getRight()) / 2;
                double y = (recognition.getTop() + recognition.getBottom()) / 2;

                telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
                telemetry.addData("- Position", "%.0f / %.0f", x, y);
                telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
                telemetry.addData("SpikeMark", spikeMark);
            }
        }   // end for() loop
        return foundX;
    }   // end method telemetryTfod()

    private void moveRobot(int distance, MoveRobot moveRobot) {
        moveRobot(distance, moveRobot, AUTONOMOUS_SPEED);
    }

    private void moveRobot(int distance, MoveRobot moveRobot, double speed) {
        // Reset encoders
        resetDriveEncoders();
        double heading;
        int lfDirection = 1;
        int rfDirection = 1;
        int lbDirection = 1;
        int rbDirection = 1;
        switch (moveRobot) {
            case STRAFE_RIGHT:
                heading = 0;
                lbDirection = -1;
                rfDirection = -1;
                break;
            case STRAFE_LEFT:
                heading = 0;
                lfDirection = -1;
                rbDirection = -1;
                break;
            case FORWARD:
                heading = 0;
                break;
            case BACKWARD:
                heading = 0;
                lfDirection = -1;
                lbDirection = -1;
                rfDirection = -1;
                rbDirection = -1;
                break;
            case TANK_TURN_LEFT:
                heading = -90;
                lfDirection = -1;
                lbDirection = -1;
                break;
            case TANK_TURN_RIGHT:
                heading = 90;
                rfDirection = -1;
                rbDirection = -1;
                break;
            default:
                heading = 0;
        }
        leftFront.setTargetPosition(distance * lfDirection);
        rightFront.setTargetPosition(distance * rfDirection);
        leftBack.setTargetPosition(distance * lbDirection);
        rightBack.setTargetPosition(distance * rbDirection);

        // Set motors to run to position
        setDriveRunToPosition();
        // Set motors power
        setDriveMotorsPower(speed);

        // Wait for motors to reach target position
        while (opModeIsActive() && areDriveMotorsBusy()) {
            telemetry.addData("Status", moveRobot);
            telemetry.addData("Heading- Target : Current", "%5.3f : %5.3f", heading, getHeading());
            telemetry.addData("Distance to go", distance);
            telemetry.addData("Left Front Motor", leftFront.getCurrentPosition() + "  busy=" + leftFront.isBusy());
            telemetry.addData("Left Back Motor", leftBack.getCurrentPosition() + "  busy=" + leftBack.isBusy());
            telemetry.addData("Right Front Motor", rightFront.getCurrentPosition() + "  busy=" + rightFront.isBusy());
            telemetry.addData("Right Back Motor", rightBack.getCurrentPosition() + "  busy=" + rightBack.isBusy());
            telemetry.update();
            idle();
        }

        setDriveMotorsPower(.05);

        // Stop the motors
        stopDriveMotors();

//        // Set motors back to normal mode
        stopDriveRunUsingEncoder();
    }

    void fixRobotYaw(double heading) {
        if (Math.abs(heading - getHeading()) > 1) {
            //Fix
            if (heading < getHeading())
                moveRobot(60, TANK_TURN_RIGHT, AUTONOMOUS_SPEED/2);
            else
                moveRobot(60, TANK_TURN_LEFT, AUTONOMOUS_SPEED/2);
        }
    }

    // Other autonomous actions and methods can be added here

    private void stopRobot() {
        stopDriveMotors();
        // Save more CPU resources when camera is no longer needed.
        visionPortal.close();
    }
}
