package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.MoveRobot.BACKWARD;
import static org.firstinspires.ftc.teamcode.MoveRobot.FORWARD;
import static org.firstinspires.ftc.teamcode.MoveRobot.STRAFE_LEFT;
import static org.firstinspires.ftc.teamcode.MoveRobot.STRAFE_RIGHT;
import static org.firstinspires.ftc.teamcode.MoveRobot.TANK_TURN_LEFT;
import static org.firstinspires.ftc.teamcode.MoveRobot.TANK_TURN_RIGHT;
import static org.firstinspires.ftc.teamcode.XBot.ARM_POSITION_UP;
import static org.firstinspires.ftc.teamcode.XBot.MAX_ARM_POSITION;
import static org.firstinspires.ftc.teamcode.XBot.MAX_AUTO_SPEED;
import static org.firstinspires.ftc.teamcode.XBot.MAX_AUTO_STRAFE;
import static org.firstinspires.ftc.teamcode.XBot.MAX_AUTO_TURN;
import static org.firstinspires.ftc.teamcode.XBot.MAX_WRIST_POS;
import static org.firstinspires.ftc.teamcode.XBot.MIN_ARM_POSITION;
import static org.firstinspires.ftc.teamcode.XBot.SPEED_GAIN;
import static org.firstinspires.ftc.teamcode.XBot.STRAFE_GAIN;
import static org.firstinspires.ftc.teamcode.XBot.TURN_GAIN;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

public abstract class XBotAutoOpMode extends XBotOpMode {
    boolean DEBUG = false; //Use to test certain code / bypass some
    static final double AUTONOMOUS_SPEED = 0.6;  // Adjust as needed
    //Define motors and sensors
    int desiredTagId = -1;                  // change based on spikeMark identification
    //Game Mode
    GameMode gameMode = GameMode.AUTO_OP_MODE;
    SpikeMark spikeMark = SpikeMark.RIGHT; //Default
    float detectionConfidence = 0;
    boolean teamPropDetectionCompleted = false;
    boolean spikeMarkPixelDropped = false; //Purple
    boolean aTagPixelDropped = false; //Yellow
    boolean arrivedAtBackDropTagPosition = false;

    boolean detectDesiredAprilTag(int tagId) {
        boolean aprilTagFound = false;
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            if ((detection.metadata != null) && (detection.id == tagId)) {
                aprilTagFound = true;
                desiredTagDetectionObj = detection;
                break;  // don't look any further.
            } else {
                telemetry.addData("Unknown Target", "Tag ID %d is not in TagLibrary\n", detection.id);
            }
        }
        if (!aprilTagFound) telemetry.addData("April Tag not found", tagId);
        return aprilTagFound;
    }

    void initializeAuto() {
        initialize();
        initializeIMU();
        initDriveMotorsToUseEncoders();
        closeBothClaws();
        gameMode = GameMode.AUTO_OP_MODE;
        if (!DEBUG) {
            detectTeamPropMultipleTries();
        }
        telemetry.addData("Status", "Initialized");
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.addData("SpikeMark", spikeMark + ", confidence" + detectionConfidence);
        telemetry.addData(">", "Robot Heading = %4.0f", getHeading());
        telemetry.update();
    }

    void moveRobot(int distance, MoveRobot moveRobot) {
        moveRobot(distance, moveRobot, AUTONOMOUS_SPEED);
    }

    private void detectTeamPropMultipleTries() {
        int tries = 400;
        while (!detectTeamProp() && (tries > 0)) {
            sleep(10);
            tries -= 1;
        }
    }

    boolean detectTeamProp() {
        boolean foundX = false;
        List<Recognition> currentRecognitions = tfod.getRecognitions();
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

                telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
                telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
                telemetry.addData("SpikeMark", spikeMark);
            }
        }   // end for() loop
        return foundX;
    } //detectTeamProp

    void detectTeamPropAndSwitchCameraToAprilTag() {
        teamPropDetectionCompleted = detectTeamProp();
        if ((runtime.milliseconds() > 1500) && (!teamPropDetectionCompleted)) {
            //Give up -- assume RIGHT
            teamPropDetectionCompleted = true;
            spikeMark = SpikeMark.RIGHT;
        }
        if (teamPropDetectionCompleted) {
            switchToAprilTagCamera();
            // Save CPU resources; can resume streaming when needed.
            visionPortal.stopStreaming(); //Stop until we are ready
        }
    } //detectTeamPropAndSwitchCameraToAprilTag

    void leftSpikeMark(Alliance alliance, SpikeMark spikeMark, DistanceFromBackdrop distanceFromBackdrop, Parking parking) {
        moveRobot(400, BACKWARD);
        moveArmToPosition(ARM_POSITION_UP);

        moveRobot(350, FORWARD);
        fixRobotYaw(0);
        moveRobot(710, STRAFE_RIGHT);
        fixRobotYaw(0);
        moveArmToPosition(1770);
        moveRobot(170, BACKWARD);
        if (alliance == Alliance.RED) {
            openLeftClaw();
        } else {
            openRightClaw();
        }
        sleep(100);
        moveArmToPosition(ARM_POSITION_UP);
        if (alliance == Alliance.RED) {
            moveRobot(1030, TANK_TURN_RIGHT);
            moveRobot(140, STRAFE_LEFT);
            fixRobotYaw(-90);
        } else {
            moveRobot(1030, TANK_TURN_LEFT);
            moveRobot(140, STRAFE_RIGHT);
            fixRobotYaw(90);
        }
        moveArmToPosition(200);
        if (distanceFromBackdrop == DistanceFromBackdrop.FAR) {
            moveRobot(3000, BACKWARD);
        } else {
            moveRobot(100, BACKWARD);
        }
        if (alliance == Alliance.RED) {
            moveRobot(1000, STRAFE_RIGHT);
        } else {
            moveRobot(1000, STRAFE_LEFT);
        }
        //April Tag Nav
        if (alliance == Alliance.RED) {
            desiredTagId = 4;
        } else {
            desiredTagId = 1;
        }
    }

    void rightSpikeMark(Alliance alliance, SpikeMark spikeMark, DistanceFromBackdrop distanceFromBackdrop, Parking parking) {
        moveRobot(400, BACKWARD);
        moveArmToPosition(ARM_POSITION_UP);

        moveRobot(400, FORWARD);
        moveRobot(520, STRAFE_LEFT);
        fixRobotYaw(0);
        moveArmToPosition(1770);
        moveRobot(150, BACKWARD);
        if (alliance == Alliance.RED) {
            openLeftClaw();
        } else {
            openRightClaw();
        }
        sleep(100);
        moveArmToPosition(ARM_POSITION_UP);
        moveRobot(550, STRAFE_RIGHT);
        fixRobotYaw(0);
        if (alliance == Alliance.RED) {
            moveRobot(1025, TANK_TURN_RIGHT);
            moveRobot(175, STRAFE_LEFT);
            fixRobotYaw(-90);
        } else {
            moveRobot(1025, TANK_TURN_LEFT);
            moveRobot(175, STRAFE_RIGHT);
            fixRobotYaw(90);
        }
        moveArmToPosition(200);
        if (distanceFromBackdrop == DistanceFromBackdrop.FAR) {
            moveRobot(3000, BACKWARD);
        } else {
            moveRobot(100, BACKWARD);
        }
        if (alliance == Alliance.RED) {
            moveRobot(1000, STRAFE_RIGHT);
        } else {
            moveRobot(1000, STRAFE_LEFT);
        }
        //April Tag Nav
        if (alliance == Alliance.RED) {
            desiredTagId = 6;
        } else {
            desiredTagId = 3;
        }
    }

    void centerSpikeMark(Alliance alliance, SpikeMark spikeMark, DistanceFromBackdrop distanceFromBackdrop, Parking parking) {
        if (alliance == Alliance.RED) {
            moveRobot(400, BACKWARD);
            moveArmToPosition(ARM_POSITION_UP);
            fixRobotYaw(0);
            moveRobot(400, STRAFE_RIGHT);
        } else {
            moveRobot(380, BACKWARD);
            moveArmToPosition(ARM_POSITION_UP);
            fixRobotYaw(0);
            moveRobot(400, STRAFE_LEFT);
        }
        fixRobotYaw(0);
        moveArmToPosition(1770);
        wristPosition = MAX_WRIST_POS;
        setWristPosition(wristPosition);
        moveRobot(250, BACKWARD);
        stopDriveMotors();
        if (alliance == Alliance.RED) {
            openLeftClaw();
        } else {
            openRightClaw();
        }
        sleep(100);
        moveArmToPosition(200);
        if (alliance == Alliance.RED) {
            moveRobot(1030, TANK_TURN_RIGHT);
            moveRobot(620, STRAFE_LEFT);
            fixRobotYaw(-90);
        } else {
            moveRobot(1030, TANK_TURN_LEFT);
            moveRobot(700, STRAFE_RIGHT);
            fixRobotYaw(90);
        }
        if (distanceFromBackdrop == DistanceFromBackdrop.FAR) {
            moveRobot(3300, BACKWARD);
        } else {
            moveRobot(100, BACKWARD);
        }
        if (alliance == Alliance.RED) {
            moveRobot(800, STRAFE_RIGHT);
        } else {
            moveRobot(800, STRAFE_LEFT);
        }
        //April Tag Nav
        if (alliance == Alliance.RED) {
            desiredTagId = 5;
        } else {
            desiredTagId = 2;
        }
    }

    void spikeMarkPixelDroppedGetReadyForATagNav() {
        spikeMarkPixelDropped = true;
        visionPortal.resumeStreaming();
        stopDriveMotors();
        stopDriveRunUsingEncoder();
        autoDrive = true; //drive in reverse
    }

    void aprilTagNavMoveToDesiredTagPosition(Alliance alliance) {
        int yellowPixelOffset = (alliance == Alliance.BLUE) ? 5 : -5;
        //Robot should be in front on the board to start AprilTag Nav
        boolean targetFound = detectDesiredAprilTag(desiredTagId);

        if (targetFound) {
            telemetry.addData("Found", "ID %d (%s)", desiredTagDetectionObj.id, desiredTagDetectionObj.metadata.name);
            telemetry.addData("Range", "%5.1f inches", desiredTagDetectionObj.ftcPose.range);
            telemetry.addData("Bearing", "%3.0f degrees", desiredTagDetectionObj.ftcPose.bearing);
            telemetry.addData("Yaw", "%3.0f degrees", desiredTagDetectionObj.ftcPose.yaw);
            telemetry.update();

            // Navigate using April Tag
            // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
            double rangeError = (desiredTagDetectionObj.ftcPose.range - XBot.DESIRED_DISTANCE);
            double headingError = desiredTagDetectionObj.ftcPose.bearing + yellowPixelOffset;
            double yawError = desiredTagDetectionObj.ftcPose.yaw;

            // Use the speed and turn "gains" to calculate how we want the robot to move.
            double drive = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
            double strafe = Range.clip(headingError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);
            double yawTurn = Range.clip(-yawError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);

            moveRobot(drive, strafe, yawTurn);

            if ((rangeError < 0.5) && (rangeError > -0.5)) {
                arrivedAtBackDropTagPosition = true;
                visionPortal.close();
                stopDriveMotors();
            }
        } else {
            telemetry.addData("Looking for Tag", desiredTagId + "\n");
            telemetry.update();
        }
    }

    void dropYellowPixel() {
        moveArmToPosition(MAX_ARM_POSITION);
        openLeftClaw();
        sleep(100);
        moveArmToPosition(200);
        aTagPixelDropped = true;
    }

    void parkRobot(Alliance alliance, Parking parking, SpikeMark spikeMark) {
        if (parking == Parking.LEFT) {
            switch (spikeMark) {
                case LEFT:
                    moveRobot(750, STRAFE_RIGHT);
                    break;
                case CENTER:
                    moveRobot(900, STRAFE_RIGHT);
                    break;
                case RIGHT:
                    moveRobot(1050, STRAFE_RIGHT);
                    break;
            }
        } else {
            switch (spikeMark) {
                case LEFT:
                    moveRobot(1050, STRAFE_LEFT);
                    break;
                case CENTER:
                    moveRobot(900, STRAFE_LEFT);
                    break;
                case RIGHT:
                    moveRobot(750, STRAFE_LEFT);
                    break;
            }
        }
        moveRobot(100, BACKWARD);
        moveArmToPosition(MIN_ARM_POSITION);
    }

    void autonomousPlay(Alliance alliance, DistanceFromBackdrop distanceFromBackdrop, Parking parking) {
        if (!teamPropDetectionCompleted) {
            detectTeamPropAndSwitchCameraToAprilTag();
        } else {
            telemetry.addData("SpikeMark", spikeMark + ", confidence" + detectionConfidence);
            telemetry.addData(">", "Robot Heading = %4.0f", getHeading());

            if (!spikeMarkPixelDropped) {
                switch (spikeMark) {
                    case LEFT:
                        leftSpikeMark(alliance, spikeMark, distanceFromBackdrop, parking);
                        break;
                    case RIGHT:
                        rightSpikeMark(alliance, spikeMark, distanceFromBackdrop, parking);
                        break;
                    case CENTER:
                        centerSpikeMark(alliance, spikeMark, distanceFromBackdrop, parking);
                }
                telemetry.addData("Back Side", "Ready for April Tag Nav");
                spikeMarkPixelDroppedGetReadyForATagNav();
            }
            if (!aTagPixelDropped) {
                if (!arrivedAtBackDropTagPosition) {
                    telemetry.addData("Looking for April Tag", desiredTagId);
                    aprilTagNavMoveToDesiredTagPosition(alliance);
                } else {
                    telemetry.addData("Arrived", "Dropping Pixel now");
                    dropYellowPixel();
                    //Park Now
                    parkRobot(alliance, parking, spikeMark);
                }
            }
            telemetry.update();
        }
    }

    void fixRobotYaw(double heading) {
        int tries = 5;
        double error = Math.abs(heading - getHeading());
        while ((error > 2) && (tries > 0)) {
            //Fix
            double speed = Range.clip(error, 0, AUTONOMOUS_SPEED);
            int distanceTicks = (int)Range.scale(error, 0, 5, 0, 30);
            if (heading < getHeading())
                moveRobot(distanceTicks, TANK_TURN_RIGHT, speed, true);
            else
                moveRobot(distanceTicks, TANK_TURN_LEFT, speed, true);
            tries -= 1;
        }
    }
}
