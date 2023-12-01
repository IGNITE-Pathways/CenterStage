package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.MoveRobot.BACKWARD;
import static org.firstinspires.ftc.teamcode.MoveRobot.FORWARD;
import static org.firstinspires.ftc.teamcode.MoveRobot.STRAFE_LEFT;
import static org.firstinspires.ftc.teamcode.MoveRobot.STRAFE_RIGHT;
import static org.firstinspires.ftc.teamcode.MoveRobot.TANK_TURN_LEFT;
import static org.firstinspires.ftc.teamcode.MoveRobot.TANK_TURN_RIGHT;
import static org.firstinspires.ftc.teamcode.XBot.ARM_PICK_POSITION;
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
import static org.firstinspires.ftc.teamcode.XBot.WRIST_PICK_POSITION;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

public abstract class XBotAutoOpMode extends XBotOpMode {
    boolean DEBUG = false; //Use to test certain code / bypass some
    static final double AUTONOMOUS_SPEED = 0.6;  // Adjust as needed
    int desiredTagId = -1;                  // change based on spikeMark identification
    SpikeMark spikeMark = SpikeMark.RIGHT; //Default
    float detectionConfidence = 0;
    boolean teamPropDetectionCompleted = false;
    boolean spikeMarkPixelDropped = false; //Purple
    boolean aTagPixelDropped = false; //Yellow
    boolean arrivedAtBackDropTagPosition = false;
    boolean targetFoundAtLeastOnce = false;

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

    void leftSpikeMark(Alliance alliance, SpikeMark spikeMark, DistanceFromBackdrop distanceFromBackdrop) {
        telemetry.addData("SpikeMark", spikeMark + ", confidence" + detectionConfidence);
        if (alliance == Alliance.RED) {
            //RED ALLIANCE
            moveRobot(200, BACKWARD);
            moveArmToPosition(ARM_POSITION_UP);
            fixRobotYaw(0);
            if (distanceFromBackdrop == DistanceFromBackdrop.FAR) {
                moveRobot(710, STRAFE_RIGHT);
            } else {
                moveRobot(520, STRAFE_LEFT);
            }
        } else {
            //BLUE ALLIANCE
            moveRobot(200, BACKWARD);
            moveArmToPosition(ARM_POSITION_UP);
            fixRobotYaw(0);
            if (distanceFromBackdrop == DistanceFromBackdrop.FAR) {
                moveRobot(520, STRAFE_LEFT);
            } else {
                moveRobot(520, STRAFE_RIGHT);
            }
        }

        fixRobotYaw(0);
//        moveArmToPosition(1770);
//        wristPosition = MAX_WRIST_POS;
//        setWristPosition(wristPosition);
//        moveRobot(250, BACKWARD);
//        stopDriveMotors();

        if ( ((alliance == Alliance.BLUE) && (distanceFromBackdrop == DistanceFromBackdrop.FAR)) ||
                ((alliance == Alliance.RED) && (distanceFromBackdrop == DistanceFromBackdrop.NEAR))) {
            //Close to Truss
            moveRobot(1030, TANK_TURN_LEFT);
            moveArmToPosition(ARM_PICK_POSITION+4);
            wristPosition = WRIST_PICK_POSITION;
            setWristPosition(wristPosition);
            sleep(100);
            moveRobot(220, FORWARD);
            fixRobotYaw(90);
        } else {
            //Away from Truss
            moveArmToPosition(1770);
            wristPosition = MAX_WRIST_POS;
            setWristPosition(wristPosition);
            sleep(100);
            moveRobot(250, BACKWARD);
        }

        //Drop Purple Pixel
        if (alliance == Alliance.RED) {
            if (distanceFromBackdrop == DistanceFromBackdrop.FAR) {
                openLeftClaw();
            } else {
                openRightClaw();
            }
        } else {
            if (distanceFromBackdrop == DistanceFromBackdrop.FAR) {
                openRightClaw();
            } else {
                openLeftClaw();
            }
        }

        if ( ((alliance == Alliance.BLUE) && (distanceFromBackdrop == DistanceFromBackdrop.FAR)) ||
                ((alliance == Alliance.RED) && (distanceFromBackdrop == DistanceFromBackdrop.NEAR))) {
            //Move Arm up before we turn Robot 90 degrees
            moveArmToPosition(ARM_POSITION_UP);

            //Strafe back towards center of the tile
            moveRobot(550, STRAFE_LEFT);
        }

        //Move arm lower to be able to go under the truss
        moveArmToPosition(200);

        //Turn back (camera side) towards back board and strafe to get ready to go under the truss
        if (alliance == Alliance.RED) {
            moveRobot(1030, TANK_TURN_RIGHT);
            //Strafe so we can go under the truss
            if (distanceFromBackdrop == DistanceFromBackdrop.FAR) {
                moveRobot(175, STRAFE_LEFT);
            }
            fixRobotYaw(-90);
        } else {
            moveRobot(1030, TANK_TURN_LEFT);
            //Strafe so we can go under the truss
            if (distanceFromBackdrop == DistanceFromBackdrop.FAR) {
                moveRobot(175, STRAFE_RIGHT);
            }
            fixRobotYaw(90);
        }

        if (distanceFromBackdrop == DistanceFromBackdrop.FAR) {
            moveRobot(3000, BACKWARD);
//        } else {
//            moveRobot(100, BACKWARD);
        }

        //Strafe to be in front of april tag
        if (alliance == Alliance.RED) {
            if (distanceFromBackdrop == DistanceFromBackdrop.FAR) {
                moveRobot(890, STRAFE_RIGHT);
            } else {
                moveRobot(300, STRAFE_RIGHT);
            }
            fixRobotYaw(-90);
        } else {
            if (distanceFromBackdrop == DistanceFromBackdrop.FAR) {
                moveRobot(890, STRAFE_LEFT);
            } else {
                moveRobot(500, STRAFE_LEFT);
            }
            fixRobotYaw(90);
        }

//        if (alliance == Alliance.RED) {
//            moveRobot(1000, STRAFE_RIGHT);
//        } else {
//            moveRobot(1000, STRAFE_LEFT);
//        }
        //April Tag Nav
        if (alliance == Alliance.RED) {
            desiredTagId = 4;
        } else {
            desiredTagId = 1;
        }

    }

    void rightSpikeMark(Alliance alliance, SpikeMark spikeMark, DistanceFromBackdrop distanceFromBackdrop) {
        telemetry.addData("SpikeMark", spikeMark + ", confidence" + detectionConfidence);
        if (alliance == Alliance.RED) {
            moveRobot(400, BACKWARD);
            moveArmToPosition(ARM_POSITION_UP);
            moveRobot(400, FORWARD);
            if (distanceFromBackdrop == DistanceFromBackdrop.FAR) {
                moveRobot(450, STRAFE_LEFT);
            } else {
                moveRobot(520, STRAFE_RIGHT);
            }
        } else {
            moveRobot(200, BACKWARD);
            sleep(100);
            moveArmToPosition(ARM_POSITION_UP);
            if (distanceFromBackdrop == DistanceFromBackdrop.FAR) {
                //Away from truss
                moveRobot(520, STRAFE_RIGHT);
            } else {
                //close to truss, so we move forward
                moveRobot(980, BACKWARD);
            }
        }

        fixRobotYaw(0);

        if ( ((alliance == Alliance.BLUE) && (distanceFromBackdrop == DistanceFromBackdrop.FAR)) ||
                ((alliance == Alliance.RED) && (distanceFromBackdrop == DistanceFromBackdrop.NEAR))) {
            //Away from Truss
            moveArmToPosition(1770);
            wristPosition = MAX_WRIST_POS;
            setWristPosition(wristPosition);
            sleep(100);
            moveRobot(150, BACKWARD);
        } else {
            //Close to Truss
            moveRobot(1030, TANK_TURN_LEFT);
            moveArmToPosition(ARM_PICK_POSITION+4);
            wristPosition = WRIST_PICK_POSITION;
            setWristPosition(wristPosition);
            sleep(100);
            moveRobot(220, FORWARD);
            fixRobotYaw(90);
        }

        //Drop Purple Pixel
        if (alliance == Alliance.RED) {
            if (distanceFromBackdrop == DistanceFromBackdrop.FAR) {
                openLeftClaw();
            } else {
                openRightClaw();
            }
        } else {
            if (distanceFromBackdrop == DistanceFromBackdrop.FAR) {
                openRightClaw();
            } else {
                openLeftClaw();
            }
        }

//        if ( ((alliance == Alliance.BLUE) && (distanceFromBackdrop == DistanceFromBackdrop.NEAR)) ||
//                ((alliance == Alliance.RED) && (distanceFromBackdrop == DistanceFromBackdrop.FAR))){
//            //Move Arm up before we turn Robot 90 degrees
//            moveArmToPosition(ARM_POSITION_UP);
//
//            //Strafe back towards center of the tile
//            moveRobot(600, STRAFE_RIGHT);
//        }

        //Move arm lower to be able to go under the truss
        moveArmToPosition(200);

        //Turn back (camera side) towards back board and strafe to get ready to go under the truss
        if (alliance == Alliance.RED) {
            moveRobot(1030, TANK_TURN_RIGHT);
            //Strafe so we can go under the truss
            if (distanceFromBackdrop == DistanceFromBackdrop.FAR) {
                moveRobot(175, STRAFE_LEFT);
            }
            fixRobotYaw(-90);
        } else {
            if (distanceFromBackdrop == DistanceFromBackdrop.FAR) {
                moveRobot(1030, TANK_TURN_LEFT);
                //Strafe so we can go under the truss
                moveRobot(175, STRAFE_RIGHT);
            } else {
                //move forward toward april tag
                moveRobot(850, BACKWARD);
            }
            fixRobotYaw(90);
        }

        if (distanceFromBackdrop == DistanceFromBackdrop.FAR) {
            moveRobot(3000, BACKWARD);
        }

        //Strafe to be in front of april tag
        if (alliance == Alliance.RED) {
            if (distanceFromBackdrop == DistanceFromBackdrop.FAR) {
                moveRobot(890, STRAFE_RIGHT);
            } else {
                moveRobot(300, STRAFE_RIGHT);
            }
            fixRobotYaw(-90);
        } else {
            //BLUE
            if (distanceFromBackdrop == DistanceFromBackdrop.FAR) {
                moveRobot(890, STRAFE_LEFT);
                fixRobotYaw(90);
            }
        }

        //April Tag Nav
        if (alliance == Alliance.RED) {
            desiredTagId = 6;
        } else {
            desiredTagId = 3;
        }

    }

    void centerSpikeMark(Alliance alliance, SpikeMark spikeMark, DistanceFromBackdrop distanceFromBackdrop) {
        telemetry.addData("SpikeMark", spikeMark + ", confidence" + detectionConfidence);
        if (alliance == Alliance.RED) {
            moveRobot(420, BACKWARD);
            moveArmToPosition(ARM_POSITION_UP);
            fixRobotYaw(0);
            if (distanceFromBackdrop == DistanceFromBackdrop.FAR) {
                moveRobot(350, STRAFE_RIGHT);
            } else {
                moveRobot(400, STRAFE_LEFT);
            }
        } else {
            moveRobot(420, BACKWARD);
            moveArmToPosition(ARM_POSITION_UP);
            fixRobotYaw(0);
            if (distanceFromBackdrop == DistanceFromBackdrop.FAR) {
                moveRobot(400, STRAFE_LEFT);
            } else {
                moveRobot(350, STRAFE_RIGHT);
            }
        }
        fixRobotYaw(0);
        moveArmToPosition(1770);
        wristPosition = MAX_WRIST_POS;
        setWristPosition(wristPosition);
        moveRobot(250, BACKWARD);
        stopDriveMotors();

        //Drop Purple Pixel
        if (alliance == Alliance.RED) {
            if (distanceFromBackdrop == DistanceFromBackdrop.FAR) {
                openLeftClaw();
            } else {
                openRightClaw();
            }
        } else {
            if (distanceFromBackdrop == DistanceFromBackdrop.FAR) {
                openRightClaw();
            } else {
                openLeftClaw();
            }
        }

        //Move arm lower to be able to go under the truss
        moveArmToPosition(200);

        //Turn back (camera side) towards back board and strafe to get ready to go under the truss
        if (alliance == Alliance.RED) {
            moveRobot(1030, TANK_TURN_RIGHT);
            //Strafe so we can go under the truss
            if (distanceFromBackdrop == DistanceFromBackdrop.FAR) {
                moveRobot(620, STRAFE_LEFT);
            }
            fixRobotYaw(-90);
        } else {
            moveRobot(1030, TANK_TURN_LEFT);
            //Strafe so we can go under the truss
            if (distanceFromBackdrop == DistanceFromBackdrop.FAR) {
                moveRobot(720, STRAFE_RIGHT);
            }
            fixRobotYaw(90);
        }

        //drive towards back side
        if (distanceFromBackdrop == DistanceFromBackdrop.FAR) {
            moveRobot(3250, BACKWARD);
//        } else {
//            moveRobot(100, BACKWARD);
        }

        //Strafe to be in front of april tag
        if (alliance == Alliance.RED) {
            if (distanceFromBackdrop == DistanceFromBackdrop.FAR) {
                moveRobot(890, STRAFE_RIGHT);
            } else {
                moveRobot(300, STRAFE_RIGHT);
            }
            fixRobotYaw(-90);
        } else {
            if (distanceFromBackdrop == DistanceFromBackdrop.FAR) {
                moveRobot(890, STRAFE_LEFT);
            } else {
                moveRobot(300, STRAFE_LEFT);
            }
            fixRobotYaw(90);
        }

        //Set the desired April Tag Id
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
        targetFoundAtLeastOnce = false;
    }

    void aprilTagNavMoveToDesiredTagPosition(Alliance alliance, DistanceFromBackdrop distanceFromBackdrop) {
        int yellowPixelOffset = (alliance == Alliance.BLUE)
                ? ((distanceFromBackdrop == DistanceFromBackdrop.FAR) ? 8 : -8)
                : ((distanceFromBackdrop == DistanceFromBackdrop.FAR) ? -8 : 8);

        //Robot should be in front on the board to start AprilTag Nav
        boolean targetFound = detectDesiredAprilTag(desiredTagId);
        if (targetFound) {
            targetFoundAtLeastOnce = true;
            telemetry.addData("Found", "ID %d (%s)", desiredTagDetectionObj.id, desiredTagDetectionObj.metadata.name);
            telemetry.addData("Range", "%5.1f inches", desiredTagDetectionObj.ftcPose.range);
            telemetry.addData("Bearing", "%3.0f degrees", desiredTagDetectionObj.ftcPose.bearing);
            telemetry.addData("Yaw", "%3.0f degrees", desiredTagDetectionObj.ftcPose.yaw);
            telemetry.update();

            // Navigate using April Tag
            // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
            double rangeError = (desiredTagDetectionObj.ftcPose.range - XBot.AUTOOP_DESIRED_DISTANCE);
            double headingError = desiredTagDetectionObj.ftcPose.bearing + yellowPixelOffset;
            double yawError = desiredTagDetectionObj.ftcPose.yaw;

            // Use the speed and turn "gains" to calculate how we want the robot to move.
            double drive = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
            double strafe = Range.clip(headingError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);
            double yawTurn = Range.clip(-yawError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);

            moveRobot(drive, strafe, yawTurn);

            if ((rangeError < 0.1) && (rangeError > -0.1)) {
                arrivedAtBackDropTagPosition = true;
                visionPortal.close();
                stopDriveMotors();
            }
        } else {
            telemetry.addData("Didn't find Desired Tag", desiredTagId + "\n");
            telemetry.update();
            //Reasons for not able to find -- exposure or not in-front of aTags
            if (targetFoundAtLeastOnce) {
                arrivedAtBackDropTagPosition = true;
                visionPortal.close();
                stopDriveMotors();
            }
        }
    }

    void dropYellowPixel(Alliance alliance, DistanceFromBackdrop distanceFromBackdrop) {
        moveArmToPosition(MAX_ARM_POSITION);
        if (alliance == Alliance.RED) {
            if (distanceFromBackdrop == DistanceFromBackdrop.FAR) {
                openRightClaw();
            } else {
                openLeftClaw();
            }
        } else {
            if (distanceFromBackdrop == DistanceFromBackdrop.FAR) {
                openLeftClaw();
            } else {
                openRightClaw();
            }
        }
        sleep(100);
        moveArmToPosition(200);
        aTagPixelDropped = true;
    }

    void parkRobot(Alliance alliance, Parking parking, SpikeMark spikeMark, DistanceFromBackdrop distanceFromBackdrop) {
        if (alliance == Alliance.RED) {
            fixRobotYaw(-90);
        } else {
            fixRobotYaw(90);
        }
        if (parking == Parking.LEFT) {
            //LEFT SIDE PARKING
            switch (spikeMark) {
                case LEFT:
                    moveRobot(900, STRAFE_RIGHT);
                    break;
                case CENTER:
                    moveRobot(1100, STRAFE_RIGHT);
                    break;
                case RIGHT:
                    moveRobot(1300, STRAFE_RIGHT);
                    break;
            }
        } else {
            //RIGHT SIDE PARKING
            switch (spikeMark) {
                case LEFT:
                    moveRobot(1150, STRAFE_LEFT);
                    break;
                case CENTER:
                    if (distanceFromBackdrop == DistanceFromBackdrop.FAR) {
                        //yellow pixel on right so the robot has to strafe less, so strafe back less
                        moveRobot(1000, STRAFE_LEFT);
                    } else {
                        //yellow pixel on left so the robot has to strafe more, so strafe back more
                        moveRobot(1200, STRAFE_LEFT);
                    }
                    break;
                case RIGHT:
                    if (distanceFromBackdrop == DistanceFromBackdrop.FAR) {
                        //yellow pixel on right so the robot has to strafe less, so strafe back less
                        moveRobot(850, STRAFE_LEFT);
                    } else {
                        //yellow pixel on left so the robot has to strafe more, so strafe back more
                        moveRobot(950, STRAFE_LEFT);
                    }
                    break;
            }
        }
        moveRobot(300, BACKWARD);
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
                        leftSpikeMark(alliance, spikeMark, distanceFromBackdrop);
                        break;
                    case RIGHT:
                        rightSpikeMark(alliance, spikeMark, distanceFromBackdrop);
                        break;
                    case CENTER:
                        centerSpikeMark(alliance, spikeMark, distanceFromBackdrop);
                }
                telemetry.addData("Back Side", "Ready for April Tag Nav");
                spikeMarkPixelDroppedGetReadyForATagNav();
            }
            if (!aTagPixelDropped) {
                if (!arrivedAtBackDropTagPosition) {
                    telemetry.addData("Looking for April Tag", desiredTagId);
                    aprilTagNavMoveToDesiredTagPosition(alliance, distanceFromBackdrop);
                } else {
                    telemetry.addData("Arrived", "Dropping Pixel now");
                    dropYellowPixel(alliance, distanceFromBackdrop);
                    //Park Now
                    telemetry.addData("Parking", "");
                    parkRobot(alliance, parking, spikeMark, distanceFromBackdrop);
                }
            }
            telemetry.update();
        }
    }

    void fixRobotYaw(double heading) {
        int tries = 5;
        double error = Math.abs(heading - getHeading());
        while ((error > 1) && (tries > 0)) {
            //Fix
            error = Math.abs(heading - getHeading());
            double speed = Range.clip(error, 0, AUTONOMOUS_SPEED);
            int distanceTicks = (int)Range.scale(error, 0, 5, 0, 20);
            if (heading < getHeading())
                moveRobot(distanceTicks, TANK_TURN_RIGHT, speed, true);
            else
                moveRobot(distanceTicks, TANK_TURN_LEFT, speed, true);
            tries -= 1;
        }
    }
}
