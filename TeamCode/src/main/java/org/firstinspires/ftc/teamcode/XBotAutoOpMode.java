package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.MoveRobot.BACKWARD;
import static org.firstinspires.ftc.teamcode.MoveRobot.FORWARD;
import static org.firstinspires.ftc.teamcode.MoveRobot.STRAFE_LEFT;
import static org.firstinspires.ftc.teamcode.MoveRobot.STRAFE_RIGHT;
import static org.firstinspires.ftc.teamcode.MoveRobot.TANK_TURN_LEFT;
import static org.firstinspires.ftc.teamcode.MoveRobot.TANK_TURN_RIGHT;
import static org.firstinspires.ftc.teamcode.XBot.ARM_PICK_POSITION;
import static org.firstinspires.ftc.teamcode.XBot.ARM_POSITION_UP;
import static org.firstinspires.ftc.teamcode.XBot.DEFAULT_DROP_ARM_POSITION;
import static org.firstinspires.ftc.teamcode.XBot.MAX_ARM_POSITION;
import static org.firstinspires.ftc.teamcode.XBot.MAX_AUTO_SPEED;
import static org.firstinspires.ftc.teamcode.XBot.MAX_AUTO_STRAFE;
import static org.firstinspires.ftc.teamcode.XBot.MAX_AUTO_TURN;
import static org.firstinspires.ftc.teamcode.XBot.MIN_ARM_POSITION;
import static org.firstinspires.ftc.teamcode.XBot.MIN_WRIST_POS;
import static org.firstinspires.ftc.teamcode.XBot.SPEED_GAIN;
import static org.firstinspires.ftc.teamcode.XBot.STRAFE_GAIN;
import static org.firstinspires.ftc.teamcode.XBot.TURN_GAIN;
import static org.firstinspires.ftc.teamcode.XBot.WRIST_FLAT_TO_GROUND;
import static org.firstinspires.ftc.teamcode.XBot.WRIST_VERTICAL;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

public abstract class XBotAutoOpMode extends XBotOpMode {
    static final double AUTONOMOUS_SPEED = 0.6;  // Adjust as needed
    boolean DEBUG = false; //Use to test certain code / bypass some
    int desiredTagId = -1;                  // change based on spikeMark identification
    SpikeMark spikeMark = SpikeMark.RIGHT; //Default
    float detectionConfidence = 0;
    boolean teamPropDetectionCompleted = false;
    boolean spikeMarkPixelDropped = false; //Purple
    boolean aTagPixelDropped = false; //Yellow
    boolean arrivedAtBackDropTagPosition = false;
    int timesTargetFound = 0;
    boolean reachedAprilTag = false;
    SampleMecanumDrive xDrive = null;

    Double DROP_LINE_X = 39.8;
    Double WHITE_STACK_Y = 11.0;
    Double WHITE_STACK_X = -51.5;
    Double PARKING_OFFSET = 24.0;

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
        initDriveMotorsToUseEncoders();
        xDrive = new SampleMecanumDrive(hardwareMap);
        xDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        setWristPosition(WRIST_VERTICAL);
        closeBothClaws();
        gameMode = GameMode.AUTO_OP_MODE;
        if (!DEBUG) {
            detectTeamPropMultipleTries();
        }
        telemetry.addData("Status", "Initialized");
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.addData("SpikeMark", spikeMark + ", confidence" + detectionConfidence);
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
        if ((runtime.milliseconds() > 1600) && (!teamPropDetectionCompleted)) {
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

    boolean leftSpikeMark(Alliance alliance, SpikeMark spikeMark, DistanceFromBackdrop distanceFromBackdrop) {
        telemetry.addData("SpikeMark", spikeMark + ", confidence" + detectionConfidence);
        if (alliance == Alliance.RED) {
            //RED ALLIANCE
            moveRobot(200, BACKWARD);
            sleep(100);
            moveArmToPosition(ARM_POSITION_UP);
            if (distanceFromBackdrop == DistanceFromBackdrop.FAR) {
                moveRobot(710, STRAFE_RIGHT);
            } else {
                //Close to truss -- use different strategy
                //close to truss, so we move forward
                moveRobot(950, BACKWARD);
            }
        } else {
            //BLUE ALLIANCE
            moveRobot(200, BACKWARD);
            moveArmToPosition(ARM_POSITION_UP);
            if (distanceFromBackdrop == DistanceFromBackdrop.FAR) {
                //CLOSE to truss
                moveRobot(600, STRAFE_LEFT); //Else the arm will hit the truss
                moveRobot(980, BACKWARD);
            } else {
                moveRobot(520, STRAFE_RIGHT);
            }
        }

//        fixRobotYaw(0);

        if (((alliance == Alliance.BLUE) && (distanceFromBackdrop == DistanceFromBackdrop.FAR)) ||
                ((alliance == Alliance.RED) && (distanceFromBackdrop == DistanceFromBackdrop.NEAR))) {
            //CLOSE to truss
            if (alliance == Alliance.BLUE) {
                //distance = FAR, alliance = BLUE
                moveRobot(1030, TANK_TURN_LEFT);
                moveArmToPosition(1770);
                wristPosition = MIN_WRIST_POS;
                setWristPosition(wristPosition);
                sleep(100);
                moveRobot(420, BACKWARD);
            } else {
                //distance = NEAR, alliance = RED
                moveRobot(1030, TANK_TURN_RIGHT);
                moveArmToPosition(ARM_PICK_POSITION + 4);
                wristPosition = WRIST_FLAT_TO_GROUND;
                setWristPosition(wristPosition);
                sleep(100);
                moveRobot(200, FORWARD); //Robot Start position is almost centered
            }
        } else {
            //Away from Truss
            moveArmToPosition(1770);
            wristPosition = MIN_WRIST_POS;
            setWristPosition(wristPosition);
            sleep(100);
            //@TODO: Test for FAR, RED (BLUE, NEAR works)
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

        if ((alliance == Alliance.BLUE) && (distanceFromBackdrop == DistanceFromBackdrop.FAR)) {
            moveRobot(420, FORWARD);
        }

        //Move arm lower to be able to go under the truss
        moveArmToPosition(200);

        //Turn back (camera side) towards back board and strafe to get ready to go under the truss
        if (alliance == Alliance.RED) {
            //RED
            if (distanceFromBackdrop == DistanceFromBackdrop.FAR) {
                moveRobot(1035, TANK_TURN_RIGHT);
                //Strafe so we can go under the truss
                moveRobot(240, STRAFE_LEFT);
            } else {
                //Close to truss -- use different strategy
                moveRobot(850, BACKWARD); //Last Move
            }
        } else {
            //BLUE, Strafe so we can go under the truss
            if (distanceFromBackdrop == DistanceFromBackdrop.FAR) {
                moveRobot(500, STRAFE_RIGHT);
            } else {
                moveRobot(1030, TANK_TURN_LEFT);
            }
        }

        if (distanceFromBackdrop == DistanceFromBackdrop.FAR) {
            if (alliance == Alliance.RED) {
                moveRobot(3500, BACKWARD);
            } else {
                //BLUE
//                moveRobot(3250, BACKWARD); //@TODO: Steps before this need calibration
            }
        }

        //Strafe to be in front of april tag
        if (alliance == Alliance.RED) {
            if (distanceFromBackdrop == DistanceFromBackdrop.FAR) {
                moveRobot(1090, STRAFE_RIGHT);
            }
        } else {
            if (distanceFromBackdrop == DistanceFromBackdrop.FAR) {
//                moveRobot(890, STRAFE_LEFT); //@TODO: Steps before this need calibration
            } else {
                //BLUE, NEAR
                moveRobot(350, STRAFE_LEFT); //Left April Tag Id 1 on left, strafe less
            }
        }

        //April Tag Nav
        if (alliance == Alliance.RED) {
            desiredTagId = 4;
        } else {
            desiredTagId = 1;
        }

        //Since we didn't go through truss, we return false
        return (alliance != Alliance.BLUE) || (distanceFromBackdrop != DistanceFromBackdrop.FAR);
    }

    boolean rightSpikeMark(Alliance alliance, SpikeMark spikeMark, DistanceFromBackdrop distanceFromBackdrop) {
        telemetry.addData("SpikeMark", spikeMark + ", confidence" + detectionConfidence);
        if (alliance == Alliance.RED) {
            //RED ALLIANCE
            moveRobot(200, BACKWARD);
            moveArmToPosition(ARM_POSITION_UP);
//            fixRobotYaw(0);
            if (distanceFromBackdrop == DistanceFromBackdrop.FAR) {
                //CLOSE to truss
                moveRobot(600, STRAFE_RIGHT); //Else the arm will hit the truss
                moveRobot(980, BACKWARD);
            } else {
                moveRobot(580, STRAFE_LEFT);
            }
        } else {
            //BLUE ALLIANCE
            moveRobot(200, BACKWARD);
            sleep(100);
            moveArmToPosition(ARM_POSITION_UP);
            if (distanceFromBackdrop == DistanceFromBackdrop.FAR) {
                //Away from truss
                moveRobot(710, STRAFE_LEFT); //@TODO
            } else {
                //CLOSE to truss
                moveRobot(980, BACKWARD);
            }
        }

        if (((alliance == Alliance.BLUE) && (distanceFromBackdrop == DistanceFromBackdrop.FAR)) ||
                ((alliance == Alliance.RED) && (distanceFromBackdrop == DistanceFromBackdrop.NEAR))) {
            //Away from Truss
            moveArmToPosition(1770);
            wristPosition = MIN_WRIST_POS;
            setWristPosition(wristPosition);
            sleep(100);
            //@TODO: Test for FAR, BLUE (RED, NEAR works)
            moveRobot(250, BACKWARD);
        } else {
            //CLOSE to truss
            if (alliance == Alliance.RED) {
                //distance = FAR, alliance = RED
                moveRobot(1030, TANK_TURN_RIGHT);
                moveArmToPosition(1770);
                wristPosition = MIN_WRIST_POS;
                setWristPosition(wristPosition);
                sleep(100);
                moveRobot(420, BACKWARD); //Push it under the truss
            } else {
                //distance = NEAR, alliance = BLUE
                moveRobot(1030, TANK_TURN_LEFT);
                moveArmToPosition(ARM_PICK_POSITION + 4);
                wristPosition = WRIST_FLAT_TO_GROUND;
                setWristPosition(wristPosition);
                sleep(100);
                moveRobot(220, FORWARD);
            }
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

        if ((alliance == Alliance.RED) && (distanceFromBackdrop == DistanceFromBackdrop.FAR)) {
            moveRobot(420, FORWARD);
        }

        //Move arm lower to be able to go under the truss
        moveArmToPosition(200);

        //Turn back (camera side) towards back board and strafe to get ready to go under the truss
        if (alliance == Alliance.RED) {
            //Strafe so we can go under the truss
            if (distanceFromBackdrop == DistanceFromBackdrop.FAR) {
                //@TODO: Needs testing
                moveRobot(500, STRAFE_LEFT);
            } else {
                //NEAR
                moveRobot(1030, TANK_TURN_RIGHT);
            }
        } else {
            //BLUE alliance
            if (distanceFromBackdrop == DistanceFromBackdrop.FAR) {
                moveRobot(1030, TANK_TURN_LEFT);
                //Strafe so we can go under the truss
                moveRobot(240, STRAFE_RIGHT); //@TODO
            } else {
                //CLOSE to truss -- move forward toward april tag
                moveRobot(850, BACKWARD);
            }
        }

        if (distanceFromBackdrop == DistanceFromBackdrop.FAR) {
            if (alliance == Alliance.RED) {
//                moveRobot(4000, BACKWARD); //@TODO: Steps before this need calibration
            } else {
                moveRobot(3250, BACKWARD);
            }
        }

        //Strafe to be in front of april tag
        if (alliance == Alliance.RED) {
            if (distanceFromBackdrop == DistanceFromBackdrop.FAR) {
                //Close to truss
                //moveRobot(890, STRAFE_RIGHT); //@TODO: Steps before this need calibration
            } else {
                moveRobot(400, STRAFE_RIGHT);
            }
        } else {
            //BLUE
            if (distanceFromBackdrop == DistanceFromBackdrop.FAR) {
                //@TODO: Needs testing
                moveRobot(1090, STRAFE_LEFT);
            }
        }

        //April Tag Nav
        if (alliance == Alliance.RED) {
            desiredTagId = 6;
        } else {
            desiredTagId = 3;
        }

        //Since we didn't go through truss, we return false
        return (alliance != Alliance.RED) || (distanceFromBackdrop != DistanceFromBackdrop.FAR);
    }

    boolean centerSpikeMark(Alliance alliance, SpikeMark spikeMark, DistanceFromBackdrop distanceFromBackdrop) {
        telemetry.addData("SpikeMark", spikeMark + ", confidence" + detectionConfidence);
        if (alliance == Alliance.RED) {
            moveRobot(420, BACKWARD);
            moveArmToPosition(ARM_POSITION_UP);
            if (distanceFromBackdrop == DistanceFromBackdrop.FAR) {
                moveRobot(350, STRAFE_RIGHT); //Robot away from truss, so strafe less
            } else {
                moveRobot(400, STRAFE_LEFT); //Robot close to truss, so strafe more
            }
        } else {
            moveRobot(420, BACKWARD);
            moveArmToPosition(ARM_POSITION_UP);
            if (distanceFromBackdrop == DistanceFromBackdrop.FAR) {
                //@TODO: Needs testing
                moveRobot(400, STRAFE_LEFT); //Robot close to truss, so strafe more
            } else {
                moveRobot(350, STRAFE_RIGHT); //Robot away from truss, so strafe less
            }
        }
        moveArmToPosition(1770);
        wristPosition = MIN_WRIST_POS;
        setWristPosition(wristPosition);
        moveRobot(300, BACKWARD);
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
            moveRobot(1035, TANK_TURN_RIGHT);
            //Strafe so we can go under the truss
            if (distanceFromBackdrop == DistanceFromBackdrop.FAR) {
                moveRobot(660, STRAFE_LEFT);
            }
        } else {
            moveRobot(1030, TANK_TURN_LEFT);
            //Strafe so we can go under the truss
            if (distanceFromBackdrop == DistanceFromBackdrop.FAR) {
                //@TODO: Needs testing
                moveRobot(720, STRAFE_RIGHT);
            }
        }

        //drive towards back side
        if (distanceFromBackdrop == DistanceFromBackdrop.FAR) {
            moveRobot(3250, BACKWARD);
        }

        //Strafe to be in front of april tag
        if (alliance == Alliance.RED) {
            if (distanceFromBackdrop == DistanceFromBackdrop.FAR) {
                moveRobot(890, STRAFE_RIGHT);
            } else {
                moveRobot(300, STRAFE_RIGHT);
            }
        } else {
            if (distanceFromBackdrop == DistanceFromBackdrop.FAR) {
                moveRobot(890, STRAFE_LEFT);
            } else {
                moveRobot(300, STRAFE_LEFT);
            }
        }

        //Set the desired April Tag Id
        if (alliance == Alliance.RED) {
            desiredTagId = 5;
        } else {
            desiredTagId = 2;
        }
        return true;
    }

    void spikeMarkPixelDroppedGetReadyForATagNav() {
        spikeMarkPixelDropped = true;
        visionPortal.resumeStreaming();
        stopDriveMotors();
        stopDriveRunUsingEncoder();
        autoDrive = true; //drive in reverse
        timesTargetFound = 0;
    }

    void aprilTagNavMoveToDesiredTagPosition(Alliance alliance, DistanceFromBackdrop distanceFromBackdrop) {
        int yellowPixelOffset = (alliance == Alliance.BLUE)
                ? ((distanceFromBackdrop == DistanceFromBackdrop.FAR) ? 8 : -8)
                : ((distanceFromBackdrop == DistanceFromBackdrop.FAR) ? -8 : 8);

        //Robot should be in front on the board to start AprilTag Nav
        boolean targetFound = detectDesiredAprilTag(desiredTagId);
        if (targetFound) {
            timesTargetFound += 1;
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
                visionPortal.getActiveCamera().close();
                stopDriveMotors();
            }
        } else {
            telemetry.addData("Didn't find Desired Tag", desiredTagId + "\n");
            telemetry.update();
            //Reasons for not able to find -- exposure or not in-front of aTags
            if (timesTargetFound > 5) {
                arrivedAtBackDropTagPosition = true;
                visionPortal.getActiveCamera().close();
                stopDriveMotors();
            }
        }
    }

    void dropYellowPixel() {
        moveArmToPosition(MAX_ARM_POSITION);
        openBothClaws();
        moveArmToPosition(200);
        aTagPixelDropped = true;
    }

    void parkRobot(Alliance alliance, Parking parking, SpikeMark spikeMark, DistanceFromBackdrop distanceFromBackdrop) {
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
                    moveRobot(1250, STRAFE_LEFT);
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
        resetWristAndClawPosition();
    }

    void autonomousPlay(Alliance alliance, DistanceFromBackdrop distanceFromBackdrop, Parking parking) {
        if (!teamPropDetectionCompleted) {
            detectTeamPropAndSwitchCameraToAprilTag();
        } else {
            telemetry.addData("SpikeMark", spikeMark + ", confidence" + detectionConfidence);

            if (!spikeMarkPixelDropped) {
                switch (spikeMark) {
                    case LEFT:
                        reachedAprilTag = leftSpikeMark(alliance, spikeMark, distanceFromBackdrop);
                        break;
                    case RIGHT:
                        reachedAprilTag = rightSpikeMark(alliance, spikeMark, distanceFromBackdrop);
                        break;
                    case CENTER:
                        reachedAprilTag = centerSpikeMark(alliance, spikeMark, distanceFromBackdrop);
                }
                telemetry.addData("Back Side", "Ready for April Tag Nav");
                spikeMarkPixelDroppedGetReadyForATagNav();
            }
            if (reachedAprilTag) {
                if (!aTagPixelDropped) {
                    if (!arrivedAtBackDropTagPosition) {
                        telemetry.addData("Looking for April Tag", desiredTagId);
                        aprilTagNavMoveToDesiredTagPosition(alliance, distanceFromBackdrop);
                    } else {
                        telemetry.addData("Arrived", "Dropping Pixel now");
                        dropYellowPixel();
                        //Park Now
                        telemetry.addData("Parking", "");
                        parkRobot(alliance, parking, spikeMark, distanceFromBackdrop);
                    }
                }
            }
            telemetry.update();
        }
    }

    public void grabAndDropWhitePixels(TrajectorySequence trajToPickWhitePixels, TrajectorySequence trajSeq4, TrajectorySequence trajSeq5, TrajectorySequence trajBackToDropWhitePixles) {
        //STEP 3 -- Go to pick 2 White Pixels
        moveArmToPosition(MIN_ARM_POSITION + 60); //sleep(200);
        xDrive.followTrajectorySequence(trajToPickWhitePixels);
        setWristPosition(WRIST_FLAT_TO_GROUND);
        sleep(200);

        //STEP 4 -- Move forward to grab pixels
        xDrive.followTrajectorySequence(trajSeq4);
        sleep(100);
        closeBothClaws();
        sleep(300);

        //STEP 5 -- Move back to make sure pixels are in claw
        xDrive.followTrajectorySequence(trajSeq5);
        sleep(100);
        setWristPosition(WRIST_VERTICAL);

        //STEP 6 -- Going to drop white Pixels
        xDrive.followTrajectorySequence(trajBackToDropWhitePixles);
        sleep(100);
//        moveArmToPosition(DEFAULT_DROP_ARM_POSITION - 80);
//        sleep(1400);

        moveArmToPosition(DEFAULT_DROP_ARM_POSITION - 300);
        sleep(1000);
        moveArmToPosition(DEFAULT_DROP_ARM_POSITION + 27, 0.3);
        sleep(600);

        openBothClaws();
        sleep(200);
    }
}
