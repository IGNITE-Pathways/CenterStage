package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.MoveRobot.BACKWARD;
import static org.firstinspires.ftc.teamcode.MoveRobot.STRAFE_LEFT;
import static org.firstinspires.ftc.teamcode.MoveRobot.STRAFE_RIGHT;
import static org.firstinspires.ftc.teamcode.MoveRobot.TANK_TURN_LEFT;
import static org.firstinspires.ftc.teamcode.MoveRobot.TANK_TURN_RIGHT;
import static org.firstinspires.ftc.teamcode.XBot.ARM_POSITION_HIGH;
import static org.firstinspires.ftc.teamcode.XBot.ARM_POSITION_UP;
import static org.firstinspires.ftc.teamcode.XBot.MAX_ARM_POSITION;
import static org.firstinspires.ftc.teamcode.XBot.MAX_AUTO_SPEED;
import static org.firstinspires.ftc.teamcode.XBot.MAX_AUTO_STRAFE;
import static org.firstinspires.ftc.teamcode.XBot.MAX_AUTO_TURN;
import static org.firstinspires.ftc.teamcode.XBot.MIN_ARM_POSITION;
import static org.firstinspires.ftc.teamcode.XBot.SPEED_GAIN;
import static org.firstinspires.ftc.teamcode.XBot.STRAFE_GAIN;
import static org.firstinspires.ftc.teamcode.XBot.TURN_GAIN;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.Range;

@Autonomous(name = "Auto Blue Far Left", group = "Concept")
public class AutoBlueFarLeft extends XBotOpMode implements AutoOpMode {
    Alliance alliance = Alliance.BLUE;
    DistanceFromBackdrop distanceFromBackdrop = DistanceFromBackdrop.FAR;
    Parking parking = Parking.LEFT;
    boolean spikeMarkPixelDropped = false;
    boolean aTagPixelDropped = false;
    boolean arrivedAtBackDropTagPosition = false;

    int YELLOW_PIXEL_OFFSET = 5;
    @Override
    public void runOpMode() {
        // Initialize hardware
        initializeAuto(DEBUG);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                if (!teamPropDetectionCompleted) {
                    detectTeamPropAndSwitchCameraToAprilTag();
                } else {
                    telemetry.addData("SpikeMark", spikeMark + ", confidence" + detectionConfidence);
                    telemetry.addData(">", "Robot Heading = %4.0f", getHeading());
                    telemetry.update();

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
                        spikeMarkPixelDropped = true;
                        visionPortal.resumeStreaming();
                        stopDriveMotors();
                        stopDriveRunUsingEncoder();
                        autoDrive = true; //drive in reverse
                    }
                    if (!aTagPixelDropped) {
                        if (!arrivedAtBackDropTagPosition) {
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
                                double headingError = desiredTagDetectionObj.ftcPose.bearing + YELLOW_PIXEL_OFFSET;
                                double yawError = desiredTagDetectionObj.ftcPose.yaw;

                                // Use the speed and turn "gains" to calculate how we want the robot to move.
                                double drive = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                                double strafe = Range.clip(headingError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);
                                double yawTurn = Range.clip(-yawError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);

                                moveRobot(drive, strafe, yawTurn);

                                if ((rangeError < 0.5) && (rangeError > -0.5)) {
                                    arrivedAtBackDropTagPosition = true;
//                                fixRobotYaw(90);
                                    visionPortal.close();
                                    stopDriveMotors();
                                }
                            } else {
                                telemetry.addData("Looking for Tag", desiredTagId + "\n");
                                telemetry.update();
//                            moveRobot(20, STRAFE_LEFT);
//                            fixRobotYaw(90);
                            }
                        } else {
                            telemetry.addData("Arrived", "Dropping Pixel");
                            telemetry.update();
                            moveArmToPosition(MAX_ARM_POSITION);
                            openLeftClaw();
                            sleep(100);
                            //Park Now
                            moveArmToPosition(200);
                            moveRobot(900, STRAFE_RIGHT);
                            moveRobot(100, BACKWARD);
                            moveArmToPosition(MIN_ARM_POSITION);
                            aTagPixelDropped = true;
                        }
                    }
                }
                sleep(10);
            }
        }
        stopRobot();
    }
}
