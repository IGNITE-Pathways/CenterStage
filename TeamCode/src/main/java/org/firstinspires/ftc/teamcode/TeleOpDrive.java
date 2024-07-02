package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.XBot.ARM_PICK_POSITION;
import static org.firstinspires.ftc.teamcode.XBot.ARM_POSITION_HIGH;
import static org.firstinspires.ftc.teamcode.XBot.ARM_POSITION_ROBOT_HANGING;
import static org.firstinspires.ftc.teamcode.XBot.DEFAULT_DROP_ARM_POSITION;
import static org.firstinspires.ftc.teamcode.XBot.FULL_CIRCLE;
import static org.firstinspires.ftc.teamcode.XBot.LEFT_CLAW_CLOSE_POSITION;
import static org.firstinspires.ftc.teamcode.XBot.LEFT_CLAW_OPEN_POSITION;
import static org.firstinspires.ftc.teamcode.XBot.MAX_ARM_POSITION;
import static org.firstinspires.ftc.teamcode.XBot.MAX_AUTO_SPEED;
import static org.firstinspires.ftc.teamcode.XBot.MAX_AUTO_STRAFE;
import static org.firstinspires.ftc.teamcode.XBot.MAX_AUTO_TURN;
import static org.firstinspires.ftc.teamcode.XBot.MAX_WRIST_POS;
import static org.firstinspires.ftc.teamcode.XBot.MIN_ARM_POSITION;
import static org.firstinspires.ftc.teamcode.XBot.MIN_WRIST_POS;
import static org.firstinspires.ftc.teamcode.XBot.RIGHT_CLAW_CLOSE_POSITION;
import static org.firstinspires.ftc.teamcode.XBot.RIGHT_CLAW_OPEN_POSITION;
import static org.firstinspires.ftc.teamcode.XBot.SPEED_GAIN;
import static org.firstinspires.ftc.teamcode.XBot.STRAFE_GAIN;
import static org.firstinspires.ftc.teamcode.XBot.TELEOP_DESIRED_DISTANCE;
import static org.firstinspires.ftc.teamcode.XBot.TURN_GAIN;
import static org.firstinspires.ftc.teamcode.XBot.WRIST_FLAT_TO_GROUND;
import static org.firstinspires.ftc.teamcode.XBot.WRIST_VERTICAL;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import java.util.concurrent.TimeUnit;

/*
 * This is the main manual OpMode
 */
@TeleOp(name = "Manual Drive", group = "Concept")
public class TeleOpDrive extends XBotOpMode {
    double leftClawPosition = LEFT_CLAW_OPEN_POSITION;
    double rightClawPosition = RIGHT_CLAW_OPEN_POSITION;

    int saved_drop_arm_position = DEFAULT_DROP_ARM_POSITION;
    int armPosition = ARM_PICK_POSITION;

    @Override
    public void runOpMode() {

        // Initialize April Tag Variables
        boolean aprilTagFound = false;
        boolean lookForAprilTag = false;

        // Initialize all motors and sensors
        initialize();
        resetWristAndClawPosition();

        SampleMecanumDrive mecanumDrive = new SampleMecanumDrive(hardwareMap);
        mecanumDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mecanumDrive.setPoseEstimate(new Pose2d(10, 10, Math.toRadians(90)));

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Arm: Position", leftArmMotor.getCurrentPosition() + ": " + rightArmMotor.getCurrentPosition());
        telemetry.addData("Claw: Position", leftClaw.getPosition() + ": " + rightClaw.getPosition());
        telemetry.addData("Wrist: Position", leftWrist.getPosition() + " : " + rightWrist.getPosition());
        telemetry.update();

        waitForStart();
        runtime.reset();

        // Change Game Mode on PLAY
        changeGameMode(GameMode.GOING_TO_PICK_PIXELS);
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                mecanumDrive.update();

                // Retrieve your pose
                Pose2d myPose = mecanumDrive.getPoseEstimate();
                droneServo.setPosition(DRONE_LOADED);
                autoDrive = aprilTagFound && lookForAprilTag && gamepad1.left_bumper;

                if (runtime.time(TimeUnit.SECONDS) >= 90) {
                    if (gamepad1.circle) {
                        droneServo.setPosition(DRONE_LAUNCHED);
                        sleep(500);
                    }
                }
                if (gamepad2.dpad_left) {
                    saved_drop_arm_position = armPosition;
                }
                if (gamepad2.x) {
                    changeGameMode(GameMode.PICKING_PIXELS);
                    leftPixelInClaw = false;
                    rightPixelInClaw = false;
                }

                if (gamepad2.left_bumper) {
                    //OPEN LEFT CLAW
                    openLeftClaw();
                    leftPixelInClaw = false;
                } else if (gamepad2.left_trigger > 0.1) {
                    pickLeftPixel();
                }

                if (gamepad2.right_bumper) {
                    //OPEN RIGHT CLAW
                    openRightClaw();
                    rightPixelInClaw = false;
                } else if (gamepad2.right_trigger > 0.1) {
                    pickRightPixel();
                }

                int moveArmBy = (int) -(gamepad2.left_stick_y * 40);
                if (moveArmBy != 0) {
                    armPosition += moveArmBy;
                    armPosition = Math.max(MIN_ARM_POSITION, armPosition); // cannot go below MIN_ARM_POSITION
                    armPosition = Math.min(MAX_ARM_POSITION, armPosition); // cannot go above MAX_ARM_POSITION
                    moveArmToPosition(armPosition);
                }

                double moveWristBy = -(gamepad2.right_stick_y / 100.0);
                if (moveWristBy != 0) {
                    wristPosition += moveWristBy;
                    wristPosition = Math.max(MIN_WRIST_POS, wristPosition); // cannot go below MIN_ARM_POSITION
                    wristPosition = Math.min(MAX_WRIST_POS, wristPosition); // cannot go above MAX_ARM_POSITION
                    setWristPosition(wristPosition);
                }
                //Hanging mode
                if (gamepad2.dpad_up) {
                    changeGameMode(GameMode.HANGING);
                }

                // POV Mode uses left joystick to go forward & yawTurn, and right joystick to rotate.
                double drive = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
                double strafe = -gamepad1.left_stick_x;
                double yawTurn = -gamepad1.right_stick_x;

                switch (gameMode) {
                    case GOING_TO_PICK_PIXELS:
                        // ARM = AUTO, WRIST = AUTO, CLAWS = AUTO
                        if (gameModeChanged) {
                            goToGoingToPickPixelPosition();
//                            setWristPosition(WRIST_VERTICAL);
                            gameModeChanged = Boolean.FALSE;
                        }
                        break;
                    case PICKING_PIXELS:
                        // ARM = AUTO, WRIST = AUTO, CLAWS = OPEN or CLOSE
                        if (gameModeChanged) {
//                            driveSpeed = SPEED_WHEN_PICKING_PIXELS; //Slow down, need precision to pick pixels
                            goToPickPixelPosition();
                            gameModeChanged = Boolean.FALSE;
                        }
                        if (gamepad2.circle) pickPixels(); //Manual Grab
                        break;
                    case GOING_TO_DROP_PIXELS:
                        // WRIST = AUTO (also allows Manual), CLAW = CLOSE POSITION (holding pixels)
                        // Robot will be looking for april tag and will switch mode automatically once found
                        lookForAprilTag = true;

                        //Look for April Tag(s)
                        aprilTagFound = detectAnyAprilTag();

                        if (gameModeChanged) {
                            gameModeChanged = Boolean.FALSE;
                        }
                        //Y Button = Manual override only required if April Tag Nav doesn't work
                        //Pressing Y Button will skill April Tag Navigation
                        if (gamepad2.y) changeGameMode(GameMode.DROPPING_PIXELS);
                        break;
                    case APRIL_TAG_NAVIGATION:
                        if (gameModeChanged) {
                            gameModeChanged = Boolean.FALSE;
                        }
                        aprilTagFound = detectAnyAprilTag();
                        if (aprilTagFound && lookForAprilTag && gamepad1.left_bumper) {
                            double rangeError = (desiredTagDetectionObj.ftcPose.range - TELEOP_DESIRED_DISTANCE);
                            double headingError = desiredTagDetectionObj.ftcPose.bearing;
                            double yawError = desiredTagDetectionObj.ftcPose.yaw;

                            // Use the speed and strafe "gains" to calculate how we want the robot to move.
                            drive = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                            strafe = Range.clip(headingError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);
                            yawTurn = Range.clip(-yawError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);

                            //Y Button = Manual override only required if April Tag Nav doesn't work
                            telemetry.addData("Range Error", rangeError);
                            if (((rangeError < 0.5) && (rangeError > -0.5)) || gamepad2.y) {
                                changeGameMode(GameMode.DROPPING_PIXELS);
                            }
                        }
                        //break out of april tag nav if Y is pressed
                        if (gamepad2.y) changeGameMode(GameMode.DROPPING_PIXELS);
                        break;
                    case DROPPING_PIXELS:
                        // ARM = AUTO, WRIST = NONE, CLAWS = OPEN
                        lookForAprilTag = false;
//                        driveSpeed = SPEED_WHEN_DROPPING_PIXELS; //Robot should not move, except yawTurn if needed

                        if (gameModeChanged) {
                            gameModeChanged = Boolean.FALSE;
                            // Move Arm to back board -- only once
                            armPosition = saved_drop_arm_position - 40;
                            moveArmToPosition(armPosition);
                            sleep(200);
                        }
                        //User Action :: Press O (or if distance sensor is close) to drop pixels
//                        if (gamepad2.circle || isDistanceSensorClose()) {
                        if (gamepad2.circle) {
                            dropPixels();
                            sleep(200);

                            // Move robot back a bit
//                            moveRobotBack();

                            // Change Mode, let's go get next set of pixels
                            changeGameMode(GameMode.GOING_TO_PICK_PIXELS);
                        }
                        break;
                    case GOING_TO_HANG:
                        // ARM = AUTO, WRIST = AUTO, CLAW = AUTO
                        break;
                    case HANGING:
                        // ARM = AUTO and ENGAGED, WRIST = AUTO, CLAW = AUTO
                        //Move Arm to perfect position
                        if (gameModeChanged) {
                            gameModeChanged = Boolean.FALSE;
                            armPosition = ARM_POSITION_ROBOT_HANGING;
                            moveArmToPosition(armPosition);
                            sleep(200);
                        }
                        if (gamepad2.dpad_down) {
                            //Hang the Robot
                            armPosition = 300;
                            moveArmToPosition(armPosition);
                        }
                        break;
                }

                mecanumDrive.setWeightedDrivePower(
                        new Pose2d(
                                drive,
                                strafe,
                                yawTurn
                        )
                );

                mecanumDrive.update();

                // Show the elapsed game time and wheel power.
                telemetry.addData("Status", "Run Time: " + runtime);
                telemetry.addData("GameMode", gameMode);
                telemetry.addData("AprilTag Nav: ", aprilTagFound + " " + lookForAprilTag);

                if (autoDrive) {
                    telemetry.addData("Auto", "Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, yawTurn, strafe);
                } else {
                    telemetry.addData("Manual", "Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, yawTurn, strafe);
                }
                telemetry.addData("Arm: Left=", leftArmMotor.getCurrentPosition() + ", right=" + rightArmMotor.getCurrentPosition());
                telemetry.addData("Arm Angle", ((leftArmMotor.getCurrentPosition() * 360) / FULL_CIRCLE));
                telemetry.addData("Arm: Position", armPosition);
                telemetry.addData("Wrist: Position", wristPosition);
                telemetry.addData("Claw: Left", leftClawPosition + " Right=" + rightClawPosition);
                telemetry.addData("x", myPose.getX() + ", y " + myPose.getY() + ", heading " + myPose.getHeading());
                telemetry.update();
                idle();
            }
        }
        visionPortal.close();
    }

    private void waitAndMoveArmAndResetDistance() {
        sleep(500);
        // Move Arm up to remove friction and get clearance the ground
        armPosition = ARM_POSITION_HIGH + 50;
        changeGameMode(GameMode.GOING_TO_DROP_PIXELS);
        moveArmToPosition(armPosition);
        wristPosition = WRIST_VERTICAL;
        setWristPosition(wristPosition);
    }

    private void pickPixels() {
        leftClaw.setPosition(LEFT_CLAW_CLOSE_POSITION);
        rightClaw.setPosition(RIGHT_CLAW_CLOSE_POSITION);
        leftPixelInClaw = true;
        rightPixelInClaw = true;
        waitAndMoveArmAndResetDistance();
    }

    private void pickLeftPixel() {
        leftClaw.setPosition(LEFT_CLAW_CLOSE_POSITION);
        leftPixelInClaw = true;
        if (rightPixelInClaw) {
            waitAndMoveArmAndResetDistance();
        }
    }

    private void pickRightPixel() {
        rightClaw.setPosition(RIGHT_CLAW_CLOSE_POSITION);
        rightPixelInClaw = true;
        if (leftPixelInClaw) {
            waitAndMoveArmAndResetDistance();
        }
    }

    private void dropPixels() {
        leftClaw.setPosition(LEFT_CLAW_OPEN_POSITION);
        rightClaw.setPosition(RIGHT_CLAW_OPEN_POSITION);
        sleep(500);
        leftPixelInClaw = false;
        rightPixelInClaw = false;
        changeGameMode(GameMode.GOING_TO_PICK_PIXELS);
    }

    private void goToPickPixelPosition() {
        setClawsToPixelPickPosition();
        wristPosition = WRIST_FLAT_TO_GROUND;
        setWristPosition(wristPosition);
        moveArmToPosition(ARM_PICK_POSITION);
    }

    private void goToGoingToPickPixelPosition() {
        setClawsToPixelPickPosition();
        wristPosition = WRIST_VERTICAL;
        setWristPosition(wristPosition);
        moveArmToPosition(ARM_POSITION_HIGH);
    }

    private void setClawsToPixelPickPosition() {
        leftClaw.setPosition(LEFT_CLAW_OPEN_POSITION);
        rightClaw.setPosition(RIGHT_CLAW_OPEN_POSITION);
    }

//    public void moveRobotBack() {
//        leftFront.setPower(0.2);
//        rightFront.setPower(0.2);
//        leftBack.setPower(0.2);
//        rightBack.setPower(0.2);
//        sleep(500);
//        leftFront.setPower(0);
//        rightFront.setPower(0);
//        leftBack.setPower(0);
//        rightBack.setPower(0);
//    }

}
