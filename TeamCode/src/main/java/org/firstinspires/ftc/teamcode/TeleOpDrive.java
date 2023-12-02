package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.XBot.ARM_HOLD_SPEED;
import static org.firstinspires.ftc.teamcode.XBot.ARM_PICK_POSITION;
import static org.firstinspires.ftc.teamcode.XBot.ARM_POSITION_HIGH;
import static org.firstinspires.ftc.teamcode.XBot.ARM_POSITION_ROBOT_HANGING;
import static org.firstinspires.ftc.teamcode.XBot.ARM_SPEED;
import static org.firstinspires.ftc.teamcode.XBot.FULL_CIRCLE;
import static org.firstinspires.ftc.teamcode.XBot.LEFT_CLAW_CLOSE_POSITION;
import static org.firstinspires.ftc.teamcode.XBot.LEFT_CLAW_OPEN_POSITION;
import static org.firstinspires.ftc.teamcode.XBot.MAX_ARM_POSITION;
import static org.firstinspires.ftc.teamcode.XBot.MAX_AUTO_SPEED;
import static org.firstinspires.ftc.teamcode.XBot.MAX_AUTO_STRAFE;
import static org.firstinspires.ftc.teamcode.XBot.MAX_AUTO_TURN;
import static org.firstinspires.ftc.teamcode.XBot.MAX_SPEED;
import static org.firstinspires.ftc.teamcode.XBot.MIN_ARM_POSITION;
import static org.firstinspires.ftc.teamcode.XBot.RIGHT_CLAW_CLOSE_POSITION;
import static org.firstinspires.ftc.teamcode.XBot.RIGHT_CLAW_OPEN_POSITION;
import static org.firstinspires.ftc.teamcode.XBot.SPEED_GAIN;
import static org.firstinspires.ftc.teamcode.XBot.SPEED_WHEN_DROPPING_PIXELS;
import static org.firstinspires.ftc.teamcode.XBot.SPEED_WHEN_ON_APRIL_TAG_NAV;
import static org.firstinspires.ftc.teamcode.XBot.SPEED_WHEN_PICKING_PIXELS;
import static org.firstinspires.ftc.teamcode.XBot.STARTING_LEFT_CLAW_POS;
import static org.firstinspires.ftc.teamcode.XBot.STARTING_RIGHT_CLAW_POS;
import static org.firstinspires.ftc.teamcode.XBot.STARTING_WRIST_POSITION;
import static org.firstinspires.ftc.teamcode.XBot.STRAFE_GAIN;
import static org.firstinspires.ftc.teamcode.XBot.TELEOP_DESIRED_DISTANCE;
import static org.firstinspires.ftc.teamcode.XBot.TURN_GAIN;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Queue;

/*
 * This is the main manual OpMode
 */
@TeleOp(name = "Manual Drive", group = "Concept")
public class TeleOpDrive extends XBotOpMode {
    double leftClawPosition = STARTING_LEFT_CLAW_POS;
    double rightClawPosition = STARTING_RIGHT_CLAW_POS;
    int armPosition = ARM_PICK_POSITION;
    //Robot Speed
    double driveSpeed = MAX_SPEED;
    // Declare OpMode members for each of the 4 motors.
    Queue<Double> distanceQueue = new SizeLimitedQueue<>(10);
    double calculatedDistance = DistanceSensor.distanceOutOfRange;
    @Override
    public void runOpMode() {

        // Initialize April Tag Variables
        boolean aprilTagFound = false;
        boolean lookForAprilTag = false;

        // Initialize all motors and sensors
        initialize();
        resetWristAndClawPosition();

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Arm: Target", armPosition);
        telemetry.addData("Arm: Left Motor Position", leftArmMotor.getCurrentPosition() + "  busy=" + leftArmMotor.isBusy());
        telemetry.addData("Arm: Right Motor Position", rightArmMotor.getCurrentPosition() + "  busy=" + rightArmMotor.isBusy());
        telemetry.addData("Wrist: Position", wristServo.getPosition());
        telemetry.update();

        waitForStart();
        runtime.reset();

        // Change Game Mode on PLAY
        changeGameMode(GameMode.GOING_TO_PICK_PIXELS);
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                autoDrive = aprilTagFound && lookForAprilTag && gamepad1.left_bumper;
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
                        driveSpeed = MAX_SPEED;
                        if (gameModeChanged) {
                            armPosition = goToGoingToPickPixelPosition();
                            wristServo.setPosition(STARTING_WRIST_POSITION);
                            gameModeChanged = Boolean.FALSE;
                            resetDistanceSensor();
                        }
                        break;
                    case PICKING_PIXELS:
                        // ARM = AUTO, WRIST = AUTO, CLAWS = OPEN or CLOSE
                        if (gameModeChanged) {
                            driveSpeed = SPEED_WHEN_PICKING_PIXELS; //Slow down, need precision to pick pixels
                            armPosition = goToPickPixelPosition();
                            gameModeChanged = Boolean.FALSE;
                        }
                        if (gamepad2.circle) pickPixels(); //Manual Grab
                        if (isLeftPixelInReach()) pickLeftPixel(); //Auto Grab
                        if (isRightPixelInReach()) pickRightPixel(); //Auto Grab
                        break;
                    case GOING_TO_DROP_PIXELS:
                        // WRIST = AUTO (also allows Manual), CLAW = CLOSE POSITION (holding pixels)
                        driveSpeed = MAX_SPEED; //Full speed from front to back

                        // Manual Arm Movement is allowed, but not required
//                        int moveArmBy = (int) -(gamepad2.left_stick_y * 40);
//                        if (moveArmBy != 0) {
//                            armPosition += moveArmBy;
//                            armPosition = Math.max(MIN_ARM_POSITION, armPosition); // cannot go below MIN_ARM_POSITION
//                            armPosition = Math.min(MAX_ARM_POSITION, armPosition); // cannot go above MAX_ARM_POSITION
//                            moveArmToPosition(armPosition);
//                        }

                        // Robot will be looking for april tag and will switch mode automatically once found
                        lookForAprilTag = true;
                        // drive using manual POV Joystick mode.  Slow things down to make the robot more controlable.
//                        drive = -gamepad1.left_stick_y / 2.0;  // Reduce drive rate to 50%.
//                        strafe = -gamepad1.left_stick_x / 2.0;  // Reduce yawTurn rate to 50%.
//                        yawTurn = -gamepad1.right_stick_x / 3.0;  // Reduce strafe rate to 33%.

                        //Look for April Tag(s)
                        aprilTagFound = detectAnyAprilTag();

                        if (gameModeChanged) {
                            gameModeChanged = Boolean.FALSE;
                            resetDistanceSensor();
                        }
                        updateDistance();
                        //Y Button = Manual override only required if April Tag Nav doesn't work
                        //Pressing Y Button will skill April Tag Navigation
                        if (gamepad2.y) changeGameMode(GameMode.DROPPING_PIXELS);
                        break;
                    case APRIL_TAG_NAVIGATION:
                            if (gameModeChanged) {
                                gameModeChanged = Boolean.FALSE;
                            }
                            updateDistance();
                            aprilTagFound = detectAnyAprilTag();
                            if (aprilTagFound && lookForAprilTag && gamepad1.left_bumper) {
                                driveSpeed = SPEED_WHEN_ON_APRIL_TAG_NAV;
                                double rangeError = (desiredTagDetectionObj.ftcPose.range - TELEOP_DESIRED_DISTANCE);
                                double headingError = desiredTagDetectionObj.ftcPose.bearing;
                                double yawError = desiredTagDetectionObj.ftcPose.yaw;

                                // Use the speed and strafe "gains" to calculate how we want the robot to move.
                                drive = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                                strafe = Range.clip( headingError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);
                                yawTurn = Range.clip(-yawError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);

                                //Y Button = Manual override only required if April Tag Nav doesn't work
                                if (((rangeError < 0.1) && (rangeError > -0.1)) || gamepad2.y) {
                                    changeGameMode(GameMode.DROPPING_PIXELS);
                                }
                            }
                            //break out of april tag nav if Y is pressed
                            if (gamepad2.y) changeGameMode(GameMode.DROPPING_PIXELS);
                        break;
                    case DROPPING_PIXELS:
                        // ARM = AUTO, WRIST = NONE, CLAWS = OPEN
                        lookForAprilTag = false;
                        driveSpeed = SPEED_WHEN_DROPPING_PIXELS; //Robot should not move, except yawTurn if needed
                        updateDistance();

                        if (gameModeChanged) {
                            gameModeChanged = Boolean.FALSE;
                            // Move Arm to back board -- only once
                            armPosition = MAX_ARM_POSITION;
                            moveArmToPosition(armPosition);
                            sleep(200);
                        }
                        //User Action :: Press O (or if distance sensor is close) to drop pixels
                        if (gamepad2.circle || isDistanceSensorClose()) {
                            dropPixels();
                            sleep(200);

                            // Move robot back a bit
                            moveRobotBack();
                            resetDistanceSensor();

                            // Change Mode, let's go get next set of pixels
                            changeGameMode(GameMode.GOING_TO_PICK_PIXELS);
                        }
                        break;
                    case GOING_TO_HANG:
                        // ARM = AUTO, WRIST = AUTO, CLAW = AUTO
                        driveSpeed = MAX_SPEED;
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

                moveRobot(drive * driveSpeed, strafe * driveSpeed, yawTurn * driveSpeed);

                // Show the elapsed game time and wheel power.
                telemetry.addData("Status", "Run Time: " + runtime.toString());
                telemetry.addData("GameMode", gameMode);
                telemetry.addData("AprilTag Nav: ", aprilTagFound + " " + lookForAprilTag);

                if (autoDrive) {
                    telemetry.addData("Auto", "Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, yawTurn, strafe);
                } else {
                    telemetry.addData("Manual", "Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, yawTurn, strafe);
                }
                telemetry.addData("Arm: Left Motor Position", leftArmMotor.getCurrentPosition() + "  busy=" + leftArmMotor.isBusy());
                telemetry.addData("Arm: Right Motor Position", rightArmMotor.getCurrentPosition() + "  busy=" + rightArmMotor.isBusy());

                telemetry.addData("Arm Angle", ((leftArmMotor.getCurrentPosition() * 360) / FULL_CIRCLE));
                telemetry.addData("Arm: Position", armPosition);
                telemetry.addData("Wrist: Position", wristPosition);
                telemetry.addData("Claw: Left", leftClawPosition + " Right=" + rightClawPosition);
                telemetry.addData("Touch: Left", leftPixelInClaw + " Right=" + rightPixelInClaw);
                if (calculatedDistance != DistanceSensor.distanceOutOfRange) {
                    telemetry.addData("Distance", "%.01f mm, %.01f mm", sensorDistance.getDistance(DistanceUnit.MM), calculatedDistance);
                }
                telemetry.update();
                idle();
            }
        }
        visionPortal.close();
    }

    private boolean isRightPixelInReach() {
        //return rightClawTouchSensor.isPressed();
        return (rightClawDistance.getDistance(DistanceUnit.MM) < 20);
    }

    private boolean isLeftPixelInReach() {
        //return leftClawTouchSensor.isPressed()
        return (leftClawDistance.getDistance(DistanceUnit.MM) < 20);
    }


    private void waitAndMoveArmAndResetDistance() {
        sleep(500);
        // Move Arm up to remove friction and get clearance the ground
        armPosition = ARM_POSITION_HIGH + 50;
        moveArmToPosition(armPosition);
        changeGameMode(GameMode.GOING_TO_DROP_PIXELS);
        resetDistanceSensor();
    }
    private void pickPixels() {
        leftClawServo.setPosition(LEFT_CLAW_CLOSE_POSITION);
        rightClawServo.setPosition(RIGHT_CLAW_CLOSE_POSITION);
        leftPixelInClaw = true;
        rightPixelInClaw = true;
        waitAndMoveArmAndResetDistance();
    }
    private void pickLeftPixel() {
        leftClawServo.setPosition(LEFT_CLAW_CLOSE_POSITION);
        leftPixelInClaw = true;
        if (rightPixelInClaw) {
            waitAndMoveArmAndResetDistance();
        }
    }
    private void pickRightPixel() {
        rightClawServo.setPosition(RIGHT_CLAW_CLOSE_POSITION);
        rightPixelInClaw = true;
        if (leftPixelInClaw) {
            waitAndMoveArmAndResetDistance();
        }
    }

    private void resetDistanceSensor() {
        distanceQueue.clear();
        calculatedDistance = DistanceSensor.distanceOutOfRange;
    }

    private boolean isDistanceSensorClose() {
        return calculatedDistance < 40;
    }

    private void dropPixels() {
        leftClawServo.setPosition(LEFT_CLAW_OPEN_POSITION);
        rightClawServo.setPosition(RIGHT_CLAW_OPEN_POSITION);
        sleep(500);
        leftPixelInClaw = false;
        rightPixelInClaw = false;
        changeGameMode(GameMode.GOING_TO_PICK_PIXELS);
    }

    private int goToPickPixelPosition() {
        setClawsToPixelPickPosition();
        return moveArmToPosition(ARM_PICK_POSITION);
    }

    private int goToGoingToPickPixelPosition() {
        setClawsToPixelPickPosition();
        return moveArmToPosition(ARM_POSITION_HIGH);
    }
    private void setClawsToPixelPickPosition() {
        leftClawServo.setPosition(LEFT_CLAW_OPEN_POSITION);
        rightClawServo.setPosition(RIGHT_CLAW_OPEN_POSITION);
    }

    int moveArmToPosition(int armPosition) {
        int savePos = armPosition;
        // set motors to run forward for 5000 encoder counts.
        leftArmMotor.setTargetPosition(armPosition);
        rightArmMotor.setTargetPosition(armPosition);

        // set motors to run to target encoder position and stop with brakes on.
        leftArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftArmMotor.setPower(ARM_SPEED);
        rightArmMotor.setPower(ARM_SPEED);

        while (opModeIsActive() && rightArmMotor.isBusy())   //leftMotor.getCurrentPosition() < leftMotor.getTargetPosition())
        {
            armPosition = leftArmMotor.getCurrentPosition();
            wristPosition = getWristPosition(armPosition);
            setWristPosition(wristPosition);
            telemetry.addData("Arm: Target", savePos);
            telemetry.addData("Arm: Left Motor Position", leftArmMotor.getCurrentPosition() + "  busy=" + leftArmMotor.isBusy());
            telemetry.addData("Arm: Right Motor Position", rightArmMotor.getCurrentPosition() + "  busy=" + rightArmMotor.isBusy());
            telemetry.update();
        }

        leftArmMotor.setPower(ARM_HOLD_SPEED);
        rightArmMotor.setPower(ARM_HOLD_SPEED);
        
        return armPosition;
    }

    public void moveRobotBack() {
        leftFront.setPower(0.2);
        rightFront.setPower(0.2);
        leftBack.setPower(0.2);
        rightBack.setPower(0.2);
        sleep(500);
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
    }

    private void addNewDistanceValue(double distance) {
        distanceQueue.add(distance);
    }

    private void updateDistance() {
        double distance = sensorDistance.getDistance(DistanceUnit.MM);
        if ((distance != DistanceSensor.distanceOutOfRange)) {
            addNewDistanceValue(distance);
        }
        calculatedDistance = calculateDistance();
    }

    private Double calculateDistance() {
        //Must have multiple entries
        if (distanceQueue.size() < 2) return DistanceSensor.distanceOutOfRange;
        Double total = 0.0;
        for (Double d : distanceQueue) {
            total += d;
        }
        return total/distanceQueue.size();
    }

}
