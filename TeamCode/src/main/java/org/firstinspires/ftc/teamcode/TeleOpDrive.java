package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.XBot.*;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.Queue;
import java.util.concurrent.TimeUnit;

/*
 * This is the main manual OpMode
 */
@TeleOp(name = "Manual Drive", group = "Concept")
public class TeleOpDrive extends XBotOpMode {

    double wristPosition = STARTING_WRIST_POSITION;
    double leftClawPosition = STARTING_CLAW_POS;
    double rightClawPosition = STARTING_CLAW_POS;

    //Robot Speed
    double driveSpeed = MAX_SPEED;

    // Declare OpMode members for each of the 4 motors.
    private final ElapsedTime runtime = new ElapsedTime();
    Queue<Double> distanceQueue = new SizeLimitedQueue<>(10);
    double calculatedDistance = DistanceSensor.distanceOutOfRange;

    @Override
    public void runOpMode() {
        int armPosition = 0;

        // Initialize April Tag Variables
        boolean aprilTagFound = false;
        boolean lookForAprilTag = false;
        boolean autoDrive;

        // Initialize all motors and sensors
        initialize();
        resetWristAndClawPosition();

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // Change Game Mode on PLAY
        changeGameMode(GameMode.GOING_TO_PICK_PIXELS);

        while (opModeIsActive()) {
            autoDrive = aprilTagFound && lookForAprilTag;
            if (gamepad2.x) changeGameMode(GameMode.PICKING_PIXELS);

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double drive = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double turn = -gamepad1.left_stick_x;
            double strafe = -gamepad1.right_stick_x;

            switch (gameMode) {
                case GOING_TO_PICK_PIXELS:
                    // ARM = AUTO, WRIST = AUTO, CLAWS = AUTO
                    driveSpeed = MAX_SPEED;
                    if (gameModeChanged) {
                        armPosition = goToGoingToPickPixelPosition();
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
                    if (leftClawTouchSensor.isPressed()) pickLeftPixel(); //Auto Grab
                    if (rightClawTouchSensor.isPressed()) pickRightPixel(); //Auto Grab
                    break;
                case GOING_TO_DROP_PIXELS:
                    // WRIST = AUTO (also allows Manual), CLAW = CLOSE POSITION (holding pixels)
                    driveSpeed = MAX_SPEED; //Full speed from front to back

                    // Manual Wrist Movement is allowed, but not required
                    armPosition += (int) -(gamepad2.left_stick_y * 20);
                    armPosition = Math.max(MIN_ARM_POSITION, armPosition); // cannot go below MIN_ARM_POSITION
                    armPosition = Math.min(MAX_ARM_POSITION, armPosition); // cannot go above MAX_ARM_POSITION
                    moveArmToPosition(armPosition);

                    // Robot will be looking for april tag and will switch mode automatically once found
                    lookForAprilTag = true;
                    // drive using manual POV Joystick mode.  Slow things down to make the robot more controlable.
                    drive = -gamepad1.left_stick_y / 2.0;  // Reduce drive rate to 50%.
                    strafe = -gamepad1.left_stick_x / 2.0;  // Reduce strafe rate to 50%.
                    turn = -gamepad1.right_stick_x / 3.0;  // Reduce turn rate to 33%.

                    //Look for April Tag(s)
                    aprilTagFound = detectAprilTags();

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
                    if (aprilTagFound && lookForAprilTag) {
                        driveSpeed = SPEED_WHEN_ON_APRIL_TAG_NAV;
                        if (gameModeChanged) {
                            gameModeChanged = Boolean.FALSE;
                        }
                        updateDistance();
                        aprilTagFound = detectAprilTags();
                        double rangeError = (desiredTag.ftcPose.range - DESIRED_DISTANCE);
                        double headingError = desiredTag.ftcPose.bearing;
                        double yawError = desiredTag.ftcPose.yaw;

                        // Use the speed and turn "gains" to calculate how we want the robot to move.
                        drive = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                        turn = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
                        strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);

                        //Y Button = Manual override only required if April Tag Nav doesn't work
                        if (((rangeError < 0.02) && (rangeError > -0.02)) || gamepad2.y) {
                            changeGameMode(GameMode.DROPPING_PIXELS);
                        }
                    }
                    break;
                case DROPPING_PIXELS:
                    // ARM = AUTO, WRIST = NONE, CLAWS = OPEN
                    lookForAprilTag = false;
                    driveSpeed = SPEED_WHEN_DROPPING_PIXELS; //Robot should not move, except strafe if needed
                    updateDistance();

                    if (gameModeChanged) {
                        gameModeChanged = Boolean.FALSE;
                        // Move Arm to back board -- only once
                        armPosition = MAX_ARM_POSITION;
                        moveArmToPosition(armPosition);
                        sleep(400);
                    }
                    //User Action :: Press O (or if distance sensor is close) to drop pixels
                    if (gamepad2.circle || isDistanceSensorClose()) {
                        dropPixels();
                        sleep(400);

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
                    break;
            }

            moveRobot(drive * driveSpeed, turn * driveSpeed, strafe * driveSpeed);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("GameMode", gameMode);
            if (autoDrive) {
                telemetry.addData("Auto", "Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
            } else {
                telemetry.addData("Manual", "Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
            }
            telemetry.addData("Arm: Left Motor Position", leftArmMotor.getCurrentPosition() + "  busy=" + leftArmMotor.isBusy());
            telemetry.addData("Arm: Right Motor Position", rightArmMotor.getCurrentPosition() + "  busy=" + rightArmMotor.isBusy());
            telemetry.addData("Arm Angle", ((leftArmMotor.getCurrentPosition() * 360) / FULL_CIRCLE));
            telemetry.addData("Wrist: Position", wristPosition);
            telemetry.addData("Claw: Left", leftClawPosition + " Right=" + rightClawPosition);
            telemetry.addData("Distance", "%.01f mm, %.01f mm", sensorDistance.getDistance(DistanceUnit.MM), calculatedDistance);
            telemetry.update();
            idle();
        }
    }

    private void resetWristAndClawPosition() {
        wristServo.setPosition(STARTING_WRIST_POSITION);
        leftClawServo.setPosition(STARTING_CLAW_POS);
        rightClawServo.setPosition(STARTING_CLAW_POS);
    }
    private void waitAndMoveArmAndResetDistance() {
        sleep(1000);
        // Move Arm up to remove friction and get clearance the ground
        moveArmToPosition(ARM_PICK_POSITION + 50);
        changeGameMode(GameMode.GOING_TO_DROP_PIXELS);
        resetDistanceSensor();
    }
    private void pickPixels() {
        leftClawServo.setPosition(CLAW_CLOSE_POSITION);
        rightClawServo.setPosition(CLAW_CLOSE_POSITION);
        leftPixelInClaw = true;
        rightPixelInClaw = true;
        waitAndMoveArmAndResetDistance();
    }
    private void pickLeftPixel() {
        leftClawServo.setPosition(CLAW_CLOSE_POSITION);
        leftPixelInClaw = true;
        if (rightPixelInClaw) {
            waitAndMoveArmAndResetDistance();
        }
    }
    private void pickRightPixel() {
        rightClawServo.setPosition(CLAW_CLOSE_POSITION);
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
        leftClawServo.setPosition(CLAW_OPEN_POSITION);
        rightClawServo.setPosition(CLAW_OPEN_POSITION);
        sleep(1000);
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
        leftClawServo.setPosition(CLAW_OPEN_POSITION);
        rightClawServo.setPosition(CLAW_OPEN_POSITION);
    }

    private int moveArmToPosition(int armPosition) {
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
            wristPosition = Math.min(MAX_WRIST_POS, Math.max(MIN_WRIST_POS, wristPosition));
            wristServo.setPosition(wristPosition);
        }

        leftArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        return armPosition;
    }

    //calculate wrist position based on armPosition and pick or drop intent
    private double getWristPosition(int armPosition) {
        if ((gameMode == GameMode.GOING_TO_PICK_PIXELS) || (gameMode == GameMode.PICKING_PIXELS)) {
            if (isCloseToGround(armPosition)) {
                //Claw needs to face the ground
                return WRIST_PICK_POSITION;
            } else if (isArmFacingBack(armPosition)) {
                int angleA = ((armPosition * 360) / FULL_CIRCLE);
                return (150 - (0.383 * angleA)) / 100;
            } else {
                return wristPosition;
            }
        } else if ((gameMode == GameMode.GOING_TO_DROP_PIXELS) || (gameMode == GameMode.APRIL_TAG_NAVIGATION)) {
            //Calculate claw position based on arm position
            if (isArmFacingBack(armPosition)) {
                int angleA = ((armPosition * 360) / FULL_CIRCLE);
                return (150 - (0.383 * angleA)) / 100;
            } else {
                return wristPosition;
            }
        } else {
            //HOME
            return MIN_WRIST_POS;
        }
    }

    private boolean isArmFacingBack(double armPosition) {
        return armPosition > 380;
    }

    private boolean isCloseToGround(double armPosition) {
        return armPosition > 700;
    }

    public void moveRobotBack() {
        leftFront.setPower(-0.2);
        rightFront.setPower(-0.2);
        leftBack.setPower(-0.2);
        rightBack.setPower(-0.2);
        sleep(1000);
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
    }

    public void moveRobot(double x, double y, double yaw) {
        // Calculate wheel powers.
        double leftFrontPower = x - y - yaw;
        double rightFrontPower = x + y + yaw;
        double leftBackPower = x + y - yaw;
        double rightBackPower = x - y + yaw;

        // Normalize wheel powers to be less than 1.0
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        // Send powers to the wheels.
        leftFront.setPower(leftFrontPower);
        rightBack.setPower(rightFrontPower);
        leftBack.setPower(leftBackPower);
        rightBack.setPower(rightBackPower);
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
