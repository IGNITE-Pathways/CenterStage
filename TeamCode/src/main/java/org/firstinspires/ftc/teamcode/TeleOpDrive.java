package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.Iterator;
import java.util.List;
import java.util.Queue;
import java.util.concurrent.TimeUnit;

/*
 * This is the main manual OpMode
 */
@TeleOp(name = "Manual Drive", group = "Concept")
public class TeleOpDrive extends LinearOpMode {

    // ARM
    static final int MAX_ARM_POSITION = 500;
    static final int MIN_ARM_POSITION = 0;
    static final int FULL_CIRCLE = 1075;
    static final int ARM_PICK_POSITION = MIN_ARM_POSITION + 5; //Robot must move slow
    static final int ARM_POSITION_HIGH = MIN_ARM_POSITION + 20; //Robot running across field
    static final double ARM_SPEED = 0.3;

    // WRIST
    static final double MAX_WRIST_POS = 0.95;     // Maximum rotational position
    static final double MIN_WRIST_POS = 0.27;     // Minimum rotational position
    static final double STARTING_WRIST_POSITION = MIN_WRIST_POS;
    static final double WRIST_PICK_POSITION = MIN_WRIST_POS;

    //CLAW
    static final double CLAW_OPEN_POSITION = 0.63;
    static final double CLAW_CLOSE_POSITION = 0.70;
    static final double STARTING_CLAW_POS = CLAW_OPEN_POSITION;

    //DRIVE
    static final double MAX_SPEED = 1.0;
    static final double SPEED_WHEN_PICKING_PIXELS = 0.3; //gameMode = PICKING_PIXELS
    static final double SPEED_WHEN_DROPPING_PIXELS = 0.3; //gameMode = DROPPING_PIXELS
    static final double SPEED_WHEN_ON_APRIL_TAG_NAV = 0.5; //gameMode = APRIL_TAG_NAVIGATION

    //APRIL TAG
    private static final int DESIRED_TAG_ID = -1;     // Choose the tag you want to approach or set to -1 for ANY tag.
    final double MAX_AUTO_SPEED = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_STRAFE = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_TURN = 0.3;   //  Clip the turn speed to this max value (adjust for your robot)
    // Use DESIRED_DISTANCE to set how close you want the robot to get to the target.
    final double DESIRED_DISTANCE = 7.0; //  this is how close the camera should get to the target (inches)
    // Speed and Turn sensitivity can be adjusted using the SPEED_GAIN, STRAFE_GAIN and TURN_GAIN constants.
    final double SPEED_GAIN = 0.02;   //  Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    final double STRAFE_GAIN = 0.015;   //  Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
    final double TURN_GAIN = 0.01;   //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)
    GameMode gameMode = GameMode.NONE;
    DcMotor rightFront, leftFront, rightBack, leftBack = null;
    DcMotor leftArmMotor, rightArmMotor = null;
    Servo wristServo = null;
    Servo leftClawServo, rightClawServo = null;
    double wristPosition = STARTING_WRIST_POSITION;
    double leftClawPosition = STARTING_CLAW_POS;
    double rightClawPosition = STARTING_CLAW_POS;
    //Robot Speed
    double driveSpeed = MAX_SPEED;
    Boolean gameModeChanged = Boolean.FALSE;
    // Declare OpMode members for each of the 4 motors.
    private final ElapsedTime runtime = new ElapsedTime();
    private DistanceSensor sensorDistance;
    Queue<Double> distanceQueue = new SizeLimitedQueue<>(10);
    double calculatedDistance = DistanceSensor.distanceOutOfRange;

    private VisionPortal visionPortal;               // Used to manage the video source.
    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
    private AprilTagDetection desiredTag = null;     // Used to hold the data for a detected AprilTag

    @Override
    public void runOpMode() {
        int armPosition = 0;

        // Initialize April Tag Variables
        boolean aprilTagFound = false;
        boolean lookForAprilTag = false;
        boolean autoDrive;

        // Initialize all motors and sensors
        initialize();
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
                    driveSpeed = MAX_SPEED;
                    // ARM = AUTO, WRIST = AUTO, CLAWS = AUTO
                    if (gameModeChanged) {
                        armPosition = goToGoingToPickPixelPosition();
                        gameModeChanged = Boolean.FALSE;
                        resetDistanceSensor();
                    }
                    break;
                case PICKING_PIXELS:
                    // ARM = AUTO, WRIST = AUTO, CLAWS = OPEN or CLOSE
                    if (gameModeChanged) {
                        driveSpeed = SPEED_WHEN_PICKING_PIXELS;
                        armPosition = goToPickPixelPosition();
                        gameModeChanged = Boolean.FALSE;
                    }
                    //@TODO: This needs to be automated with touch-sensors
                    if (gamepad2.circle)  pickPixels();
                    break;
                case GOING_TO_DROP_PIXELS:
                    // WRIST = AUTO, CLAW = CLOSED (holding pixels)
                    driveSpeed = MAX_SPEED;

                    armPosition += (int) -(gamepad2.left_stick_y * 20);
                    armPosition = Math.max(0, armPosition); // cannot go below zero
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

                        // Change Mode, let's go get next set of pixels
                        changeGameMode(GameMode.GOING_TO_PICK_PIXELS);
                    }
                    break;
                case GOING_TO_HANG:
                    // ARM = AUTO, WRIST = AUTO, CLAW = AUTO
                    driveSpeed = MAX_SPEED;
                    break;
                case HANGING:
                    driveSpeed = 0;
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

    private void resetDistanceSensor() {
        distanceQueue.clear();
        calculatedDistance = DistanceSensor.distanceOutOfRange;
    }

    private boolean isDistanceSensorClose() {
        return calculatedDistance < 40;
    }

    private boolean detectAprilTags() {
        boolean aprilTagFound = false;
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            if ((detection.metadata != null) &&
                    ((DESIRED_TAG_ID < 0) || (detection.id == DESIRED_TAG_ID))) {
                aprilTagFound = true;
                //We found April Tag, change game mode to APRIL TAG NAV
                changeGameMode(GameMode.APRIL_TAG_NAVIGATION);
                desiredTag = detection;
                break;  // don't look any further.
            } else {
                telemetry.addData("Unknown Target", "Tag ID %d is not in TagLibrary\n", detection.id);
            }
        }
        return aprilTagFound;
    }

    private void dropPixels() {
        leftClawServo.setPosition(CLAW_OPEN_POSITION);
        rightClawServo.setPosition(CLAW_OPEN_POSITION);
        sleep(1000);
        changeGameMode(GameMode.GOING_TO_PICK_PIXELS);
    }

    private void changeGameMode(GameMode mode) {
        gameMode = mode;
        gameModeChanged = Boolean.TRUE;
        gamepad2.rumble(1000);
    }

    private void pickPixels() {
        leftClawServo.setPosition(CLAW_CLOSE_POSITION);
        rightClawServo.setPosition(CLAW_CLOSE_POSITION);
        sleep(1000);
        // Move Arm up to remove friction and get clearance the ground
        moveArmToPosition(ARM_PICK_POSITION + 50);
        changeGameMode(GameMode.GOING_TO_DROP_PIXELS);
        resetDistanceSensor();
    }

    private void initialize() {
        gameMode = GameMode.INIT;
        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFront = hardwareMap.get(DcMotor.class, "leftfront");
        leftBack = hardwareMap.get(DcMotor.class, "leftback");
        rightFront = hardwareMap.get(DcMotor.class, "rightfront");
        rightBack = hardwareMap.get(DcMotor.class, "rightback");

        // Initialize Motors
        leftArmMotor = hardwareMap.get(DcMotor.class, "leftArmMotor");
        rightArmMotor = hardwareMap.get(DcMotor.class, "rightArmMotor");

        // Initialize Rev 2M Distance sensor
        sensorDistance = hardwareMap.get(DistanceSensor.class, "distance");

        // Initialize April Tag
        aprilTag = new AprilTagProcessor.Builder().build();
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();
        setManualExposure(6, 250);  // Use low exposure time to reduce motion blur

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.FORWARD);

        //Left Motor is in reverse
        leftArmMotor.setDirection(DcMotor.Direction.REVERSE);

        //Using Encoders
        leftArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Initialize Servo
        wristServo = hardwareMap.get(Servo.class, "wrist");
        leftClawServo = hardwareMap.get(Servo.class, "leftClaw");
        rightClawServo = hardwareMap.get(Servo.class, "rightClaw");

        //Reset encoders -- making start position as zero
        resetArmEncoders();
        resetWristAndClawPosition();
    }

    private void resetArmEncoders() {
        leftArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    private void resetWristAndClawPosition() {
        wristServo.setPosition(STARTING_WRIST_POSITION);
        leftClawServo.setPosition(STARTING_CLAW_POS);
        rightClawServo.setPosition(STARTING_CLAW_POS);
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
        rightBack.setPower(-0.2);
        leftBack.setPower(-0.2);
        rightBack.setPower(-0.2);
        sleep(1000);
        leftFront.setPower(0);
        rightBack.setPower(0);
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

    private void setManualExposure(int exposureMS, int gain) {
        // Wait for the camera to be open, then use the controls

        if (visionPortal == null) {
            return;
        }

        // Make sure camera is streaming before we try to set the exposure controls
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        // Set camera controls unless we are stopping.
        if (!isStopRequested()) {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure(exposureMS, TimeUnit.MILLISECONDS);
            sleep(20);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            sleep(20);
        }
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
        if (distanceQueue.size() == 0) return DistanceSensor.distanceOutOfRange;
        Double total = 0.0;
        for (Double d : distanceQueue) {
            total += d;
        }
        return total/distanceQueue.size();
    }

}
