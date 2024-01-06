package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.XBot.ARM_SPEED;
import static org.firstinspires.ftc.teamcode.XBot.FULL_CIRCLE;
import static org.firstinspires.ftc.teamcode.XBot.MAX_WRIST_POS;
import static org.firstinspires.ftc.teamcode.XBot.MIN_WRIST_POS;
import static org.firstinspires.ftc.teamcode.XBot.STARTING_LEFT_CLAW_POS;
import static org.firstinspires.ftc.teamcode.XBot.STARTING_RIGHT_CLAW_POS;
import static org.firstinspires.ftc.teamcode.XBot.WRIST_FLAT_TO_GROUND;
import static org.firstinspires.ftc.teamcode.XBot.WRIST_VERTICAL;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;
import org.opencv.core.Mat;

import java.util.List;
import java.util.concurrent.TimeUnit;

public abstract class XBotOpMode extends LinearOpMode {
    private static final String TFOD_MODEL_ASSET = "TeaXProp_TFOD.tflite";
    private static final String[] LABELS = {"X"};
    final ElapsedTime runtime = new ElapsedTime();
    //Define motors and sensors
    DcMotor rightFront = null, leftFront = null, rightBack = null, leftBack = null;
    DcMotor leftArmMotor = null, rightArmMotor = null;
    Servo leftWrist = null;
    Servo rightWrist = null;
    Servo leftClaw = null;
    Servo rightClaw = null;
    //    private IMU imu = null;      // Control Hub IMU
    WebcamName webcam1;
    TfodProcessor tfod;
    VisionPortal visionPortal;               // Used to manage the video source.
    AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
    //Desired April Tag
    AprilTagDetection desiredTagDetectionObj = null;     // Used to hold the data for a detected AprilTag
    //Game Mode
    GameMode gameMode = GameMode.NONE;
    Boolean gameModeChanged = Boolean.FALSE;
    double wristPosition = WRIST_FLAT_TO_GROUND;
    boolean autoDrive = false;
    boolean leftPixelInClaw = false, rightPixelInClaw = false;
    double previousLeftFrontPower, previousLeftBackPower, previousRightFrontPower, previousRightBackPower;

//    void initializeIMU() {
//        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD;
//        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.UP;
//        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
//        imu = hardwareMap.get(IMU.class, "imu");
//        imu.initialize(new IMU.Parameters(orientationOnRobot));
//        imu.resetYaw();
//    }

    void initialize() {
        gameMode = GameMode.INIT;
        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFront = hardwareMap.get(DcMotor.class, "leftfront");
        leftBack = hardwareMap.get(DcMotor.class, "leftback");
        rightFront = hardwareMap.get(DcMotor.class, "rightfront");
        rightBack = hardwareMap.get(DcMotor.class, "rightback");
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.FORWARD);

        // Initialize Motors
        leftArmMotor = hardwareMap.get(DcMotor.class, "leftarm");
        rightArmMotor = hardwareMap.get(DcMotor.class, "rightarm");
        rightArmMotor.setDirection(DcMotor.Direction.REVERSE);

        leftWrist = hardwareMap.get(Servo.class, "leftwrist");
        rightWrist = hardwareMap.get(Servo.class, "rightwrist");
        rightWrist.setDirection(Servo.Direction.REVERSE);

        leftClaw = hardwareMap.get(Servo.class, "leftclaw");
        rightClaw = hardwareMap.get(Servo.class, "rightclaw");
        rightClaw.setDirection(Servo.Direction.REVERSE);

        //CAM
        webcam1 = hardwareMap.get(WebcamName.class, "Webcam 1");
//        webcam2 = hardwareMap.get(WebcamName.class, "Webcam 2");
        CameraName switchableCamera = ClassFactory.getInstance()
                .getCameraManager().nameForSwitchableCamera(webcam1);

        // Create the TensorFlow processor by using a builder.
        tfod = new TfodProcessor.Builder()
                .setModelAssetName(TFOD_MODEL_ASSET)
                .setModelLabels(LABELS)
                //.setIsModelTensorFlow2(true)
                //.setIsModelQuantized(true)
                //.setModelInputSize(300)
                //.setModelAspectRatio(16.0 / 9.0)
                .build();

        // Set confidence threshold for TFOD recognitions, at any time.
        tfod.setMinResultConfidence(0.80f);

        aprilTag = new AprilTagProcessor.Builder().build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(switchableCamera)
                .addProcessors(aprilTag, tfod)
//                .setAutoStopLiveView(false)
//                .setCameraResolution(new Size(640, 480))
//                .enableLiveView(true)
//                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                .build();

        if (this instanceof AutoOpMode) {
            switchToTFODCamera();
        } else {
            switchToAprilTagCamera();
        }

        leftArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Reset encoders -- making start position as zero
        resetArmEncoders();
    }

    void switchToAprilTagCamera() {
        visionPortal.setProcessorEnabled(tfod, false);
        visionPortal.setProcessorEnabled(aprilTag, true);
        makeSureCamIsReadyandExposureIsSet(XBot.APRIL_TAG_CAM_EXPOSURE, 250, "AprilTag Cam");  // Use low exposure time to reduce motion blur
//        visionPortal.setActiveCamera(webcam1);
    }

    void switchToTFODCamera() {
        visionPortal.setProcessorEnabled(tfod, true);
        visionPortal.setProcessorEnabled(aprilTag, false);
        makeSureCamIsReadyandExposureIsSet(XBot.TFOD_CAM_EXPOSURE, 250, "TFOD Cam");  // Use low exposure time to reduce motion blur
//        visionPortal.setActiveCamera(webcam2);
    }

    void initDriveMotorsToUseEncoders() {
        //Using Encoders
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    void makeSureCamIsReadyandExposureIsSet(long exposureMS, int gain, String cam) {
        // Wait for the camera to be open, then use the controls
        if (visionPortal == null) {
            return;
        }
        // Make sure camera is streaming before we try to set the exposure controls
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData(cam, "Waiting");
            telemetry.update();
            while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            telemetry.addData(cam, "Ready");
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
            if (gainControl != null) {
                gainControl.setGain(gain);
                sleep(20);
            }
        }
    }

    long getExposure() {
        ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
        return exposureControl.getExposure(TimeUnit.MILLISECONDS);
    }

    int getGain() {
        GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
        return gainControl == null ? -1 : gainControl.getGain();
    }

    boolean detectAnyAprilTag() {
        boolean aprilTagFound = false;
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            if ((detection.metadata != null) &&
                    ((XBot.DESIRED_TAG_ID < 0) || (detection.id == XBot.DESIRED_TAG_ID))) {
                aprilTagFound = true;
                //We found April Tag, change game mode to APRIL TAG NAV
                changeGameMode(GameMode.APRIL_TAG_NAVIGATION);
                desiredTagDetectionObj = detection;
                break;  // don't look any further.
            } else {
                telemetry.addData("Unknown Target", "Tag ID %d is not in TagLibrary\n", detection.id);
            }
        }
        return aprilTagFound;
    }

    void resetArmEncoders() {
        leftArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    void changeGameMode(GameMode mode) {
        gameMode = mode;
        gameModeChanged = Boolean.TRUE;
        gamepad2.rumble(1000);
    }

    boolean areDriveMotorsBusy() {
        return leftBack.isBusy();// || leftFront.isBusy() || rightFront.isBusy() || rightBack.isBusy();
    }

    void setDriveMotorsPower(double speed) {
        setDriveMotorsPower(speed, speed, speed, speed);
    }

    void setDriveMotorsPower(double lfspeed, double lbspeed, double rfspeed, double rbspeed) {

        double max = Math.max(Math.abs(lfspeed), Math.abs(rfspeed));
        max = Math.max(max, Math.max(Math.abs(lbspeed), Math.abs(rbspeed)));
        if (max > 1.0) {
            lfspeed /= max;
            lbspeed /= max;
            rfspeed /= max;
            rbspeed /= max;
        }

        leftFront.setPower(lfspeed);
        rightFront.setPower(rfspeed);
        leftBack.setPower(lbspeed);
        rightBack.setPower(rbspeed);
    }

    //x=drive, y=strafe, yaw=tanTurn Yaw
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

//        if (Math.abs(previousLeftBackPower - leftBackPower) > 0.2) {
//            leftBackPower = previousLeftBackPower + (leftBackPower - previousLeftBackPower)*0.2; //20% increments
//            previousLeftBackPower = leftBackPower;
//        }

        if (autoDrive) {
            //Drive in reverse
            leftFront.setPower(-leftFrontPower);
            rightFront.setPower(-rightFrontPower);
            leftBack.setPower(-leftBackPower);
            rightBack.setPower(-rightBackPower);
        } else {
            // Send powers to the wheels.
            leftFront.setPower(leftFrontPower);
            rightFront.setPower(rightFrontPower);
            leftBack.setPower(leftBackPower);
            rightBack.setPower(rightBackPower);
        }
    }

    public void stopDriveMotors() {
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        setDriveMotorsPower(0, 0, 0, 0);
    }

    public void setDriveRunToPosition() {
        // set motors to run to target encoder position and stop with brakes on.
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    void resetDriveEncoders() {
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    void stopDriveRunUsingEncoder() {
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    void openLeftClaw() {
        sleep(100);
        leftClaw.setPosition(XBot.LEFT_CLAW_OPEN_POSITION);
        sleep(100);
    }

    void openRightClaw() {
        sleep(100);
        rightClaw.setPosition(XBot.RIGHT_CLAW_OPEN_POSITION);
        sleep(100);
    }

    void closeLeftClaw() {
        leftClaw.setPosition(XBot.LEFT_CLAW_CLOSE_POSITION);
    }

    void closeRightClaw() {
        rightClaw.setPosition(XBot.RIGHT_CLAW_CLOSE_POSITION);
    }

    void openBothClaws() {
        openLeftClaw();
        openRightClaw();
    }

    void closeBothClaws() {
        closeLeftClaw();
        closeRightClaw();
    }

    void moveArmToPosition(int armPosition, boolean increasing) {
        int slowSpeedMovement = Math.abs(armPosition - leftArmMotor.getCurrentPosition()) / 10;
        if (increasing) {
            //Arm going up
            moveArmToPosition(armPosition - slowSpeedMovement);
            moveArmToPosition(slowSpeedMovement, ARM_SPEED / 2);
        } else {
            moveArmToPosition(slowSpeedMovement);
            moveArmToPosition(armPosition, ARM_SPEED / 2);
        }
    }

    void moveArmToPosition(int armPosition) {
        moveArmToPosition(armPosition, ARM_SPEED);
    }

    void moveArmToPosition(int armPosition, double speed) {
//        int savePos = armPosition;
        // set motors to run forward for 5000 encoder counts.
        leftArmMotor.setTargetPosition(armPosition);
        rightArmMotor.setTargetPosition(armPosition);

        // set motors to run to target encoder position and stop with brakes on.
        leftArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftArmMotor.setPower(speed);
        rightArmMotor.setPower(speed);

        while (leftArmMotor.isBusy())   //leftMotor.getCurrentPosition() < leftMotor.getTargetPosition())
        {
            if (gameMode != GameMode.AUTO_OP_MODE) {
                armPosition = leftArmMotor.getCurrentPosition();
                wristPosition = getWristPosition(armPosition);
                setWristPosition(wristPosition);
            }
//            telemetry.addData("Arm: Target", savePos);
//            telemetry.addData("Arm: Left Motor Position", leftArmMotor.getCurrentPosition() + "  busy=" + leftArmMotor.isBusy());
//            telemetry.addData("Arm: Right Motor Position", rightArmMotor.getCurrentPosition() + "  busy=" + rightArmMotor.isBusy());
//            telemetry.update();
            sleep(10);
        }

//        leftArmMotor.setPower(ARM_HOLD_SPEED);
//        rightArmMotor.setPower(ARM_HOLD_SPEED);
    }

    void setWristPosition(double wristPosition) {
        wristPosition = Math.min(MAX_WRIST_POS, Math.max(MIN_WRIST_POS, wristPosition));
        leftWrist.setPosition(wristPosition);
        rightWrist.setPosition(wristPosition);
    }

    //calculate wrist position based on armPosition and pick or drop intent
    double getWristPosition(int armPosition) {
        if ((gameMode == GameMode.GOING_TO_PICK_PIXELS)) {
            return WRIST_VERTICAL;
        }
        if (gameMode == GameMode.PICKING_PIXELS) {
            return WRIST_FLAT_TO_GROUND;
        } else if ((gameMode == GameMode.GOING_TO_DROP_PIXELS)
                || (gameMode == GameMode.APRIL_TAG_NAVIGATION)
                || (gameMode == GameMode.DROPPING_PIXELS)) {
            int angleA = ((armPosition * 360) / FULL_CIRCLE);
            if (angleA < 340) {
                return MIN_WRIST_POS;
            } else {
                return Math.min(MAX_WRIST_POS, Math.max(MIN_WRIST_POS, (-115 + (.303 * angleA)) / 100));
            }
        } else if ((gameMode == GameMode.AUTO_OP_MODE)) {
            if (armPosition > 1500) {
                return WRIST_VERTICAL;
            } else if (isArmFacingBack(armPosition)) {
                int angleA = ((armPosition * 360) / FULL_CIRCLE);
                return Math.min(MIN_WRIST_POS, Math.max(MAX_WRIST_POS, (123 - (0.196 * angleA)) / 100));
            } else {
                return MAX_WRIST_POS;
            }
        } else {
            //HOME
            return MAX_WRIST_POS;
        }
    }

    private boolean isArmFacingBack(double armPosition) {
        return armPosition > 1000;
    }

    private boolean isCloseToGround(double armPosition) {
        return armPosition < 20;
    }

//    public double getHeading() {
//        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
//        return orientation.getYaw(AngleUnit.DEGREES);
//    }

    void moveRobot(int distance, MoveRobot moveRobot, double speed) {
        moveRobot(distance, moveRobot, speed, false);
    }

    void moveRobot(int distance, MoveRobot moveRobot, double speed, boolean yawfix) {
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
            if (!yawfix) {
                telemetry.addData("Status", moveRobot);
//                telemetry.addData("Heading- Target : Current", "%5.3f : %5.3f", heading, getHeading());
                telemetry.addData("Distance to go", distance);
                telemetry.addData("Left Front Motor", leftFront.getCurrentPosition() + "  busy=" + leftFront.isBusy());
                telemetry.addData("Left Back Motor", leftBack.getCurrentPosition() + "  busy=" + leftBack.isBusy());
                telemetry.addData("Right Front Motor", rightFront.getCurrentPosition() + "  busy=" + rightFront.isBusy());
                telemetry.addData("Right Back Motor", rightBack.getCurrentPosition() + "  busy=" + rightBack.isBusy());
                telemetry.update();
            }
            idle();
        }

        setDriveMotorsPower(.05);
        // Stop the motors
        stopDriveMotors();
//        // Set motors back to normal mode
//        stopDriveRunUsingEncoder();

    }

    void stopRobot() {
        stopDriveMotors();
        // Save more CPU resources when camera is no longer needed.
        visionPortal.close();
    }

    void setAprilTagDecimation() {
        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        aprilTag.setDecimation(2);
    }


    void resetWristAndClawPosition() {
        wristPosition = WRIST_FLAT_TO_GROUND;
        setWristPosition(wristPosition);
        leftClaw.setPosition(STARTING_LEFT_CLAW_POS);
        rightClaw.setPosition(STARTING_RIGHT_CLAW_POS);
    }
}
