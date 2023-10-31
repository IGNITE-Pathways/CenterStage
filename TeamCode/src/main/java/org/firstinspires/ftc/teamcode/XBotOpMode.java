package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

public abstract class XBotOpMode extends LinearOpMode {
    //Define motors and sensors
    DcMotor rightFront, leftFront, rightBack, leftBack = null;
    DcMotor leftArmMotor, rightArmMotor = null;
    Servo wristServo, leftClawServo, rightClawServo = null;
    DistanceSensor sensorDistance = null;
    TouchSensor leftClawTouchSensor, rightClawTouchSensor = null;
    boolean leftPixelInClaw, rightPixelInClaw = false;
    VisionPortal visionPortal;               // Used to manage the video source.
    AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.

    //Desired April Tag
    AprilTagDetection desiredTag = null;     // Used to hold the data for a detected AprilTag

    //Game Mode
    GameMode gameMode = GameMode.NONE;
    Boolean gameModeChanged = Boolean.FALSE;

    void initialize() {
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
        leftClawTouchSensor = hardwareMap.get(TouchSensor.class, "leftClawTouch");
        rightClawTouchSensor = hardwareMap.get(TouchSensor.class, "rightClawTouch");

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
    }

    void initDriveMotorsToUseEncoders() {
        //Using Encoders
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    void setManualExposure(int exposureMS, int gain) {
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

    boolean detectAprilTags() {
        boolean aprilTagFound = false;
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            if ((detection.metadata != null) &&
                    ((XBot.DESIRED_TAG_ID < 0) || (detection.id == XBot.DESIRED_TAG_ID))) {
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

    void resetArmEncoders() {
        leftArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    void changeGameMode(GameMode mode) {
        gameMode = mode;
        gameModeChanged = Boolean.TRUE;
        gamepad2.rumble(1000);
    }

    boolean areDriveMotorsBusy() {
        return leftFront.isBusy() || leftBack.isBusy() || rightFront.isBusy() || rightBack.isBusy();
    }

    void setDriveMotorsPower(double speed) {
        leftFront.setPower(speed);
        rightFront.setPower(speed);
        leftBack.setPower(speed);
        rightBack.setPower(speed);
    }

    public void stopDriveMotors() {
        setDriveMotorsPower(0);
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
}
