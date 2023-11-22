package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

@Autonomous(name = "Autonomous Drive", group = "Concept")
public class AutoOpMode extends XBotOpMode {

    // Constants for autonomous movement
    private static final double AUTONOMOUS_SPEED = 0.5;  // Adjust as needed
    private static final int DISTANCE_TO_DRIVE = 1000;  // Adjust as needed

    private static final String TFOD_MODEL_ASSET = "TeaXProp_TFOD.tflite";

    private final ElapsedTime runtime = new ElapsedTime();
    private TfodProcessor tfod;
    private VisionPortal visionPortal;
    private static final String[] LABELS = {
            "X",
    };

    @Override
    public void runOpMode() {
        // Initialize hardware
        initialize();
        initDriveMotorsToUseEncoders();
        initTfod();

        telemetry.addData("Status", "Initialized");
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        if (opModeIsActive()) {
            while (opModeIsActive()) {

                telemetryTfod();

                // Push telemetry to the Driver Station.
                telemetry.update();

                // Save CPU resources; can resume streaming when needed.
                if (gamepad1.dpad_down) {
                    visionPortal.stopStreaming();
                } else if (gamepad1.dpad_up) {
                    visionPortal.resumeStreaming();
                }

                // Share the CPU.
                sleep(20);

                // Autonomous driving
//                driveForwardForDistance(DISTANCE_TO_DRIVE);
            }
        }

        // Stop the robot
        stopRobot();
    }

    private void initTfod() {

        // Create the TensorFlow processor by using a builder.
        tfod = new TfodProcessor.Builder()
                .setModelAssetName(TFOD_MODEL_ASSET)
                .setModelLabels(LABELS)
                //.setIsModelTensorFlow2(true)
                //.setIsModelQuantized(true)
                //.setModelInputSize(300)
                //.setModelAspectRatio(16.0 / 9.0)
                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 2"));

        // Choose a camera resolution. Not all cameras support all resolutions.
        //builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(tfod);
        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Set confidence threshold for TFOD recognitions, at any time.
        //tfod.setMinResultConfidence(0.75f);

        // Disable or re-enable the TFOD processor at any time.
        //visionPortal.setProcessorEnabled(tfod, true);

    }

    private void telemetryTfod() {

        List<Recognition> currentRecognitions = tfod.getRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions.size());

        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
            double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;

            telemetry.addData(""," ");
            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Position", "%.0f / %.0f", x, y);
            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
        }   // end for() loop

    }   // end method telemetryTfod()

    private void driveForwardForDistance(int distance) {
        // Reset encoders
        resetDriveEncoders();

        // Set target position for the motors
        leftFront.setTargetPosition(distance);
        rightBack.setTargetPosition(distance);
        leftBack.setTargetPosition(distance);
        rightBack.setTargetPosition(distance);

        // Set motors to run to position
        setDriveRunToPosition();

        // Set motors power
        setDriveMotorsPower(AUTONOMOUS_SPEED);

        // Wait for motors to reach target position
        while (opModeIsActive() && areDriveMotorsBusy()) {
            // Additional actions or checks can be added here
            telemetry.addData("Status", "Driving forward...");
            telemetry.update();
            idle();
        }

        // Stop the motors
        stopDriveMotors();

        // Set motors back to normal mode
        stopDriveRunUsingEncoder();
    }

    // Other autonomous actions and methods can be added here

    private void stopRobot() {
        stopDriveMotors();
        // Save more CPU resources when camera is no longer needed.
        visionPortal.close();
    }


}
