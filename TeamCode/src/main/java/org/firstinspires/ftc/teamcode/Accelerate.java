package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

/*
 * This OpMode illustrates the basics of AprilTag recognition and pose estimation,
 * including Java Builder structures for specifying Vision parameters.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list.
 */
@TeleOp
@Disabled
public class Accelerate extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    DcMotor rightfront = null;
    DcMotor leftfront = null;
    DcMotor rightback = null;
    DcMotor leftback = null;

    DcMotor leftArmMotor = null;
    DcMotor rightArmMotor = null;

    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftfront = hardwareMap.get(DcMotor.class, "leftfront");
        leftback = hardwareMap.get(DcMotor.class, "leftback");
        rightfront = hardwareMap.get(DcMotor.class, "rightfront");
        rightback = hardwareMap.get(DcMotor.class, "rightback");
//        leftArmMotor = hardwareMap.get(DcMotor.class, "leftArmMotor");
//        rightArmMotor = hardwareMap.get(DcMotor.class, "rightArmMotor");

        // ########################################################################################
        // !!!            IMPORTANT Drive Information. Test your motor directions.            !!!!!
        // ########################################################################################
        // Most robots need the motors on one side to be reversed to drive forward.
        // The motor reversals shown here are for a "direct drive" robot (the wheels turn the same direction as the motor shaft)
        // If your robot has additional gear reductions or uses a right-angled drive, it's important to ensure
        // that your motors are turning in the correct direction.  So, start out with the reversals here, BUT
        // when you first test your robot, push the left joystick forward and observe the direction the wheels turn.
        // Reverse the direction (flip FORWARD <-> REVERSE ) of any wheel that runs backward
        // Keep testing until ALL the wheels move the robot forward when you push the left joystick forward.
        leftfront.setDirection(DcMotor.Direction.REVERSE);
        leftback.setDirection(DcMotor.Direction.REVERSE);
        rightfront.setDirection(DcMotor.Direction.FORWARD);
        rightback.setDirection(DcMotor.Direction.FORWARD);

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double max;

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral = gamepad1.left_stick_x;
            double yaw = gamepad1.right_stick_x;
            double power = 0.5;
            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double leftFrontPower = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower = axial - lateral + yaw;
            double rightBackPower = axial + lateral - yaw;
//            double leftArmMotorPower = -gamepad2.left_stick_y;
//            double rightArmMotorPower = gamepad2.left_stick_y;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max < .5) {
                leftFrontPower /= max;
                rightFrontPower /= max;
                leftBackPower /= max;
                rightBackPower /= max;
            }
            if (gamepad1.right_trigger > 0) {
                power = power*1.5;
            }
            else{
                power = 0.5;
            }
            // This is test code:
            //
            // Uncomment the following code to test your motor directions.
            // Each button should make the corresponding motor run FORWARD.
            //   1) First get all the motors to take to correct positions on the robot
            //      by adjusting your Robot Configuration if necessary.
            //   2) Then make sure they run in the correct direction by modifying the
            //      the setDirection() calls above.
            // Once the correct motors move in the correct direction re-comment this code.

            /*
            leftFrontPower  = gamepad1.x ? 1.0 : 0.0;  // X gamepad
            leftBackPower   = gamepad1.a ? 1.0 : 0.0;  // A gamepad
            rightFrontPower = gamepad1.y ? 1.0 : 0.0;  // Y gamepad
            rightBackPower  = gamepad1.b ? 1.0 : 0.0;  // B gamepad
            */

            // Send calculated power to wheels
            leftfront.setPower(leftFrontPower * power);
            leftfront.setPower(leftFrontPower * power);
            rightfront.setPower(rightFrontPower * power);
            leftback.setPower(leftBackPower * power);
            rightback.setPower(rightBackPower * power);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.update();
        }

        /*
         * This OpMode illustrates the basics of AprilTag recognition and pose estimation,
         * including Java Builder structures for specifying Vision parameters.
         *
         * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
         * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list.
         */
        class AprilTag extends LinearOpMode {

            private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

            /**
             * The variable to store our instance of the AprilTag processor.
             */
            private AprilTagProcessor aprilTag;

            /**
             * The variable to store our instance of the vision portal.
             */
            private VisionPortal visionPortal;

            @Override
            public void runOpMode() {

                initAprilTag();

                // Wait for the DS start button to be touched.
                telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
                telemetry.addData(">", "Touch Play to start OpMode");
                telemetry.update();
                waitForStart();

                if (opModeIsActive()) {
                    while (opModeIsActive()) {

                        telemetryAprilTag();

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
                    }
                }

                // Save more CPU resources when camera is no longer needed.
                visionPortal.close();

            }   // end method runOpMode()

            /**
             * Initialize the AprilTag processor.
             */
            private void initAprilTag() {

                // Create the AprilTag processor.
                aprilTag = new AprilTagProcessor.Builder()
                        //.setDrawAxes(false)
                        //.setDrawCubeProjection(false)
                        //.setDrawTagOutline(true)
                        //.setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                        //.setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                        //.setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)

                        // == CAMERA CALIBRATION ==
                        // If you do not manually specify calibration parameters, the SDK will attempt
                        // to load a predefined calibration for your camera.
                        //.setLensIntrinsics(578.272, 578.272, 402.145, 221.506)

                        // ... these parameters are fx, fy, cx, cy.

                        .build();

                // Create the vision portal by using a builder.
                VisionPortal.Builder builder = new VisionPortal.Builder();

                // Set the camera (webcam vs. built-in RC phone camera).
                if (USE_WEBCAM) {
                    builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
                } else {
                    builder.setCamera(BuiltinCameraDirection.BACK);
                }

                // Choose a camera resolution. Not all cameras support all resolutions.
                //builder.setCameraResolution(new Size(640, 480));

                // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
                //builder.enableCameraMonitoring(true);

                // Set the stream format; MJPEG uses less bandwidth than default YUY2.
                //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

                // Choose whether or not LiveView stops if no processors are enabled.
                // If set "true", monitor shows solid orange screen if no processors enabled.
                // If set "false", monitor shows camera view without annotations.
                //builder.setAutoStopLiveView(false);

                // Set and enable the processor.
                builder.addProcessor(aprilTag);

                // Build the Vision Portal, using the above settings.
                visionPortal = builder.build();

                // Disable or re-enable the aprilTag processor at any time.
                //visionPortal.setProcessorEnabled(aprilTag, true);

            }   // end method initAprilTag()


            /**
             * Add telemetry about AprilTag detections.
             */
            private void telemetryAprilTag() {

                List<AprilTagDetection> currentDetections = aprilTag.getDetections();
                telemetry.addData("# AprilTags Detected", currentDetections.size());

                // Step through the list of detections and display info for each one.
                for (AprilTagDetection detection : currentDetections) {
                    if (detection.metadata != null) {
                        telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                        telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                        telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                        telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
                    } else {
                        telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                        telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
                    }
                }   // end for() loop

                // Add "key" information to telemetry
                telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
                telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
                telemetry.addLine("RBE = Range, Bearing & Elevation");

            }   // end method telemetryAprilTag()
        }
// end class
    }
}
