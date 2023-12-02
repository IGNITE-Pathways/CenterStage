package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

@TeleOp(name="Exposure Test", group = "Concept")
public class GetExposureSettings extends XBotOpMode
{
    private static final int DESIRED_TAG_ID = 1;     // Choose the tag you want to approach or set to -1 for ANY tag.

    @Override public void runOpMode()
    {
        boolean targetFound     = false;    // Set to true when an AprilTag target is detected
        double  drive           = 0;        // Desired forward power/speed (-1 to +1)
        double  strafe          = 0;        // Desired strafe power/speed (-1 to +1)
        double  yawTurn            = 0;        // Desired turning power/speed (-1 to +1)

        // Initialize the Apriltag Detection process
        initialize();
        switchToAprilTagCamera();
        // Wait for driver to press start
        telemetry.addData("Camera preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();
        waitForStart();

        while (opModeIsActive())
        {
            targetFound = false;
            desiredTagDetectionObj = null;
            moveArmToPosition(200);
            autoDrive = true; //drive in reverse

            // Step through the list of detected tags and look for a matching tag
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            for (AprilTagDetection detection : currentDetections) {
                // Look to see if we have size info on this tag.
                if (detection.metadata != null) {
                    //  Check to see if we want to track towards this tag.
                    if ((DESIRED_TAG_ID < 0) || (detection.id == DESIRED_TAG_ID)) {
                        // Yes, we want to use this tag.
                        targetFound = true;
                        desiredTagDetectionObj = detection;
                        break;  // don't look any further.
                    } else {
                        // This tag is in the library, but we do not want to track it right now.
                        telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
                    }
                } else {
                    // This tag is NOT in the library, so we don't have enough information to track to it.
                    telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
                }
            }

            // Tell the driver what we see, and what to do.
            if (targetFound) {
                telemetry.addData("\n>","HOLD Left-Bumper to Drive to Target\n");
                telemetry.addData("Found", "ID %d (%s)", desiredTagDetectionObj.id, desiredTagDetectionObj.metadata.name);
                telemetry.addData("Range",  "%5.1f inches", desiredTagDetectionObj.ftcPose.range);
                telemetry.addData("Bearing","%3.0f degrees", desiredTagDetectionObj.ftcPose.bearing);
                telemetry.addData("Yaw","%3.0f degrees", desiredTagDetectionObj.ftcPose.yaw);
                telemetry.addData("Exposure", getExposure());
            } else {
                makeSureCamIsReadyandExposureIsSet(getExposure() + 1, 250, "Cam");
            }

            telemetry.update();
            sleep(10);
        }
    }
}
