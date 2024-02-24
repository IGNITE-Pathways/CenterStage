package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.XBot.DEFAULT_DROP_ARM_POSITION;
import static org.firstinspires.ftc.teamcode.XBot.MIN_ARM_POSITION;
import static org.firstinspires.ftc.teamcode.XBot.SKIP_PICKING_WHITE_PIXELS_FAR;
import static org.firstinspires.ftc.teamcode.XBot.WAIT_TIME_FOR_ALLIANCE_TO_CLEAR;
import static org.firstinspires.ftc.teamcode.XBot.WRIST_FLAT_TO_GROUND;
import static org.firstinspires.ftc.teamcode.XBot.WRIST_VERTICAL;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

public abstract class XBotBlueFar extends XBotBlue {
    public void autoBlueFar(Parking parking) {
        super.initializeAuto(new Pose2d(-36.5, 63.5, Math.toRadians(90)), DistanceFromBackdrop.FAR, parking);

        if (opModeIsActive()) {
            while (!teamPropDetectionCompleted) {
                detectTeamPropAndSwitchCameraToAprilTag();
            }
            
            telemetry.addData("SpikeMark", spikeMark + ", confidence" + detectionConfidence);

            if (spikeMark != SpikeMark.RIGHT) {
                if (spikeMark == SpikeMark.LEFT) {
                    //Meep Meep === blueFarLeftPixelLeftParking

                    trajectorySeqToDropPurplePixel = xDrive.trajectorySequenceBuilder(startPose)
                            .back(27.5)
                            .turn(Math.toRadians(-90))
                            //Push the Team Prop
                            .forward(10)
                            //Move Back to drop Pixel
                            .back(11)
                            .build();

                    trajectorySeqToDropYellowPixel = xDrive.trajectorySequenceBuilder(trajectorySeqToDropPurplePixel.end())
                            .back(4)
                            .strafeTo(new Vector2d(-41, 12.5))
                            //180 degree turn
                            .turn(Math.toRadians(180))
//                            .turn(Math.toRadians(90))
                            .lineTo(new Vector2d(DROP_LINE_X, 12.5))
                            .strafeTo(new Vector2d(DROP_LINE_X, 43)) //ID 1 Blue
                            .build();

                } else if (spikeMark == SpikeMark.CENTER) {
                    trajectorySeqToDropPurplePixel = xDrive.trajectorySequenceBuilder(startPose)
                            .strafeTo(new Vector2d(-50, 30))
                            .splineToConstantHeading(new Vector2d(-44, 15.5), Math.toRadians(90))
                            .build();

                    trajectorySeqToDropYellowPixel = xDrive.trajectorySequenceBuilder(trajectorySeqToDropPurplePixel.end())
                            .turn(Math.toRadians(90))
                            .lineTo(new Vector2d(DROP_LINE_X, 15.5))
                            .strafeTo(new Vector2d(DROP_LINE_X, 38)) //ID 2 Blue
                            .build();
                }
            }

            if (isStopRequested()) return;

            if (spikeMark == SpikeMark.CENTER && SKIP_PICKING_WHITE_PIXELS_FAR) {
                sleep(WAIT_TIME_FOR_ALLIANCE_TO_CLEAR); //10s sleep so alliance robot can park
            }

            //STEP 1 -- Purple Pixel Drop on spike mark
            xDrive.followTrajectorySequence(trajectorySeqToDropPurplePixel);
            setWristPosition(WRIST_FLAT_TO_GROUND);
            sleep(200);
            openRightClaw();
            sleep(200);

            //STEP 2 -- Yellow Pixel to back board
            setWristPosition(WRIST_VERTICAL);
            sleep(50);

            if (spikeMark == SpikeMark.LEFT && SKIP_PICKING_WHITE_PIXELS_FAR) {
                sleep(WAIT_TIME_FOR_ALLIANCE_TO_CLEAR - 3000); //10s sleep so alliance robot can park
            } else if (spikeMark == SpikeMark.RIGHT && SKIP_PICKING_WHITE_PIXELS_FAR) {
                sleep(WAIT_TIME_FOR_ALLIANCE_TO_CLEAR); //10s sleep so alliance robot can park
            }

            xDrive.followTrajectorySequence(trajectorySeqToDropYellowPixel);

            moveArmToPosition(DEFAULT_DROP_ARM_POSITION - 300);
            sleep(1200);
            moveArmToPosition(DEFAULT_DROP_ARM_POSITION + 10, 0.3);
            sleep(600);
            openLeftClaw();
            sleep(200);

            if (!SKIP_PICKING_WHITE_PIXELS_FAR) {
                //STEP 3 to 6 -- Grab White Pixels and drop the on backboard
                grabAndDropWhitePixels(trajectorySeqToPickWhitePixels, inchForwardSeq, inchBackwardSeq, trajectorySeqToDropWhitePixels);
            }
            //STEP 7 -- Park
            moveArmToPosition(MIN_ARM_POSITION);
            xDrive.followTrajectorySequence(parkingSeq);
        }
        stopRobot();
    }
}
