package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.XBot.DEFAULT_DROP_ARM_POSITION;
import static org.firstinspires.ftc.teamcode.XBot.MIN_ARM_POSITION;
import static org.firstinspires.ftc.teamcode.XBot.SKIP_PICKING_WHITE_PIXELS_FAR;
import static org.firstinspires.ftc.teamcode.XBot.WRIST_FLAT_TO_GROUND;
import static org.firstinspires.ftc.teamcode.XBot.WRIST_VERTICAL;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public abstract class XBotRedFar extends XBotRed {
    public void autoRedFar(Parking parking) {
        super.initializeAuto(new Pose2d(-36, -63.5, Math.toRadians(-90)), DistanceFromBackdrop.FAR, parking);

        if (opModeIsActive()) {
            while (!teamPropDetectionCompleted) {
                detectTeamPropAndSwitchCameraToAprilTag();
            }
//            spikeMark = SpikeMark.RIGHT;
            telemetry.addData("SpikeMark", spikeMark + ", confidence" + detectionConfidence);
            if (spikeMark != SpikeMark.RIGHT) {
                if (spikeMark == SpikeMark.LEFT) {
                    trajectorySeqToDropPurplePixel = xDrive.trajectorySequenceBuilder(startPose)
                            .back(29)
                            .turn(Math.toRadians(-90))
                            .forward(5)
                            .back(9)
//                            .back(27.5)
//                            .turn(Math.toRadians(-90))
//                            .forward(7)
//                            .back(9)
                            .build();

                    trajectorySeqToDropYellowPixel = xDrive.trajectorySequenceBuilder(trajectorySeqToDropPurplePixel.end())
                            .strafeTo(new Vector2d(-32, -12.5))
                            .lineTo(new Vector2d(DROP_LINE_X, -12.5))
                            .strafeTo(new Vector2d(DROP_LINE_X, -32.5))
                            .build();

                    trajectoryToDropYellowPixel = xDrive.trajectoryBuilder(trajectorySeqToDropPurplePixel.end(), true)
                            .back(40)
                            .splineTo(new Vector2d(DROP_LINE_X, -32.5), 0,
                                    SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                    SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                            .build();
                } else if (spikeMark == SpikeMark.CENTER) {
                    trajectorySeqToDropPurplePixel = xDrive.trajectorySequenceBuilder(startPose)
                            .back(51)
//                            .back(49)
                            .build();

                    trajectorySeqToDropYellowPixel = xDrive.trajectorySequenceBuilder(trajectorySeqToDropPurplePixel.end())
                            .turn(Math.toRadians(-90))
                            .lineTo(new Vector2d(DROP_LINE_X, -12.5))
                            .strafeTo(new Vector2d(DROP_LINE_X, -36))
                            .build();

                    trajectoryToDropYellowPixel = xDrive.trajectoryBuilder(trajectorySeqToDropPurplePixel.end(), true)
                            .splineTo(new Vector2d(10, -10), 0,
                                    SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                    SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                            .splineToConstantHeading(new Vector2d(DROP_LINE_X, -38), 0,
                                    SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                    SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                            .build();
                }
                sleep(10);
            }

            if (isStopRequested()) return;
            //STEP 1 -- Purple Pixel Drop on spike mark
            xDrive.followTrajectorySequence(trajectorySeqToDropPurplePixel);
            setWristPosition(WRIST_FLAT_TO_GROUND);
            sleep(200);
            openRightClaw();
            sleep(200);

            //STEP 2 -- Yellow Pixel to back board
            setWristPosition(WRIST_VERTICAL);
            sleep(50);
            if (SKIP_PICKING_WHITE_PIXELS_FAR) {
                sleep(14000); //10s sleep so alliance robot can park
            }

            if (trajectorySeqToDropYellowPixel != null) {
                xDrive.followTrajectorySequence(trajectorySeqToDropYellowPixel);
            } else {
                xDrive.followTrajectory(trajectoryToDropYellowPixel); //sleep(100);
            }
            moveArmToPosition(DEFAULT_DROP_ARM_POSITION, 0.4);
            sleep(1400);
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
