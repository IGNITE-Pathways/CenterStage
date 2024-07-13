package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.XBot.DEFAULT_DROP_ARM_POSITION;
import static org.firstinspires.ftc.teamcode.XBot.MIN_ARM_POSITION;
import static org.firstinspires.ftc.teamcode.XBot.SKIP_PICKING_WHITE_PIXELS_FAR;
import static org.firstinspires.ftc.teamcode.XBot.SKIP_PICKING_WHITE_PIXELS_NEAR;
import static org.firstinspires.ftc.teamcode.XBot.WRIST_FLAT_TO_GROUND;
import static org.firstinspires.ftc.teamcode.XBot.WRIST_VERTICAL;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public abstract class XBotRedNear extends XBotRed {
    public void autoRedNear(Parking parking) {
        super.initializeAuto(new Pose2d(12.0, -63.5, Math.toRadians(-90)), DistanceFromBackdrop.NEAR, parking);

        if (opModeIsActive()) {
            while (!teamPropDetectionCompleted) {
                detectTeamPropAndSwitchCameraToAprilTag();
            }
//            spikeMark = SpikeMark.RIGHT;

            telemetry.addData("SpikeMark", spikeMark + ", confidence" + detectionConfidence);
            if (spikeMark != SpikeMark.RIGHT) {

                if (spikeMark == SpikeMark.LEFT) {
                    trajectorySeqToDropPurplePixel = xDrive.trajectorySequenceBuilder(startPose)
                            .setReversed(true)
                            .splineTo(new Vector2d(20, -35), 0)
                            .forward(11)
                            .back(5)
                            .build();

                    trajectorySeqToDropYellowPixel = xDrive.trajectorySequenceBuilder(trajectorySeqToDropPurplePixel.end())
                            .strafeTo(new Vector2d(DROP_LINE_X, -32.5)) //ID 4 Red
//                            .splineToConstantHeading(new Vector2d(DROP_LINE_X, -32.5), 0,
//                                    SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                                    SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                            .build();
                } else if (spikeMark == SpikeMark.CENTER) {
                    trajectorySeqToDropPurplePixel = xDrive.trajectorySequenceBuilder(startPose)
                            .setReversed(true)
                            .splineTo(new Vector2d(30, -34), 0,
                                    SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                    SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                            .strafeRight(6)
                            .build();

                    trajectorySeqToDropYellowPixel = xDrive.trajectorySequenceBuilder(trajectorySeqToDropPurplePixel.end())
                            .splineToConstantHeading(new Vector2d(DROP_LINE_X, -37), 0,
                                    SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                    SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                            .build();
                }
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
            xDrive.followTrajectorySequence(trajectorySeqToDropYellowPixel);

            moveArmToPosition(DEFAULT_DROP_ARM_POSITION - 300);
            sleep(1200);
            moveArmToPosition(DEFAULT_DROP_ARM_POSITION + 40, 0.3);
            sleep(600);

            openLeftClaw();
            sleep(200);

            if (!SKIP_PICKING_WHITE_PIXELS_NEAR) {
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
