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
        super.initializeAuto(new Pose2d(12.0, -63.5, Math.toRadians(-90)));

        if (opModeIsActive()) {
            while (!teamPropDetectionCompleted) {
                detectTeamPropAndSwitchCameraToAprilTag();
            }

            telemetry.addData("SpikeMark", spikeMark + ", confidence" + detectionConfidence);
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

                trajectorySeqToPickWhitePixels = xDrive.trajectorySequenceBuilder(trajectorySeqToDropYellowPixel.end())
                        .strafeTo(new Vector2d(DROP_LINE_X, -WHITE_STACK_Y - 1.5))
                        .lineTo(new Vector2d(WHITE_STACK_X, -WHITE_STACK_Y - 1.5))
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

                trajectorySeqToPickWhitePixels = xDrive.trajectorySequenceBuilder(trajectorySeqToDropYellowPixel.end())
                        .strafeTo(new Vector2d(DROP_LINE_X, -WHITE_STACK_Y - 1.5))
                        .lineTo(new Vector2d(WHITE_STACK_X, -WHITE_STACK_Y - 1.5))
                        .build();

            } else if (spikeMark == SpikeMark.RIGHT) {
                trajectorySeqToDropPurplePixel = xDrive.trajectorySequenceBuilder(startPose)
                        .back(27.5)
                        .turn(Math.toRadians(-90))
                        .back(23)
                        .build();

                trajectorySeqToDropYellowPixel = xDrive.trajectorySequenceBuilder(trajectorySeqToDropPurplePixel.end())
                        .strafeTo(new Vector2d(DROP_LINE_X, -43))
                        .build();

                trajectorySeqToPickWhitePixels = xDrive.trajectorySequenceBuilder(trajectorySeqToDropYellowPixel.end())
                        .strafeTo(new Vector2d(DROP_LINE_X, -WHITE_STACK_Y - 2))
                        .lineTo(new Vector2d(WHITE_STACK_X, -WHITE_STACK_Y - 2))
                        .build();
            }

            inchForwardSeq = xDrive.trajectorySequenceBuilder(trajectorySeqToPickWhitePixels.end()).forward(5).build();
            inchBackwardSeq = xDrive.trajectorySequenceBuilder(inchForwardSeq.end()).back(10).build();

            trajectorySeqToDropWhitePixels = xDrive.trajectorySequenceBuilder(inchBackwardSeq.end())
                    .lineTo(new Vector2d(DROP_LINE_X, -WHITE_STACK_Y - 1.5))
                    .strafeTo(new Vector2d(DROP_LINE_X, -39))
                    .build();

            Double strafeDistance = PARKING_OFFSET;
            if (parking == Parking.RIGHT) {
                parkingSeq = xDrive.trajectorySequenceBuilder(trajectorySeqToDropWhitePixels.end()).strafeLeft(strafeDistance).back(15).build();
            } else {
                parkingSeq = xDrive.trajectorySequenceBuilder(trajectorySeqToDropWhitePixels.end()).strafeRight(strafeDistance).back(15).build();
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
            moveArmToPosition(DEFAULT_DROP_ARM_POSITION + 20, 0.3);
            sleep(600);

            openLeftClaw();
            sleep(200);

            if (!SKIP_PICKING_WHITE_PIXELS_NEAR) {
                //STEP 3 to 6 -- Grab White Pixels and drop the on backboard
                grabAndDropWhitePixels(trajectorySeqToPickWhitePixels, inchForwardSeq, inchBackwardSeq, trajectorySeqToDropWhitePixels);
                if (parking == Parking.RIGHT) {
                    parkingSeq = xDrive.trajectorySequenceBuilder(trajectorySeqToDropWhitePixels.end()).strafeLeft(strafeDistance).back(15).build();
                } else {
                    parkingSeq = xDrive.trajectorySequenceBuilder(trajectorySeqToDropWhitePixels.end()).strafeRight(strafeDistance).back(15).build();
                }
            } else {
                if (parking == Parking.RIGHT) {
                    parkingSeq = xDrive.trajectorySequenceBuilder(trajectorySeqToDropYellowPixel.end()).strafeLeft(PARKING_OFFSET).back(15).build();
                } else {
                    parkingSeq = xDrive.trajectorySequenceBuilder(trajectorySeqToDropYellowPixel.end()).strafeRight(PARKING_OFFSET).back(15).build();
                }
            }
            //STEP 7 -- Park
            moveArmToPosition(MIN_ARM_POSITION);
            xDrive.followTrajectorySequence(parkingSeq);
        }
        stopRobot();
    }
}
