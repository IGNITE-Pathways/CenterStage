package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.XBot.DEFAULT_DROP_ARM_POSITION;
import static org.firstinspires.ftc.teamcode.XBot.MIN_ARM_POSITION;
import static org.firstinspires.ftc.teamcode.XBot.WRIST_FLAT_TO_GROUND;
import static org.firstinspires.ftc.teamcode.XBot.WRIST_VERTICAL;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public abstract class XBotRedNear extends XBotRed {
    public void autoRedNear(Parking parking) {
        super.initializeAuto(new Pose2d(12, -63.5, Math.toRadians(-90)), DistanceFromBackdrop.FAR, parking);

        if (opModeIsActive()) {
            while (!teamPropDetectionCompleted) {
                detectTeamPropAndSwitchCameraToAprilTag();
            }
            telemetry.addData("SpikeMark", spikeMark + ", confidence" + detectionConfidence);
            if (spikeMark != SpikeMark.RIGHT) {

                if (spikeMark == SpikeMark.LEFT) {
                    trajectoryToDropPurplePixel = xDrive.trajectorySequenceBuilder(startPose)
                            .setReversed(true)
                            .splineTo(new Vector2d(20, -35), 0)
                            .forward(11)
                            .back(5)
                            .build();

                    trajectoryToDropYellowPixel = xDrive.trajectoryBuilder(trajectoryToDropPurplePixel.end(), true)
                            .splineTo(new Vector2d(DROP_LINE_X, -32.5), 0,
                                    SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                    SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                            .build();
                } else if (spikeMark == SpikeMark.CENTER) {
                    trajectoryToDropPurplePixel = xDrive.trajectorySequenceBuilder(startPose)
                            .setReversed(true)
                            .splineTo(new Vector2d(15, -29), Math.toRadians(90),
                                    SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                    SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                            .turn(Math.toRadians(-90))
                            .back(8)
                            .build();

                    trajectoryToDropYellowPixel = xDrive.trajectoryBuilder(trajectoryToDropPurplePixel.end(), true)
                            .splineTo(new Vector2d(DROP_LINE_X, -38), 0,
                                    SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                    SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                            .build();
                }
            }

            if (isStopRequested()) return;
            //STEP 1 -- Purple Pixel Drop on spike mark
            xDrive.followTrajectorySequence(trajectoryToDropPurplePixel);
            setWristPosition(WRIST_FLAT_TO_GROUND);
            sleep(200);
            openRightClaw();
            sleep(200);

            //STEP 2 -- Yellow Pixel to back board
            setWristPosition(WRIST_VERTICAL);
            sleep(50);
            xDrive.followTrajectory(trajectoryToDropYellowPixel); //sleep(100);
            moveArmToPosition(DEFAULT_DROP_ARM_POSITION);
            sleep(1400);
            openLeftClaw();
            sleep(200);

            //STEP 3 to 6 -- Grab White Pixels and drop the on backboard
            grabAndDropWhitePixels(trajectoryToPickWhitePixels, inchForward, inchBackward, trajectoryToDropWhitePixels);

            //STEP 7 -- Park
            moveArmToPosition(MIN_ARM_POSITION);
            xDrive.followTrajectorySequence(parkingSeq);
        }
        stopRobot();
    }
}
