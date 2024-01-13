package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.XBot.DEFAULT_DROP_ARM_POSITION;
import static org.firstinspires.ftc.teamcode.XBot.MIN_ARM_POSITION;
import static org.firstinspires.ftc.teamcode.XBot.WRIST_FLAT_TO_GROUND;
import static org.firstinspires.ftc.teamcode.XBot.WRIST_VERTICAL;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public abstract class XBotRedFar extends XBotRed {
    public void autoRedFar(Parking parking) {
        // Initialize hardware
        initializeAuto();
        Pose2d startPose = new Pose2d(-38, -63.5, Math.toRadians(-90));
        xDrive.setPoseEstimate(startPose);
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        if (opModeIsActive()) {
            while (!teamPropDetectionCompleted) {
                detectTeamPropAndSwitchCameraToAprilTag();
            }
            telemetry.addData("SpikeMark", spikeMark + ", confidence" + detectionConfidence);

            TrajectorySequence trajectoryToDropPurplePixel = xDrive.trajectorySequenceBuilder(startPose)
                    .back(27.5)
                    .turn(Math.toRadians(-90))
                    .back(26)
                    .build();

            Trajectory trajectoryToDropYellowPixel = xDrive.trajectoryBuilder(trajectoryToDropPurplePixel.end(), true)
                    .back(40)
                    .splineTo(new Vector2d(DROP_LINE_X, -42.5), 0,
                            SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                    .build();

            if (spikeMark == SpikeMark.LEFT) {
                 trajectoryToDropPurplePixel = xDrive.trajectorySequenceBuilder(startPose)
                        .back(27.5)
                        .turn(Math.toRadians(-90))
                        .forward(5)
                        .back(9)
                        .build();

                 trajectoryToDropYellowPixel = xDrive.trajectoryBuilder(trajectoryToDropPurplePixel.end(), true)
                        .back(40)
                        .splineTo(new Vector2d(DROP_LINE_X, -32.5), 0,
                                SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                        .build();
            } else if (spikeMark == SpikeMark.CENTER) {
                trajectoryToDropPurplePixel = xDrive.trajectorySequenceBuilder(startPose)
                        .back(51.5)
                        .build();

                trajectoryToDropYellowPixel = xDrive.trajectoryBuilder(trajectoryToDropPurplePixel.end(), true)
                        .splineTo(new Vector2d(10, -10), 0,
                                SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                        .splineToConstantHeading(new Vector2d(DROP_LINE_X, -38), 0,
                                SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                        .build();
            }

            TrajectorySequence trajectoryToPickWhitePixels = xDrive.trajectorySequenceBuilder(trajectoryToDropYellowPixel.end())
                    .strafeTo(new Vector2d(DROP_LINE_X, -WHITE_STACK_Y))
                    .lineTo(new Vector2d(WHITE_STACK_X, -WHITE_STACK_Y))
                    .build();

            TrajectorySequence inchForward = xDrive.trajectorySequenceBuilder(trajectoryToPickWhitePixels.end())
                    .forward(5)
                    .build();

            TrajectorySequence inchBackward = xDrive.trajectorySequenceBuilder(inchForward.end())
                    .back(10)
                    .build();

            TrajectorySequence trajectoryToDropWhitePixels = xDrive.trajectorySequenceBuilder(inchBackward.end())
                    .lineTo(new Vector2d(DROP_LINE_X, -WHITE_STACK_Y))
                    .strafeTo(new Vector2d(DROP_LINE_X, -36))
                    .build();

            TrajectorySequence parkingSeq = xDrive.trajectorySequenceBuilder(trajectoryToDropWhitePixels.end())
                    .strafeRight(22.5)
                    .back(15)
                    .build();

            if (parking == Parking.RIGHT) {
                parkingSeq = xDrive.trajectorySequenceBuilder(trajectoryToDropWhitePixels.end())
                        .strafeLeft(22.5)
                        .back(15)
                        .build();
            }

            sleep(10);

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

            grabAndDropWhitePixels(trajectoryToPickWhitePixels, inchForward, inchBackward, trajectoryToDropWhitePixels);

            //STEP 7 -- Park
            moveArmToPosition(MIN_ARM_POSITION);
            xDrive.followTrajectorySequence(parkingSeq);
        }

        stopRobot();
    }
}
