package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.XBot.DEFAULT_DROP_ARM_POSITION;
import static org.firstinspires.ftc.teamcode.XBot.MIN_ARM_POSITION;
import static org.firstinspires.ftc.teamcode.XBot.WRIST_FLAT_TO_GROUND;
import static org.firstinspires.ftc.teamcode.XBot.WRIST_VERTICAL;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "Auto Blue Far Right", group = "Concept")
public class AutoBlueFarRight extends XBotAutoOpMode implements AutoOpMode {
    @Override
    public void runOpMode() {
        // Initialize hardware
        initializeAuto();
        Pose2d startPose = new Pose2d(-32, 63.5, Math.toRadians(90));
        xDrive.setPoseEstimate(startPose);
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        if (opModeIsActive()) {
            while (!teamPropDetectionCompleted) {
                detectTeamPropAndSwitchCameraToAprilTag();
            }
            telemetry.addData("SpikeMark", spikeMark + ", confidence" + detectionConfidence);
            //spikeMark is set

            spikeMark = SpikeMark.CENTER; //@TODO:TESTING

            //        autonomousPlay(Alliance.BLUE, DistanceFromBackdrop.FAR, Parking.RIGHT);
            TrajectorySequence trajToDropPurplePixel = xDrive.trajectorySequenceBuilder(startPose)
                    .back(27.5)
                    .turn(Math.toRadians(90))
                    .back(21)
                    .build();

            Trajectory trajToDropYellowPixel = xDrive.trajectoryBuilder(trajToDropPurplePixel.end(), true)
                    .back(40)
                    .splineTo(new Vector2d(43.5, 44.5), 0,
                            SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                    .build();

            if (spikeMark == SpikeMark.RIGHT) {
                trajToDropPurplePixel = xDrive.trajectorySequenceBuilder(startPose)
                        .back(27.5)
                        .turn(Math.toRadians(90))
                        .forward(10)
                        .back(9)
                        .build();

                trajToDropYellowPixel = xDrive.trajectoryBuilder(trajToDropPurplePixel.end(), true)
                        .back(40)
                        .splineTo(new Vector2d(43.5, 32.5), 0,
                                SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                        .build();
            } else if (spikeMark == SpikeMark.CENTER) {
                trajToDropPurplePixel = xDrive.trajectorySequenceBuilder(startPose)
                        .back(27.5)
                        .turn(Math.toRadians(135)) //@TODO: Calibrate
                        .build();

                trajToDropYellowPixel = xDrive.trajectoryBuilder(trajToDropPurplePixel.end(), true)
                        .splineTo(new Vector2d(-32, 36), 0,
                                SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                        .back(40)
                        .splineTo(new Vector2d(43.5, 38), 0,
                                SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                        .build();
            }

            TrajectorySequence trajToPickWhitePixels = xDrive.trajectorySequenceBuilder(trajToDropYellowPixel.end())
                    .strafeTo(new Vector2d(43.5, 9.5))
                    .lineTo(new Vector2d(-50, 9.5))
                    .build();

            TrajectorySequence inchForward = xDrive.trajectorySequenceBuilder(trajToPickWhitePixels.end())
                    .forward(5)
                    .build();

            TrajectorySequence inchBackward = xDrive.trajectorySequenceBuilder(inchForward.end())
                    .back(10)
                    .build();

            TrajectorySequence trajBackToDropWhitePixles = xDrive.trajectorySequenceBuilder(inchBackward.end())
                    .back(88.5)
                    .strafeTo(new Vector2d(43.5, 36))
                    .build();

            TrajectorySequence parkingRightSeq = xDrive.trajectorySequenceBuilder(trajBackToDropWhitePixles.end())
                    .strafeLeft(22)
                    .back(15)
                    .build();

            sleep(10);

            if (isStopRequested()) return;
            //STEP 1 -- Purple Pixel Drop on spike mark
            xDrive.followTrajectorySequence(trajToDropPurplePixel);
            setWristPosition(WRIST_FLAT_TO_GROUND); sleep(200);
            openLeftClaw(); sleep(200);

            //STEP 2 -- Yellow Pixel to back board
            setWristPosition(WRIST_VERTICAL); sleep(50);
            xDrive.followTrajectory(trajToDropYellowPixel); //sleep(100);
            moveArmToPosition(DEFAULT_DROP_ARM_POSITION ); sleep(1400);
            openRightClaw(); sleep(200);

            grabAndDropWhitePixels(trajToPickWhitePixels, inchForward, inchBackward, trajBackToDropWhitePixles);

            //STEP 7 -- Park
            moveArmToPosition(MIN_ARM_POSITION );
            xDrive.followTrajectorySequence(parkingRightSeq);
        }
        stopRobot();
    }
}
