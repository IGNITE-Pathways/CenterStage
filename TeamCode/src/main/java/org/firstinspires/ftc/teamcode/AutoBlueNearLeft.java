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

@Autonomous(name = "Auto Blue Near Left", group = "Concept")
public class AutoBlueNearLeft extends XBotAutoOpMode implements AutoOpMode {
    @Override
    public void runOpMode() {
        // Initialize hardware
        Double DROP_LINE_X = 43.0;
        Double WHITE_STACK_Y = 9.0;
        Double WHITE_STACK_X = -50.0;

        initializeAuto();
        Pose2d startPose = new Pose2d(15, 63.5, Math.toRadians(90));
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

//            spikeMark = SpikeMark.CENTER; //@TODO:TESTING

            //autonomousPlay(Alliance.BLUE, DistanceFromBackdrop.NEAR, Parking.LEFT);
            TrajectorySequence trajToDropPurplePixel = xDrive.trajectorySequenceBuilder(startPose)
                    .back(27.5)
                    .turn(Math.toRadians(90))
                    .back(21)
                    .build();

            Trajectory trajToDropYellowPixel = xDrive.trajectoryBuilder(trajToDropPurplePixel.end(), true)
                    .splineTo(new Vector2d(DROP_LINE_X, 44.5), 0,
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
                        .splineTo(new Vector2d(DROP_LINE_X, 32.5), 0,
                                SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                        .build();
            } else if (spikeMark == SpikeMark.CENTER) {
                trajToDropPurplePixel = xDrive.trajectorySequenceBuilder(startPose)
                        .back(34)
                        .turn(Math.toRadians(90))
                        .back(8)
                        .build();

                trajToDropYellowPixel = xDrive.trajectoryBuilder(trajToDropPurplePixel.end(), true)
                        .splineTo(new Vector2d(DROP_LINE_X, 38), 0,
                                SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                        .build();
            }

//            Trajectory trajToDropYellowPixel = xDrive.trajectoryBuilder(trajToDropPurplePixel.end())
//                    .strafeRight(8.5)
//                    .back(6.5)
//                    .build();

            TrajectorySequence trajToPickWhitePixels = xDrive.trajectorySequenceBuilder(trajToDropYellowPixel.end())
                    .strafeTo(new Vector2d(DROP_LINE_X, WHITE_STACK_Y))
                    .lineTo(new Vector2d(WHITE_STACK_X, WHITE_STACK_Y))
                    .build();

            TrajectorySequence inchForward = xDrive.trajectorySequenceBuilder(trajToPickWhitePixels.end())
                    .forward(5)
                    .build();

            TrajectorySequence inchBackward = xDrive.trajectorySequenceBuilder(inchForward.end())
                    .back(10)
                    .build();

            TrajectorySequence trajBackToDropWhitePixles = xDrive.trajectorySequenceBuilder(inchBackward.end())
                    .lineTo(new Vector2d(DROP_LINE_X, WHITE_STACK_Y))
                    .strafeTo(new Vector2d(DROP_LINE_X, 36))
                    .build();

            TrajectorySequence parkingLeftSeq = xDrive.trajectorySequenceBuilder(trajBackToDropWhitePixles.end())
                    .strafeRight(22.5)
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
            xDrive.followTrajectorySequence(parkingLeftSeq);
        }
        stopRobot();
    }


}
