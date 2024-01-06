package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.XBot.DEFAULT_DROP_ARM_POSITION;
import static org.firstinspires.ftc.teamcode.XBot.MIN_ARM_POSITION;
import static org.firstinspires.ftc.teamcode.XBot.WRIST_FLAT_TO_GROUND;
import static org.firstinspires.ftc.teamcode.XBot.WRIST_VERTICAL;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "Auto Blue Near Left", group = "Concept")
public class AutoBlueNearLeft extends XBotAutoOpMode implements AutoOpMode {
    @Override
    public void runOpMode() {
        // Initialize hardware
        initializeAuto();
        Pose2d startPose = new Pose2d(16, 63.5, Math.toRadians(90));
        mecanumDrive.setPoseEstimate(startPose);
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        if (opModeIsActive()) {
//            while (opModeIsActive()) {
            //autonomousPlay(Alliance.BLUE, DistanceFromBackdrop.NEAR, Parking.LEFT);
            TrajectorySequence trajSeq1 = mecanumDrive.trajectorySequenceBuilder(startPose)
                    .back(27.5)
                    .turn(Math.toRadians(90))
                    .back(21)
                    .build();

            TrajectorySequence trajSeq2 = mecanumDrive.trajectorySequenceBuilder(trajSeq1.end())
                    .strafeRight(8.5)
                    .back(6.5)
                    .build();

            TrajectorySequence trajSeq3 = mecanumDrive.trajectorySequenceBuilder(trajSeq2.end())
                    .strafeTo(new Vector2d(42.5, 9.5))
                    .lineTo(new Vector2d(-50, 9.5))
                    .build();

            TrajectorySequence trajSeq4 = mecanumDrive.trajectorySequenceBuilder(trajSeq3.end())
                    .forward(5)
                    .build();

            TrajectorySequence trajSeq5 = mecanumDrive.trajectorySequenceBuilder(trajSeq4.end())
                    .back(10)
                    .build();

            TrajectorySequence trajSeq6 = mecanumDrive.trajectorySequenceBuilder(trajSeq5.end())
                    .back(87)
                    .strafeTo(new Vector2d(42.5, 36))
                    .build();

            TrajectorySequence parkSeq6 = mecanumDrive.trajectorySequenceBuilder(trajSeq6.end())
                    .strafeRight(21)
                    .back(15)
                    .build();

//                Trajectory spline1 = mecanumDrive.trajectoryBuilder(new Pose2d())
//                        .splineTo(new Vector2d(36, 36), 180,
//                                SampleMecanumDrive.getVelocityConstraint(30,
//                                        DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                        .build();
            sleep(10);

            if (isStopRequested()) return;
            mecanumDrive.followTrajectorySequence(trajSeq1);

            setWristPosition(WRIST_FLAT_TO_GROUND); sleep(200);

            //Purple in right claw
            openLeftClaw(); sleep(200);

            //Move Wrist up
            setWristPosition(WRIST_VERTICAL); sleep(100);

            mecanumDrive.followTrajectorySequence(trajSeq2); sleep(100);

            moveArmToPosition(DEFAULT_DROP_ARM_POSITION);
            sleep(1200);

            //Yellow Pixel on board
            openRightClaw();
            sleep(200);
            moveArmToPosition(MIN_ARM_POSITION + 45);
            sleep(200);

            mecanumDrive.followTrajectorySequence(trajSeq3);
            setWristPosition(WRIST_FLAT_TO_GROUND);
            sleep(500);

            mecanumDrive.followTrajectorySequence(trajSeq4); sleep(100);

            //Pick White Pixels
            closeBothClaws();
            sleep(200);

            mecanumDrive.followTrajectorySequence(trajSeq5); sleep(100);
            setWristPosition(WRIST_VERTICAL);

            mecanumDrive.followTrajectorySequence(trajSeq6); sleep(100);

            moveArmToPosition(DEFAULT_DROP_ARM_POSITION);
            sleep(1200);

            openBothClaws();
            sleep(200);

            moveArmToPosition(MIN_ARM_POSITION);
            mecanumDrive.followTrajectorySequence(parkSeq6);
//            }
        }
//        stopRobot();
    }
}
