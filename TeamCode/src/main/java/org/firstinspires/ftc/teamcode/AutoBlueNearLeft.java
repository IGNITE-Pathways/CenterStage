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
        xDrive.setPoseEstimate(startPose);
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        if (opModeIsActive()) {
            //autonomousPlay(Alliance.BLUE, DistanceFromBackdrop.NEAR, Parking.LEFT);
            TrajectorySequence trajToDropFirstPixel = xDrive.trajectorySequenceBuilder(startPose)
                    .back(27.5)
                    .turn(Math.toRadians(90))
                    .back(21)
                    .build();

            TrajectorySequence trajToDropSecondPixel = xDrive.trajectorySequenceBuilder(trajToDropFirstPixel.end())
                    .strafeRight(8.5)
                    .back(6.5)
                    .build();

            TrajectorySequence trajToPickWhitePixels = xDrive.trajectorySequenceBuilder(trajToDropSecondPixel.end())
                    .strafeTo(new Vector2d(42.5, 9.5))
                    .lineTo(new Vector2d(-50, 9.5))
                    .build();

            TrajectorySequence trajSeq4 = xDrive.trajectorySequenceBuilder(trajToPickWhitePixels.end())
                    .forward(5)
                    .build();

            TrajectorySequence trajSeq5 = xDrive.trajectorySequenceBuilder(trajSeq4.end())
                    .back(10)
                    .build();

            TrajectorySequence trajBackToDropWhitePixles = xDrive.trajectorySequenceBuilder(trajSeq5.end())
                    .back(87)
                    .strafeTo(new Vector2d(42.5, 36))
                    .build();

            TrajectorySequence parkingSeq = xDrive.trajectorySequenceBuilder(trajBackToDropWhitePixles.end())
                    .strafeRight(21)
                    .back(15)
                    .build();

            sleep(10);

            if (isStopRequested()) return;
            //STEP 1 -- Purple Pixel Drop on spike mark
            xDrive.followTrajectorySequence(trajToDropFirstPixel);
            setWristPosition(WRIST_FLAT_TO_GROUND); sleep(200);
            openLeftClaw(); sleep(200);

            //STEP 2 -- Yellow Pixel to back board
            setWristPosition(WRIST_VERTICAL); sleep(50);
            xDrive.followTrajectorySequence(trajToDropSecondPixel); //sleep(100);
            moveArmToPosition(DEFAULT_DROP_ARM_POSITION, true); sleep(1400);
            openRightClaw(); sleep(200);

            //STEP 3 -- Go to pick 2 White Pixels
            moveArmToPosition(MIN_ARM_POSITION + 40, false); //sleep(200);
            xDrive.followTrajectorySequence(trajToPickWhitePixels);
            setWristPosition(WRIST_FLAT_TO_GROUND); sleep(200);

            //STEP 4 -- Move forward to grab pixels
            xDrive.followTrajectorySequence(trajSeq4); sleep(100);
            closeBothClaws(); sleep(200);

            //STEP 5 -- Move back to make sure pixels are in claw
            xDrive.followTrajectorySequence(trajSeq5); sleep(100);
            setWristPosition(WRIST_VERTICAL);

            //STEP 6 -- Going to drop white Pixels
            xDrive.followTrajectorySequence(trajBackToDropWhitePixles); sleep(100);
            moveArmToPosition(DEFAULT_DROP_ARM_POSITION, true); sleep(1400);
            openBothClaws(); sleep(200);

            //STEP 7 -- Park
            moveArmToPosition(MIN_ARM_POSITION, false);
            xDrive.followTrajectorySequence(parkingSeq);
        }
        stopRobot();
    }
}
