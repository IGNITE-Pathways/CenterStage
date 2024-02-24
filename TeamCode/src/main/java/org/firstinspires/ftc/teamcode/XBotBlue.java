package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public abstract class XBotBlue extends XBotAutoOpMode {
    TrajectorySequence trajectorySeqToDropPurplePixel = null;
    Trajectory trajectoryToDropYellowPixel = null;
    TrajectorySequence trajectorySeqToDropYellowPixel = null;
    TrajectorySequence trajectorySeqToPickWhitePixels = null;
    TrajectorySequence inchForwardSeq = null;
    TrajectorySequence inchBackwardSeq = null;
    TrajectorySequence trajectorySeqToDropWhitePixels = null;
    TrajectorySequence parkingSeq = null;
    Pose2d startPose = null;

    public void initializeAuto(Pose2d pose2d, DistanceFromBackdrop distanceFromBackdrop, Parking parking) {
        startPose = pose2d;
        super.initializeAuto();
        xDrive.setPoseEstimate(startPose);

        //RIGHT Spike Mark -- Initialized for default
        if (distanceFromBackdrop == DistanceFromBackdrop.FAR) {
            //FAR
            trajectorySeqToDropPurplePixel = xDrive.trajectorySequenceBuilder(startPose)
                    .back(29)
                    .turn(Math.toRadians(90))
                    .forward(10)
                    .back(11.5)
                    .build();

            trajectorySeqToDropYellowPixel = xDrive.trajectorySequenceBuilder(trajectorySeqToDropPurplePixel.end())
                    .strafeTo(new Vector2d(-37, 12.5))
                    .lineTo(new Vector2d(DROP_LINE_X, 12.5))
                    .strafeTo(new Vector2d(DROP_LINE_X, 44.5))
                    .build();

//            trajectoryToDropYellowPixel = xDrive.trajectoryBuilder(trajectorySeqToDropPurplePixel.end(), true)
//                    .back(40)
//                    .splineTo(new Vector2d(DROP_LINE_X, 32.5), 0,
//                            SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                    .build();
        } else {
            //NEAR
            trajectorySeqToDropPurplePixel = xDrive.trajectorySequenceBuilder(startPose)
                    .setReversed(true)
                    .splineTo(new Vector2d(20, 35), 0)
                    .forward(10)
                    .back(5)
                    .build();

            trajectoryToDropYellowPixel = xDrive.trajectoryBuilder(trajectorySeqToDropPurplePixel.end(), true)
                    .splineTo(new Vector2d(DROP_LINE_X, 32.5), 0,
                            SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                    .build();
        }

        trajectorySeqToPickWhitePixels = xDrive.trajectorySequenceBuilder(trajectoryToDropYellowPixel.end())
                .strafeTo(new Vector2d(DROP_LINE_X, WHITE_STACK_Y))
                .lineTo(new Vector2d(WHITE_STACK_X, WHITE_STACK_Y))
                .build();

        inchForwardSeq = xDrive.trajectorySequenceBuilder(trajectorySeqToPickWhitePixels.end()).forward(5).build();
        inchBackwardSeq = xDrive.trajectorySequenceBuilder(inchForwardSeq.end()).back(10).build();

        trajectorySeqToDropWhitePixels = xDrive.trajectorySequenceBuilder(inchBackwardSeq.end())
                .lineTo(new Vector2d(DROP_LINE_X, WHITE_STACK_Y))
                .strafeTo(new Vector2d(DROP_LINE_X, 36))
                .build();

        if (parking == Parking.RIGHT) {
            parkingSeq = xDrive.trajectorySequenceBuilder(trajectorySeqToDropWhitePixels.end())
                    .strafeLeft(PARKING_OFFSET)
                    .back(15).build();
        } else {
            parkingSeq = xDrive.trajectorySequenceBuilder(trajectorySeqToDropWhitePixels.end())
                    .strafeRight(PARKING_OFFSET)
                    .back(15).build();
        }

        waitForStart();
        runtime.reset();
    }
}
