package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public abstract class XBotRed extends XBotAutoOpMode {
    TrajectorySequence trajectoryToDropPurplePixel = null;
    Trajectory trajectoryToDropYellowPixel = null;
    TrajectorySequence trajectoryToPickWhitePixels = null;
    TrajectorySequence inchForward = null;
    TrajectorySequence inchBackward = null;
    TrajectorySequence trajectoryToDropWhitePixels = null;
    TrajectorySequence parkingSeq = null;
    Pose2d startPose = null;

    public void initializeAuto(Pose2d pose2d, DistanceFromBackdrop distanceFromBackdrop, Parking parking) {
        startPose = pose2d;
        super.initializeAuto();
        xDrive.setPoseEstimate(startPose);

        trajectoryToDropPurplePixel = xDrive.trajectorySequenceBuilder(startPose)
                .back(27.5)
                .turn(Math.toRadians(-90))
                .back(26)
                .build();

        if (distanceFromBackdrop == DistanceFromBackdrop.FAR) {
            trajectoryToDropYellowPixel = xDrive.trajectoryBuilder(trajectoryToDropPurplePixel.end(), true)
                    .back(40)
                    .splineTo(new Vector2d(DROP_LINE_X, -42.5), 0,
                            SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                    .build();
        } else {
            trajectoryToDropYellowPixel = xDrive.trajectoryBuilder(trajectoryToDropPurplePixel.end(), true)
                    .splineTo(new Vector2d(DROP_LINE_X, -44.5), 0,
                            SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                    .build();
        }

        trajectoryToPickWhitePixels = xDrive.trajectorySequenceBuilder(trajectoryToDropYellowPixel.end())
                .strafeTo(new Vector2d(DROP_LINE_X, -WHITE_STACK_Y))
                .lineTo(new Vector2d(WHITE_STACK_X, -WHITE_STACK_Y))
                .build();

        inchForward = xDrive.trajectorySequenceBuilder(trajectoryToPickWhitePixels.end()).forward(5).build();
        inchBackward = xDrive.trajectorySequenceBuilder(inchForward.end()).back(10).build();

        trajectoryToDropWhitePixels = xDrive.trajectorySequenceBuilder(inchBackward.end())
                .lineTo(new Vector2d(DROP_LINE_X, -WHITE_STACK_Y))
                .strafeTo(new Vector2d(DROP_LINE_X, -36))
                .build();

        if (parking == Parking.RIGHT) {
            parkingSeq = xDrive.trajectorySequenceBuilder(trajectoryToDropWhitePixels.end()).strafeLeft(22.5).back(15).build();
        } else {
            parkingSeq = xDrive.trajectorySequenceBuilder(trajectoryToDropWhitePixels.end()).strafeRight(22.5).back(15).build();
        }

        waitForStart();
        runtime.reset();
    }
}
