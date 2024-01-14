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
        //RED OVERRIDE
//        WHITE_STACK_X = -50.0;

        startPose = pose2d;
        super.initializeAuto();
        xDrive.setPoseEstimate(startPose);

        trajectoryToDropPurplePixel = xDrive.trajectorySequenceBuilder(startPose)
                .back(27.5)
                .turn(Math.toRadians(-90))
                .back(23)
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
                    .strafeTo(new Vector2d(DROP_LINE_X, -44.5))
//                    .splineToConstantHeading(new Vector2d(DROP_LINE_X, -44.5), 0,
//                            SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                    .build();
        }

        trajectoryToPickWhitePixels = xDrive.trajectorySequenceBuilder(trajectoryToDropYellowPixel.end())
                .strafeTo(new Vector2d(DROP_LINE_X, -WHITE_STACK_Y - 1.5))
                .lineTo(new Vector2d(WHITE_STACK_X, -WHITE_STACK_Y - 1.5))
                .build();

        inchForward = xDrive.trajectorySequenceBuilder(trajectoryToPickWhitePixels.end()).forward(5).build();
        inchBackward = xDrive.trajectorySequenceBuilder(inchForward.end()).back(10).build();

        trajectoryToDropWhitePixels = xDrive.trajectorySequenceBuilder(inchBackward.end())
                .lineTo(new Vector2d(DROP_LINE_X, -WHITE_STACK_Y - 1.5))
                .strafeTo(new Vector2d(DROP_LINE_X, -39))
                .build();

        Double strafeDistance = PARKING_OFFSET;
        if (parking == Parking.RIGHT) {
            parkingSeq = xDrive.trajectorySequenceBuilder(trajectoryToDropWhitePixels.end()).strafeLeft(strafeDistance).back(15).build();
        } else {
            parkingSeq = xDrive.trajectorySequenceBuilder(trajectoryToDropWhitePixels.end()).strafeRight(strafeDistance).back(15).build();
        }

        waitForStart();
        runtime.reset();
    }
}
