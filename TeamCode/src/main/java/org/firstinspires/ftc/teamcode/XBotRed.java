package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public abstract class XBotRed extends XBotAutoOpMode {
    TrajectorySequence trajectorySeqToDropPurplePixel = null;
    TrajectorySequence trajectorySeqToDropYellowPixel = null;
    TrajectorySequence trajectorySeqToPickWhitePixels = null;
    TrajectorySequence inchForwardSeq = null;
    TrajectorySequence inchBackwardSeq = null;
    TrajectorySequence trajectorySeqToDropWhitePixels = null;
    TrajectorySequence parkingSeq = null;
    Pose2d startPose = null;

    public void initializeAuto(Pose2d pose2d, DistanceFromBackdrop distanceFromBackdrop, Parking parking) {
        //RED OVERRIDE
//        WHITE_STACK_X = -50.0;

        startPose = pose2d;
        super.initializeAuto();
        xDrive.setPoseEstimate(startPose);

        if (distanceFromBackdrop == DistanceFromBackdrop.FAR) {
            trajectorySeqToDropPurplePixel = xDrive.trajectorySequenceBuilder(startPose)
                    .back(28.5)
                    .turn(Math.toRadians(90))
                    //Push the Team Prop
                    .forward(10)
                    //Move Back to drop Pixel
                    .back(11.5)
                    .build();

            trajectorySeqToDropYellowPixel = xDrive.trajectorySequenceBuilder(trajectorySeqToDropPurplePixel.end())
//                    .back(10)
                    .strafeTo(new Vector2d(-41, -12.5))
                    //U Turn
                    .turn(Math.toRadians(180))
                    .lineTo(new Vector2d(DROP_LINE_X, -12.5))
                    .strafeTo(new Vector2d(DROP_LINE_X, -42.5)) //ID 6 Red
                    .build();

            trajectorySeqToPickWhitePixels = xDrive.trajectorySequenceBuilder(trajectorySeqToDropYellowPixel.end())
                    .strafeTo(new Vector2d(DROP_LINE_X, -WHITE_STACK_Y - 2))
                    .lineTo(new Vector2d(WHITE_STACK_X, -WHITE_STACK_Y - 2))
                    .build();

        } else {
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

        waitForStart();
        runtime.reset();
    }
}
