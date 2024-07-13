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

    public void initializeAuto(Pose2d pose2d) {
        //RED OVERRIDE
//        WHITE_STACK_X = -50.0;

        startPose = pose2d;
        super.initializeAuto();
        xDrive.setPoseEstimate(startPose);

        waitForStart();
        runtime.reset();
    }
}
