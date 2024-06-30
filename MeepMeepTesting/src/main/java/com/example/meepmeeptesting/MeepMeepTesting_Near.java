package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueLight;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedLight;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.DriveTrainType;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting_Near {
    public static void main(String[] args) {
        Double DROP_LINE_X = 42.0;
        Double WHITE_STACK_Y = 9.0;
        Double WHITE_STACK_X = -50.0;

        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity blueNearLeftPixelLeftParking = new DefaultBotBuilder(meepMeep)
                .setDimensions(14,14)
                .setDriveTrainType(DriveTrainType.MECANUM)
                .setColorScheme(new ColorSchemeBlueLight())
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(52.48291908330528, 52.48291908330528,
                        5.426119932065176, Math.toRadians(238.65474285714288), 12.75)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(12, 63.5, Math.toRadians(90)))
                                .back(27.5)
                                .turn(Math.toRadians(90))
                                .back(21)
                                .waitSeconds(1)
                                .setReversed(true)
//                                .strafeTo(new Vector2d(DROP_LINE_X, 44.5))
                                .splineToConstantHeading(new Vector2d(DROP_LINE_X, 44.5), 0)
                                //Drop Pixel on Backdrop
                                .waitSeconds(1)

//                                .strafeTo(new Vector2d(DROP_LINE_X, WHITE_STACK_Y))
//                                .lineTo(new Vector2d(-50, WHITE_STACK_Y))
//                                .forward(5)
//                                .waitSeconds(1)
//                                .back(10)
//                                .lineTo(new Vector2d(DROP_LINE_X, WHITE_STACK_Y))
//                                .strafeTo(new Vector2d(DROP_LINE_X, 36))
//                                .waitSeconds(1)
//                                .strafeRight(22.5)
//                                .back(15)
                                .build()
                );

        RoadRunnerBotEntity blueNearCenterPixelLeftParking = new DefaultBotBuilder(meepMeep)
                .setDimensions(14,14)
                .setDriveTrainType(DriveTrainType.MECANUM)
                .setColorScheme(new ColorSchemeBlueLight())
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(52.48291908330528, 52.48291908330528,
                        5.426119932065176, Math.toRadians(238.65474285714288), 12.75)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(12, 63.5, Math.toRadians(90)))
                                .strafeTo(new Vector2d(26, 30))
                                .splineToConstantHeading(new Vector2d(20, 15.5), Math.toRadians(90))
                                //DROP
                                .waitSeconds(1)
                                .setReversed(true)
                                .splineTo(new Vector2d(DROP_LINE_X, 38), 0)
                                .waitSeconds(1)
                                //DROP
//                                .strafeTo(new Vector2d(DROP_LINE_X, WHITE_STACK_Y))
//                                .lineTo(new Vector2d(WHITE_STACK_X, WHITE_STACK_Y))
//                                .forward(5)
//                                .back(10)
//                                .lineTo(new Vector2d(DROP_LINE_X, WHITE_STACK_Y))
//                                .strafeTo(new Vector2d(DROP_LINE_X, 36))
//                                .waitSeconds(1)

                                .strafeRight(22.5)
                                .back(15)
                                .build()
                );

        RoadRunnerBotEntity blueNearRightPixelLeftParking = new DefaultBotBuilder(meepMeep)
                .setDimensions(14,14)
                .setDriveTrainType(DriveTrainType.MECANUM)
                .setColorScheme(new ColorSchemeBlueLight())
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(52.48291908330528, 52.48291908330528,
                        5.426119932065176, Math.toRadians(238.65474285714288), 12.75)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(12, 63.5, Math.toRadians(90)))
                                .setReversed(true)
                                .splineTo(new Vector2d(20, 35), 0)
//                                .splineToConstantHeading(new Vector2d(15, 35), 0)
//                                .back(27.5)
//                                .turn(Math.toRadians(90))
                                .forward(5)
//                                .back(9)
                                .waitSeconds(1)
                                //Drop Purple Pixel on Spikemark
                                .setReversed(true)
                                .splineTo(new Vector2d(DROP_LINE_X, 32.5), 0)
                                //Drop Yellow Pixel on Backdrop
//                                .waitSeconds(1)
//                                //Go get White Pixels
//                                .strafeTo(new Vector2d(DROP_LINE_X, WHITE_STACK_Y))
//                                .lineTo(new Vector2d(WHITE_STACK_X, WHITE_STACK_Y))
//                                .forward(5)
//                                .waitSeconds(1)
//                                .back(10)
//                                .lineTo(new Vector2d(DROP_LINE_X, WHITE_STACK_Y))
//                                .strafeTo(new Vector2d(DROP_LINE_X, 36))
//                                .waitSeconds(1)
                                //Park
//                                .splineTo(new Vector2d(DROP_LINE_X, 32.5), 0)
//                                .strafeRight(22.5)
//                                .back(15)
                                .build()
                );

        RoadRunnerBotEntity redNearLeftPixelRightParking = new DefaultBotBuilder(meepMeep)
                .setDimensions(14,14)
                .setDriveTrainType(DriveTrainType.MECANUM)
                .setColorScheme(new ColorSchemeRedLight())
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(52.48291908330528, 52.48291908330528,
                        5.426119932065176, Math.toRadians(238.65474285714288), 12.75)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(12, -63.5, Math.toRadians(-90)))
                                .setReversed(true)
                                .splineTo(new Vector2d(20, -35), 0)
                                .forward(10)
                                .back(5)
                                //Drop Pixel on Spikemark
                                .waitSeconds(1)
                                .setReversed(true)
                                .splineTo(new Vector2d(DROP_LINE_X, -32.5), 0)
                                //Drop Pixel on Backdrop
                                .waitSeconds(1)

//                                .strafeTo(new Vector2d(DROP_LINE_X, -WHITE_STACK_Y))
//                                .lineTo(new Vector2d(WHITE_STACK_X, -WHITE_STACK_Y))
//                                .forward(5)
//                                .waitSeconds(1)
//                                .back(10)
//                                .lineTo(new Vector2d(DROP_LINE_X, -WHITE_STACK_Y))
//                                .strafeTo(new Vector2d(DROP_LINE_X, -36))
//                                .waitSeconds(1)
//                                .strafeLeft(22.5)
//                                .back(15)
                                .build()
                );

        RoadRunnerBotEntity redNearCenterPixelRightParking = new DefaultBotBuilder(meepMeep)
                .setDimensions(14,14)
                .setDriveTrainType(DriveTrainType.MECANUM)
                .setColorScheme(new ColorSchemeRedLight())
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(52.48291908330528, 52.48291908330528,
                        5.426119932065176, Math.toRadians(238.65474285714288), 12.75)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(12, -63.5, Math.toRadians(-90)))
                                .setReversed(true)
                                .splineTo(new Vector2d(30, -34), 0)
                                .strafeRight(6)

                                .waitSeconds(1)
                                .splineTo(new Vector2d(DROP_LINE_X, -37.5), 0)
                                .waitSeconds(1)

//                                .strafeTo(new Vector2d(DROP_LINE_X, -WHITE_STACK_Y - 1 ))
//                                .lineTo(new Vector2d(WHITE_STACK_X, -WHITE_STACK_Y - 1))
//                                .forward(5)
//                                .waitSeconds(1)
//                                .back(10)
//                                .lineTo(new Vector2d(DROP_LINE_X, -WHITE_STACK_Y - 1))
//                                .strafeTo(new Vector2d(DROP_LINE_X, -36))
//                                .waitSeconds(1)
//                                .strafeLeft(22.5)
//                                .back(15)
                                .build()
                );

        RoadRunnerBotEntity redNearRightPixelRightParking = new DefaultBotBuilder(meepMeep)
                .setDimensions(14,14)
                .setDriveTrainType(DriveTrainType.MECANUM)
                .setColorScheme(new ColorSchemeRedLight())
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(52.48291908330528, 52.48291908330528,
                        5.426119932065176, Math.toRadians(238.65474285714288), 12.75)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(12, -63.5, Math.toRadians(-90)))
                                .back(27.5)
                                .turn(Math.toRadians(-90))
                                .back(23)
                                .waitSeconds(1)
                                .strafeTo(new Vector2d(DROP_LINE_X, -44.5))
////                                .setReversed(true)
////                                .splineToConstantHeading(new Vector2d(DROP_LINE_X, -44.5), 0)
//                                .waitSeconds(1)
//                                .strafeTo(new Vector2d(DROP_LINE_X, -WHITE_STACK_Y))
//                                .lineTo(new Vector2d(WHITE_STACK_X, -WHITE_STACK_Y))
//                                .forward(5)
//                                .waitSeconds(1)
//                                .back(10)
//                                .lineTo(new Vector2d(DROP_LINE_X, -WHITE_STACK_Y))
//                                .strafeTo(new Vector2d(DROP_LINE_X, -36))
//                                .waitSeconds(1)
//                                .strafeLeft(22.5)
//                                .back(15)
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_LIGHT)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)

//                .addEntity(blueNearLeftPixelLeftParking)
//                .addEntity(redNearRightPixelRightParking)

//                .addEntity(blueNearCenterPixelLeftParking)
//                .addEntity(redNearCenterPixelRightParking)
//
                .addEntity(blueNearRightPixelLeftParking)
//                .addEntity(redNearLeftPixelRightParking)

                .start();
    }
}