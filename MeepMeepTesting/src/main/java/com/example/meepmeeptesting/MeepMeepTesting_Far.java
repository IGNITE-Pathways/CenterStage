package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueLight;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedLight;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.DriveTrainType;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting_Far {
    public static void main(String[] args) {
        Double DROP_LINE_X = 42.0;
        Double WHITE_STACK_Y = 9.0;
        Double WHITE_STACK_X = -50.0;

        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity redFarCenterPixelRightParking = new DefaultBotBuilder(meepMeep)
                .setDimensions(14,14)
                .setDriveTrainType(DriveTrainType.MECANUM)
                .setColorScheme(new ColorSchemeRedLight())
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(52.48291908330528, 52.48291908330528,
                        5.426119932065176, Math.toRadians(238.65474285714288), 12.75)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-36, -63.5, Math.toRadians(-90)))
                                .setReversed(true)
                                .strafeTo(new Vector2d(-50, -30))
                                .splineToConstantHeading(new Vector2d(-43, -15.5), Math.toRadians(-90))

//                                .back(51)
                                //Drop Pixel on Spikemark
                                .waitSeconds(1)
                                .turn(Math.toRadians(-90))
                                .lineTo(new Vector2d(DROP_LINE_X, -15.5))
                                .strafeTo(new Vector2d(DROP_LINE_X, -36))

//                                .splineTo(new Vector2d(25, -21), Math.toRadians(-35))
//                                .splineTo(new Vector2d(DROP_LINE_X, -36), 0)
                                //Drop Yellow Pixel on Backboard
                                .waitSeconds(1)
                                .strafeTo(new Vector2d(DROP_LINE_X, -WHITE_STACK_Y))
                                .lineTo(new Vector2d(WHITE_STACK_X, -WHITE_STACK_Y))
                                .forward(5)
                                //Pick White Pixels
                                .waitSeconds(0.5)
                                .back(10)
                                //Go Back
                                .lineTo(new Vector2d(DROP_LINE_X, -WHITE_STACK_Y))
                                .strafeTo(new Vector2d(DROP_LINE_X, -36))
                                //Drop White Pixels
                                .waitSeconds(1)
                                .strafeLeft(22.5)
                                //Park
                                .back(15)
                                .build()
                );

        RoadRunnerBotEntity blueFarCenterPixelLeftParking = new DefaultBotBuilder(meepMeep)
                .setDimensions(14,14)
                .setDriveTrainType(DriveTrainType.MECANUM)
                .setColorScheme(new ColorSchemeBlueLight())
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(52.48291908330528, 52.48291908330528,
                        5.426119932065176, Math.toRadians(238.65474285714288), 12.75)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-36, 63.5, Math.toRadians(90)))
                                .setReversed(true)
                                .strafeTo(new Vector2d(-50, 30))
                                .splineToConstantHeading(new Vector2d(-44, 15.5), Math.toRadians(90))
                                //Drop Pixel on Spikemark
                                .waitSeconds(1)
                                .turn(Math.toRadians(90))
                                .lineTo(new Vector2d(DROP_LINE_X, 15.5))
                                .strafeTo(new Vector2d(DROP_LINE_X, 36))
                                //Drop Yellow Pixel on Backboard
                                .waitSeconds(1)
                                .strafeTo(new Vector2d(DROP_LINE_X, WHITE_STACK_Y))
                                .lineTo(new Vector2d(WHITE_STACK_X, WHITE_STACK_Y))
                                .forward(5)
                                //Pick White Pixels
                                .waitSeconds(1)
                                .back(10)
                                //Go Back
                                .lineTo(new Vector2d(DROP_LINE_X, WHITE_STACK_Y))
                                .strafeTo(new Vector2d(DROP_LINE_X, 36))
                                //Drop White Pixels
                                .waitSeconds(1)
                                .strafeRight(22.5)
                                //Park
                                .back(15)
                                .build()
                );

        RoadRunnerBotEntity blueFarLeftPixelLeftParking = new DefaultBotBuilder(meepMeep)
                .setDimensions(14,14)
                .setDriveTrainType(DriveTrainType.MECANUM)
                .setColorScheme(new ColorSchemeBlueLight())
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(52.48291908330528, 52.48291908330528,
                        5.426119932065176, Math.toRadians(238.65474285714288), 12.75)
                .followTrajectorySequence(drive ->
                                drive.trajectorySequenceBuilder(new Pose2d(-36, 63.5, Math.toRadians(90)))
                                        .setReversed(true)
                                        .back(29)
                                        .turn(Math.toRadians(-90))
                                        //Push the Team Prop
                                        .forward(10)
                                        //Move Back to drop Pixel
                                        .back(5)

                                        //Drop Pixel on Spikemark
                                        .waitSeconds(1)
                                        .back(10)
                                        .strafeTo(new Vector2d(-41, 12.5))
                                        //180 degree turn
                                        .turn(Math.toRadians(90))
                                        .turn(Math.toRadians(90))
                                        .lineTo(new Vector2d(DROP_LINE_X, 12.5))
                                        .strafeTo(new Vector2d(DROP_LINE_X, 42))

                                        //Drop Yellow Pixel on Backboard
                                        .waitSeconds(1)
                                        .strafeTo(new Vector2d(DROP_LINE_X, WHITE_STACK_Y))
                                        .lineTo(new Vector2d(WHITE_STACK_X, WHITE_STACK_Y))
                                        .forward(5)
                                        //Pick White Pixels
                                        .waitSeconds(1)
                                        .back(10)
                                        //Go Back
                                        .lineTo(new Vector2d(DROP_LINE_X, WHITE_STACK_Y))
                                        .strafeTo(new Vector2d(DROP_LINE_X, 36))
                                        //Drop White Pixels
                                        .waitSeconds(1)
                                        .strafeRight(22.5)
                                        //Park
                                        .back(15)
                                        .build()
                );

        RoadRunnerBotEntity redFarRightPixelRightParking = new DefaultBotBuilder(meepMeep)
                .setDimensions(14,14)
                .setDriveTrainType(DriveTrainType.MECANUM)
                .setColorScheme(new ColorSchemeRedLight())
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(52.48291908330528, 52.48291908330528,
                        5.426119932065176, Math.toRadians(238.65474285714288), 12.75)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-36, -63.5, Math.toRadians(-90)))
                                .setReversed(true)
                                .back(28.5)
                                .turn(Math.toRadians(90))
                                //Push the Team Prop
                                .forward(10)
                                //Move Back to drop Pixel
                                .back(11.5)

                                //Drop Pixel on Spikemark
                                .waitSeconds(1)
                                .strafeTo(new Vector2d(-41, -12.5))
                                //U Turn
                                .turn(Math.toRadians(180))
//                                .turn(Math.toRadians(90))
                                .lineTo(new Vector2d(DROP_LINE_X, -12.5))
                                .strafeTo(new Vector2d(DROP_LINE_X, -42.5))

                                //Drop Pixel on Spikemark
                                .waitSeconds(1)
                                .strafeTo(new Vector2d(DROP_LINE_X, -WHITE_STACK_Y))
                                .lineTo(new Vector2d(WHITE_STACK_X, -WHITE_STACK_Y))
                                .forward(5)
                                .waitSeconds(1)
                                .back(10)
                                .lineTo(new Vector2d(DROP_LINE_X, -WHITE_STACK_Y))
                                .strafeTo(new Vector2d(DROP_LINE_X, -36))
                                .waitSeconds(1)
                                .strafeLeft(22.5)
                                .back(15)
                                .build()
                );


        RoadRunnerBotEntity blueFarRightPixelLeftParking = new DefaultBotBuilder(meepMeep)
                .setDimensions(14,14)
                .setDriveTrainType(DriveTrainType.MECANUM)
                .setColorScheme(new ColorSchemeBlueLight())
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(52.48291908330528, 52.48291908330528,
                        5.426119932065176, Math.toRadians(238.65474285714288), 12.75)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-36, 63.5, Math.toRadians(90)))
                                .setReversed(true)
                                .back(29)
                                .turn(Math.toRadians(90))
                                .forward(10)
                                .back(9)

                                //Drop Pixel on Spikemark
                                .waitSeconds(1)
                                .strafeTo(new Vector2d(-37, 12.5))
                                .lineTo(new Vector2d(DROP_LINE_X, 12.5))
                                .strafeTo(new Vector2d(DROP_LINE_X, 44.5))

                                //Drop Yellow Pixel on Backboard
                                .waitSeconds(1)
                                .strafeTo(new Vector2d(DROP_LINE_X, WHITE_STACK_Y))
                                .lineTo(new Vector2d(WHITE_STACK_X, WHITE_STACK_Y))
                                .forward(5)
                                //Pick White Pixels
                                .waitSeconds(1)
                                .back(10)
                                //Go Back
                                .lineTo(new Vector2d(DROP_LINE_X, WHITE_STACK_Y))
                                .strafeTo(new Vector2d(DROP_LINE_X, 36))
                                //Drop White Pixels
                                .waitSeconds(1)
                                .strafeRight(22.5)
                                //Park
                                .back(15)
                                .build()
                );

        RoadRunnerBotEntity redFarLeftPixelRightParking = new DefaultBotBuilder(meepMeep)
                .setDimensions(14,14)
                .setDriveTrainType(DriveTrainType.MECANUM)
                .setColorScheme(new ColorSchemeRedLight())
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(52.48291908330528, 52.48291908330528,
                        5.426119932065176, Math.toRadians(238.65474285714288), 12.75)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-36, -63.5, Math.toRadians(-90)))
                                .setReversed(true)
                                .back(27.5)
                                .turn(Math.toRadians(-90))
                                .forward(10)
                                .back(11)

                                //Drop Pixel on Spikemark
                                .waitSeconds(1)
                                .strafeTo(new Vector2d(-37, -12.5))
                                .lineTo(new Vector2d(DROP_LINE_X, -12.5))
                                .strafeTo(new Vector2d(DROP_LINE_X, -32.5)) //ID 4 Red

                                //Drop Pixel on Spikemark
                                .waitSeconds(1)
                                .strafeTo(new Vector2d(DROP_LINE_X, -WHITE_STACK_Y))
                                .lineTo(new Vector2d(WHITE_STACK_X, -WHITE_STACK_Y))
                                .forward(5)
                                .waitSeconds(1)
                                .back(10)
                                .lineTo(new Vector2d(DROP_LINE_X, -WHITE_STACK_Y))
                                .strafeTo(new Vector2d(DROP_LINE_X, -36))
                                .waitSeconds(1)
                                .strafeLeft(22.5)
                                .back(15)
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_LIGHT)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
//
//                .addEntity(blueFarLeftPixelLeftParking)
//                .addEntity(redFarRightPixelRightParking)

//                .addEntity(blueFarCenterPixelLeftParking)
                .addEntity(redFarCenterPixelRightParking)
////
//                .addEntity(blueFarRightPixelLeftParking)
//                .addEntity(redFarLeftPixelRightParking)

                .start();
    }
}