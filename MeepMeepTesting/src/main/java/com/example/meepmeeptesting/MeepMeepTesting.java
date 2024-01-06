package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.ColorScheme;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueLight;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedLight;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.DriveTrainType;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);
        RoadRunnerBotEntity blueFarLeft = new DefaultBotBuilder(meepMeep)
                .setDimensions(14,14)
                .setDriveTrainType(DriveTrainType.MECANUM)
                .setColorScheme(new ColorSchemeBlueDark())
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(52.48291908330528, 52.48291908330528,
                        5.426119932065176, Math.toRadians(238.65474285714288), 12.75)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-32, 63.5, Math.toRadians(90)))
                                .back(34)
                                .turn(Math.toRadians(90))
                                .back(21)
                                //Drop Pixel on Spikemark
                                .strafeRight(10)
                                .back(4)
                                //Drop Pixel on Backdrop
                                .strafeRight(21)
                                .back(15)
                                .build()
                );

        RoadRunnerBotEntity blueNearLeft = new DefaultBotBuilder(meepMeep)
                .setDimensions(14,14)
                .setDriveTrainType(DriveTrainType.MECANUM)
                .setColorScheme(new ColorSchemeBlueLight())
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(52.48291908330528, 52.48291908330528,
                        5.426119932065176, Math.toRadians(238.65474285714288), 12.75)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(15, 63.5, Math.toRadians(90)))
                                .back(34)
                                .turn(Math.toRadians(90))
                                .back(21)
                                //Drop Pixel on Spikemark
                                .strafeRight(10)
                                .back(4)
                                //Drop Pixel on Backdrop
                                .strafeRight(21)
                                .back(15)
                                .build()
                );

        RoadRunnerBotEntity blueNearRight = new DefaultBotBuilder(meepMeep)
                .setDimensions(14,14)
                .setDriveTrainType(DriveTrainType.MECANUM)
                .setColorScheme(new ColorSchemeBlueLight())
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(52.48291908330528, 52.48291908330528,
                        5.426119932065176, Math.toRadians(238.65474285714288), 12.75)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(15, 63.5, Math.toRadians(90)))
                                .back(34)
                                .turn(Math.toRadians(90))
                                .back(21)
                                //Drop Pixel on Spikemark
                                .strafeRight(10)
                                .back(4)
                                //Drop Pixel on Backdrop
                                .strafeLeft(29)
                                .back(15)
                                .build()
                );

        RoadRunnerBotEntity redNearRight = new DefaultBotBuilder(meepMeep)
                .setDimensions(14,14)
                .setDriveTrainType(DriveTrainType.MECANUM)
                .setColorScheme(new ColorSchemeRedLight())
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(52.48291908330528, 52.48291908330528,
                        5.426119932065176, Math.toRadians(238.65474285714288), 12.75)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(15, -63.5, Math.toRadians(-90)))
                                .back(34)
                                .turn(Math.toRadians(-90))
                                .back(21)
                                //Drop Pixel on Spikemark
                                .strafeLeft(10)
                                .back(4)
                                //Drop Pixel on Backdrop
                                .strafeLeft(21)
                                .back(15)
                                .build()
                );

        RoadRunnerBotEntity redNearLeft = new DefaultBotBuilder(meepMeep)
                .setDimensions(14,14)
                .setDriveTrainType(DriveTrainType.MECANUM)
                .setColorScheme(new ColorSchemeRedLight())
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(52.48291908330528, 52.48291908330528,
                        5.426119932065176, Math.toRadians(238.65474285714288), 12.75)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(15, -63.5, Math.toRadians(-90)))
                                .back(34)
                                .turn(Math.toRadians(-90))
                                .back(21)
                                //Drop Pixel on Spikemark
                                .strafeLeft(10)
                                .back(4)
                                //Drop Pixel on Backdrop
                                .strafeRight(29)
                                .back(15)
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_LIGHT)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(blueFarLeft)
                .addEntity(blueNearLeft)
                .addEntity(redNearRight)
                .addEntity(redNearLeft)
                .addEntity(blueNearRight)
                .start();
    }
}