package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.MoveRobot.TANK_TURN_LEFT;
import static org.firstinspires.ftc.teamcode.MoveRobot.TANK_TURN_RIGHT;
import static org.firstinspires.ftc.teamcode.XBot.ARM_POSITION_HIGH;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Auto Red Far Left", group = "Concept")
public class AutoRedFarLeft extends XBotAutoOpMode implements AutoOpMode {
    @Override
    public void runOpMode() {
        // Initialize hardware
        initializeAuto();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                if (DEBUG) {
                    //Debug
                    moveArmToPosition(ARM_POSITION_HIGH);
                    moveRobot(1025, TANK_TURN_LEFT);
                    moveRobot(1025, TANK_TURN_RIGHT);
                    fixRobotYaw(0);
                    continue;
                }

                autonomousPlay(Alliance.RED, DistanceFromBackdrop.FAR, Parking.LEFT);
                sleep(10);
            }
        }
        stopRobot();
    }
}
