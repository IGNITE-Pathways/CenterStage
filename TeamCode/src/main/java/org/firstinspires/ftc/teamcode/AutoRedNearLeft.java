package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.MoveRobot.BACKWARD;
import static org.firstinspires.ftc.teamcode.MoveRobot.TANK_TURN_LEFT;
import static org.firstinspires.ftc.teamcode.MoveRobot.TANK_TURN_RIGHT;
import static org.firstinspires.ftc.teamcode.XBot.ARM_POSITION_HIGH;
import static org.firstinspires.ftc.teamcode.XBot.ARM_POSITION_UP;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Auto Red Near Left", group = "Concept")
public class AutoRedNearLeft extends XBotOpMode implements AutoOpMode {
    Alliance alliance = Alliance.RED;
    DistanceFromBackdrop distanceFromBackdrop = DistanceFromBackdrop.NEAR;
    Parking parking = Parking.LEFT;
    boolean spikeMarkPixelDropped = false;
    boolean aTagPixelDropped = false;

    @Override
    public void runOpMode() {
        // Initialize hardware
        initializeAuto(DEBUG);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                if (!teamPropDetectionCompleted) {
                    detectTeamPropAndSwitchCameraToAprilTag();
                } else {
                    telemetry.addData("SpikeMark", spikeMark + ", confidence" + detectionConfidence);
                    telemetry.addData(">", "Robot Heading = %4.0f", getHeading());
                    telemetry.update();
                    if (!spikeMarkPixelDropped) {
                        switch (spikeMark) {
                            case LEFT:
                                leftSpikeMark(alliance, spikeMark, distanceFromBackdrop, parking);
                                break;
                            case RIGHT:
                                rightSpikeMark(alliance, spikeMark, distanceFromBackdrop, parking);
                                break;
                            case CENTER:
                                centerSpikeMark(alliance, spikeMark, distanceFromBackdrop, parking);
                        }
                        spikeMarkPixelDropped = true;
                    }
                }
                sleep(20);
            }
        }
        stopRobot();
    }
}
