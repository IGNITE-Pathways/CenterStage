package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.MoveRobot.BACKWARD;
import static org.firstinspires.ftc.teamcode.XBot.ARM_POSITION_UP;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Auto Blue Near Left", group = "Concept")
public class AutoBlueNearLeft extends XBotOpMode implements AutoOpMode {
    Alliance alliance = Alliance.BLUE;
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
                        moveRobot(400, BACKWARD);
                        moveArmToPosition(ARM_POSITION_UP);

                        switch (spikeMark) {
                            case LEFT:
                                leftSpikeMark(alliance, distanceFromBackdrop, parking);
                                break;
                            case RIGHT:
                                rightSpikeMark(alliance, distanceFromBackdrop, parking);
                                break;
                            case CENTER:
                                centerSpikeMark(alliance, distanceFromBackdrop, parking);
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
