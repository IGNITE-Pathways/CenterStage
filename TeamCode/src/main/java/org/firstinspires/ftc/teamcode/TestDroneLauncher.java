package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Test Drone", group = "Concept")
public class TestDroneLauncher extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private final ElapsedTime runtime = new ElapsedTime();
    Servo droneServo = null;
    static final double DRONE_LOADED = 1.0;     // Maximum rotational position
    static final double DRONE_LAUNCHED = 0.0;     // Maximum rotational position
    @Override
    public void runOpMode() {
        droneServo = hardwareMap.get(Servo.class, "drone");
        droneServo.setPosition(DRONE_LOADED);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            if (gamepad1.circle) {
                droneServo.setPosition(DRONE_LAUNCHED);
            }

            telemetry.addData("Drone Servo Pos:", droneServo.getPosition());
            telemetry.update();
        }
    }
}
