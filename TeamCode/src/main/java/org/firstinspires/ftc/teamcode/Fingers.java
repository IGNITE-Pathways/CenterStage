package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Test Claw", group = "Concept")
public class Fingers extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    static double MIN_POS = 0.5;
    static double MAX_POS = 0.8;
    static double STARTING_POS = 0.6;

    Servo   leftClaw = null;
    Servo   rightClaw = null;


    @Override
    public void runOpMode() throws InterruptedException {
        int armPosition = 0;
        //Start with No Intent
        double leftClawPosition = STARTING_POS;
        double rightClawPosition = STARTING_POS;

        initialize();

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // wait for start button.
        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            leftClawPosition += gamepad2.left_stick_x / 1000;
            rightClawPosition += gamepad2.right_stick_x / 1000;
            leftClawPosition = Math.min(MAX_POS, Math.max(MIN_POS, leftClawPosition));
            rightClawPosition = Math.min(MAX_POS, Math.max(MIN_POS, rightClawPosition));

            leftClaw.setPosition(leftClawPosition);
            rightClaw.setPosition(rightClawPosition);

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Left Claw Position", leftClawPosition);
            telemetry.addData("Right Claw Position", rightClawPosition);
            telemetry.update();
        }
    }

    private void initialize() {
        //Initialize Servo
        leftClaw = hardwareMap.get(Servo.class, "leftClaw");
        rightClaw = hardwareMap.get(Servo.class, "rightClaw");

    }

}
