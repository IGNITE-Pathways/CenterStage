package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.XBot.MAX_ARM_POSITION;
import static org.firstinspires.ftc.teamcode.XBot.MIN_ARM_POSITION;

import static java.lang.Math.signum;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Test Claw", group = "Concept")
public class TestClaw extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private final ElapsedTime runtime = new ElapsedTime();
    Servo leftClaw = null;
    Servo rightClaw = null;
    Servo leftWrist = null;
    Servo rightWrist = null;
    static final double CLAW_OPEN = 0.1272;     // Maximum rotational position
    static final double CLAW_CLOSED = 0.5;     // Maximum rotational position
        static final double WRIST_FLAT_TO_GROUND = 0.95;     // Maximum rotational position
    static final double WRIST_VERTICAL = 0.49;     // Maximum rotational position
    static final double MIN_WRIST_POSITION = 0.0;     // Maximum rotational position
    static final double MAX_WRIST_POSITION = 1.0;     // Maximum rotational position
    double clawPos = 0.2;
    double wristPosition = 0;

    @Override
    public void runOpMode() {
        leftClaw = hardwareMap.get(Servo.class, "leftclaw");
        rightClaw = hardwareMap.get(Servo.class, "rightclaw");
        leftWrist = hardwareMap.get(Servo.class, "leftwrist");
        rightWrist = hardwareMap.get(Servo.class, "rightwrist");
        rightWrist.setDirection(Servo.Direction.REVERSE);
        rightClaw.setDirection(Servo.Direction.REVERSE);

        leftClaw.setPosition(CLAW_OPEN);
        rightClaw.setPosition(CLAW_OPEN);
        leftWrist.setPosition(WRIST_FLAT_TO_GROUND);
        rightWrist.setPosition(WRIST_FLAT_TO_GROUND);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double moveWristBy = -gamepad1.right_stick_y;
            if (moveWristBy != 0) {
                wristPosition += signum(moveWristBy) * .01;
                wristPosition = Math.max(MIN_WRIST_POSITION, wristPosition); // cannot go below MIN_ARM_POSITION
                wristPosition = Math.min(MAX_WRIST_POSITION, wristPosition); // cannot go above MAX_ARM_POSITION
                leftWrist.setPosition(wristPosition);
                rightWrist.setPosition(wristPosition);
            }

            double moveClawBy = -gamepad1.left_stick_x;
            if (moveClawBy != 0) {
                clawPos = moveClawBy;
                clawPos = Math.max(0.0, clawPos); // cannot go below MIN_ARM_POSITION
                clawPos = Math.min(1.0, clawPos); // cannot go above MAX_ARM_POSITION
                leftClaw.setPosition(clawPos);
                rightClaw.setPosition(clawPos);
            }

            if (gamepad1.left_trigger > 0.1) {
                leftClaw.setPosition(CLAW_CLOSED);
            }

            if (gamepad1.right_trigger > 0.1) {
                rightClaw.setPosition(CLAW_CLOSED);
            }

            if (gamepad1.left_bumper) {
                leftClaw.setPosition(CLAW_OPEN);
            }

            if (gamepad1.right_bumper) {
                rightClaw.setPosition(CLAW_OPEN);
            }

            telemetry.addData("Claw Left Pos:", leftClaw.getPosition());
            telemetry.addData("Claw Right Pos:", rightClaw.getPosition());
            telemetry.addData("Wrist Left Pos:", leftWrist.getPosition());
            telemetry.addData("Wrist Right Pos:", rightWrist.getPosition());
            telemetry.update();
        }
    }
}
