package org.firstinspires.ftc.teamcode.drive.opmode;

import android.os.Build;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
@TeleOp(group = "drive")
public class LocalizationTest extends LinearOpMode {
    Servo leftWrist = null;
    Servo rightWrist = null;
    Servo leftClaw = null;
    Servo rightClaw = null;
    static final double WRIST_FLAT_TO_GROUND = 0.95;     // Maximum rotational position
    static final double CLAW_OPEN = 0.1272;     // Maximum rotational position
    static final double CLAW_CLOSED = 0.5;     // Maximum rotational position
    static final double WRIST_VERTICAL = 0.49;     // Maximum rotational position

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        leftWrist = hardwareMap.get(Servo.class, "leftwrist");
        rightWrist = hardwareMap.get(Servo.class, "rightwrist");
        rightWrist.setDirection(Servo.Direction.REVERSE);
        leftClaw = hardwareMap.get(Servo.class, "leftclaw");
        rightClaw = hardwareMap.get(Servo.class, "rightclaw");
        rightClaw.setDirection(Servo.Direction.REVERSE);

        leftWrist.setPosition(WRIST_FLAT_TO_GROUND);
        rightWrist.setPosition(WRIST_FLAT_TO_GROUND);
        leftClaw.setPosition(CLAW_OPEN);
        rightClaw.setPosition(CLAW_OPEN);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while (!isStopRequested()) {
            if (gamepad2.left_trigger > 0.1) {
                leftClaw.setPosition(CLAW_CLOSED);
            }

            if (gamepad2.right_trigger > 0.1) {
                rightClaw.setPosition(CLAW_CLOSED);
            }

            if (gamepad2.left_bumper) {
                leftClaw.setPosition(CLAW_OPEN);
            }

            if (gamepad2.right_bumper) {
                rightClaw.setPosition(CLAW_OPEN);
            }

            if (gamepad2.circle) {
                leftWrist.setPosition(WRIST_VERTICAL);
                rightWrist.setPosition(WRIST_VERTICAL);
            }

            if (gamepad2.square) {
                leftWrist.setPosition(WRIST_FLAT_TO_GROUND);
                rightWrist.setPosition(WRIST_FLAT_TO_GROUND);
            }

            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    )
            );

            drive.update();

            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();
        }
    }
}
