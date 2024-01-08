package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.XBot.ARM_HOLD_SPEED;
import static org.firstinspires.ftc.teamcode.XBot.ARM_SPEED;
import static org.firstinspires.ftc.teamcode.XBot.MAX_ARM_POSITION;
import static org.firstinspires.ftc.teamcode.XBot.MIN_ARM_POSITION;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Test Motors", group = "Concept")
@Disabled
public class TestMotor extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private final ElapsedTime runtime = new ElapsedTime();
    DcMotor rightFront = null;
    DcMotor leftFront = null;
    DcMotor rightBack = null;
    DcMotor leftBack = null;
    DcMotor leftArmMotor, rightArmMotor = null;
    int armPosition = 0;

    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFront = hardwareMap.get(DcMotor.class, "leftfront");
        leftBack = hardwareMap.get(DcMotor.class, "leftback");
        rightFront = hardwareMap.get(DcMotor.class, "rightfront");
        rightBack = hardwareMap.get(DcMotor.class, "rightback");

        // Initialize Motors
        leftArmMotor = hardwareMap.get(DcMotor.class, "leftArmMotor");
        rightArmMotor = hardwareMap.get(DcMotor.class, "rightArmMotor");

        //Left Motor is in reverse
        rightArmMotor.setDirection(DcMotor.Direction.REVERSE);

        //Using Encoders
        leftArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.FORWARD);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double drive = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double turn = -gamepad1.left_stick_x;
            double strafe = -gamepad1.right_stick_x;

//            moveRobot(drive, turn, strafe);

            int moveArmBy = (int) -(gamepad2.left_stick_y * 120);
            if (moveArmBy != 0) {
                armPosition += moveArmBy;
                armPosition = Math.max(MIN_ARM_POSITION, armPosition); // cannot go below MIN_ARM_POSITION
                armPosition = Math.min(MAX_ARM_POSITION, armPosition); // cannot go above MAX_ARM_POSITION
                moveArmToPosition(armPosition);
            }

            if (gamepad2.x) {
                armPosition += 8000;
                moveArmToPosition(armPosition);
            }

            telemetry.addData("Manual", "Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
            telemetry.addData("Arm: Position", armPosition);
            telemetry.addData("Arm: Left Motor Position", leftArmMotor.getCurrentPosition() + "  busy=" + leftArmMotor.isBusy());
            telemetry.addData("Arm: Right Motor Position", rightArmMotor.getCurrentPosition() + "  busy=" + rightArmMotor.isBusy());
            telemetry.addData("MoveArmBy", moveArmBy);
            telemetry.update();
        }
    }

    public void moveRobot(double x, double y, double yaw) {
        // Calculate wheel powers.
        double leftFrontPower = x - y - yaw;
        double rightFrontPower = x + y + yaw;
        double leftBackPower = x + y - yaw;
        double rightBackPower = x - y + yaw;

        // Normalize wheel powers to be less than 1.0
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        // Send powers to the wheels.
        leftFront.setPower(leftFrontPower);
        rightFront.setPower(rightFrontPower);
        leftBack.setPower(leftBackPower);
        rightBack.setPower(rightBackPower);
    }

    private int moveArmToPosition(int armPosition) {
        // set motors to run forward for 5000 encoder counts.
        leftArmMotor.setTargetPosition(armPosition);
        rightArmMotor.setTargetPosition(armPosition);

        // set motors to run to target encoder position and stop with brakes on.
        leftArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftArmMotor.setPower(ARM_SPEED);
        rightArmMotor.setPower(ARM_SPEED);

        while (opModeIsActive() && rightArmMotor.isBusy())   //leftMotor.getCurrentPosition() < leftMotor.getTargetPosition())
        {
            telemetry.addData("Arm: Target", armPosition);
            telemetry.addData("Arm: Left Motor Position", leftArmMotor.getCurrentPosition() + "  busy=" + leftArmMotor.isBusy());
            telemetry.addData("Arm: Right Motor Position", rightArmMotor.getCurrentPosition() + "  busy=" + rightArmMotor.isBusy());
            telemetry.update();
        }

        leftArmMotor.setPower(ARM_HOLD_SPEED);
        rightArmMotor.setPower(ARM_HOLD_SPEED);

        return armPosition;
    }
}
