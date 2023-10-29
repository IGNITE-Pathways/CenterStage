package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Test Count Ticks", group = "Concept")
@Disabled
public class CountTicks extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    double MOTOR_SPEED = 0.4;
    DcMotor motor = null;
    @Override
    public void runOpMode() throws InterruptedException {
        int position = 0;
        initialize();

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Count Ticks: Initialized");
        telemetry.update();

        // wait for start button.
        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            position += (int)-(gamepad2.left_stick_y * 10);
            moveArmToPosition(position);
        }
    }

    private void initialize() {
        //Initialize motors
        motor = hardwareMap.dcMotor.get("motor");

        //Using Encoders
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Reset encoders -- making start position as zero
        resetArmPosition();
    }

    private void resetArmPosition() {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    private int moveArmToPosition(int pos) {
        // set motors to run forward for 5000 encoder counts.
        motor.setTargetPosition(pos);

        // set motors to run to target encoder position and stop with brakes on.
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motor.setPower(MOTOR_SPEED);

        while (opModeIsActive() && motor.isBusy())
        {
            telemetry.addData("Motor Position", motor.getCurrentPosition() + "  busy=" + motor.isBusy());
            telemetry.update();
            idle();
        }

        return pos;
    }

}
