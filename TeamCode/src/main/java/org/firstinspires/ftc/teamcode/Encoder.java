// autonomous program that drives bot forward a set distance, stops then
// backs up to the starting point using encoders to measure the distance.

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="Drive Encoder", group="Exercises")
//@Disabled
public class Encoder extends LinearOpMode
{
    DcMotor leftArmMotor;
    DcMotor rightArmMotor;

    @Override
    public void runOpMode() throws InterruptedException
    {
        leftArmMotor = hardwareMap.dcMotor.get("leftArmMotor");
        rightArmMotor = hardwareMap.dcMotor.get("rightArmMotor");


        float abc = gamepad1.left_trigger;
        // You will need to set this based on your robot's
        // gearing to get forward control input to result in
        // forward motion.
        leftArmMotor.setDirection(DcMotor.Direction.REVERSE);

        // reset encoder counts kept by motors.
        leftArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // set motors to run forward for 5000 encoder counts.
        leftArmMotor.setTargetPosition(100);
        rightArmMotor.setTargetPosition(100);

        // set motors to run to target encoder position and stop with brakes on.
        leftArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        telemetry.addData("Mode", "waiting");
        telemetry.addData("left_trigger", abc);
        telemetry.update();


        // wait for start button.

        waitForStart();

        telemetry.addData("Mode", "running");
        telemetry.update();

        // set both motors to 25% power. Movement will start. Sign of power is
        // ignored as sign of target encoder position controls direction when
        // running to position.

        //leftArmMotor.setPower(0.25);
        //rightArmMotor.setPower(0.25);

        // wait while opmode is active and left motor is busy running to position.

//        while (opModeIsActive() && leftArmMotor.isBusy())   //leftMotor.getCurrentPosition() < leftMotor.getTargetPosition())
//        {
//            telemetry.addData("encoder-fwd-left", leftArmMotor.getCurrentPosition() + "  busy=" + leftArmMotor.isBusy());
//            telemetry.addData("encoder-fwd-right", rightArmMotor.getCurrentPosition() + "  busy=" + rightArmMotor.isBusy());
//            telemetry.update();
//            idle();
//        }

        // set motor power to zero to turn off motors. The motors stop on their own but
        // power is still applied so we turn off the power.

        //leftArmMotor.setPower(0.0);
        //rightArmMotor.setPower(0.0);

        // wait 5 sec to you can observe the final encoder position.

//        resetStartTime();

        while (opModeIsActive() && getRuntime() < 5)
        {
            telemetry.addData("encoder-fwd-left-end", leftArmMotor.getCurrentPosition());
            telemetry.addData("encoder-fwd-right-end", rightArmMotor.getCurrentPosition());
            telemetry.update();
            idle();
        }

        // From current position back up to starting point. In this example instead of
        // having the motor monitor the encoder we will monitor the encoder ourselves.

        leftArmMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightArmMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftArmMotor.setTargetPosition(0);
        rightArmMotor.setTargetPosition(0);

        // Power sign matters again as we are running without encoder.
        leftArmMotor.setPower(-0.25);
        rightArmMotor.setPower(-0.25);

        while (opModeIsActive() && leftArmMotor.getCurrentPosition() > leftArmMotor.getTargetPosition())
        {
            telemetry.addData("encoder-back-left", leftArmMotor.getCurrentPosition());
            telemetry.addData("encoder-back-right", rightArmMotor.getCurrentPosition());
            telemetry.update();
            idle();
        }

        // set motor power to zero to stop motors.

        leftArmMotor.setPower(0.0);
        rightArmMotor.setPower(0.0);

        resetStartTime();

        while (opModeIsActive() && getRuntime() < 5)
        {
            telemetry.addData("encoder-right-arm-motor", leftArmMotor.getCurrentPosition());
            telemetry.addData("encoder-left-arm-motor", rightArmMotor.getCurrentPosition());
            telemetry.update();
            idle();
        }
    }

    private void resetStartTime() {
    }
}
