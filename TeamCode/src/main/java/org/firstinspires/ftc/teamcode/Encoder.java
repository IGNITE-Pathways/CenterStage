// autonomous program that drives bot forward a set distance, stops then
// backs up to the starting point using encoders to measure the distance.

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="Test Encoder", group="Concept")
@Disabled
public class Encoder extends LinearOpMode
{
    DcMotor leftArmMotor;
    DcMotor rightArmMotor;

    @Override
    public void runOpMode() throws InterruptedException
    {
        leftArmMotor = hardwareMap.dcMotor.get("leftArmMotor");
        rightArmMotor = hardwareMap.dcMotor.get("rightArmMotor");

        // You will need to set this based on your robot's
        // gearing to get forward control input to result in
        // forward motion.
        leftArmMotor.setDirection(DcMotor.Direction.REVERSE);
        leftArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // wait for start button.
        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.left_trigger > 0.5) {
                // reset encoder counts kept by motors.
                leftArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                // set motors to run forward for 5000 encoder counts.
                leftArmMotor.setTargetPosition(1000);
                rightArmMotor.setTargetPosition(1000);

                // set motors to run to target encoder position and stop with brakes on.
                leftArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                leftArmMotor.setPower(0.4);
                rightArmMotor.setPower(0.4);
                while (opModeIsActive() && rightArmMotor.isBusy())   //leftMotor.getCurrentPosition() < leftMotor.getTargetPosition())
                {
                    telemetry.addData("encoder-fwd-left", leftArmMotor.getCurrentPosition() + "  busy=" + leftArmMotor.isBusy());
                    telemetry.addData("encoder-fwd-right", rightArmMotor.getCurrentPosition() + "  busy=" + rightArmMotor.isBusy());
                    telemetry.update();
                    idle();
                }
            }
            telemetry.addData("Mode", "waiting");
            telemetry.update();

            // set motor power to zero to turn off motors. The motors stop on their own but
            // power is still applied so we turn off the power.

            leftArmMotor.setPower(0.0);
            rightArmMotor.setPower(0.0);
        }
    }

    private void resetStartTime() {
    }
}
