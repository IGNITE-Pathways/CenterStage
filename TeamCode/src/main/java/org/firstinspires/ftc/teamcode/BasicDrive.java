package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp
public class BasicDrive extends LinearOpMode {
    private DcMotor rightBack;
    private DcMotor rightFront;
    private DcMotor leftBack;
    private DcMotor leftFront;
    @Override
    public void runOpMode() throws InterruptedException {
        rightBack=hardwareMap.get(DcMotor.class ,"rightback");
        rightFront=hardwareMap.get(DcMotor.class ,"rightfront");
        leftBack=hardwareMap.get(DcMotor.class ,"leftback");
        leftFront=hardwareMap.get(DcMotor.class ,"leftfront");
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
        waitForStart();
        double powery = 0;
        double powerx = 0;
        while(opModeIsActive()){
            powery = this.gamepad1.left_stick_y;
                    rightFront.setPower(powery);
                    rightBack.setPower(powery);
                    leftFront.setPower(powery);
                    leftBack.setPower(powery);
            powerx = this.gamepad1.left_stick_x;
                    rightFront.setPower(-powerx);
                    rightBack.setPower(powerx);
                    leftFront.setPower(-powerx);
                    leftBack.setPower(powerx);
            telemetry.addData("powerx", powerx);
            telemetry.addData("powery", powery);
            telemetry.update();

        }
    }
}
