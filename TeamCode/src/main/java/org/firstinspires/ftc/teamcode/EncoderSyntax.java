package org.firstinspires.ftc.teamcode;


import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.linearOpMode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous
public class EncoderSyntax {

    DcMotor rightArmMotor;
    DcMotor leftArmMotor;

    private int leftPos;
    private int rightPos;

    public void runOpMode() {
        leftArmMotor = hardwareMap.get(DcMotor.class, "leftArmMotor");
        rightArmMotor = hardwareMap.get(DcMotor.class, "rightArmMotor");

        leftArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rightArmMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        leftPos = 0;
        rightPos = 0;


        drive(1000,1000, .25);
        drive(1000,-1000, .25);

    }



    private void drive(int leftTarget, int rightTarget, double speed) {
        leftPos += leftTarget;
        rightPos += rightTarget;

        leftArmMotor.setTargetPosition(leftPos);
        rightArmMotor.setTargetPosition(rightPos);

        leftArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftArmMotor.setPower(speed);
        rightArmMotor.setPower(speed);

        }






    }


