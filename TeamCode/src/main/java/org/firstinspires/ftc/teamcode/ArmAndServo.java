package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Arm and Servo", group = "Concept")
public class ArmAndServo extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    static final double MAX_POS     =  0.91;     // Maximum rotational position
    static final double MIN_POS     =  0.07;     // Minimum rotational position
    static final double RANGE = MAX_POS - MIN_POS;
    static final double  WRIST_MID_POSITION = (MAX_POS + MIN_POS) / 2; // Start at halfway position
    static final int ARM_PICK_POSITION = 840;
    //Arm Speed
    double ARM_SPEED = 0.4;
    int MAX_ARM_POSITION = 840;

    int FULL_CIRCLE = 1075;

    DcMotor leftArmMotor = null;
    DcMotor rightArmMotor = null;
    Servo   servo = null;

    @Override
    public void runOpMode() throws InterruptedException {
        int armPosition = 0;
        //Start with No Intent
        ClawPosition clawPosition = ClawPosition.HOME;

        initialize();

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // wait for start button.
        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            if (gamepad2.x) {
                clawPosition = ClawPosition.FACING_DOWN;
            }
            if (gamepad2.y) {
                clawPosition = ClawPosition.FACING_BACK_BOARD;
            }

            switch (clawPosition) {
                case FACING_BACK_BOARD:
                    //New Arm Position
                    armPosition += (int)-(gamepad2.left_stick_y * 10);
                    armPosition = Math.max(0,armPosition); // cannot go below zero
                    armPosition = Math.min(MAX_ARM_POSITION,armPosition); // cannot go above 840
                    moveArmToPosition(armPosition, ClawPosition.FACING_BACK_BOARD);
                    double wristPosition = getWristPosition(armPosition, ClawPosition.FACING_BACK_BOARD);
                    servo.setPosition(wristPosition);
                    break;
                case FACING_DOWN:
                    armPosition = goToPickPixelPosition();
                    break;
                case HOME:
                    //Initial state or while driving
                    break;
            }

//            telemetry.addData("Status", "Run Time: " + runtime.toString());
//            telemetry.addData("Arm Position", leftArmMotor.getCurrentPosition());
//            telemetry.update();
        }
    }

    private void initialize() {
        //Initialize motors
        leftArmMotor = hardwareMap.dcMotor.get("leftArmMotor");
        rightArmMotor = hardwareMap.dcMotor.get("rightArmMotor");

        //Left Motor is in reverse
        leftArmMotor.setDirection(DcMotor.Direction.REVERSE);

        //Using Encoders
        leftArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Initialize Servo
        servo = hardwareMap.get(Servo.class, "wrist");

        //Reset encoders -- making start position as zero
        resetArmPosition();
    }

    private int goToPickPixelPosition() {
        return moveArmToPosition(ARM_PICK_POSITION, ClawPosition.FACING_DOWN);
    }

    private void resetArmPosition() {
        leftArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    private int moveArmToPosition(int armPosition, ClawPosition action) {
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
            armPosition = leftArmMotor.getCurrentPosition();
            double wristPosition = getWristPosition(armPosition, action);
            servo.setPosition(wristPosition);

            telemetry.addData("Arm: Left Motor Position", leftArmMotor.getCurrentPosition() + "  busy=" + leftArmMotor.isBusy());
            telemetry.addData("Arm: Right Motor Position", rightArmMotor.getCurrentPosition() + "  busy=" + rightArmMotor.isBusy());
            telemetry.addData("Angle", ((leftArmMotor.getCurrentPosition() * 360)/FULL_CIRCLE));
            telemetry.addData("Wrist: Servo Position", wristPosition);
            telemetry.update();
            idle();
        }

//        leftArmMotor.setPower(0);
//        rightArmMotor.setPower(0);

        return armPosition;
    }

    //calculate wrist position based on armPosition and pick or drop intent
    private double getWristPosition(int armPosition, ClawPosition clawPosition) {
        if (clawPosition == ClawPosition.FACING_DOWN) {
            //Claw needs to face the ground
            return MIN_POS * 5;
        } else if (clawPosition == ClawPosition.FACING_BACK_BOARD) {
            //Calculate claw position based on arm position
            int angleA = 320 - ((armPosition * 360) / FULL_CIRCLE);
            int angleW = 60 - angleA;
            return MAX_POS;
        } else {
            //HOME
            return MIN_POS;
        }
    }
}
