package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

@TeleOp(name = "Arm and Servo", group = "Concept")
public class ArmAndServo extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    static final double MAX_WRIST_POS =  0.95;     // Maximum rotational position
    static final double MIN_WRIST_POS =  0.07;     // Minimum rotational position
    static final double  STARTING_WRIST_POSITION = MAX_WRIST_POS;

    //Picking pixels -- Arm, Wrist and Claw
    static final int ARM_PICK_POSITION = 740;
    static final double  WRIST_PICK_POSITION = 0.73;
    static final double  CLAW_PICK_POSITION = 0.55;

    static final double MIN_CLAW_POS = 0.5;
    static final double MAX_CLAW_POS = 0.8;
    static final double STARTING_CLAW_POS = 0.67;
    
    //Arm Speed
    double ARM_SPEED = 0.3;
    int MAX_ARM_POSITION = 740;
    int FULL_CIRCLE = 1075;

    double wristPosition = STARTING_WRIST_POSITION;
    double leftClawPosition = STARTING_CLAW_POS;
    double rightClawPosition = STARTING_CLAW_POS;

    DcMotor leftArmMotor = null;
    DcMotor rightArmMotor = null;
    Servo   wristServo = null;
    Servo   leftClawServo = null;
    Servo   rightClawServo = null;

    @Override
    public void runOpMode() throws InterruptedException {
        int armPosition = 0;
        //Start with No Intent
        ClawPosition clawPosition = ClawPosition.HOME;
        //Start with No Intent
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
//                    wristPosition = getWristPosition(armPosition, ClawPosition.FACING_BACK_BOARD);
//                    wristServo.setPosition(wristPosition);
                    break;
                case FACING_DOWN:
                    armPosition = goToPickPixelPosition();
//                    wristPosition = getWristPosition(armPosition, ClawPosition.FACING_DOWN);
                    break;
                case HOME:
                    //Initial state or while driving
                    break;
            }

//            wristPosition += gamepad2.right_stick_y / 1000;
//            wristPosition = Math.min(MAX_WRIST_POS, Math.max(MIN_WRIST_POS, wristPosition));
//            wristServo.setPosition(wristPosition);

            telemetry.addData("Arm: Left Motor Position", leftArmMotor.getCurrentPosition() + "  busy=" + leftArmMotor.isBusy());
            telemetry.addData("Arm: Right Motor Position", rightArmMotor.getCurrentPosition() + "  busy=" + rightArmMotor.isBusy());
            telemetry.addData("Arm Angle", ((leftArmMotor.getCurrentPosition() * 360)/FULL_CIRCLE));
            telemetry.addData("Wrist: Servo Position", wristPosition);
            telemetry.addData("Left Claw Position", leftClawPosition);
            telemetry.addData("Right Claw Position", rightClawPosition);
            telemetry.update();
            idle();

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
        wristServo = hardwareMap.get(Servo.class, "wrist");
        leftClawServo = hardwareMap.get(Servo.class, "leftClaw");
        rightClawServo = hardwareMap.get(Servo.class, "rightClaw");

        //Reset encoders -- making start position as zero
        resetArmPosition();
        resetWristAndClawPosition();

    }

    private int goToPickPixelPosition() {
        setClawsToPixelPickPosition();
        moveArmToPosition(ARM_PICK_POSITION - 50, ClawPosition.FACING_DOWN);
        return moveArmToPosition(ARM_PICK_POSITION, ClawPosition.FACING_DOWN);
    }

    private void setClawsToPixelPickPosition() {
        leftClawServo.setPosition(CLAW_PICK_POSITION);
        rightClawServo.setPosition(CLAW_PICK_POSITION);
    }

    private void resetArmPosition() {
        leftArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    private void resetWristAndClawPosition() {
        wristServo.setPosition(STARTING_WRIST_POSITION);
        leftClawServo.setPosition(STARTING_CLAW_POS);
        rightClawServo.setPosition(STARTING_CLAW_POS);
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
            wristPosition = getWristPosition(armPosition, action);
            wristPosition = Math.min(MAX_WRIST_POS, Math.max(MIN_WRIST_POS, wristPosition));
            wristServo.setPosition(wristPosition);
        }

//        leftArmMotor.setPower(0);
//        rightArmMotor.setPower(0);

        return armPosition;
    }

    //calculate wrist position based on armPosition and pick or drop intent
    private double getWristPosition(int armPosition, ClawPosition clawPosition) {
        if (clawPosition == ClawPosition.FACING_DOWN) {
            if (isCloseToGround(armPosition)) {
                //Claw needs to face the ground
                return WRIST_PICK_POSITION;
            } else if (isArmFacingBack(armPosition)) {
                int angleA = ((armPosition * 360) / FULL_CIRCLE);
                return (150 - (0.383 * angleA))/100;
            } else {
                return wristPosition;
            }
        } else if (clawPosition == ClawPosition.FACING_BACK_BOARD) {
            //Calculate claw position based on arm position
            if (isArmFacingBack(armPosition)) {
                int angleA = ((armPosition * 360) / FULL_CIRCLE);
                return (150 - (0.383 * angleA))/100;
            } else {
                return wristPosition;
            }
        } else {
            //HOME
            return MIN_WRIST_POS;
        }
    }

    private boolean isArmFacingBack(double armPosition) {
        return armPosition > 380;
    }

    private boolean isCloseToGround(double armPosition) {
        return armPosition > 700;
    }
}
