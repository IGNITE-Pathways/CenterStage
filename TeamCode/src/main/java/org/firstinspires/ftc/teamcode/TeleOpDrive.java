package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * This is the main manual OpMode
 */
@TeleOp(name = "Manual Drive", group = "Concept")
public class TeleOpDrive extends LinearOpMode {

    static final int MAX_ARM_POSITION = 500;
    static final int MIN_ARM_POSITION = 0;
    static final int FULL_CIRCLE = 1075;

    static final double MAX_WRIST_POS =  0.95;     // Maximum rotational position
    static final double MIN_WRIST_POS =  0.08;     // Minimum rotational position
    static final double  STARTING_WRIST_POSITION = MAX_WRIST_POS;

    //Picking pixels -- Arm, Wrist and Claw
    static final int ARM_PICK_POSITION = MIN_ARM_POSITION + 20;
    static final double  WRIST_PICK_POSITION = MIN_WRIST_POS;
    static final double CLAW_OPEN_POSITION = 0.55;
    static final double CLAW_CLOSE_POSITION = 0.65;


    static final double MIN_CLAW_POS = 0.5;
    static final double MAX_CLAW_POS = 0.8;
    static final double STARTING_CLAW_POS = 0.67;

    //Arm Speed
    static final double ARM_SPEED = 0.3;

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    GameMode gameMode = GameMode.NONE;
    DcMotor rightfront = null;
    DcMotor leftfront = null;
    DcMotor rightback = null;
    DcMotor leftback = null;
    DcMotor leftArmMotor = null;
    DcMotor rightArmMotor = null;
    Servo wristServo = null;
    Servo   leftClawServo = null;
    Servo   rightClawServo = null;
    double wristPosition = STARTING_WRIST_POSITION;
    double leftClawPosition = STARTING_CLAW_POS;
    double rightClawPosition = STARTING_CLAW_POS;



    @Override
    public void runOpMode() {
        int armPosition = 0;
        initialize();
        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        runtime.reset();
        gameMode = GameMode.GOING_TO_PICK_PIXELS;
        // run until the end of the match (driver presses STOP)
        if (gamepad2.x){
            gameMode = GameMode.PICKING_PIXELS;
        }
        if (gamepad2.y){
            gameMode = GameMode.DROPPING_PIXELS;
        }

        while (opModeIsActive()) {
            switch(gameMode){
                case GOING_TO_PICK_PIXELS:
                    // ARM = AUTO, WRIST = AUTO, CLAWS = AUTO
                    armPosition = goToPickPixelPosition();
                    break;
                case PICKING_PIXELS:
                    // ARM = AUTO, WRIST = AUTO, CLAWS = OPEN or CLOSE
                    pickPixels();
                    break;
                case GOING_TO_DROP_PIXELS:
                    // WRIST = AUTO, CLAW = CLOSED (holding pixels)
                    break;
                case APRIL_TAG_NAVIGATION:
                    break;
                case DROPPING_PIXELS:
                    // ARM = MANUAL, WRIST = NONE, CLAWS = OPEN
                    armPosition += (int)-(gamepad2.left_stick_y * 10);
                    armPosition = Math.max(0,armPosition); // cannot go below zero
                    armPosition = Math.min(MAX_ARM_POSITION,armPosition); // cannot go above 840
                    moveArmToPosition(armPosition);
                    break;
                case GOING_TO_HANG:
                    // ARM = AUTO, WRIST = AUTO, CLAW = AUTO
                    break;
                case HANGING:
                    // ARM = AUTO and ENGAGED, WRIST = AUTO, CLAW = AUTO
                    break;
            }
            double max;

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral = gamepad1.left_stick_x;
            double yaw = gamepad1.right_stick_x;

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double leftFrontPower = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower = axial - lateral + yaw;
            double rightBackPower = axial + lateral - yaw;
            double leftArmMotorPower = -gamepad2.left_stick_y;
            double rightArmMotorPower = gamepad2.left_stick_y;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower /= max;
                rightFrontPower /= max;
                leftBackPower /= max;
                rightBackPower /= max;
            }

            // Send calculated power to wheels
            leftfront.setPower(leftFrontPower);
            rightfront.setPower(rightFrontPower);
            leftback.setPower(leftBackPower);
            rightback.setPower(rightBackPower);
            leftArmMotor.setPower(leftArmMotorPower*ARM_SPEED);
            rightArmMotor.setPower(rightArmMotorPower*ARM_SPEED);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
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

    private void pickPixels() {
        leftClawServo.setPosition(CLAW_CLOSE_POSITION);
        rightClawServo.setPosition(CLAW_CLOSE_POSITION);
        // Move Arm up to remove friction and get clearance the ground
        moveArmToPosition(ARM_PICK_POSITION + 100);
        gameMode = GameMode.GOING_TO_DROP_PIXELS;
    }

    private void initialize() {
        gameMode = GameMode.INIT;
        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftfront = hardwareMap.get(DcMotor.class, "leftfront");
        leftback = hardwareMap.get(DcMotor.class, "leftback");
        rightfront = hardwareMap.get(DcMotor.class, "rightfront");
        rightback = hardwareMap.get(DcMotor.class, "rightback");
        // Initialize Motors
        leftArmMotor = hardwareMap.get(DcMotor.class, "leftArmMotor");
        rightArmMotor = hardwareMap.get(DcMotor.class, "rightArmMotor");

        leftfront.setDirection(DcMotor.Direction.REVERSE);
        leftback.setDirection(DcMotor.Direction.REVERSE);
        rightfront.setDirection(DcMotor.Direction.FORWARD);
        rightback.setDirection(DcMotor.Direction.FORWARD);


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
        resetArmEncoders();
        resetWristAndClawPosition();
    }
    private void resetArmEncoders() {
        leftArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    private void resetWristAndClawPosition() {
        wristServo.setPosition(STARTING_WRIST_POSITION);
        leftClawServo.setPosition(STARTING_CLAW_POS);
        rightClawServo.setPosition(STARTING_CLAW_POS);
    }
    private int goToPickPixelPosition() {
        setClawsToPixelPickPosition();
//        moveArmToPosition(ARM_PICK_POSITION - 50, ClawPosition.FACING_DOWN);
        return moveArmToPosition(ARM_PICK_POSITION);
    }

    private void setClawsToPixelPickPosition() {
        leftClawServo.setPosition(CLAW_OPEN_POSITION);
        rightClawServo.setPosition(CLAW_OPEN_POSITION);
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
            armPosition = leftArmMotor.getCurrentPosition();
            wristPosition = getWristPosition(armPosition);
            wristPosition = Math.min(MAX_WRIST_POS, Math.max(MIN_WRIST_POS, wristPosition));
            wristServo.setPosition(wristPosition);
        }

//        leftArmMotor.setPower(0);
//        rightArmMotor.setPower(0);

        return armPosition;
    }

    //calculate wrist position based on armPosition and pick or drop intent
    private double getWristPosition(int armPosition) {
        if ((gameMode == GameMode.GOING_TO_PICK_PIXELS) || (gameMode == GameMode.PICKING_PIXELS)) {
            if (isCloseToGround(armPosition)) {
                //Claw needs to face the ground
                return WRIST_PICK_POSITION;
            } else if (isArmFacingBack(armPosition)) {
                int angleA = ((armPosition * 360) / FULL_CIRCLE);
                return (150 - (0.383 * angleA))/100;
            } else {
                return wristPosition;
            }
        } else if ((gameMode == GameMode.GOING_TO_DROP_PIXELS) ||  (gameMode == GameMode.APRIL_TAG_NAVIGATION)) {
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
