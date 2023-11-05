package org.firstinspires.ftc.teamcode;

        import com.qualcomm.robotcore.eventloop.opmode.Disabled;
        import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
        import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
        import com.qualcomm.robotcore.hardware.DcMotor;
        import com.qualcomm.robotcore.util.ElapsedTime;

        import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
        import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
        import org.firstinspires.ftc.vision.VisionPortal;
        import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
        import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

        import java.util.List;

@TeleOp
public class Motor extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    DcMotor rightfront = null;
    DcMotor leftfront = null;
    DcMotor rightback = null;
    DcMotor leftback = null;


    @Override
    public void runOpMode() {
        double speed;

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftfront = hardwareMap.get(DcMotor.class, "leftfront");
        leftback = hardwareMap.get(DcMotor.class, "leftback");
        rightfront = hardwareMap.get(DcMotor.class, "rightfront");
        rightback = hardwareMap.get(DcMotor.class, "rightback");

        leftfront.setDirection(DcMotor.Direction.REVERSE);
        leftback.setDirection(DcMotor.Direction.REVERSE);
        rightfront.setDirection(DcMotor.Direction.FORWARD);
        rightback.setDirection(DcMotor.Direction.FORWARD);


        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double max;

            double power = 0.5;
            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double leftFrontPower = power;
            double rightFrontPower = power;
            double leftBackPower = power;
            double rightBackPower = power;

            if (gamepad2.x) {
                leftfront.setPower(leftFrontPower);
            }
            if (gamepad2.y) {
                leftback.setPower(leftFrontPower);
            }
            if (gamepad2.a) {
                rightfront.setPower(leftFrontPower);
            }
            if (gamepad2.b) {
                rightback.setPower(leftFrontPower);
            }


            }


        }

        }


