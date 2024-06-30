package org.firstinspires.ftc.teamcode;

public class XBot {
    // ARM
    static final int FULL_CIRCLE = 1425; //3895.9 for 43RPM, 1425.1 for 117 RPM
    static final int ARM_MOTOR_GEAR_RATIO = 2;
    static final int MIN_ARM_POSITION = 0;
    static final int DEFAULT_DROP_ARM_POSITION = 1540;
    static final int MAX_ARM_POSITION = 1600; //FULL_CIRCLE * ARM_MOTOR_GEAR_RATIO * 60/100; //50%
    static final int ARM_POSITION_HIGH = MIN_ARM_POSITION + FULL_CIRCLE * ARM_MOTOR_GEAR_RATIO * 4 / 100;
    static final int ARM_POSITION_UP = 800;
    // 4% //Robot running across field
    static final int ARM_POSITION_ROBOT_HANGING = 800;
    static final double ARM_SPEED = 0.75;
    static final double ARM_HOLD_SPEED = 0.05;
    // WRIST
    static final double MIN_WRIST_POS = 0.0;     // Minimum rotational position
    static final double MAX_WRIST_POS = 1.0;     // Maximum rotational position (closer to ground)
    static final double WRIST_FLAT_TO_GROUND = 0.62;     // Maximum rotational position
    static final double WRIST_VERTICAL = 0.0;     // Maximum rotational position
    //CLAW
    static final double LEFT_CLAW_OPEN_POSITION = 0.13;
    static final double LEFT_CLAW_CLOSE_POSITION = 0.51;
    static final double RIGHT_CLAW_OPEN_POSITION = 0.06;
    static final double RIGHT_CLAW_CLOSE_POSITION = 0.45;
    //APRIL TAG
    static final int DESIRED_TAG_ID = -1;     // Choose the tag you want to approach or set to -1 for ANY tag.
    static final double MAX_AUTO_SPEED = 0.4;   //  Clip the approach speed to this max value (adjust for your robot)
    static final double MAX_AUTO_STRAFE = 0.4;   //  Clip the approach speed to this max value (adjust for your robot)
    static final double MAX_AUTO_TURN = 0.3;   //  Clip the turn speed to this max value (adjust for your robot)
    static final double TELEOP_DESIRED_DISTANCE = 13; //  this is how close the camera should get to the target (inches)
    static final double AUTOOP_DESIRED_DISTANCE = 12; //  this is how close the camera should get to the target (inches)
    static final double SPEED_GAIN = 0.1;   //  Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    static final double STRAFE_GAIN = 0.015;   //  Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
    static final double TURN_GAIN = 0.01;   //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)
    static final int APRIL_TAG_CAM_EXPOSURE = 4; //Change this based on the amount of light in the room. Lower exposure if room is bright and vice-versa
    static int ARM_PICK_POSITION = MIN_ARM_POSITION; //Robot must move slow

    //Variables to change / Tune
    static final int TFOD_CAM_EXPOSURE = 7; //Change this based on the amount of light in the room. Lower exposure if room is bright and vice-versa
    static boolean SKIP_PICKING_WHITE_PIXELS_FAR = true;
    static boolean SKIP_PICKING_WHITE_PIXELS_NEAR = false;
    static long WAIT_TIME_FOR_ALLIANCE_TO_CLEAR = 11000; //milliseconds
}
