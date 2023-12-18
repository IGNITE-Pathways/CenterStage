package org.firstinspires.ftc.teamcode;

public class XBot {
    // ARM
    static final int FULL_CIRCLE = 1425; //3895.9 for 43RPM, 1425.1 for 117 RPM
    static final int ARM_MOTOR_GEAR_RATIO = 2;
    static final int MIN_ARM_POSITION = 2;
    static final int MAX_ARM_POSITION = FULL_CIRCLE * ARM_MOTOR_GEAR_RATIO * 50/100; //50%
    static final int AUTO_MAX_ARM_POSITION = FULL_CIRCLE * ARM_MOTOR_GEAR_RATIO * 55/100; //60%
    static int ARM_PICK_POSITION = MIN_ARM_POSITION + 0; //Robot must move slow
    static final int ARM_POSITION_HIGH = MIN_ARM_POSITION + FULL_CIRCLE * ARM_MOTOR_GEAR_RATIO * 4/100;; // 4% //Robot running across field
    static final int ARM_POSITION_UP = 800;
    static final int ARM_POSITION_ROBOT_HANGING = 730;
    static final double ARM_SPEED = 0.3;

    static final double ARM_HOLD_SPEED = 0.05;

    // WRIST
    static final double MAX_WRIST_POS = 0.65;     // Maximum rotational position
    static final double MIN_WRIST_POS = 0.11;     // Minimum rotational position
    static final double STARTING_WRIST_POSITION = MIN_WRIST_POS;
    static final double WRIST_PICK_POSITION = MIN_WRIST_POS;

    //CLAW
    static final double LEFT_CLAW_OPEN_POSITION = 0.71;
    static final double LEFT_CLAW_CLOSE_POSITION = 0.77;
    static final double STARTING_LEFT_CLAW_POS = LEFT_CLAW_OPEN_POSITION;

    static final double RIGHT_CLAW_OPEN_POSITION = 0.61;
    static final double RIGHT_CLAW_CLOSE_POSITION = 0.67;
    static final double STARTING_RIGHT_CLAW_POS = RIGHT_CLAW_OPEN_POSITION;

    //DRIVE
    static final double MAX_SPEED = 1.0; //Increase as operator gets better
    static final double SPEED_WHEN_PICKING_PIXELS = 0.3; //gameMode = PICKING_PIXELS
    static final double SPEED_WHEN_DROPPING_PIXELS = 0.3; //gameMode = DROPPING_PIXELS
    static final double SPEED_WHEN_ON_APRIL_TAG_NAV = 0.5; //gameMode = APRIL_TAG_NAVIGATION

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
    static final int TFOD_CAM_EXPOSURE = 4; //Change this based on the amount of light in the room. Lower exposure if room is bright and vice-versa
}
