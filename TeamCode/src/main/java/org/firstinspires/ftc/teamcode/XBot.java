package org.firstinspires.ftc.teamcode;

public class XBot {
    // ARM
    static final int MAX_ARM_POSITION = 500;
    static final int MIN_ARM_POSITION = 0;
    static final int FULL_CIRCLE = 1075;
    static final int ARM_PICK_POSITION = MIN_ARM_POSITION + 5; //Robot must move slow
    static final int ARM_POSITION_HIGH = MIN_ARM_POSITION + 20; //Robot running across field
    static final double ARM_SPEED = 0.3;

    // WRIST
    static final double MAX_WRIST_POS = 0.95;     // Maximum rotational position
    static final double MIN_WRIST_POS = 0.27;     // Minimum rotational position
    static final double STARTING_WRIST_POSITION = MIN_WRIST_POS;
    static final double WRIST_PICK_POSITION = MIN_WRIST_POS;

    //CLAW
    static final double CLAW_OPEN_POSITION = 0.63;
    static final double CLAW_CLOSE_POSITION = 0.70;
    static final double STARTING_CLAW_POS = CLAW_OPEN_POSITION;

    //DRIVE
    static final double MAX_SPEED = 1.0;
    static final double SPEED_WHEN_PICKING_PIXELS = 0.3; //gameMode = PICKING_PIXELS
    static final double SPEED_WHEN_DROPPING_PIXELS = 0.3; //gameMode = DROPPING_PIXELS
    static final double SPEED_WHEN_ON_APRIL_TAG_NAV = 0.5; //gameMode = APRIL_TAG_NAVIGATION

    //APRIL TAG
    static final int DESIRED_TAG_ID = -1;     // Choose the tag you want to approach or set to -1 for ANY tag.
    static final double MAX_AUTO_SPEED = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    static final double MAX_AUTO_STRAFE = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    static final double MAX_AUTO_TURN = 0.3;   //  Clip the turn speed to this max value (adjust for your robot)
    static final double DESIRED_DISTANCE = 7.0; //  this is how close the camera should get to the target (inches)
    static final double SPEED_GAIN = 0.02;   //  Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    static final double STRAFE_GAIN = 0.015;   //  Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
    static final double TURN_GAIN = 0.01;   //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)
}