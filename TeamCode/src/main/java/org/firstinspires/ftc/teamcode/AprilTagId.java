package org.firstinspires.ftc.teamcode;

public enum AprilTagId {
    BLUE_LEFT(1, 44.5),
    BLUE_CENTER(2, 38.0),
    BLUE_RIGHT(3, 34.0),
    RED_LEFT(4, -32.0),
    RED_CENTER(5, -36.0),
    RED_RIGHT(6, - 42.5);

    private final int value;
    private final double yPos;

    AprilTagId(final int newValue, double y) {
        value = newValue;
        yPos = y;
    }

    public int getValue() {
        return value;
    }
    public double getYPos() { return yPos; }
}
