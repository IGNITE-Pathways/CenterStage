package org.firstinspires.ftc.teamcode;

public enum Alliance {
    RED(1),
    BLUE(2);
    private final int value;

    Alliance(final int newValue) {
        value = newValue;
    }

    public int getValue() { return value; }
}
