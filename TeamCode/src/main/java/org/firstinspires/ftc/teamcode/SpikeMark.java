package org.firstinspires.ftc.teamcode;

public enum SpikeMark {
    LEFT(1),
    CENTER(2),
    RIGHT(3);
    private final int value;

    SpikeMark(final int newValue) {
        value = newValue;
    }

    public int getValue() {
        return value;
    }
}
