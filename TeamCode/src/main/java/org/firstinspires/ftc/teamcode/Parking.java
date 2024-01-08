package org.firstinspires.ftc.teamcode;

public enum Parking {
    LEFT(1),
    RIGHT(2);
    private final int value;

    Parking(final int newValue) {
        value = newValue;
    }

    public int getValue() {
        return value;
    }
}
