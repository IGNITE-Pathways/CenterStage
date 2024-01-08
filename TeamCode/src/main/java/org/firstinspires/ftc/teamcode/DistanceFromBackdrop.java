package org.firstinspires.ftc.teamcode;

public enum DistanceFromBackdrop {
    NEAR(1),
    FAR(2);
    private final int value;

    DistanceFromBackdrop(final int newValue) {
        value = newValue;
    }

    public int getValue() {
        return value;
    }
}
