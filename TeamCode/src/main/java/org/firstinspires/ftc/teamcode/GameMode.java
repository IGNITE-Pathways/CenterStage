package org.firstinspires.ftc.teamcode;

// INIT -> GOING_TO_PICK_PIXELS ->
// PICKING_PIXELS -> GOING_TO_DROP_PIXELS ->
// DROPPING_PIXELS -> GOING_TO_PICK_PIXELS
public enum GameMode {
    NONE,
    // Manual Game Initialized, Waiting for Play
    // ENTER MODE == Hit the INIT button on Driver Station
    // EXIT MODE == Hit the Play button on Driver Station
    INIT,

    // User driving Robot toward pixels, Robot is in motion
    // Arm back side, wrist down, claw open, clearance from ground > 0
    // ARM = AUTO, WRIST = AUTO, CLAWS = AUTO
    // ENTER MODE == AUTO (after pixel drop) or Initial Play Button,
    // EXIT MODE == USER ACTION changes game mode to PICKING_PIXELS
    // Operator 1 = DRIVER, Operator 2 = ACTION BUTTON (engage at the right time)
    GOING_TO_PICK_PIXELS,

    // User picking pixels, Robot is already on top of pixels, ARM lowers down to ground level
    // Arm back side, wrist down, claw manually operated, clearance from ground = 0
    // ARM = AUTO, WRIST = AUTO, CLAWS = OPEN or CLOSE
    // ENTER MODE = USER ACTION, -- lowers the arm closer to ground to pick pixels
    // EXIT MODE == USER ACTION changes game mode to GOING_TO_DROP_PIXELS
    // Operator 1 = DRIVER (Light engagement), Operator 2 = CLAWS and ACTION BUTTON (engage at the right time)
    PICKING_PIXELS,

    // User driving Robot toward back board, Robot is in motion
    // Arm back side, wrist facing board, claw holding pixels
    // WRIST = AUTO, CLAW = CLOSED (holding pixels)
    // ENTER MODE = USER ACTION, -- Once pixels are picked
    // EXIT MODE == USER ACTION changes game mode to DROPPING_PIXELS
    // Operator 1 = DRIVER, Operator 2 = Move ARM to right height and ACTION BUTTON
    GOING_TO_DROP_PIXELS,

    // Detected April Tag and Arm turns towards back board, and drops pixel
    // Arm back side, wrist facing board, claw open
    // ARM = AUTO, WRIST = AUTO, CLAWS = OPEN
    // ENTER MODE = AUTO NAVIGATION
    // EXIT MODE == AUTO -- right when pixels drop
    // Operator 1 = DRIVER (Light engagement), Operator 2 = ARM CONTROL, OPEN CLAWS
    APRIL_TAG_NAVIGATION,

    // Arm back side, wrist facing board, claw open
    // ARM = MANUAL, WRIST = NONE, CLAWS = OPEN
    // ENTER MODE = USER ACTION,
    // EXIT MODE == AUTO -- right when pixels drop
    // Operator 1 = DRIVER (Light engagement), Operator 2 = ARM CONTROL, OPEN CLAWS
    DROPPING_PIXELS,

    // Mode should only be activated when Robot is in position
    // Moves ARM high up to hang the Robot
    // ARM = AUTO, WRIST = AUTO, CLAW = AUTO
    // ENTER MODE = USER ACTION,
    // EXIT MODE == AUTO -- When Robot arm reaches the right position
    // Operator 1 = Operate ARM
    GOING_TO_HANG,

    // ARM = AUTO and ENGAGED, WRIST = AUTO, CLAW = AUTO
    // ENTER MODE = AUTO -- right after Robot ARM reaches the right height and angle
    // EXIT MODE == AUTO
    // Operators = NONE
    HANGING,
    AUTO_OP_MODE
}
