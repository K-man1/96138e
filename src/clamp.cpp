#include "clamp.hpp"
#include "config.hpp"

static bool isClampManualMoving = false;

// R2 = intake, L2 = outtake
static void clampHold() {
    // if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2) == true) {
    //     clamp.move(127);
    //     isClampManualMoving = true;
    // } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2) == true) {
    //     clamp.move(-127);
    //     isClampManualMoving = true;
    // // only stop on release, so the X toggle below isn't overridden
    // } else if (isClampManualMoving) {
    //     clamp.move(0);
    //     isClampManualMoving = false;
    // }
}

// X steps through: forward -> stop -> reverse -> stop
static void clampToggle() {
    static int sequence = 0;

    if (!controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
        return;
    }

    switch (sequence) {
        case 0:
            clamp.move(127);
            sequence = 1;
            break;
        case 1:
            clamp.move(0);
            sequence = 2;
            break;
        case 2:
            clamp.move(-127);
            sequence = 3;
            break;
        default:
            clamp.move(0);
            sequence = 0;
            break;
    }
}

bool clampWhenClose(int triggerMm, int timeoutMs) {
    uint32_t start = pros::millis();
    int closest = 9999; // closest reading seen, for the screen

    while (pros::millis() - start < (uint32_t)timeoutMs) {
        int dist = stackSensor.get_distance(); // mm

        // ignore bad readings (PROS_ERR / 0 when nothing is seen)
        if (dist > 0 && dist != PROS_ERR) {
            if (dist < closest) closest = dist;

            if (dist <= triggerMm) {
                clamp.move(127);
                pros::lcd::print(6, "Clamp: hit at %d mm", dist);
                return true;
            }
        }

        pros::delay(10); // sensor updates about every 33ms, 10ms is plenty
    }

    // never got close enough: intake anyway so the auton doesn't miss the stack,
    // and show the closest reading so you can tune triggerMm
    clamp.move(127);
    pros::lcd::print(6, "Clamp: TIMEOUT closest %d mm", closest);
    return false;
}

void clampControl() {
    // clampHold();
    clampToggle();
}
