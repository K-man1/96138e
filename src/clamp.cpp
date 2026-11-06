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

void clampControl() {
    // clampHold();
    clampToggle();
}
