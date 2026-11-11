#include "lift.hpp"
#include "config.hpp"

static bool isLiftManualMoving = false;

// true if any controller button other than Y is held
static bool anyOtherButtonPressed() {
    using namespace pros;
    return controller.get_digital(E_CONTROLLER_DIGITAL_R1) ||
           controller.get_digital(E_CONTROLLER_DIGITAL_UP) ||
           controller.get_digital(E_CONTROLLER_DIGITAL_DOWN) ||
           controller.get_digital(E_CONTROLLER_DIGITAL_LEFT) ||
           controller.get_digital(E_CONTROLLER_DIGITAL_RIGHT) ||
           controller.get_digital(E_CONTROLLER_DIGITAL_X) ||
           controller.get_digital(E_CONTROLLER_DIGITAL_A) ||
           controller.get_digital(E_CONTROLLER_DIGITAL_B);
}

void liftControl() {
    // R1 = raise lift, L2 = lower lift
    bool r1 = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1);
    bool l2 = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2);

    if (r1) {
        liftmotors.move(127);
        isLiftManualMoving = true;
    }
    else if (l2) {
        liftmotors.move(-70);
        isLiftManualMoving = true;
    }

    else if (isLiftManualMoving) {
        liftmotors.move_velocity(0);
        double currentPos = liftmotors.get_position();
        liftmotors.move_absolute(currentPos, 100);
        isLiftManualMoving = false;
    }

    // matchloader
    else if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
        while (rotation_sensor.get_position() / 100.0 > 2) {
            // any other button cancels; the main loop then handles that button
            if (anyOtherButtonPressed()) break;
            liftmotors.move(-80);
            // wrist.move_absolute(-45,127);
            pros::delay(10);
        }
        // Lock at arrival spot
        double currentPos = liftmotors.get_position();
        liftmotors.move_absolute(currentPos, 100);
    }
}

// the two lift sides, so auton can power them separately
// (same ports/directions as liftmotors in config.cpp)
static pros::Motor liftLeft(15);
static pros::Motor liftRight(-16);

// how hard to pull the sides back together, power per degree of difference
constexpr double kLiftSync = 3.0;

// drive the lift at `power`, but slow down whichever side is ahead so the
// two sides move together. if one side gets stuck (e.g. resting on a goal),
// the other side stops too instead of going on alone and tilting the lift.
static void moveLiftLevel(int power) {
    double diff = liftLeft.get_position() - liftRight.get_position();
    int correction = static_cast<int>(diff * kLiftSync);
    liftLeft.move(power - correction);
    liftRight.move(power + correction);
}

void liftAutoHeight(double target_deg) {
    liftmotors.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

    double current_deg = rotation_sensor.get_position() / 100.0;
    int timeout = 0;
    double tolerance = 0.5; // Stops within 0.5 degrees of target

    // Move UP if lower than target
    if (current_deg < target_deg - tolerance) {
        while (current_deg < target_deg - tolerance && timeout < 100) {
            moveLiftLevel(127);
            pros::delay(10);
            current_deg = rotation_sensor.get_position() / 100.0;
            timeout++;
        }
    }
    // Move DOWN if higher than target
    else if (current_deg > target_deg + tolerance) {
        while (current_deg > target_deg + tolerance && timeout < 100) {
            moveLiftLevel(-80);
            pros::delay(10);
            current_deg = rotation_sensor.get_position() / 100.0;
            timeout++;
        }
    }

    // hold both sides at the spot halfway between them, so neither side
    // gets told to go somewhere it physically can't reach
    liftmotors.move_velocity(0);
    double holdPos = (liftLeft.get_position() + liftRight.get_position()) / 2.0;
    liftmotors.move_absolute(holdPos, 100);
}
