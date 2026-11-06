#include "main.h"
#include "config.hpp"
#include "lift.hpp"
#include "clamp.hpp"
#include "autons.hpp"

// Where things live:
//   config.cpp  - ports, motors, sensors, LemLib chassis + PID
//   lift.cpp    - lift driver control (R1/L1/B/Y) + liftAutoHeight
//   clamp.cpp   - clamp driver control (R2/L2/X)
//   autons.cpp  - autonomous routines

// farthest Y reached since auton started (for linear PID tuning)
double maxY = 0;

// prints odom + lift info to the brain screen
void screenTask() {
    while (true) {

        double degrees = rotation_sensor.get_position() / 100.0 ;

        if (chassis.getPose().y > maxY) maxY = chassis.getPose().y;
        pros::lcd::print(5, "Max Y: %.2f", maxY);

        pros::lcd::print(0, "X: %.2f", chassis.getPose().x);
        pros::lcd::print(1, "Y: %.2f", chassis.getPose().y);
        pros::lcd::print(2, "Theta: %.2f", chassis.getPose().theta);
        pros::lcd::print(3, "Lift: %.2f", liftmotors.get_position());
        pros::lcd::print(4, "Rotation: %.2f degrees", degrees);

        pros::delay(50);
    }
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
    pros::lcd::initialize();

    chassis.calibrate();

    chassis.setPose(0, 0, 0);
    rotation_sensor.reset_position();
    pros::delay(50);

    liftmotors.tare_position();
    // claw.tare_position();
    // wrist.tare_position();

    liftmotors.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    // wrist.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

    pros::Task screen(screenTask);
}

/*
 * Runs while the robot is disabled
 */
void disabled() {}

/*
 * runs after initialize if the robot is connected to field control
*/
void competition_initialize() {}

/**
 * Runs during auton
 */
void autonomous() {
    // set position to x:0, y:0, heading:0
    // liftmotors.move(127);
    // pros::delay(1200);
    // liftmotors.move(0);
    // liftmotors.brake();
    // // pros::delay(500);
    // chassis.setPose(0, 0, 0);
    // chassis.turnToHeading(90, 2000);
    chassis.setPose(0, 0, 0);
    maxY = 0;
    // turn to face heading 90 with a very long timeout
    // chassis.turnToHeading(90, 2000);

    // raise the lift all the way, then hold it there
    // liftmotors.move(127);
    // pros::delay(1000);
    // liftmotors.brake(); // brake mode is HOLD (set in initialize)

    // // let the robot settle before driving
    // pros::delay(500);

    // LINEAR PID TUNING: drive 48" (2 tiles) forward, long timeout
    uint32_t startTime = pros::millis();
    chassis.moveToPoint(0, 48, 10000);
    chassis.waitUntilDone();
    pros::lcd::print(6, "Move time: %d ms", (int)(pros::millis() - startTime));
    // chassis.setPose(0, 0, 0);
    // // turn to face heading 90 with a very long timeout
    // chassis.turnToHeading(180, 2000);
    // // auton2();
}

/**
 * Runs in driver control
 */
void opcontrol() {
    // 1. Force-cancel any leftover LemLib movements from autonomous
    chassis.cancelAllMotions();

    // 2. Set brake modes
    liftmotors.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    // wrist.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

    // 3. Lock motors instantly without triggering blocking PID calls
    liftmotors.move_velocity(0);
    // wrist.move_velocity(0);

    while (true) {
        // Get joystick positions
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        // Move chassis
        chassis.arcade(leftY, rightX);

        clampControl();
        liftControl();

        pros::delay(10); // Lowered to 10ms for faster driver response (100Hz)
    }
}
