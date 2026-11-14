#include "main.h"
#include "config.hpp"
#include "lift.hpp"
#include "clamp.hpp"
#include "autons.hpp"
#include <cmath>

// Where things live:
//   config.cpp  - ports, motors, sensors, LemLib chassis + PID
//   lift.cpp    - lift driver control (R1/L1/B/Y) + liftAutoHeight
//   clamp.cpp   - clamp driver control (R2/L2/X)
//   autons.cpp  - autonomous routines

// Which side auton1 runs on.
//   false = normal routine            -> upload to slot 1
//   true  = mirrored, other alliance goal -> upload to slot 2
// Keep this false in the code; flip it to true only to upload slot 2.
constexpr bool kMirrorAuton = false;

// farthest Y reached since auton started (for linear PID tuning)
double maxY = 0;

// prints odom + lift info to the brain screen
void screenTask() {
    int controllerTick = 0;
    while (true) {

        double degrees = rotation_sensor.get_position() / 100.0 ;

        if (chassis.getPose().y > maxY) maxY = chassis.getPose().y;
        pros::lcd::print(5, "Max Y: %.2f", maxY);

        pros::lcd::print(0, "X: %.2f", chassis.getPose().x);
        pros::lcd::print(1, "Y: %.2f", chassis.getPose().y);
        pros::lcd::print(2, "Theta: %.2f", chassis.getPose().theta);
        pros::lcd::print(3, "Lift L: %.1f  R: %.1f", liftmotors.get_position(0), liftmotors.get_position(1));
        pros::lcd::print(4, "Rot: %.2f deg  Lift temp: %.0f/%.0f C", degrees,
                         liftmotors.get_temperature(0), liftmotors.get_temperature(1));
        pros::lcd::print(7, "Stack: %d mm  Back: %d mm", stackSensor.get_distance(), backSensor.get_distance());

        // X on controller row 2 (rows 0-1 are used for errors / "BOB").
        // Only every 3rd loop (150 ms): the controller drops prints sent
        // <50 ms apart, so this leaves room for other controller.print calls.
        // Trailing spaces clear leftover digits when the number gets shorter.
        if (++controllerTick >= 3) {
            controllerTick = 0;
            controller.print(2, 0, "X: %.2f      ", chassis.getPose().x);
        }

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

    // chassis.calibrate() can return while the IMU is still calibrating.
    // If odom reads the IMU then, it gets "infinity", the heading becomes NaN,
    // and every LemLib move (moveToPoint, turnToHeading...) silently does nothing
    // for the rest of the run. chassis.tank() still works since it skips odom.
    // So wait here until the IMU gives a real number (max 3 s).
    uint32_t imuWaitStart = pros::millis();
    while ((imu.is_calibrating() || !std::isfinite(imu.get_rotation())) &&
           pros::millis() - imuWaitStart < 3000) {
        pros::delay(10);
    }
    if (!std::isfinite(imu.get_rotation())) {
        controller.print(0, 0, "IMU NOT READY"); // controller only has rows 0-2
    }
    pros::delay(50); // let odom take a few good IMU readings before setPose clears the heading

    chassis.setPose(0, 0, 0);
    rotation_sensor.reset_position();
    pros::delay(50);

    liftmotors.tare_position_all();
    // claw.tare_position();
    // wrist.tare_position();

    liftmotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
    // wrist.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

    pros::Task screen(screenTask);

    double start = 144 - right.get_distance() * 0.0393701 - 6; // convert mm to inches
    double startY = backSensor.get_distance() * 0.0393701 + 3.6; // back wall is y = 0, convert mm to inches, +3 sensor-to-center
    chassis.setPose(start, startY, 0); //start
    pros::lcd::print(6, "Start: %.2f, %.2f in", start, startY);

    pros::delay(50); // controller ignores prints sent less than 50 ms apart
    controller.print(1, 0, "BOB");
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
    auton1(kMirrorAuton);
}

/**
 * Runs in driver control
 */
void opcontrol() {
    // 1. Force-cancel any leftover LemLib movements from autonomous
    chassis.cancelAllMotions();

    // 2. Lock the lift (holds if raised, rests passively if at the bottom)
    liftHold();
    // wrist.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

    // 3. Lock motors instantly without triggering blocking PID calls
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
