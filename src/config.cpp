#include "config.hpp"

// ---------------------------------------------------------------------------
// Ports
// ---------------------------------------------------------------------------

// creates handheld controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// creates motor groups, function together when variable is called
// negative numbers refer to reversed motors
pros::MotorGroup rightmotors({-1, 10}, pros::MotorGearset::blue);
pros::MotorGroup leftmotors({9, -20}, pros::MotorGearset::blue);

// lift motors
pros::MotorGroup liftmotors({15, -16}, pros::MotorGearset::red);
pros::Motor clamp(7);

pros::IMU imu(4);
pros::Rotation rotation_sensor(14);
pros::Distance front_1(); //unplugged rn
pros::Distance front_2(); //unplugged rn
pros::Distance right(13); // distance sensor on the right side
pros::Distance stackSensor(6); // distance sensor that sees the stacks

/*
// creating sensors, track how many degrees vertical and horizontal tracking wheels have spun
pros::Rotation horizontalenc(5);
pros::Rotation verticalenc1(8);
pros::Rotation verticalenc2(11);
*/

// ---------------------------------------------------------------------------
// LemLib setup
// ---------------------------------------------------------------------------

/*
// horizontal tracking wheel
// 2.75" diameter, 5.75" offset, back of the robot (negative)
lemlib::TrackingWheel horizontal(&horizontalenc,
                                 lemlib::Omniwheel::NEW_275, -5.75);
// vertical tracking wheel 1
// 2.75" diameter, 2.5" offset, left of the robot (negative)
lemlib::TrackingWheel vertical1(&verticalenc1,
                               lemlib::Omniwheel::NEW_275, -2.5);

// vertical tracking wheel 2
// 2.75" diameter, 2.5" offset, left of the robot (negative)
lemlib::TrackingWheel vertical2(&verticalenc2,
                               lemlib::Omniwheel::NEW_275, -2.5);
*/

// drivetrain settings
lemlib::Drivetrain drivetrain(&leftmotors, // left motor group
                              &rightmotors, // right motor group
                              11.5, // 11.5 inch track width
                              lemlib::Omniwheel::NEW_325, // using new 3.25" omnis
                              360, // drivetrain rpm is 360
                              2 // horizontal drift is 2
);

// PID setup

// lateral motion controller (forward/backward)
lemlib::ControllerSettings linearController(20, // proportional gain (kP)
                                            0, // integral gain (kI)
                                            60, // derivative gain (kD)
                                            0, // anti windup
                                            1, // small error range, in inches
                                            100, // small error range timeout, in milliseconds
                                            3, // large error range, in inches
                                            500, // large error range timeout, in milliseconds
                                            0 // maximum acceleration (slew)
);

// angular motion controller
lemlib::ControllerSettings angularController(6, // proportional gain (kP)
                                             0, // integral gain (kI)
                                             37, // derivative gain (kD)
                                             0.48, // anti windup (only used if kI > 0)
                                             1, // small error range, in degrees
                                             100, // small error range timeout, in milliseconds
                                             3, // large error range, in degrees
                                             500, // large error range timeout, in milliseconds
                                             0 // maximum acceleration (slew)
);

// tuning angular PID

/*
void autonomous() {
    chassis.setPose(0, 0, 0);

    chassis.turnToHeading(90, 2000);
}
*/

// horizontal tracking wheel encoder
pros::Rotation horizontal_encoder(-8);
// horizontal tracking wheel
lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_encoder, lemlib::Omniwheel::NEW_2, 2.25);


// sensors for odom
lemlib::OdomSensors sensors(nullptr, // vertical tracking wheel
                            nullptr, // vertical tracking wheel 2
                            &horizontal_tracking_wheel, // horizontal tracking wheel
                            nullptr, // horizontal tracking wheel 2
                            // set to nullptr since we don't have a second one
                            &imu // inertial sensor
);

// input curve for throttle input during driver control
// left joystick in arcade drive
lemlib::ExpoDriveCurve throttleCurve(3, // joystick deadband out of 127
                                     10, // minimum output where drivetrain will move out of 127
                                     1.019 // expo curve gain
);

// input curve for steer input during driver control
lemlib::ExpoDriveCurve steerCurve(3, // joystick deadband out of 127
                                  10, // minimum output where drivetrain will move out of 127
                                  1.019 // expo curve gain
);

// create the chassis
lemlib::Chassis chassis(drivetrain, linearController,
angularController, sensors, &throttleCurve, &steerCurve);
