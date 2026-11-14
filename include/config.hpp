#pragma once

#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep

// All devices are created in src/config.cpp.
// Include this file anywhere you need to use them.

// build number: bump this every upload so you can check the robot has the new code
// (shown on the controller screen and brain line 7)
constexpr int BUILD_NUM = 4;

// handheld controller
extern pros::Controller controller;

// drivetrain motor groups (negative ports = reversed motors)
extern pros::MotorGroup leftmotors;
extern pros::MotorGroup rightmotors;

// subsystem motors
extern pros::MotorGroup liftmotors;
extern pros::Motor clamp;

// sensors
extern pros::IMU imu;
extern pros::Rotation rotation_sensor;
extern pros::Distance right; // right-side distance sensor (port 13)
extern pros::Distance stackSensor; // sees the stacks in front of the clamp (port 6)
extern pros::Distance backSensor; // back of the robot, measures starting Y (port 18)

// LemLib chassis (drivetrain + PID + odom)
extern lemlib::Chassis chassis;

// field measurements
constexpr double encoderUnitsPerInch = 100;
constexpr double alliance_goal_height = 3.25;
constexpr double neutral_goal_height = 5.8;
constexpr double pin_height = 6.5;
constexpr double cup_height = 6.5;
constexpr double buffer = 1.45;
