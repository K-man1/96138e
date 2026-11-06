#pragma once

#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep

// All devices are created in src/config.cpp.
// Include this file anywhere you need to use them.

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

// LemLib chassis (drivetrain + PID + odom)
extern lemlib::Chassis chassis;

// field measurements
constexpr double encoderUnitsPerInch = 100;
constexpr double alliance_goal_height = 3.25;
constexpr double neutral_goal_height = 5.8;
constexpr double pin_height = 6.5;
constexpr double cup_height = 6.5;
constexpr double buffer = 1.45;
