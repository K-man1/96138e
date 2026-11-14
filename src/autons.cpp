#include "autons.hpp"
#include "config.hpp"
#include "lift.hpp"
#include "clamp.hpp"
#include <algorithm>
#include <cmath>
#include <vector>

// get a path used for pure pursuit
// this needs to be put outside a function
ASSET(example_txt);

// ---------------------------------------------------------------------------
// Wrist (not on the robot right now)
// ---------------------------------------------------------------------------

// wrist limit for auto
void raiseWristAuto() {
    const double ticks_per_degree = 0.6 * M_PI;
    double target_ticks = 630.0 * ticks_per_degree;
    // wrist.move_absolute(target_ticks,127);
}

void lowerWristAuto() {
    const double ticks_per_degree = 0.6 * M_PI;
    double target_ticks = 0.0 * ticks_per_degree;
    // wrist.move_absolute(target_ticks,127);
}

/*
// old wrist driver control
bool isWristManualMoving = false;

// Y = move wrist up
if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y)) {
        wrist.move(127);
        isManualMoving = true;
} else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_B)) {
    wrist.move(-127);
    isManualMoving = true;
}

else if (isManualMoving) {
    wrist.move(0);
    isManualMoving = false;
}

else if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
    wristIsUp = !wristIsUp;

    if (wristIsUp) {
        wrist.move_absolute(target_ticks, 127);
    } else {
        wrist.move_absolute(0,127);
    }
}
*/

// ---------------------------------------------------------------------------
// Odom check
// ---------------------------------------------------------------------------

// LemLib moves (moveToPoint, turnToHeading...) silently do nothing if the
// pose is NaN. That happens when a sensor odom reads gives an error value:
// the IMU, or any drive motor (LemLib uses the drive motor encoders since
// there's no vertical tracking wheel). chassis.tank() still works, so the
// robot does the bump and then just sits there.
// This finds the bad device and shows it on the controller + brain line 6.
static bool allFinite(const std::vector<double>& values) {
    for (double v : values) {
        if (!std::isfinite(v)) return false;
    }
    return true;
}

static bool odomOk() {
    const char* problem = nullptr;

    if (!std::isfinite(imu.get_rotation())) {
        problem = "IMU ERR port 4";
    } else if (!allFinite(leftmotors.get_position_all())) {
        problem = "L DRIVE MOTOR ERR";
    } else if (!allFinite(rightmotors.get_position_all())) {
        problem = "R DRIVE MOTOR ERR";
    } else {
        lemlib::Pose p = chassis.getPose();
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.theta)) {
            problem = "ODOM NaN";
        }
    }

    if (problem == nullptr) return true;
    controller.print(0, 0, "%s", problem);
    pros::lcd::print(6, "%s", problem);
    return false;
}

// ---------------------------------------------------------------------------
// Slow drive (no speed-up at the end)
// ---------------------------------------------------------------------------

// LemLib's moveToPoint raises maxSpeed to at least 60 once it's within 7.5"
// of the target (its "settling" mode), so {.maxSpeed = 30} speeds up right as
// it reaches the stack. This drives forward at a steady `speed` the whole way
// instead, steering toward (x, y) using the odom heading.
// Blocks until it reaches the point, drives past it, or timeoutMs runs out.
// Forwards only.
static void slowDriveTo(float x, float y, int speed, int timeoutMs) {
    constexpr float kTurnP = 1.0f;      // turn power per degree off course
    constexpr float kStopIn = 0.5f;     // done when this close (inches)
    constexpr float kNoSteerIn = 3.0f;  // stop steering this close: the angle to the point gets jumpy

    chassis.waitUntilDone(); // finish any running turn first, tank() doesn't wait for it

    const lemlib::Pose start = chassis.getPose();
    const float startDx = x - start.x;
    const float startDy = y - start.y;
    const uint32_t deadline = pros::millis() + timeoutMs;

    while (pros::millis() < deadline) {
        lemlib::Pose p = chassis.getPose();
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.theta)) break; // odom broke

        float dx = x - p.x;
        float dy = y - p.y;
        float dist = std::hypot(dx, dy);
        if (dist < kStopIn) break;
        if (dx * startDx + dy * startDy < 0) break; // drove past it

        float turn = 0;
        if (dist > kNoSteerIn) {
            // LemLib heading: degrees, 0 = +Y, clockwise is positive
            float targetHeading = std::atan2(dx, dy) * 180.0f / (float)M_PI;
            float error = std::remainder(targetHeading - p.theta, 360.0f); // -180..180
            turn = std::clamp(error * kTurnP, -speed / 2.0f, speed / 2.0f);
        }

        // true = skip the joystick drive curve so 30 really means 30
        chassis.tank((int)(speed + turn), (int)(speed - turn), true);
        pros::delay(10);
    }
    chassis.tank(0, 0, true);
}

// ---------------------------------------------------------------------------
// Routines
// ---------------------------------------------------------------------------


// mirror = false: the normal routine (slot 1).
// mirror = true:  the same routine on the other alliance goal's side of the
//                 field (slot 2), flipped over the center line x = 72.
// Every hard-coded field x goes through X() and every heading through H(),
// so tuning a number here tunes both sides. New moves: wrap them the same way.
//   X(x) = 144 - x when mirrored     H(h) = -h when mirrored
// y values don't change. turnToPoint/slowDriveTo just need X() on the x.
void auton1(bool mirror) {
    auto X = [mirror](float x) { return mirror ? 144.0f - x : x; };
    auto H = [mirror](float h) { return mirror ? -h : h; };

    chassis.setPose(0, 0, 0);

    clamp.move(127); //start intake
    pros::delay(500);
    chassis.tank(0, 0);
    pros::delay(100);
    chassis.tank(-120, -120);
    pros::delay(500);
    chassis.tank(0, 0);
    chassis.tank(90, 90);
    pros::delay(500);
    chassis.tank(0, 0);
    pros::delay(100);
    chassis.tank(-120, -120);
    pros::delay(500);
    chassis.tank(0, 0);

    chassis.turnToHeading(H(0), 1000);
    //----------------TOGGLE---------------

    double start = 144 - right.get_distance() * 0.0393701 - 6; // convert mm to inches
    double startY = backSensor.get_distance() * 0.0393701 + 3.4; // back sensor Y after the bump, mm to inches, +3 sensor-to-center
    pros::lcd::print(6, "Start: %.2f, %.2f in", start, startY);
    chassis.setPose(start, startY, 0); //start
    pros::delay(30); // let odom update a couple times so a bad sensor shows up
    odomOk();        // if this prints an error, the moves below won't run

    chassis.moveToPoint(start, 25, 5000); // align w goal (start is measured, already the real x: no X())


    chassis.turnToHeading(H(90), 2000); //turn to alliance goal

    liftAutoHeight(19); //lift lift


    chassis.moveToPoint(X(91), 25, 2000, {.maxSpeed = 50}); //go into goal

    
    chassis.waitUntilDone();
    liftAutoHeight(5);
    pros::delay(100); 
    chassis.turnToHeading(H(90), 1000);
    clamp.move(-60); //outtake
    pros::delay(300); //wait
    clamp.move(-0);

    //-------------PRELOAD + TOGGLE-----------------

    chassis.moveToPoint(X(72), 24, 2000, {.forwards = false}); //back out of goal

    // 1st stack: same 45 deg line from (72,24) through (89,41), but keep going
    // stack1ExtraIn inches past (89,41). Want it deeper -> raise it, too far -> lower it.
    const float stack1ExtraIn = 3;
    const float stack1X = X(89 + stack1ExtraIn * 0.7071f); // 0.7071 = cos(45) = sin(45)
    const float stack1Y = 41 + stack1ExtraIn * 0.7071f;

    chassis.turnToPoint(stack1X, stack1Y, 1000);
    
    clamp.move(-127);
    chassis.waitUntilDone(); // finish the turn so the timing below starts when the drive starts
    // slowDriveTo blocks, so this runs alongside it in the background:
    // reverse for the first 0.5 s of the drive, stop, then intake at 130 mm
    pros::Task([] {
        pros::delay(500);
        clamp.move(0);
        clampAtDistance(130);
    });
    slowDriveTo(stack1X, stack1Y, 30, 10000); //go to 1st stack (45 deg from 72,24), steady 30 the whole way
    // timed intake (no sensor): this many ms after the drive starts.
    // late -> lower it, early -> raise it. try steps of 50.
    // from video: drive starts ~5.42s, stack hits the rollers ~6.37s (0.95s later).
    // minus ~0.2s so the clamp has reversed and is spinning in before it arrives.
    chassis.waitUntilDone();
    clamp.move(127);
    pros::delay(2000);


    chassis.turnToHeading(H(180), 1000);
    liftAutoHeight(34);
    chassis.moveToPoint(X(96), 32, 3000, {.maxSpeed = 70}); //go to alliance goal
    chassis.waitUntilDone();
    liftAutoHeight(16);
    clamp.move(-120);
    pros::delay(2000);
    chassis.moveToPoint(X(96), 42, 3000, {.forwards = false});
    liftAutoHeight(0);

    chassis.turnToPoint(X(116), 32, 1000); //turn to second stack
    // stack sensor: intake once the stack is 125 mm away (watches for 5 s)
    // clampWhenClose(5000, 55); // speed-adjusted version
    clamp.move(-127);
    chassis.waitUntilDone(); // finish the turn so the clamp starts watching when the drive starts
    clampAtDistance(150);    // must come BEFORE slowDriveTo: it runs in the background, slowDriveTo blocks
    slowDriveTo(X(116), 32, 30, 3000); //go into second stack, steady 30 the whole way

    chassis.waitUntilDone();
    clamp.move(127);
    pros::delay(1000);
    chassis.turnToHeading(H(-90), 1000);
    liftAutoHeight(34);
    chassis.moveToPoint(X(105), 25, 3000, {.maxSpeed = 70}); //go to alliance goal
    chassis.waitUntilDone();
    liftAutoHeight(0);
    clamp.move(-127);

    pros::delay(1000);

    chassis.moveToPoint(X(131), chassis.getPose().y, 3000);
    chassis.turnToHeading(H(180), 1000);
    chassis.moveToPoint(X(131), 10, 3000);
    


    
}

void auton2() {

    chassis.cancelAllMotions();

    // wrist.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    liftmotors.tare_position_all();
    // wrist.tare_position();

    // chassis.moveToPoint(0, 0, 1000, {.forwards = false});

    chassis.moveToPoint(0.02, 3.38, 1000);
    chassis.setPose(0,0,0,1000);

    liftAutoHeight(42);
    chassis.moveToPoint(0.33, -6, 1000, {.forwards = false});
    pros::delay(1000);

    liftAutoHeight(25);
    chassis.turnToPoint(0.84, 4.0, 800);
    chassis.moveToPoint(0.84, 4.0, 1000, {
        .forwards = true,
        .maxSpeed = 127,
        .minSpeed = 127,
        .earlyExitRange = 1.0
    });
    pros::delay(200);

    /*

    chassis.moveToPose(-0.69, 8.27, 117.83, 1000);

    liftAutoHeight(25);
    pros::delay(200);

    intake.move(127);
    claw.move(127);
    pros::delay(2000);
    intake.move(0);
    claw.move(0);

    chassis.moveToPose(-0.43, 8.87, 123.15, 1000);
    raiseWristAuto();
    chassis.moveToPose(-2.92, 7.75, 63.77, 1000);
    chassis.moveToPoint(-4.38, 7.59, 1000);
    chassis.moveToPose(-9.09, 8.3, 133.51, 1000);
    chassis.moveToPoint(-14.65, 13.4, 1000);
    chassis.moveToPoint(-12.92, 13.72, 1000);
    chassis.moveToPose(-10.98, 9.77, 281,1000);
    chassis.moveToPoint(-14.65, 9.80, 1000);
    claw.move(-127);
    pros::delay(1000);
    claw.move(0);
    */

}

// ---------------------------------------------------------------------------
// Old routines (kept for reference)
// ---------------------------------------------------------------------------

// chassis.moveToPose(14.22, 10.79, 47.64, 1000);
/*
chassis.turnToHeading(88.71, 1000);
chassis.moveToPoint(6.23, 8.11, 1000);
chassis.turnToHeading(43.77, 1000);
chassis.moveToPoint(13.01, 13.90, 1000);
pros::delay(500);

claw.move(-127);
pros::delay(500);
claw.move(0);
chassis.moveToPoint(2.2, 7.58, 1000,
    {.forwards = false});
chassis.moveToPose(-1.13, 6.65, 89.65, 1000);
chassis.moveToPoint(52.01, 9.76, 1000);

chassis.moveToPose(56.13, 9.52, -3.21, 1000);
chassis.turnToHeading(1.34, 1000);
*/

/*
chassis.setPose(0.0, 0.0, 0.05);
liftAutoHeight(3.81);
intake.move(127);
claw.move(127);
pros::delay(500);
intake.move(0);
claw.move(0);
chassis.moveToPoint(-0.01, 3.83, 1000);
raiseWristAuto();
chassis.moveToPose(0.15, 6.80, 3.6, 1000);
pros::delay(500);
claw.move(-127);
pros::delay(500);
claw.move(0);
chassis.moveToPose(-0.03, -0.36, 0.52, 1000);
pros::delay(500);
lowerWristAuto();
chassis.turnToHeading(57.6, 1000);
chassis.moveToPose(28.88, 21.70, 57.17, 1000);
chassis.moveToPose(34.44, 25.76, 61.47, 1000);
chassis.turnToHeading(153.38, 1000);
*/

/*
claw.move(127);
pros::delay(500);
chassis.moveToPoint(-0.04, 0.23, 1000);
claw.move(0);
lowerWristAuto();
chassis.turnToHeading(-41.83, 1000);
chassis.moveToPoint(-9.95, 11.83, 1000);
chassis.moveToPose(-10.95, 14.77, 1.68, 1000);
raiseWristAuto();
chassis.moveToPoint(-10.33, 26.19, 1000);
claw.move(-127);
pros::delay(1000);
claw.move(0);
*/
