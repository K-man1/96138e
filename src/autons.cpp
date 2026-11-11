#include "autons.hpp"
#include "config.hpp"
#include "lift.hpp"
#include "clamp.hpp"

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
// Routines
// ---------------------------------------------------------------------------


void auton1() {

    chassis.setPose(0, 0, 0);

    clamp.move(127); //start intake
    chassis.tank(90, 90);
    pros::delay(500);
    chassis.tank(0, 0);
    pros::delay(100);
    chassis.tank(-120, -120);
    pros::delay(500);
    chassis.tank(0, 0);

    chassis.turnToHeading(0, 1000);
    //----------------TOGGLE---------------

    double start = 144 - right.get_distance() * 0.0393701 - 6; // convert mm to inches
    double startY = backSensor.get_distance() * 0.0393701 + 3.4; // back sensor Y after the bump, mm to inches, +3 sensor-to-center
    pros::lcd::print(6, "Start: %.2f, %.2f in", start, startY);
    chassis.setPose(start, startY, 0); //start

    chassis.moveToPoint(start, 26, 5000); // align w goal


    chassis.turnToHeading(90, 2000); //turn to alliance goal

    liftAutoHeight(19); //lift lift


    chassis.moveToPoint(89, 26, 2000, {.maxSpeed = 50}); //go into goal

    
    chassis.waitUntilDone();
    liftAutoHeight(5);
    pros::delay(100); 
    chassis.turnToHeading(90, 1000);
    clamp.move(-60); //outtake
    pros::delay(300); //wait
    clamp.move(-0);

    //-------------PRELOAD + TOGGLE-----------------

    chassis.moveToPoint(72, 24, 2000, {.forwards = false}); //back out of goal
    chassis.turnToPoint(96, 54, 1000);
    
    chassis.moveToPoint(96, 50, 3000, {.maxSpeed = 50}); //go to 1st stack (about 45 deg from 72,24)

    clamp.move(-127);
    clampWhenClose(150, 5000); // for the next 3s, intake if the stack is within 100mm


    chassis.turnToHeading(180, 1000);
    liftAutoHeight(19);
    chassis.moveToPoint(96, 28, 3000, {.maxSpeed = 60}); //go to alliance goal
    chassis.waitUntilDone();
    liftAutoHeight(16);
    clamp.move(-120);
    pros::delay(2000);
    chassis.moveToPoint(96, 40, 3000, {.forwards = false});
    chassis.turnToPoint(120, 24, 1000);
    chassis.moveToPoint(120, 24, 3000);
    clampWhenClose(150, 5000);
    pros::delay(1000);
    chassis.turnToHeading(-90, 1000);
    liftAutoHeight(25);
    // clamp.move(127);
    // liftAutoHeight(3);


    
}

void auton2() {

    chassis.cancelAllMotions();

    // wrist.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    liftmotors.tare_position();
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
