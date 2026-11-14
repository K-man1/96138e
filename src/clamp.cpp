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

// where the X button is in its forward -> stop -> reverse -> stop cycle.
// shared so the auto intake below can move it along too.
static int sequence = 0;

// ---------------------------------------------------------------------------
// Auto intake timing
// ---------------------------------------------------------------------------
// The clamp grabs the stack when it starts spinning with the stack ~65mm away.
// But between "sensor sees it" and "clamp is actually spinning" the robot
// keeps driving, so we have to give the command early:
//
//   trigger distance = grab distance + robot speed * total delay
//
// TUNING (watch brain line 6 after a run, it shows trigger mm and speed):
//   - grabs too late at ALL speeds  -> raise kGrabMm
//   - fine when slow, late when fast -> raise the delays
//   - fine when slow, early when fast -> lower the delays
constexpr int kGrabMm = 55;        // where the stack should be when the clamp starts spinning
constexpr int kSensorDelayMs = 60; // how old a distance reading is by the time we see it
constexpr int kMotorDelayMs = 80; // brain says move -> clamp is actually spinning
constexpr int kLoopMs = 10;        // how often we check the sensor (adds ~half a loop on average)
constexpr int kMaxTriggerMm = 300; // never trigger further out than this, whatever the speed

// how fast the robot is driving forward, in mm/s (0 if stopped or reversing)
static double driveSpeedMmPerSec() {
    constexpr double kMotorRpmMax = 600.0;          // blue cartridge
    constexpr double kWheelRpmMax = 360.0;          // drivetrain rpm (see config.cpp)
    constexpr double kWheelCircMm = 3.25 * 25.4 * 3.14159265; // 3.25" omnis

    double left = leftmotors.get_actual_velocity();   // motor rpm
    double right = rightmotors.get_actual_velocity();
    if (left == PROS_ERR_F || right == PROS_ERR_F) return 0;

    double wheelRpm = (left + right) / 2.0 * (kWheelRpmMax / kMotorRpmMax);
    double speed = wheelRpm / 60.0 * kWheelCircMm;
    return speed > 0 ? speed : 0;
}

// how far out to give the intake command right now, so the clamp is spinning
// by the time the stack is grabMm away
static int clampTriggerMm(int grabMm, double speedMmPerSec) {
    double delaySec = (kSensorDelayMs + kMotorDelayMs + kLoopMs / 2.0) / 1000.0;
    int trigger = grabMm + (int)(speedMmPerSec * delaySec);
    return trigger > kMaxTriggerMm ? kMaxTriggerMm : trigger;
}

// driver control: intake when a stack gets close enough to the stack sensor
// (speed-adjusted, see above).
// only fires the moment the stack first comes into range, so pressing X to
// stop or reverse still works while the stack is sitting there.
static void clampAutoIntake() {
    static bool wasClose = false;

    int dist = stackSensor.get_distance(); // mm
    if (dist <= 0 || dist == PROS_ERR) return; // bad reading, skip

    // once it's in range, it has to back off an extra 20mm to count as gone.
    // otherwise slowing down (which shrinks the trigger) could re-fire it.
    int trigger = clampTriggerMm(kGrabMm, driveSpeedMmPerSec());
    bool close = dist <= (wasClose ? trigger + 20 : trigger);
    if (close && !wasClose) {
        clamp.move(127);
        sequence = 1; // next X press stops it
    }
    wasClose = close;
}

// X steps through: forward -> stop -> reverse -> stop
static void clampToggle() {
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

void clampWhenClose(int timeoutMs, int grabMm) {
    // runs in the background, so the auton keeps going while this watches
    pros::Task([timeoutMs, grabMm] {
        const uint32_t deadline = pros::millis() + timeoutMs;

        while (pros::millis() < deadline) {
            int dist = stackSensor.get_distance(); // mm
            double speed = driveSpeedMmPerSec();
            int trigger = clampTriggerMm(grabMm, speed);

            // within range -> intake, done. (0 / PROS_ERR = bad reading, skip)
            if (dist > 0 && dist != PROS_ERR && dist <= trigger) {
                clamp.move(127);
                pros::lcd::print(6, "Clamp: %dmm trig %d @%dmm/s", dist, trigger, (int)speed);
                return;
            }

            pros::delay(kLoopMs);
        }

        // time ran out without seeing it -> do nothing
        pros::lcd::print(6, "Clamp: never saw it");
    });
}

void clampAtDistance(int triggerMm, int timeoutMs) {
    // runs in the background, so the auton keeps going while this watches
    pros::Task([triggerMm, timeoutMs] {
        const uint32_t deadline = pros::millis() + timeoutMs;

        while (pros::millis() < deadline) {
            int dist = stackSensor.get_distance(); // mm

            // within range -> intake, done. (0 / PROS_ERR = bad reading, skip)
            if (dist > 0 && dist != PROS_ERR && dist <= triggerMm) {
                clamp.move(127);
                pros::lcd::print(6, "Clamp: %dmm (simple)", dist);
                return;
            }

            pros::delay(kLoopMs);
        }

        // time ran out without seeing it -> do nothing
        pros::lcd::print(6, "Clamp: never saw it");
    });
}

void clampAfter(int delayMs) {
    // runs in the background, so the auton keeps going while this waits
    pros::Task([delayMs] {
        pros::delay(delayMs);
        clamp.move(127);
        pros::lcd::print(6, "Clamp: intake after %d ms", delayMs);
    });
}

void clampControl() {
    // clampHold();
    clampAutoIntake();
    clampToggle(); // after auto intake, so an X press on the same tick wins
}
