#pragma once

// Driver control for the clamp. Call once per opcontrol loop.
//   R2 (hold) = intake  (+127)
//   L2 (hold) = outtake (-127)
//   X         = step through: forward -> stop -> reverse -> stop
//   auto      = intakes when a stack gets close to the stack sensor
//               (triggers earlier the faster the robot is driving)
void clampControl();

// Auton: for the next timeoutMs, watch the stack sensor (port 6) and run
// clamp.move(127) early enough that the clamp is spinning by the time the
// stack is grabMm away. How early depends on drive speed and the delay
// constants at the top of clamp.cpp, so you don't retune per move speed.
// Runs in the background: returns immediately and never touches the drive.
// Brain line 6 shows "<dist>mm trig <trigger> @<speed>mm/s" or "never saw it".
// Auton: run clamp.move(127) delayMs from now. No sensor, just a timer.
// Call it right after starting the drive toward a stack.
// Runs in the background: returns immediately and never touches the drive.
void clampAfter(int delayMs);

void clampWhenClose(int timeoutMs = 3000, int grabMm = 55);

// Auton, simple version: for the next timeoutMs, as soon as the stack sensor
// (port 6) reads triggerMm or closer, run clamp.move(127). No speed math.
// Runs in the background: returns immediately and never touches the drive.
// Brain line 6 shows "Clamp: <dist>mm (simple)" or "never saw it".
void clampAtDistance(int triggerMm = 125, int timeoutMs = 3000);
