#pragma once

// Driver control for the clamp. Call once per opcontrol loop.
//   R2 (hold) = intake  (+127)
//   L2 (hold) = outtake (-127)
//   X         = step through: forward -> stop -> reverse -> stop
void clampControl();

// Auton: keeps reading the stack distance sensor (port 6) and runs
// clamp.move(127) the moment it reads triggerMm or closer.
// Blocks until it clamps or timeoutMs runs out. Returns true if the sensor
// triggered it. On timeout it still runs the clamp and returns false.
// Brain line 6 shows the reading it triggered at, or the closest it saw.
// LemLib motions run in the background, so call this right after moveToPoint.
bool clampWhenClose(int triggerMm = 55, int timeoutMs = 3000);
