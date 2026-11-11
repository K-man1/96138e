#pragma once

// Driver control for the clamp. Call once per opcontrol loop.
//   R2 (hold) = intake  (+127)
//   L2 (hold) = outtake (-127)
//   X         = step through: forward -> stop -> reverse -> stop
void clampControl();

// Auton: for the next timeoutMs, if the stack sensor (port 6) reads
// triggerMm or closer, run clamp.move(127). That's it.
// Runs in the background: returns immediately and never touches the drive.
// Brain line 6 shows "intake at __ mm" or "never saw it".
void clampWhenClose(int triggerMm = 100, int timeoutMs = 3000);
