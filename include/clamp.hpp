#pragma once

// Driver control for the clamp. Call once per opcontrol loop.
//   R2 (hold) = intake  (+127)
//   L2 (hold) = outtake (-127)
//   X         = step through: forward -> stop -> reverse -> stop
void clampControl();
