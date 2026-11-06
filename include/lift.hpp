#pragma once

// Driver control for the lift. Call once per opcontrol loop.
//   R1 (hold) = raise
//   L1 (hold) = lower
//   B         = raise to scoring height
//   Y         = lower to matchload height
// Holds position when the buttons are released.
void liftControl();

// Auton: drive the lift until the rotation sensor reads target_deg (1s timeout).
void liftAutoHeight(double target_deg);
