// PID line follower for the Red Robot 2026 kit.
//
// Usage: call line_follower::Update() from AutonomousLoop(). Each call does at
// most one control step and returns, so it fits the starter code's model where
// AutonomousLoop() is called repeatedly until remote control takes over.
//
// All tuning values (PID gains, auto-gain switch, calibration test switch,
// speeds, debug printing) are at the top of src/line_follower.cpp.

#ifndef RED_ROBOT_LINE_FOLLOWER_H_
#define RED_ROBOT_LINE_FOLLOWER_H_

namespace line_follower {

// Runs one iteration of the line follower (or of the calibration test, if
// enabled). Safe to call as often as you like; it rate-limits itself.
void Update();

}  // namespace line_follower

#endif  // RED_ROBOT_LINE_FOLLOWER_H_
