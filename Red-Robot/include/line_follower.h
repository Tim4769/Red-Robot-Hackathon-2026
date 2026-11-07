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

#include <stdint.h>

namespace line_follower {

// Runs one iteration of the line follower (or of the calibration test, if
// enabled). Safe to call as often as you like; it rate-limits itself.
void Update();

// For the test console (src/drive_test.cpp):

// Runs the calibration routine now (hand / sweep / full rotation, per the
// settings in line_follower.cpp) and prints the [CAL] report. Blocking.
void CalibrateNow();

// Follows the line for `duration_ms` using the same control step as
// Update(), printing [LF] status lines. Stops early if `should_abort`
// returns true. Motors are stopped on return. Returns true if it ran the
// full duration. Uses kSavedMin/kSavedMax if CalibrateNow() hasn't run.
bool FollowFor(uint32_t duration_ms, bool (*should_abort)());

}  // namespace line_follower

#endif  // RED_ROBOT_LINE_FOLLOWER_H_
