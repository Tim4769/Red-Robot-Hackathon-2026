// One-by-one actuator test for the Red Robot 2026 kit.
//
// Runs each DC motor forward then backward, then sweeps each servo, printing
// a [HW] line before every step so you can match what moves to what was
// commanded. Enable it with kHardwareTestMode in src/main.cpp.

#ifndef RED_ROBOT_HARDWARE_TEST_H_
#define RED_ROBOT_HARDWARE_TEST_H_

namespace hardware_test {

// Runs the full test once (blocking, about 25 s), then returns. Later calls
// do nothing. Call it from AutonomousLoop().
void RunOnce();

// Drive test, once: forward, backward, pivot left, pivot right (1 s each at
// 50%), printing what each wheel should be doing. Blocking, about 9 s.
void RunDriveOnce();

}  // namespace hardware_test

#endif  // RED_ROBOT_HARDWARE_TEST_H_
