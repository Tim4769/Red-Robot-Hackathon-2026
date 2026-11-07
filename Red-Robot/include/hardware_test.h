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

}  // namespace hardware_test

#endif  // RED_ROBOT_HARDWARE_TEST_H_
