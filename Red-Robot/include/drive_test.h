// Serial test console for the drivetrain (Option A).
//
// Open a serial monitor (pio device monitor -b 115200). The robot prints a
// numbered menu; type a number (or name) and Enter to run a test, with
// optional arguments, e.g. "2 60 3000". Any key during a test aborts it and
// stops the motors. Select it with kAutonMode in src/main.cpp.

#ifndef RED_ROBOT_DRIVE_TEST_H_
#define RED_ROBOT_DRIVE_TEST_H_

namespace drive_test {

// Handles serial input and runs the selected test. Call repeatedly from
// AutonomousLoop(). Returns quickly when idle; a running test blocks until it
// finishes or is aborted.
void Update();

}  // namespace drive_test

#endif  // RED_ROBOT_DRIVE_TEST_H_
