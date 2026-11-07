#include <Arduino.h>
#include <RedRobot.h>

#include "hardware_test.h"
#include "line_follower.h"

// true: run the one-by-one motor/servo test (src/hardware_test.cpp) once
// instead of the line follower. Set false for normal autonomous.
constexpr bool kHardwareTestMode = true;

// initialize any variables here
void setup() {
  RedRobot::InitRedRobot();         // call this first
  RedRobot::InitDCMotors();
  RedRobot::InitServo();
  RedRobot::InitLineSensors();
  RedRobot::InitAuton();            // call this to activate the AutonomousLoop
}

// Called repeatedly until remote control takes over (X / Button 1).
// The club's original example sequence is in git history (upstream commit
// "Working auton toggle; example starter code").
void AutonomousLoop() {
  if (kHardwareTestMode) {
    hardware_test::RunOnce();
    return;
  }
  line_follower::Update();  // Tuning and test switches: src/line_follower.cpp
}
