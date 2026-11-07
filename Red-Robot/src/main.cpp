#include <Arduino.h>
#include <RedRobot.h>

#include "hardware_test.h"
#include "line_follower.h"

// What AutonomousLoop() runs:
//   kLineFollower = normal autonomous (src/line_follower.cpp)
//   kActuatorTest = each motor, then each servo, one by one, once
//   kDriveTest    = forward, backward, pivot left, pivot right, once
enum class AutonMode { kLineFollower, kActuatorTest, kDriveTest };
constexpr AutonMode kAutonMode = AutonMode::kLineFollower;

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
  switch (kAutonMode) {
    case AutonMode::kActuatorTest:
      hardware_test::RunOnce();
      break;
    case AutonMode::kDriveTest:
      hardware_test::RunDriveOnce();
      break;
    case AutonMode::kLineFollower:
      line_follower::Update();  // Tuning/test switches: src/line_follower.cpp
      break;
  }
}
