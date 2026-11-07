#include <Arduino.h>
#include <RedRobot.h>

#include "auton.h"
#include "drive_test.h"
#include "hardware_test.h"
#include "line_follower.h"

// What AutonomousLoop() runs:
//   kFullAuton    = the match routine: leave box, find line, follow it, drop
//                   pipe, stop (src/auton.cpp)
//   kLineFollower = line following only (src/line_follower.cpp)
//   kActuatorTest = each motor, then each servo, one by one, once
//   kDriveTest    = forward, backward, pivot left, pivot right, once
//   kTestConsole  = serial menu to pick drivetrain tests (src/drive_test.cpp)
enum class AutonMode {
  kFullAuton, kLineFollower, kActuatorTest, kDriveTest, kTestConsole
};
// Test console = what was verified on the robot. For matches use kFullAuton.
constexpr AutonMode kAutonMode = AutonMode::kTestConsole;

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
    case AutonMode::kFullAuton:
      auton::Update();  // Stages and tuning: src/auton.cpp
      break;
    case AutonMode::kActuatorTest:
      hardware_test::RunOnce();
      break;
    case AutonMode::kDriveTest:
      hardware_test::RunDriveOnce();
      break;
    case AutonMode::kLineFollower:
      line_follower::Update();  // Tuning/test switches: src/line_follower.cpp
      break;
    case AutonMode::kTestConsole:
      drive_test::Update();
      break;
  }
}
