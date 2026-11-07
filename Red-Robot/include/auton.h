// Autonomous sequence for the Red Robot 2026 match (the first 20 seconds).
//
// Usage: call auton::Update() from AutonomousLoop(). Each call does a little
// work for the current stage and returns. All tuning values are at the top of
// src/auton.cpp.

#ifndef RED_ROBOT_AUTON_H_
#define RED_ROBOT_AUTON_H_

namespace auton {

// Runs one step of the autonomous sequence. Call it over and over.
void Update();

}  // namespace auton

#endif  // RED_ROBOT_AUTON_H_
