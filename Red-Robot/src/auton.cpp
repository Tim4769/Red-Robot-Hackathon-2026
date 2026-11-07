// Autonomous sequence for the Red Robot 2026 match (the first 20 seconds).
//
// This is a STATE MACHINE. The robot is always in exactly one stage. Each call
// to auton::Update() does a little work for the current stage. When a stage is
// finished, GoTo() moves to the next one:
//
//   START -> LEAVE_BOX -> FIND_LINE -> FOLLOW_LINE -> DROP_PIPE -> DONE
//
// Every stage change prints one line to the Serial Monitor, for example
//   [AUTO] t=1000ms  LEAVE_BOX -> FIND_LINE  (drove forward 1000 ms)
// so you can always see where the robot is and why it moved on.
//
// Everything you will tune on the field is in the "Settings" block below.
// Line following itself (PID gains, calibration) is in src/line_follower.cpp.

#include "auton.h"

#include <Arduino.h>
#include <RedRobot.h>

#include "line_follower.h"

namespace auton {
namespace {

// The stages, in order.
enum class Stage { kStart, kLeaveBox, kFindLine, kFollowLine, kDropPipe, kDone };

// ================================ Settings =================================

// Which stage to begin in. Normally kLeaveBox. To test ONE stage by itself,
// put the robot where that stage begins and start there. Example: kFollowLine
// with the robot already sitting on the tape.
constexpr Stage kFirstStage = Stage::kLeaveBox;

// --- Stage LEAVE_BOX: drive straight out of the start box. ---
constexpr int kLeaveSpeed = 50;         // Wheel speed, -100..100.
constexpr uint32_t kLeaveBoxMs = 1000;  // How long to drive. Measure on field.

// --- Stage FIND_LINE: creep forward until a sensor sees the tape. ---
constexpr int kFindSpeed = 30;                 // Slow, so we don't overshoot.
constexpr uint32_t kFindLineTimeoutMs = 3000;  // Give up (and stop) after this.

// --- Stage FOLLOW_LINE: Tim's PID line follower. ---
// How the robot decides it has reached the trench. Choose after you have seen
// how the tape ends on the real field:
//   kByTime       = follow for kFollowMs, then move on.
//   kAllSensorsOn = move on when kEndSensorCount or more sensors see tape at
//                   the same time (works if the line ends in a crossbar or a
//                   wide patch of tape).
enum class Arrival { kByTime, kAllSensorsOn };
constexpr Arrival kArrival = Arrival::kByTime;
constexpr uint32_t kFollowMs = 6000;          // Used by kByTime.
constexpr int kEndSensorCount = 5;            // Used by kAllSensorsOn.
constexpr uint32_t kIgnoreEndMs = 1000;       // kAllSensorsOn: ignore the end
                                              // marker right after we join
                                              // the line (we may cross it).
constexpr uint32_t kFollowTimeoutMs = 12000;  // kAllSensorsOn: give up here.

// --- Stage DROP_PIPE: placeholder until the mechanism is built. ---
constexpr uint32_t kDropMs = 1000;  // Time to let the mechanism move.

// --- Whole routine. ---
// The auton period is 20 s. Stop a little before that, no matter what.
constexpr uint32_t kAutonLimitMs = 19500;

// Drive wiring. MUST match line_follower.cpp and RemoteControl.cpp:
// motor port 0 = left wheel (wired backward, so inverted), port 2 = right.
constexpr uint8_t kLeftMotor = 0;
constexpr uint8_t kRightMotor = 2;
constexpr bool kInvertLeft = true;
constexpr bool kInvertRight = false;

// ================================== State ==================================

Stage g_stage = Stage::kStart;  // Which stage we are in right now.
uint32_t g_auton_start_ms = 0;  // millis() when the routine started.
uint32_t g_stage_start_ms = 0;  // millis() when the current stage started.

const char* StageName(Stage stage) {
  switch (stage) {
    case Stage::kStart:
      return "START";
    case Stage::kLeaveBox:
      return "LEAVE_BOX";
    case Stage::kFindLine:
      return "FIND_LINE";
    case Stage::kFollowLine:
      return "FOLLOW_LINE";
    case Stage::kDropPipe:
      return "DROP_PIPE";
    case Stage::kDone:
      return "DONE";
  }
  return "?";
}

// Sets both wheels. + = robot forward, for both wheels.
void Drive(int left, int right) {
  left = constrain(left, -100, 100);
  right = constrain(right, -100, 100);
  RedRobot::WriteDCMotor(kLeftMotor, kInvertLeft ? -left : left);
  RedRobot::WriteDCMotor(kRightMotor, kInvertRight ? -right : right);
}

void Stop() { Drive(0, 0); }

// How long we have been in the current stage, in ms.
uint32_t TimeInStage() { return millis() - g_stage_start_ms; }

// Moves to the next stage, prints why, and does that stage's one-time setup.
void GoTo(Stage next, const char* reason) {
  const uint32_t now = millis();
  Serial.printf("[AUTO] t=%lums  %s -> %s  (%s)\n",
                static_cast<unsigned long>(now - g_auton_start_ms),
                StageName(g_stage), StageName(next), reason);
  g_stage = next;
  g_stage_start_ms = now;

  // One-time setup when entering a stage.
  if (next == Stage::kFollowLine) line_follower::StartFollowing();
  if (next == Stage::kDropPipe || next == Stage::kDone) Stop();
}

}  // namespace

void Update() {
  // Sensor test mode (kCalibrationTestMode in line_follower.cpp): the motors
  // stay off and the sensor values are printed. No driving at all.
  if (line_follower::IsCalibrationTestMode()) {
    line_follower::Update();
    return;
  }

  // Safety net: whatever stage we are in, stop before the 20 s are up.
  if (g_stage != Stage::kStart && g_stage != Stage::kDone &&
      millis() - g_auton_start_ms >= kAutonLimitMs) {
    GoTo(Stage::kDone, "auton time limit reached");
    return;
  }

  char reason[64];  // Text for the [AUTO] stage-change line.

  switch (g_stage) {
    // ------------------------------------------------------------------ START
    case Stage::kStart:
      // Load the line sensor calibration BEFORE starting the clock. (If
      // kCalibrateAtStart is true in line_follower.cpp, this calibrates now.)
      line_follower::Init();
      g_auton_start_ms = millis();
      Serial.println("[AUTO] ===== Autonomous start =====");
      GoTo(kFirstStage, "starting");
      break;

    // -------------------------------------------------------------- LEAVE_BOX
    case Stage::kLeaveBox:
      Drive(kLeaveSpeed, kLeaveSpeed);
      if (TimeInStage() >= kLeaveBoxMs) {
        snprintf(reason, sizeof(reason), "drove forward %lu ms",
                 static_cast<unsigned long>(TimeInStage()));
        GoTo(Stage::kFindLine, reason);
      }
      break;

    // -------------------------------------------------------------- FIND_LINE
    case Stage::kFindLine:
      if (line_follower::SeesLine()) {
        snprintf(reason, sizeof(reason), "line found after %lu ms",
                 static_cast<unsigned long>(TimeInStage()));
        GoTo(Stage::kFollowLine, reason);
      } else if (TimeInStage() >= kFindLineTimeoutMs) {
        GoTo(Stage::kDone, "line NOT found, gave up");
      } else {
        Drive(kFindSpeed, kFindSpeed);
      }
      break;

    // ------------------------------------------------------------ FOLLOW_LINE
    case Stage::kFollowLine:
      line_follower::Update();  // One PID step. Prints [LF] lines.

      if (kArrival == Arrival::kByTime) {
        if (TimeInStage() >= kFollowMs) {
          snprintf(reason, sizeof(reason), "followed the line %lu ms",
                   static_cast<unsigned long>(TimeInStage()));
          GoTo(Stage::kDropPipe, reason);
        }
      } else {  // Arrival::kAllSensorsOn
        if (TimeInStage() >= kIgnoreEndMs &&
            line_follower::LastOnLineCount() >= kEndSensorCount) {
          snprintf(reason, sizeof(reason), "%d sensors on tape: end of line",
                   line_follower::LastOnLineCount());
          GoTo(Stage::kDropPipe, reason);
        } else if (TimeInStage() >= kFollowTimeoutMs) {
          GoTo(Stage::kDone, "end of line NOT found, gave up");
        }
      }
      break;

    // -------------------------------------------------------------- DROP_PIPE
    case Stage::kDropPipe:
      // TODO when the mechanism exists: move the servo(s) here, e.g.
      //   RedRobot::SetServo(0, 90);
      // The wheels were already stopped when we entered this stage.
      if (TimeInStage() >= kDropMs) {
        GoTo(Stage::kDone, "pipe dropped (placeholder, nothing moved)");
      }
      break;

    // ------------------------------------------------------------------- DONE
    case Stage::kDone:
      // Wheels are stopped. Wait for the driver to press Button 1.
      break;
  }
}

}  // namespace auton
