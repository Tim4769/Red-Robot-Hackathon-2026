// One-by-one actuator test for the Red Robot 2026 kit. See hardware_test.h.

#include "hardware_test.h"

#include <Arduino.h>
#include <RedRobot.h>

namespace hardware_test {
namespace {

// Motors to test. Motor 1 is skipped: the library rejects index 1.
constexpr uint8_t kMotors[] = {0, 2};
constexpr int kMotorSpeed = 50;            // Percent, -100..100.
constexpr uint32_t kMotorRunMs = 1500;     // Per direction.
constexpr uint32_t kPauseMs = 700;         // Between steps.

constexpr uint8_t kServos[] = {0, 1, 2, 3};
constexpr int kServoAngles[] = {90, 0, 90, 180, 90};  // Sweep, end centered.
constexpr uint32_t kServoStepMs = 700;

// Drive test wiring: which port drives which wheel.
constexpr uint8_t kLeftDriveMotor = 0;
constexpr uint8_t kRightDriveMotor = 2;
constexpr bool kInvertLeftDrive = true;   // Left wheel is wired backward.
constexpr bool kInvertRightDrive = false;
constexpr int kDriveSpeed = 50;           // Percent.
constexpr uint32_t kDriveRunMs = 1000;    // Per maneuver.

struct Maneuver {
  const char* name;
  const char* expect;
  int left;
  int right;
};

constexpr Maneuver kManeuvers[] = {
    {"FORWARD", "both wheels push the robot forward", kDriveSpeed,
     kDriveSpeed},
    {"BACKWARD", "both wheels pull the robot backward", -kDriveSpeed,
     -kDriveSpeed},
    {"TURN LEFT (pivot)", "left wheel back, right wheel forward",
     -kDriveSpeed, kDriveSpeed},
    {"TURN RIGHT (pivot)", "left wheel forward, right wheel back",
     kDriveSpeed, -kDriveSpeed},
};

// Waits for a serial monitor (up to 15 s) so no output is lost, then counts
// down so you can get your hands clear.
void WaitForMonitor(const char* title) {
  const uint32_t start = millis();
  while (!Serial && millis() - start < 15000) delay(10);
  Serial.println();
  Serial.printf("[HW] ===== %s =====\n", title);
  Serial.println("[HW] Wheels OFF the ground. Battery must be on: USB alone "
                 "may not power motors/servos.");
  for (int s = 3; s > 0; --s) {
    Serial.printf("[HW] Starting in %d...\n", s);
    delay(1000);
  }
}

void TestMotor(uint8_t motor, int step, int total) {
  Serial.printf("[HW] step %d/%d: motor %u FORWARD %d%% for %lu ms\n", step,
                total, motor, kMotorSpeed,
                static_cast<unsigned long>(kMotorRunMs));
  RedRobot::WriteDCMotor(motor, kMotorSpeed);
  delay(kMotorRunMs);
  RedRobot::WriteDCMotor(motor, 0);
  delay(kPauseMs);

  Serial.printf("[HW] step %d/%d: motor %u BACKWARD %d%% for %lu ms\n", step,
                total, motor, kMotorSpeed,
                static_cast<unsigned long>(kMotorRunMs));
  RedRobot::WriteDCMotor(motor, -kMotorSpeed);
  delay(kMotorRunMs);
  RedRobot::WriteDCMotor(motor, 0);
  Serial.printf("[HW]   motor %u stopped\n", motor);
  delay(kPauseMs);
}

void TestServo(uint8_t servo, int step, int total) {
  Serial.printf("[HW] step %d/%d: servo %u sweep 90 -> 0 -> 90 -> 180 -> 90\n",
                step, total, servo);
  for (const int angle : kServoAngles) {
    Serial.printf("[HW]   servo %u -> %d deg\n", servo, angle);
    RedRobot::SetServo(servo, angle);
    delay(kServoStepMs);
  }
  delay(kPauseMs);
}

bool g_done = false;

}  // namespace

void RunOnce() {
  if (g_done) return;
  g_done = true;
  WaitForMonitor("Actuator test: each motor, then each servo");

  const int total = sizeof(kMotors) + sizeof(kServos);
  int step = 1;
  for (const uint8_t motor : kMotors) TestMotor(motor, step++, total);
  for (const uint8_t servo : kServos) TestServo(servo, step++, total);

  Serial.println("[HW] ===== Done. Press reset (or replug USB) to run again. "
                 "=====");
}

void RunDriveOnce() {
  if (g_done) return;
  g_done = true;
  WaitForMonitor("Drive test: forward, backward, turn left, turn right");
  Serial.printf("[DRIVE] left = motor %u%s, right = motor %u%s, speed %d%%, "
                "%lu ms each. L/R below are wheel directions (before "
                "inversion).\n",
                kLeftDriveMotor, kInvertLeftDrive ? " (inverted)" : "",
                kRightDriveMotor, kInvertRightDrive ? " (inverted)" : "",
                kDriveSpeed, static_cast<unsigned long>(kDriveRunMs));

  const int total = sizeof(kManeuvers) / sizeof(kManeuvers[0]);
  for (int i = 0; i < total; ++i) {
    const Maneuver& m = kManeuvers[i];
    Serial.printf("[DRIVE] %d/%d %s: L=%+d R=%+d (expect: %s)\n", i + 1,
                  total, m.name, m.left, m.right, m.expect);
    RedRobot::WriteDCMotor(kLeftDriveMotor,
                           kInvertLeftDrive ? -m.left : m.left);
    RedRobot::WriteDCMotor(kRightDriveMotor,
                           kInvertRightDrive ? -m.right : m.right);
    delay(kDriveRunMs);
    RedRobot::WriteDCMotor(kLeftDriveMotor, 0);
    RedRobot::WriteDCMotor(kRightDriveMotor, 0);
    delay(kPauseMs);
  }
  Serial.println("[DRIVE] ===== Done. Press reset (or replug USB) to run "
                 "again. =====");
}

}  // namespace hardware_test
