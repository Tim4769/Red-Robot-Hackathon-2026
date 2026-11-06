// PID line follower for the Red Robot 2026 kit.
//
// Based on the Robojunkies LF-2 sample (LF2_Line_follower.ino). Kept from it:
//   * Calibration: sweep over the line, record each sensor's min and max.
//     A sensor counts as "on the line" past the midpoint, (min + max) / 2.
//   * Hard pivot when only an outer sensor sees the line.
//   * PID steering otherwise, with LF-2's optional automatic gain
//     (Kp grows as the center sensors lose the line, Kd = ratio * Kp).
//
// Changed from LF-2, because of how the 2026 kit works:
//   * The line sensors return discharge times in microseconds (0-10000), not
//     0-1023 analog values, so every reading is normalized to 0 (floor) .. 1
//     (line) with the calibration min/max before it is used.
//   * The error is a weighted average of all 6 sensors (-2.5 = far left,
//     +2.5 = far right) instead of sensor2 - sensor4.
//   * The I and D terms use the measured time step, because one sensor read
//     takes 7+ ms and varies with what the sensors see.
//   * Losing the line is handled explicitly: pivot toward where it was last
//     seen (LF-2 silently kept its last motor command).
//   * The integral is clamped and reset, so it cannot wind up.
//
// Sign convention: position < 0 means the line is on the robot's LEFT.
// correction < 0 slows the left wheel, so the robot turns toward the line.

#include "line_follower.h"

#include <Arduino.h>
#include <RedRobot.h>

namespace line_follower {
namespace {

// ============================== PID gains ==================================
// Used when kUseAutoGain is false. Units: motor percent (-100..100) per unit
// of position error (-2.5..+2.5).
constexpr float kKp = 18.0f;  // Proportional: steering per unit of error.
constexpr float kKi = 0.0f;   // Integral: per (error x second). Keep 0 at first.
constexpr float kKd = 1.2f;   // Derivative: per (error / second).

// ============================ Automatic gain ===============================
// LF-2's "automatic PID value": when true, Kp and Kd are recomputed every step
// from how strongly the center sensors see the line. Gentle when centered,
// aggressive when drifting off. kKp and kKd above are then ignored (kKi is
// still used).
constexpr bool kUseAutoGain = false;
constexpr float kAutoKpMin = 8.0f;     // Kp when the center sees the line fully.
constexpr float kAutoKpMax = 30.0f;    // Kp when the center barely sees it.
constexpr float kAutoKdRatio = 0.06f;  // Kd = kAutoKdRatio * Kp (LF-2: 10x in
                                       // its own units).

// ========================= Calibration settings ============================
// When true, Update() ONLY runs calibration, prints the results, then streams
// live sensor readings with the motors stopped. Use it to check calibration
// and sensor order before letting the robot drive.
constexpr bool kCalibrationTestMode = false;

// When true, sweep-calibrate at the start of autonomous (about 3 s).
// When false, skip it and use kSavedMin / kSavedMax. Copy those from the
// calibration test output on the real field.
constexpr bool kCalibrateAtStart = true;
constexpr int kNumSensors = 6;
constexpr uint16_t kSavedMin[kNumSensors] = {100, 100, 100, 100, 100, 100};
constexpr uint16_t kSavedMax[kNumSensors] = {2000, 2000, 2000, 2000, 2000,
                                             2000};

constexpr uint32_t kCalibrationDurationMs = 3000;
constexpr uint32_t kCalibrationSweepMs = 500;  // Time per sweep direction.
constexpr int kCalibrationSpeed = 35;          // Pivot speed while sweeping.
constexpr uint16_t kMinUsefulRange = 300;      // us. Smaller: never saw tape.
constexpr uint16_t kSensorTimeoutUs = 10000;   // Library's read timeout.

// ============================ Debug printing ===============================
constexpr bool kDebugPrint = true;
constexpr uint32_t kDebugPrintPeriodMs = 100;  // One status line per period.
constexpr uint32_t kTestPrintPeriodMs = 200;   // Calibration test stream.

// ============================== Robot setup ================================
constexpr uint8_t kLeftMotor = 2;
constexpr uint8_t kRightMotor = 3;
constexpr bool kInvertLeftMotor = false;   // Flip if the wheel spins backward.
constexpr bool kInvertRightMotor = false;
// Flip if the position sign is backwards (line on the left reads positive).
constexpr bool kReverseSensorOrder = false;
// Dark tape usually discharges slower, so it reads higher. Flip if the
// calibration test shows ~0.0 over the tape and ~1.0 over the floor.
constexpr bool kLineReadsHigher = true;

// ================================ Driving ==================================
constexpr int kBaseSpeed = 45;              // Forward speed on a straight line.
constexpr int kMinSpeed = 20;               // Forward speed floor on curves.
constexpr float kSlowdownPerError = 0.25f;  // Speed cut per unit of |error|.
constexpr int kEdgeTurnSpeed = 45;          // Outer wheel speed in a pivot.
constexpr int kLostTurnSpeed = 35;          // Pivot speed when the line is lost.
constexpr uint32_t kAutonDurationMs = 20000;
constexpr uint32_t kControlPeriodMs = 15;   // Minimum time between steps.
constexpr float kIntegralLimit = 2.0f;      // |integral| cap (error x second).

// Normalized-value thresholds (0 = floor, 1 = line).
constexpr float kOnLineThreshold = 0.5f;  // LF-2's (min + max) / 2 midpoint.
constexpr float kNoiseFloor = 0.1f;       // Ignore weaker readings.
constexpr float kLostLineSum = 0.3f;      // Total below this: line is lost.

// Sensor positions, robot's left to right, in sensor spacings from center.
constexpr float kSensorPosition[kNumSensors] = {-2.5f, -1.5f, -0.5f,
                                                0.5f,  1.5f,  2.5f};

// ================================= State ===================================
enum class Mode { kPid, kEdgeLeft, kEdgeRight, kLost, kDone };

struct SensorReading {
  uint16_t raw[kNumSensors];  // Discharge time, us, robot's left to right.
  float norm[kNumSensors];    // 0 = floor, 1 = line.
  bool on_line[kNumSensors];  // norm >= kOnLineThreshold.
  float position = 0.0f;      // Weighted average, -2.5 .. +2.5.
  float strength = 0.0f;      // Sum of weighted norms.
  bool line_visible = false;
  uint32_t read_us = 0;       // How long the read took.
};

struct PidTerms {
  float kp = 0.0f, ki = 0.0f, kd = 0.0f;
  float p = 0.0f, i = 0.0f, d = 0.0f;
  float correction = 0.0f;
};

uint16_t g_min[kNumSensors];
uint16_t g_max[kNumSensors];
bool g_initialized = false;
uint32_t g_auton_start_ms = 0;
uint32_t g_last_step_ms = 0;
uint32_t g_last_pid_us = 0;
uint32_t g_last_print_ms = 0;
float g_integral = 0.0f;
float g_previous_error = 0.0f;
bool g_has_previous_error = false;
float g_last_position = 0.0f;
Mode g_mode = Mode::kPid;

const char* ModeName(Mode mode) {
  switch (mode) {
    case Mode::kPid:
      return "PID";
    case Mode::kEdgeLeft:
      return "EDGE_L";
    case Mode::kEdgeRight:
      return "EDGE_R";
    case Mode::kLost:
      return "LOST";
    case Mode::kDone:
      return "DONE";
  }
  return "?";
}

// Sets wheel speeds in percent, -100..100, after applying motor inversion.
void SetDrive(int left, int right) {
  left = constrain(left, -100, 100);
  right = constrain(right, -100, 100);
  RedRobot::WriteDCMotor(kLeftMotor, kInvertLeftMotor ? -left : left);
  RedRobot::WriteDCMotor(kRightMotor, kInvertRightMotor ? -right : right);
}

void ResetPid() {
  g_integral = 0.0f;
  g_has_previous_error = false;
}

// Reads all sensors and fills `reading` in robot left-to-right order.
void ReadSensors(SensorReading* reading) {
  uint16_t hw[kNumSensors];
  const uint32_t start_us = micros();
  RedRobot::ReadLineSensor(hw);
  reading->read_us = micros() - start_us;

  float weighted_sum = 0.0f;
  reading->strength = 0.0f;
  for (int i = 0; i < kNumSensors; ++i) {
    const int hw_index = kReverseSensorOrder ? (kNumSensors - 1 - i) : i;
    reading->raw[i] = hw[hw_index];

    const int range = static_cast<int>(g_max[i]) - static_cast<int>(g_min[i]);
    float value = 0.0f;
    if (range >= kMinUsefulRange) {  // Skip sensors with bad calibration.
      value = static_cast<float>(reading->raw[i] - static_cast<int>(g_min[i])) /
              range;
      value = constrain(value, 0.0f, 1.0f);
      if (!kLineReadsHigher) value = 1.0f - value;
    }
    reading->norm[i] = value;
    reading->on_line[i] = value >= kOnLineThreshold;

    const float weight = value < kNoiseFloor ? 0.0f : value;
    weighted_sum += weight * kSensorPosition[i];
    reading->strength += weight;
  }

  reading->line_visible = reading->strength >= kLostLineSum;
  reading->position =
      reading->line_visible ? weighted_sum / reading->strength : 0.0f;
}

void PrintCalibration(uint32_t duration_ms, int samples) {
  Serial.printf("[CAL] Done: %lu ms, %d samples (S0 = robot's far left)\n",
                static_cast<unsigned long>(duration_ms), samples);
  Serial.println("[CAL] sensor  min_us  max_us  range_us  threshold_us");
  bool all_ok = true;
  for (int i = 0; i < kNumSensors; ++i) {
    const int range = g_max[i] - g_min[i];
    Serial.printf("[CAL]   S%d   %6u  %6u  %8d  %12d\n", i, g_min[i], g_max[i],
                  range, (g_min[i] + g_max[i]) / 2);
    if (range < kMinUsefulRange) {
      all_ok = false;
      Serial.printf(
          "[CAL] WARNING: S%d range %d us < %u us. It probably never crossed "
          "the tape (start the robot over the line) or is too high/low.\n",
          i, range, kMinUsefulRange);
    }
    if (g_max[i] >= kSensorTimeoutUs) {
      Serial.printf(
          "[CAL] WARNING: S%d hit the %u us read timeout. The surface is very "
          "dark or the sensor is disconnected.\n",
          i, kSensorTimeoutUs);
    }
  }
  Serial.println(all_ok ? "[CAL] All sensors have a usable range."
                        : "[CAL] Sensors with a bad range are IGNORED.");
  Serial.println("[CAL] To skip calibration in matches, set "
                 "kCalibrateAtStart = false and paste:");
  Serial.print("constexpr uint16_t kSavedMin[kNumSensors] = {");
  for (int i = 0; i < kNumSensors; ++i) {
    Serial.printf("%u%s", g_min[i], i < kNumSensors - 1 ? ", " : "};\n");
  }
  Serial.print("constexpr uint16_t kSavedMax[kNumSensors] = {");
  for (int i = 0; i < kNumSensors; ++i) {
    Serial.printf("%u%s", g_max[i], i < kNumSensors - 1 ? ", " : "};\n");
  }
}

// Sweeps left and right over the line, recording each sensor's min and max.
// The sweep is symmetric, so the robot ends facing about where it started.
void Calibrate() {
  Serial.printf("[CAL] Sweeping for %lu ms at speed %d. Start over the line.\n",
                static_cast<unsigned long>(kCalibrationDurationMs),
                kCalibrationSpeed);
  for (int i = 0; i < kNumSensors; ++i) {
    g_min[i] = UINT16_MAX;
    g_max[i] = 0;
  }

  const uint32_t start_ms = millis();
  int samples = 0;
  uint32_t elapsed_ms = 0;
  while ((elapsed_ms = millis() - start_ms) < kCalibrationDurationMs) {
    if (RedRobot::IsRemoteControlled()) {
      Serial.println("[CAL] Aborted: remote control took over.");
      break;
    }
    // Phases: half sweep left, then alternate full sweeps, end with half left.
    const bool turn_left =
        ((elapsed_ms + kCalibrationSweepMs / 2) / kCalibrationSweepMs) % 2 == 0;
    if (turn_left) {
      SetDrive(-kCalibrationSpeed, kCalibrationSpeed);
    } else {
      SetDrive(kCalibrationSpeed, -kCalibrationSpeed);
    }

    uint16_t hw[kNumSensors];
    RedRobot::ReadLineSensor(hw);
    for (int i = 0; i < kNumSensors; ++i) {
      const uint16_t value =
          hw[kReverseSensorOrder ? (kNumSensors - 1 - i) : i];
      if (value < g_min[i]) g_min[i] = value;
      if (value > g_max[i]) g_max[i] = value;
    }
    ++samples;
  }
  SetDrive(0, 0);
  PrintCalibration(elapsed_ms, samples);
}

void PrintConfig() {
  Serial.println("[LF] ===== Line follower config =====");
  Serial.printf("[LF] mode=%s  auto_gain=%s  calibrate_at_start=%s\n",
                kCalibrationTestMode ? "CALIBRATION_TEST" : "FOLLOW",
                kUseAutoGain ? "ON" : "OFF", kCalibrateAtStart ? "YES" : "NO");
  if (kUseAutoGain) {
    Serial.printf("[LF] Kp=%.2f..%.2f (auto)  Ki=%.3f  Kd=%.3f*Kp\n",
                  kAutoKpMin, kAutoKpMax, kKi, kAutoKdRatio);
  } else {
    Serial.printf("[LF] Kp=%.2f  Ki=%.3f  Kd=%.3f\n", kKp, kKi, kKd);
  }
  Serial.printf("[LF] base=%d min=%d period=%lums auton=%lums\n", kBaseSpeed,
                kMinSpeed, static_cast<unsigned long>(kControlPeriodMs),
                static_cast<unsigned long>(kAutonDurationMs));
  Serial.println("[LF] Status line: t, dt (time since last PID step), read "
                 "(sensor read time), mode, n (0=floor 1=line, left->right), "
                 "pos (<0 = line on left), gains, P/I/D terms, corr, speeds");
}

void PrintStatus(uint32_t now_ms, float dt_ms, const SensorReading& reading,
                 const PidTerms& pid, int speed, int left, int right) {
  Serial.printf(
      "[LF] t=%lu dt=%.1fms read=%.1fms mode=%-6s "
      "n=[%.2f %.2f %.2f %.2f %.2f %.2f] pos=%+.2f "
      "kp=%.1f ki=%.2f kd=%.2f P=%+.1f I=%+.1f D=%+.1f corr=%+.1f "
      "spd=%d L=%d R=%d\n",
      static_cast<unsigned long>(now_ms - g_auton_start_ms), dt_ms,
      reading.read_us / 1000.0f, ModeName(g_mode), reading.norm[0],
      reading.norm[1], reading.norm[2], reading.norm[3], reading.norm[4],
      reading.norm[5], reading.position, pid.kp, pid.ki, pid.kd, pid.p, pid.i,
      pid.d, pid.correction, speed, left, right);
}

void SetMode(Mode mode, uint32_t now_ms) {
  if (mode == g_mode) return;
  if (kDebugPrint) {
    Serial.printf("[LF] t=%lu mode %s -> %s (last pos=%+.2f)\n",
                  static_cast<unsigned long>(now_ms - g_auton_start_ms),
                  ModeName(g_mode), ModeName(mode), g_last_position);
  }
  // Leaving PID: forget the old error and integral so D does not spike and
  // I does not push the old way when PID resumes.
  if (g_mode == Mode::kPid) ResetPid();
  g_mode = mode;
}

// Streams sensor readings with the motors stopped.
void RunCalibrationTest(uint32_t now_ms) {
  if (now_ms - g_last_print_ms < kTestPrintPeriodMs) return;
  g_last_print_ms = now_ms;

  SensorReading reading;
  ReadSensors(&reading);
  int strongest = 0;
  for (int i = 1; i < kNumSensors; ++i) {
    if (reading.norm[i] > reading.norm[strongest]) strongest = i;
  }
  Serial.printf(
      "[TEST] raw_us=[%5u %5u %5u %5u %5u %5u] "
      "n=[%.2f %.2f %.2f %.2f %.2f %.2f] on=[%d%d%d%d%d%d] "
      "pos=%+.2f %s strongest=S%d read=%.1fms\n",
      reading.raw[0], reading.raw[1], reading.raw[2], reading.raw[3],
      reading.raw[4], reading.raw[5], reading.norm[0], reading.norm[1],
      reading.norm[2], reading.norm[3], reading.norm[4], reading.norm[5],
      reading.on_line[0], reading.on_line[1], reading.on_line[2],
      reading.on_line[3], reading.on_line[4], reading.on_line[5],
      reading.position, reading.line_visible ? "LINE" : "NO_LINE", strongest,
      reading.read_us / 1000.0f);
}

void Initialize() {
  g_initialized = true;
  PrintConfig();
  if (kCalibrationTestMode) {
    Calibrate();
    Serial.println(
        "[TEST] Motors stopped. Slide the tape under the robot: the tape "
        "should read ~1.00, the floor ~0.00, and pos should go NEGATIVE when "
        "the tape is under the robot's LEFT side.");
  } else if (kCalibrateAtStart) {
    Calibrate();
  } else {
    for (int i = 0; i < kNumSensors; ++i) {
      g_min[i] = kSavedMin[i];
      g_max[i] = kSavedMax[i];
    }
    Serial.println("[LF] Using saved calibration (kSavedMin / kSavedMax).");
  }
  ResetPid();
  g_auton_start_ms = millis();  // Start the clock after calibration.
  g_last_pid_us = micros();
}

}  // namespace

void Update() {
  const uint32_t now_ms = millis();
  if (!g_initialized) Initialize();

  if (kCalibrationTestMode) {
    RunCalibrationTest(now_ms);
    return;
  }

  if (g_mode == Mode::kDone) return;
  if (now_ms - g_auton_start_ms >= kAutonDurationMs) {
    SetDrive(0, 0);
    SetMode(Mode::kDone, now_ms);
    Serial.printf("[LF] Autonomous time up after %lu ms. Motors stopped.\n",
                  static_cast<unsigned long>(kAutonDurationMs));
    return;
  }
  if (now_ms - g_last_step_ms < kControlPeriodMs) return;
  g_last_step_ms = now_ms;

  SensorReading reading;
  ReadSensors(&reading);

  const uint32_t now_us = micros();
  const float dt_s = (now_us - g_last_pid_us) / 1e6f;
  g_last_pid_us = now_us;

  PidTerms pid;
  int speed = 0;
  int left = 0;
  int right = 0;
  const bool outer_left = reading.on_line[0];
  const bool outer_right = reading.on_line[kNumSensors - 1];

  if (outer_left && !outer_right) {
    // Like LF-2: only the far-left sensor side sees the line -> hard left.
    SetMode(Mode::kEdgeLeft, now_ms);
    left = 0;
    right = kEdgeTurnSpeed;
    g_last_position = kSensorPosition[0];
  } else if (outer_right && !outer_left) {
    SetMode(Mode::kEdgeRight, now_ms);
    left = kEdgeTurnSpeed;
    right = 0;
    g_last_position = kSensorPosition[kNumSensors - 1];
  } else if (reading.line_visible) {
    SetMode(Mode::kPid, now_ms);
    const float error = reading.position;  // Target is 0 (centered).

    if (kUseAutoGain) {
      // LF-2: Kp = 0.0006 * (1000 - center). Same idea on normalized values:
      // the less the center sees the line, the harder we steer.
      const float center = max(reading.norm[2], reading.norm[3]);
      pid.kp = kAutoKpMin + (kAutoKpMax - kAutoKpMin) * (1.0f - center);
      pid.kd = kAutoKdRatio * pid.kp;
    } else {
      pid.kp = kKp;
      pid.kd = kKd;
    }
    pid.ki = kKi;

    g_integral = constrain(g_integral + error * dt_s, -kIntegralLimit,
                           kIntegralLimit);
    const float derivative =
        (g_has_previous_error && dt_s > 0.0f)
            ? (error - g_previous_error) / dt_s
            : 0.0f;
    g_previous_error = error;
    g_has_previous_error = true;

    pid.p = pid.kp * error;
    pid.i = pid.ki * g_integral;
    pid.d = pid.kd * derivative;
    pid.correction = pid.p + pid.i + pid.d;

    // Slow down on big errors so both wheels stay in range on curves.
    speed = static_cast<int>(kBaseSpeed *
                             (1.0f - kSlowdownPerError * fabsf(error)));
    speed = max(speed, kMinSpeed);
    // Like LF-2, wheels never reverse while following.
    left = constrain(static_cast<int>(speed + pid.correction), 0, 100);
    right = constrain(static_cast<int>(speed - pid.correction), 0, 100);
    g_last_position = reading.position;
  } else {
    // Line lost: pivot toward the side it was last seen on.
    SetMode(Mode::kLost, now_ms);
    if (g_last_position < 0.0f) {
      left = -kLostTurnSpeed;
      right = kLostTurnSpeed;
    } else {
      left = kLostTurnSpeed;
      right = -kLostTurnSpeed;
    }
  }

  SetDrive(left, right);

  if (kDebugPrint && now_ms - g_last_print_ms >= kDebugPrintPeriodMs) {
    g_last_print_ms = now_ms;
    PrintStatus(now_ms, dt_s * 1000.0f, reading, pid, speed, left, right);
  }
}

}  // namespace line_follower
