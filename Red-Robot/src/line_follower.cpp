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
constexpr float kAutoKpMin = 10.0f;    // Kp when the center sees the line fully.
constexpr float kAutoKpMax = 40.0f;    // Kp when the center barely sees it.
                                       // (Was 8 / 30: turned too late.)
constexpr float kAutoKdRatio = 0.06f;  // Kd = kAutoKdRatio * Kp (LF-2: 10x in
                                       // its own units).

// ========================= Calibration settings ============================
// When true, Update() ONLY runs calibration, prints the results, then streams
// live sensor readings with the motors stopped. Use it to check calibration
// and sensor order before letting the robot drive.
constexpr bool kCalibrationTestMode = true;

// When false, calibration does NOT drive the motors: you get
// kHandCalibrationMs to slide the robot (or the tape) back and forth by hand
// so every sensor passes over the tape. Safe on a desk / plugged into USB.
// When true, the robot sweeps or spins by itself (see below).
constexpr bool kCalibrationUseMotors = false;
constexpr uint32_t kHandCalibrationMs = 10000;

// When true, calibrate at the start of autonomous (sweep or full rotation,
// see below). When false, skip it and use kSavedMin / kSavedMax. Copy those
// from the calibration test output on the real field.
constexpr bool kCalibrateAtStart = true;
constexpr int kNumSensors = 6;
// From a hand calibration on the robot (Nov 7). Recalibrate on the real field.
constexpr uint16_t kSavedMin[kNumSensors] = {966, 1036, 884, 907, 926, 886};
constexpr uint16_t kSavedMax[kNumSensors] = {2847, 2865, 2385,
                                             2484, 2760, 3212};

// Calibration motion:
//   true  = FULL ROTATION: spin in place like LF-2. Every sensor passes over
//           the tape (about 2 crossings per sensor per turn), but the robot
//           only ends facing its start direction if kFullRotationMs is right.
//   false = SWEEP: wiggle left/right over the line. Ends facing about the same
//           way, but sensors only cross the tape if the line is under the
//           robot at the start.
constexpr bool kCalibrationFullRotation = false;
// Time for ONE 360-degree turn at kCalibrationSpeed. Measure it: the
// calibration printout reports tape crossings per sensor; aim for ~2 per
// sensor per rotation. Fewer = increase this, more = decrease it.
constexpr uint32_t kFullRotationMs = 2000;
constexpr int kFullRotations = 1;               // Number of turns.
constexpr bool kFullRotationClockwise = true;   // Spin direction.

constexpr uint32_t kCalibrationDurationMs = 3000;  // Sweep mode only.
constexpr uint32_t kCalibrationSweepMs = 500;  // Time per sweep direction.
constexpr int kCalibrationSpeed = 35;          // Pivot speed (both modes).
constexpr uint16_t kMinUsefulRange = 300;      // us. Smaller: never saw tape.
constexpr uint16_t kSensorTimeoutUs = 10000;   // Library's read timeout.
constexpr int kMaxCalibrationSamples = 600;    // Buffer for crossing count.

// ============================ Debug printing ===============================
constexpr bool kDebugPrint = true;
constexpr uint32_t kDebugPrintPeriodMs = 100;  // One status line per period.
constexpr uint32_t kTestPrintPeriodMs = 200;   // Calibration test stream.

// ============================== Robot setup ================================
constexpr uint8_t kLeftMotor = 0;   // Wired: motor port 0 = left wheel.
constexpr uint8_t kRightMotor = 2;  // Wired: motor port 2 = right wheel.
// Left wheel spins backward for positive speed (drive test: "forward" turned
// left, "turn left" went forward), so it is inverted.
constexpr bool kInvertLeftMotor = true;
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
// Inner wheel may run BACKWARD up to this % while following, so sharp bends
// can be turned tighter. Only happens on big errors (correction > speed);
// straight-line following is unchanged. 0 = never reverse (LF-2).
constexpr int kMaxReverse = 30;
constexpr uint32_t kAutonDurationMs = 20000;
constexpr uint32_t kControlPeriodMs = 15;   // Minimum time between steps.
constexpr float kIntegralLimit = 2.0f;      // |integral| cap (error x second).

// Normalized-value thresholds (0 = floor, 1 = line).
constexpr float kOnLineThreshold = 0.5f;  // LF-2's (min + max) / 2 midpoint.
constexpr float kNoiseFloor = 0.2f;       // Ignore weaker readings (floor
                                          // noise) in the position average.
// The line counts as seen only if at least ONE sensor reads at least this.
// (It used to be "sum of all sensors >= 0.3", which a slightly dark floor
// reaches on 2-3 sensors, so a lost line was never detected.)
constexpr float kLineSeenThreshold = 0.4f;

// ================================= Bends ===================================
// When kWideSensorCount or more sensors see tape (a bend or an intersection),
// remember which side has more tape. If the line is then lost within
// kBranchMemoryMs, the lost-line turn goes toward that side (the bend)
// instead of the side the line was last seen, which is ~random at the end of
// a bend. Tape even on both sides (a + intersection) clears the memory, so
// intersections still go straight. false = old lost-line turn only.
constexpr bool kRememberBends = true;
constexpr int kWideSensorCount = 3;
constexpr float kBendMinOffset = 0.3f;     // |avg position| = one-sided.
constexpr uint32_t kBranchMemoryMs = 800;

// ============================ Finding the line ==============================
// SEARCH (kSearchForLine): at the start, drive straight at kSearchSpeed until
// a sensor sees the line, instead of spinning in place. Gives up (stops)
// after kSearchTimeoutMs.
constexpr bool kSearchForLine = true;
constexpr int kSearchSpeed = 30;
constexpr uint32_t kSearchTimeoutMs = 5000;

// SWEEP (FollowUntilLineEnds with find_line_first, used by controller Y):
// pivot left/right in place, each swing wider, until a sensor sees the line.
// Swing sizes are in ms of pivoting (~7.4 ms per degree at 35% on this
// robot): first swing ~35 deg each side, growing ~35 deg per swing, max
// ~120 deg each side.
constexpr int kSweepSpeed = 35;
constexpr uint32_t kSweepFirstMs = 260;
constexpr uint32_t kSweepGrowMs = 260;
constexpr uint32_t kSweepMaxMs = 890;
constexpr uint32_t kSweepTimeoutMs = 6000;

// ACQUIRE (kAcquireAtAngle): once the line is found, line up with it before
// PID, so a steep approach angle doesn't make it overshoot or turn the long
// way (the LF-2 edge rule would pivot toward the side the line touched
// first, which is the WRONG way on an angled approach).
//   1. ACQUIRE:  keep creeping straight at kAcquireSpeed, edge rule off,
//                until a center sensor (S2/S3) is on the line. Note which way
//                the line moved across the sensor bar on the way:
//                left -> right means the line's forward direction is RIGHT.
//                If that took longer than kShallowSweepMs, the angle is
//                shallow: start PID right away (PID handles shallow angles).
//   2. DRIVE_ON: keep going kDriveOnMs so the WHEELS reach the line (the bar
//                is in front of the wheels). MEASURE THIS on your robot.
//   3. ALIGN:    pivot toward the line's forward direction until a center
//                sensor leaves and then finds the line again, then PID.
constexpr bool kAcquireAtAngle = true;
constexpr int kAcquireSpeed = 25;
constexpr uint32_t kShallowSweepMs = 600;
constexpr uint32_t kDriveOnMs = 300;
constexpr int kAlignSpeed = 35;
constexpr uint32_t kAlignMinMs = 150;      // Pivot at least this long.
constexpr uint32_t kAlignTimeoutMs = 3000;  // Then give up and follow.
constexpr float kMinSweep = 0.5f;  // Line must move this far across the bar
                                   // to decide a direction (else default).
constexpr bool kDefaultTurnRight = true;  // Head-on (no clear sweep).

// Sensor positions, robot's left to right, in sensor spacings from center.
constexpr float kSensorPosition[kNumSensors] = {-2.5f, -1.5f, -0.5f,
                                                0.5f,  1.5f,  2.5f};

// ================================= State ===================================
// kSearch/kAcquire/kDriveOn/kAlign: finding the line at the start.
enum class Mode {
  kSearch, kAcquire, kDriveOn, kAlign,
  kPid, kEdgeLeft, kEdgeRight, kLost, kDone
};

struct SensorReading {
  uint16_t raw[kNumSensors];  // Discharge time, us, robot's left to right.
  float norm[kNumSensors];    // 0 = floor, 1 = line.
  bool on_line[kNumSensors];  // norm >= kOnLineThreshold.
  float position = 0.0f;      // Weighted average, -2.5 .. +2.5.
  float strength = 0.0f;      // Sum of weighted norms.
  float max_norm = 0.0f;      // Strongest sensor (decides line_visible).
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
// Raw calibration samples, kept so we can count tape crossings afterwards.
uint16_t g_cal_samples[kMaxCalibrationSamples][kNumSensors];
int g_cal_sample_count = 0;
bool g_initialized = false;
int g_last_on_line_count = 0;  // For auton: sensors on the line, last step.
bool g_calibrated = false;  // g_min/g_max hold real values.
uint32_t g_auton_start_ms = 0;
uint32_t g_last_step_ms = 0;
uint32_t g_last_pid_us = 0;
uint32_t g_last_print_ms = 0;
float g_integral = 0.0f;
float g_previous_error = 0.0f;
bool g_has_previous_error = false;
float g_last_position = 0.0f;
Mode g_mode = Mode::kPid;
int g_bend_side = 0;           // -1 left / +1 right: last one-sided wide read.
uint32_t g_bend_seen_ms = 0;   // When it was seen (0 = none).
uint32_t g_phase_start_ms = 0;  // When the current find-the-line step began.
float g_first_seen_pos = 0.0f;  // Where the line first appeared (ACQUIRE).
bool g_align_right = true;      // ALIGN pivot direction.
bool g_align_left_line = false;  // ALIGN: center has left the line once.

const char* ModeName(Mode mode) {
  switch (mode) {
    case Mode::kSearch:
      return "SEARCH";
    case Mode::kAcquire:
      return "ACQUIRE";
    case Mode::kDriveOn:
      return "DRIVE_ON";
    case Mode::kAlign:
      return "ALIGN";
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
  reading->max_norm = 0.0f;
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
    if (value > reading->max_norm) reading->max_norm = value;

    const float weight = value < kNoiseFloor ? 0.0f : value;
    weighted_sum += weight * kSensorPosition[i];
    reading->strength += weight;
  }

  reading->line_visible = reading->max_norm >= kLineSeenThreshold &&
                          reading->strength > 0.0f;
  reading->position =
      reading->line_visible ? weighted_sum / reading->strength : 0.0f;
}

// Counts how many times sensor `s` moved from floor onto tape during
// calibration, using the final threshold with a 15% hysteresis band so noise
// near the threshold is not counted twice.
int CountTapeCrossings(int s) {
  const int range = static_cast<int>(g_max[s]) - static_cast<int>(g_min[s]);
  if (range < kMinUsefulRange || g_cal_sample_count == 0) return 0;
  const int band = range * 15 / 100;
  const int high = (g_min[s] + g_max[s]) / 2 + band;  // Clearly on tape.
  const int low = (g_min[s] + g_max[s]) / 2 - band;   // Clearly on floor.
  // With kLineReadsHigher, "on tape" means a high reading; otherwise low.
  bool on_tape = kLineReadsHigher ? g_cal_samples[0][s] >= high
                                  : g_cal_samples[0][s] <= low;
  int crossings = 0;
  for (int k = 1; k < g_cal_sample_count; ++k) {
    const int v = g_cal_samples[k][s];
    const bool now_on = kLineReadsHigher ? v >= high : v <= low;
    const bool now_off = kLineReadsHigher ? v <= low : v >= high;
    if (!on_tape && now_on) {
      on_tape = true;
      ++crossings;
    } else if (on_tape && now_off) {
      on_tape = false;
    }
  }
  return crossings;
}

void PrintCalibration(uint32_t duration_ms, int samples) {
  Serial.printf("[CAL] Done: %lu ms, %d samples (S0 = robot's far left)\n",
                static_cast<unsigned long>(duration_ms), samples);
  if (samples > kMaxCalibrationSamples) {
    Serial.printf("[CAL] NOTE: only the first %d samples were kept for the "
                  "crossing count (min/max used all %d).\n",
                  kMaxCalibrationSamples, samples);
  }
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

  // Tape crossings: tells you whether the motion actually covered the line,
  // and in full-rotation mode whether kFullRotationMs is about one turn.
  Serial.print("[CAL] tape crossings per sensor: [");
  int total = 0;
  int counted = 0;
  for (int i = 0; i < kNumSensors; ++i) {
    const int crossings = CountTapeCrossings(i);
    Serial.printf("%d%s", crossings, i < kNumSensors - 1 ? " " : "]\n");
    if (g_max[i] - g_min[i] >= kMinUsefulRange) {
      total += crossings;
      ++counted;
    }
  }
  if (kCalibrationUseMotors && kCalibrationFullRotation && counted > 0) {
    const float average = static_cast<float>(total) / counted;
    const int expected = 2 * kFullRotations;
    Serial.printf("[CAL] Full rotation: average %.1f crossings/sensor, "
                  "expected ~%d for %d turn(s) of %lu ms.\n",
                  average, expected, kFullRotations,
                  static_cast<unsigned long>(kFullRotationMs));
    if (average < expected - 0.5f) {
      Serial.printf("[CAL] -> Turned LESS than %d full turn(s). Increase "
                    "kFullRotationMs (try ~%lu).\n",
                    kFullRotations,
                    static_cast<unsigned long>(kFullRotationMs * expected /
                                               max(average, 0.5f)));
    } else if (average > expected + 0.5f) {
      Serial.printf("[CAL] -> Turned MORE than %d full turn(s). Decrease "
                    "kFullRotationMs (try ~%lu).\n",
                    kFullRotations,
                    static_cast<unsigned long>(kFullRotationMs * expected /
                                               average));
    } else {
      Serial.println("[CAL] -> Rotation time looks right.");
    }
  }
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

// Moves over the line while recording each sensor's min and max.
//   Hand mode (kCalibrationUseMotors = false): motors stay off; you move the
//   robot over the tape yourself.
//   Sweep mode: symmetric left/right wiggle, so the robot ends facing about
//   where it started.
//   Full-rotation mode: constant spin in place (like LF-2) for
//   kFullRotations turns, timed by kFullRotationMs.
void Calibrate() {
  const uint32_t duration_ms =
      !kCalibrationUseMotors     ? kHandCalibrationMs
      : kCalibrationFullRotation ? kFullRotationMs * kFullRotations
                                 : kCalibrationDurationMs;
  if (!kCalibrationUseMotors) {
    Serial.printf("[CAL] Hand calibration: motors OFF. For the next %lu ms, "
                  "slide the robot back and forth over the tape so EVERY "
                  "sensor passes over it a few times.\n",
                  static_cast<unsigned long>(duration_ms));
  } else if (kCalibrationFullRotation) {
    Serial.printf("[CAL] Full rotation: %d turn(s) %s, %lu ms at speed %d.\n",
                  kFullRotations,
                  kFullRotationClockwise ? "clockwise" : "counterclockwise",
                  static_cast<unsigned long>(duration_ms), kCalibrationSpeed);
  } else {
    Serial.printf("[CAL] Sweep: %lu ms at speed %d. Start over the line.\n",
                  static_cast<unsigned long>(duration_ms), kCalibrationSpeed);
  }
  for (int i = 0; i < kNumSensors; ++i) {
    g_min[i] = UINT16_MAX;
    g_max[i] = 0;
  }
  g_cal_sample_count = 0;

  const uint32_t start_ms = millis();
  int samples = 0;
  uint32_t elapsed_ms = 0;
  while ((elapsed_ms = millis() - start_ms) < duration_ms) {
    if (RedRobot::IsRemoteControlled()) {
      Serial.println("[CAL] Aborted: remote control took over.");
      break;
    }
    bool turn_left;
    if (kCalibrationFullRotation) {
      turn_left = !kFullRotationClockwise;  // Seen from above.
    } else {
      // Phases: half sweep left, alternate full sweeps, end with half left.
      turn_left =
          ((elapsed_ms + kCalibrationSweepMs / 2) / kCalibrationSweepMs) % 2 ==
          0;
    }
    if (!kCalibrationUseMotors) {
      // Hand mode: never touch the motors.
    } else if (turn_left) {
      SetDrive(-kCalibrationSpeed, kCalibrationSpeed);
    } else {
      SetDrive(kCalibrationSpeed, -kCalibrationSpeed);
    }

    uint16_t hw[kNumSensors];
    RedRobot::ReadLineSensor(hw);
    const bool keep = g_cal_sample_count < kMaxCalibrationSamples;
    for (int i = 0; i < kNumSensors; ++i) {
      const uint16_t value =
          hw[kReverseSensorOrder ? (kNumSensors - 1 - i) : i];
      if (value < g_min[i]) g_min[i] = value;
      if (value > g_max[i]) g_max[i] = value;
      if (keep) g_cal_samples[g_cal_sample_count][i] = value;
    }
    if (keep) ++g_cal_sample_count;
    ++samples;
  }
  SetDrive(0, 0);
  PrintCalibration(elapsed_ms, samples);
  g_calibrated = true;
}

void PrintConfig() {
  Serial.println("[LF] ===== Line follower config =====");
  Serial.printf("[LF] mode=%s  auto_gain=%s  calibrate_at_start=%s\n",
                kCalibrationTestMode ? "CALIBRATION_TEST" : "FOLLOW",
                kUseAutoGain ? "ON" : "OFF", kCalibrateAtStart ? "YES" : "NO");
  Serial.printf("[LF] calibration motion=%s\n",
                !kCalibrationUseMotors     ? "BY_HAND (motors off)"
                : kCalibrationFullRotation ? "FULL_ROTATION"
                                           : "SWEEP");
  if (kUseAutoGain) {
    Serial.printf("[LF] Kp=%.2f..%.2f (auto)  Ki=%.3f  Kd=%.3f*Kp\n",
                  kAutoKpMin, kAutoKpMax, kKi, kAutoKdRatio);
  } else {
    Serial.printf("[LF] Kp=%.2f  Ki=%.3f  Kd=%.3f\n", kKp, kKi, kKd);
  }
  Serial.printf("[LF] base=%d min=%d period=%lums auton=%lums\n", kBaseSpeed,
                kMinSpeed, static_cast<unsigned long>(kControlPeriodMs),
                static_cast<unsigned long>(kAutonDurationMs));
  Serial.printf("[LF] max_reverse=%d%%  remember_bends=%s (>=%d sensors, "
                "|pos|>=%.1f, %lums)\n",
                kMaxReverse, kRememberBends ? "ON" : "OFF", kWideSensorCount,
                kBendMinOffset, static_cast<unsigned long>(kBranchMemoryMs));
  Serial.printf("[LF] line seen if a sensor >= %.2f (else LOST); position "
                "ignores readings < %.2f. Status 'max=0.xx(<seen)' = line lost.\n",
                kLineSeenThreshold, kNoiseFloor);
  Serial.println("[LF] Status line: t, dt (time since last PID step), read "
                 "(sensor read time), mode, n (0=floor 1=line, left->right), "
                 "pos (<0 = line on left), gains, P/I/D terms, corr, speeds");
}

void PrintStatus(uint32_t now_ms, float dt_ms, const SensorReading& reading,
                 const PidTerms& pid, int speed, int left, int right) {
  Serial.printf(
      "[LF] t=%lu dt=%.1fms read=%.1fms mode=%-6s "
      "n=[%.2f %.2f %.2f %.2f %.2f %.2f] max=%.2f%s pos=%+.2f "
      "kp=%.1f ki=%.2f kd=%.2f P=%+.1f I=%+.1f D=%+.1f corr=%+.1f "
      "spd=%d L=%d R=%d\n",
      static_cast<unsigned long>(now_ms - g_auton_start_ms), dt_ms,
      reading.read_us / 1000.0f, ModeName(g_mode), reading.norm[0],
      reading.norm[1], reading.norm[2], reading.norm[3], reading.norm[4],
      reading.norm[5], reading.max_norm,
      reading.line_visible ? "" : "(<seen)", reading.position, pid.kp,
      pid.ki, pid.kd, pid.p, pid.i, pid.d, pid.correction, speed, left, right);
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

void StartRun(uint32_t now_ms);  // Defined below, with the find-line steps.

void Initialize() {
  g_initialized = true;
  if (kCalibrationTestMode) {
    // Wait (up to 15 s) for a serial monitor so the calibration output is
    // not lost, then count down so you can get ready to move the robot.
    const uint32_t wait_start = millis();
    while (!Serial && millis() - wait_start < 15000) delay(10);
  }
  PrintConfig();
  if (kCalibrationTestMode) {
    for (int s = 3; s > 0; --s) {
      Serial.printf("[CAL] Calibration starts in %d...\n", s);
      delay(1000);
    }
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
    g_calibrated = true;
    Serial.println("[LF] Using saved calibration (kSavedMin / kSavedMax).");
  }
  ResetPid();
  g_auton_start_ms = millis();  // Start the clock after calibration.
  g_last_pid_us = micros();
  StartRun(g_auton_start_ms);
}

// A center sensor (S2 or S3) is on the line.
bool CenterOnLine(const SensorReading& r) {
  return r.on_line[2] || r.on_line[3];
}

void EnterPhase(Mode mode, uint32_t now_ms) {
  SetMode(mode, now_ms);
  g_phase_start_ms = now_ms;
}

// Sets the wheels for a pivot toward g_align_right.
void AlignPivot(int* left, int* right) {
  *left = g_align_right ? kAlignSpeed : -kAlignSpeed;
  *right = -*left;
}

// Puts the robot in its first mode for a new run: SEARCH, or PID directly.
void StartRun(uint32_t now_ms) {
  g_mode = kSearchForLine ? Mode::kSearch : Mode::kPid;
  g_phase_start_ms = now_ms;
  if (kDebugPrint) {
    if (kSearchForLine) {
      Serial.printf("[LF] start: SEARCH, driving straight at %d%% until the "
                    "line is seen (acquire at angle: %s)\n",
                    kSearchSpeed, kAcquireAtAngle ? "ON" : "OFF");
    } else {
      Serial.println("[LF] start: PID (robot should already be on the line)");
    }
  }
}

// Finding the line: SEARCH -> ACQUIRE -> (DRIVE_ON -> ALIGN) -> PID.
// Returns true and sets the wheels while in one of these steps. Returns
// false once the robot is in a normal following mode, so ControlStep runs
// the usual PID / edge / lost logic (also on the step where we switch).
bool FindLineStep(const SensorReading& reading, uint32_t now_ms, int* left,
                  int* right) {
  const uint32_t in_phase = now_ms - g_phase_start_ms;
  switch (g_mode) {
    case Mode::kSearch:
      if (reading.line_visible) {
        g_first_seen_pos = reading.position;
        if (kDebugPrint) {
          Serial.printf("[LF] SEARCH: line found after %lu ms at pos %+.2f\n",
                        static_cast<unsigned long>(in_phase),
                        reading.position);
        }
        if (!kAcquireAtAngle) {
          ResetPid();
          SetMode(Mode::kPid, now_ms);
          return false;
        }
        EnterPhase(Mode::kAcquire, now_ms);
        *left = *right = kAcquireSpeed;
        return true;
      }
      if (in_phase >= kSearchTimeoutMs) {
        Serial.printf("[LF] SEARCH: no line after %lu ms. Stopped.\n",
                      static_cast<unsigned long>(in_phase));
        SetMode(Mode::kDone, now_ms);
        *left = *right = 0;
        return true;
      }
      *left = *right = kSearchSpeed;
      return true;

    case Mode::kAcquire: {
      if (!reading.line_visible) {
        if (kDebugPrint) {
          Serial.println("[LF] ACQUIRE: line gone before the center saw it "
                         "(false hit?), searching again");
        }
        EnterPhase(Mode::kSearch, now_ms);
        *left = *right = kSearchSpeed;
        return true;
      }
      if (!CenterOnLine(reading)) {  // Keep creeping, edge rule off.
        *left = *right = kAcquireSpeed;
        return true;
      }
      // + sweep = the line moved left -> right across the bar.
      const float sweep = reading.position - g_first_seen_pos;
      const bool clear = fabsf(sweep) >= kMinSweep;
      const bool shallow = in_phase > kShallowSweepMs;
      g_align_right = clear ? sweep > 0.0f : kDefaultTurnRight;
      if (kDebugPrint) {
        Serial.printf(
            "[LF] ACQUIRE: center on line after %lu ms, line moved %+.2f "
            "across the bar (%s) -> %s\n",
            static_cast<unsigned long>(in_phase), sweep,
            clear ? (sweep > 0.0f ? "left->right" : "right->left")
                  : "no clear direction, using default",
            shallow ? "shallow angle: start PID"
                    : (g_align_right ? "steep: drive on, then turn RIGHT"
                                     : "steep: drive on, then turn LEFT"));
      }
      if (shallow) {
        ResetPid();
        g_last_position = reading.position;
        SetMode(Mode::kPid, now_ms);
        return false;
      }
      EnterPhase(Mode::kDriveOn, now_ms);
      *left = *right = kAcquireSpeed;
      return true;
    }

    case Mode::kDriveOn:
      if (in_phase >= kDriveOnMs) {
        g_align_left_line = !CenterOnLine(reading);
        EnterPhase(Mode::kAlign, now_ms);
        AlignPivot(left, right);
        return true;
      }
      *left = *right = kAcquireSpeed;
      return true;

    case Mode::kAlign:
      if (!CenterOnLine(reading)) g_align_left_line = true;
      if (in_phase >= kAlignMinMs && g_align_left_line &&
          CenterOnLine(reading)) {
        if (kDebugPrint) {
          Serial.printf("[LF] ALIGN: lined up after %lu ms, following\n",
                        static_cast<unsigned long>(in_phase));
        }
        ResetPid();
        g_last_position = reading.position;
        SetMode(Mode::kPid, now_ms);
        return false;
      }
      if (in_phase >= kAlignTimeoutMs) {
        Serial.printf("[LF] ALIGN: line not found after %lu ms, following "
                      "anyway\n",
                      static_cast<unsigned long>(in_phase));
        ResetPid();
        SetMode(Mode::kPid, now_ms);
        return false;
      }
      AlignPivot(left, right);
      return true;

    default:
      return false;
  }
}

// One control step: read sensors, pick a mode (PID / edge pivot / lost),
// drive the wheels, and print a status line every kDebugPrintPeriodMs.
void ControlStep(uint32_t now_ms) {
  SensorReading reading;
  ReadSensors(&reading);
  g_last_on_line_count = 0;
  for (int i = 0; i < kNumSensors; ++i) {
    if (reading.on_line[i]) ++g_last_on_line_count;
  }

  const uint32_t now_us = micros();
  const float dt_s = (now_us - g_last_pid_us) / 1e6f;
  g_last_pid_us = now_us;

  PidTerms pid;
  int speed = 0;
  int left = 0;
  int right = 0;

  if (g_mode == Mode::kDone) {  // e.g. SEARCH gave up.
    SetDrive(0, 0);
    return;
  }
  // Finding the line at the start (SEARCH / ACQUIRE / DRIVE_ON / ALIGN).
  if (FindLineStep(reading, now_ms, &left, &right)) {
    SetDrive(left, right);
    if (kDebugPrint && now_ms - g_last_print_ms >= kDebugPrintPeriodMs) {
      g_last_print_ms = now_ms;
      const int speed_now = left == right ? left : 0;  // 0 while pivoting.
      PrintStatus(now_ms, dt_s * 1000.0f, reading, pid, speed_now, left,
                  right);
    }
    return;
  }

  // Many sensors on tape: a bend (tape mostly on one side) or an
  // intersection (tape on both sides). Remember a bend's side.
  if (kRememberBends) {
    int on_count = 0;
    for (int i = 0; i < kNumSensors; ++i) on_count += reading.on_line[i];
    if (on_count >= kWideSensorCount) {
      if (fabsf(reading.position) >= kBendMinOffset) {
        g_bend_side = reading.position < 0.0f ? -1 : 1;
        g_bend_seen_ms = now_ms;
      } else {
        g_bend_seen_ms = 0;  // Even on both sides: intersection, go straight.
      }
    }
  }

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
    // The inner wheel may reverse (down to -kMaxReverse) on sharp turns.
    left = constrain(static_cast<int>(speed + pid.correction), -kMaxReverse,
                     100);
    right = constrain(static_cast<int>(speed - pid.correction), -kMaxReverse,
                      100);
    g_last_position = reading.position;
  } else {
    // Line lost: pivot toward a bend seen just before (the line turned),
    // otherwise toward the side the line was last seen on.
    const bool entering = g_mode != Mode::kLost;
    SetMode(Mode::kLost, now_ms);
    const bool recent_bend = kRememberBends && g_bend_seen_ms != 0 &&
                             now_ms - g_bend_seen_ms < kBranchMemoryMs;
    const bool turn_left =
        recent_bend ? g_bend_side < 0 : g_last_position < 0.0f;
    if (entering && kDebugPrint) {
      if (recent_bend) {
        Serial.printf("[LF] line lost: turning %s toward the bend seen %lu "
                      "ms ago\n",
                      turn_left ? "LEFT" : "RIGHT",
                      static_cast<unsigned long>(now_ms - g_bend_seen_ms));
      } else {
        Serial.printf("[LF] line lost: turning %s (last pos %+.2f, no "
                      "recent bend)\n",
                      turn_left ? "LEFT" : "RIGHT", g_last_position);
      }
    }
    if (turn_left) {
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

  ControlStep(now_ms);
}

void CalibrateNow() { Calibrate(); }

bool FollowFor(uint32_t duration_ms, bool (*should_abort)()) {
  if (!g_calibrated) {
    for (int i = 0; i < kNumSensors; ++i) {
      g_min[i] = kSavedMin[i];
      g_max[i] = kSavedMax[i];
    }
    g_calibrated = true;
    Serial.println("[LF] Not calibrated yet: using kSavedMin / kSavedMax. "
                   "Run the calibrate test first for real values.");
  }
  Serial.printf("[LF] Following for %lu ms: Kp=%.2f Ki=%.3f Kd=%.3f base=%d "
                "auto_gain=%s\n",
                static_cast<unsigned long>(duration_ms), kKp, kKi, kKd,
                kBaseSpeed, kUseAutoGain ? "ON" : "OFF");
  ResetPid();
  g_last_position = 0.0f;
  g_bend_side = 0;
  g_bend_seen_ms = 0;
  g_auton_start_ms = millis();
  g_last_step_ms = 0;
  g_last_print_ms = 0;
  g_last_pid_us = micros();
  StartRun(g_auton_start_ms);

  bool completed = true;
  while (millis() - g_auton_start_ms < duration_ms) {
    if (should_abort != nullptr && should_abort()) {
      completed = false;
      break;
    }
    const uint32_t now_ms = millis();
    if (now_ms - g_last_step_ms >= kControlPeriodMs) {
      g_last_step_ms = now_ms;
      ControlStep(now_ms);
    }
    if (g_mode == Mode::kDone) break;  // e.g. SEARCH found no line.
    delay(1);
  }
  SetDrive(0, 0);
  Serial.printf("[LF] Follow %s after %lu ms. Motors stopped.\n",
                completed ? "finished" : "ABORTED",
                static_cast<unsigned long>(millis() - g_auton_start_ms));
  return completed;
}

// ------------------------- Hooks for src/auton.cpp -------------------------

void Init() {
  if (!g_initialized) Initialize();
}

void StartFollowing() {
  if (!g_initialized) Initialize();
  ResetPid();
  g_mode = Mode::kPid;
  g_auton_start_ms = millis();  // Restart the kAutonDurationMs safety timer.
  g_last_step_ms = 0;
  g_last_pid_us = micros();
  g_last_on_line_count = 0;
  // Added in the merge with test: the line was already found by auton's
  // FIND_LINE stage, so follow right away (no SEARCH) and forget bends.
  g_bend_side = 0;
  g_bend_seen_ms = 0;
  g_last_position = 0.0f;
}

bool SeesLine() {
  if (!g_initialized) Initialize();
  SensorReading reading;
  ReadSensors(&reading);
  return reading.line_visible;
}

int LastOnLineCount() { return g_last_on_line_count; }

bool IsCalibrationTestMode() { return kCalibrationTestMode; }

bool FollowUntilLineEnds(uint32_t arm_ms, uint32_t timeout_ms,
                         bool (*should_abort)(), bool find_line_first) {
  if (!g_calibrated) {
    for (int i = 0; i < kNumSensors; ++i) {
      g_min[i] = kSavedMin[i];
      g_max[i] = kSavedMax[i];
    }
    g_calibrated = true;
    Serial.println("[LF] Not calibrated yet: using kSavedMin / kSavedMax.");
  }
  if (find_line_first) {
    // Sweep left/right in place, each swing wider, until a sensor sees the
    // line, so a robot that isn't on the line yet doesn't count as "line
    // ended" on the first step. `heading` = estimated turn from the start,
    // in ms of pivoting (+ = left).
    Serial.printf("[LF] Sweeping for the line: %d%%, first swing %lu ms, max "
                  "%lu ms each side, give up after %lu ms\n",
                  kSweepSpeed, static_cast<unsigned long>(kSweepFirstMs),
                  static_cast<unsigned long>(kSweepMaxMs),
                  static_cast<unsigned long>(kSweepTimeoutMs));
    const uint32_t search_start = millis();
    uint32_t last_ms = search_start;
    long heading = 0;
    long amplitude = kSweepFirstMs;
    int dir = 1;  // +1 = turning left, -1 = turning right.
    long target = amplitude;
    bool found = false;
    while (millis() - search_start < kSweepTimeoutMs) {
      if (should_abort != nullptr && should_abort()) {
        SetDrive(0, 0);
        return false;
      }
      SensorReading reading;
      ReadSensors(&reading);
      if (reading.line_visible) {
        found = true;
        SetDrive(0, 0);
        Serial.printf("[LF] Line found after %lu ms of sweeping (max=%.2f, "
                      "pos %+.2f)\n",
                      static_cast<unsigned long>(millis() - search_start),
                      reading.max_norm, reading.position);
        break;
      }
      const uint32_t now = millis();
      heading += dir * static_cast<long>(now - last_ms);
      last_ms = now;
      if ((dir > 0 && heading >= target) || (dir < 0 && heading <= target)) {
        amplitude = min(amplitude + static_cast<long>(kSweepGrowMs),
                        static_cast<long>(kSweepMaxMs));
        dir = -dir;
        target = dir * amplitude;
        if (kDebugPrint) {
          Serial.printf("[LF]   sweep: now turning %s (swing %ld ms)\n",
                        dir > 0 ? "LEFT" : "RIGHT", amplitude);
        }
      }
      // Pivot in place: left = left wheel back, right wheel forward.
      SetDrive(dir > 0 ? -kSweepSpeed : kSweepSpeed,
               dir > 0 ? kSweepSpeed : -kSweepSpeed);
    }
    if (!found) {
      SetDrive(0, 0);
      Serial.printf("[LF] No line found within %lu ms of sweeping. Motors "
                    "stopped.\n",
                    static_cast<unsigned long>(kSweepTimeoutMs));
      return false;
    }
  }
  Serial.printf("[LF] Following until the line ends (checking after %lu ms, "
                "timeout %lu ms): Kp=%.2f Kd=%.3f base=%d\n",
                static_cast<unsigned long>(arm_ms),
                static_cast<unsigned long>(timeout_ms), kKp, kKd, kBaseSpeed);
  ResetPid();
  g_mode = Mode::kPid;  // No line finder: the robot starts on the line.
  g_last_position = 0.0f;
  g_bend_side = 0;
  g_bend_seen_ms = 0;
  g_auton_start_ms = millis();
  g_last_step_ms = 0;
  g_last_print_ms = 0;
  g_last_pid_us = micros();

  while (millis() - g_auton_start_ms < timeout_ms) {
    if (should_abort != nullptr && should_abort()) {
      SetDrive(0, 0);
      return false;
    }
    const uint32_t now_ms = millis();
    if (now_ms - g_last_step_ms >= kControlPeriodMs) {
      g_last_step_ms = now_ms;
      ControlStep(now_ms);
      const uint32_t elapsed = now_ms - g_auton_start_ms;
      if (elapsed >= arm_ms && g_mode == Mode::kLost) {
        SetDrive(0, 0);
        Serial.printf("[LF] LINE ENDED at t=%lu ms (last pos %+.2f). Motors "
                      "stopped.\n",
                      static_cast<unsigned long>(elapsed), g_last_position);
        return true;
      }
    }
    delay(1);
  }
  SetDrive(0, 0);
  Serial.printf("[LF] Line did NOT end within %lu ms. Motors stopped.\n",
                static_cast<unsigned long>(timeout_ms));
  return false;
}

bool FollowOnLineFor(uint32_t duration_ms, bool (*should_abort)()) {
  if (!g_calibrated) {
    for (int i = 0; i < kNumSensors; ++i) {
      g_min[i] = kSavedMin[i];
      g_max[i] = kSavedMax[i];
    }
    g_calibrated = true;
    Serial.println("[LF] Not calibrated yet: using kSavedMin / kSavedMax.");
  }
  Serial.printf("[LF] Following for %lu ms (from on the line): Kp=%.2f "
                "Kd=%.3f base=%d\n",
                static_cast<unsigned long>(duration_ms), kKp, kKd, kBaseSpeed);
  ResetPid();
  g_mode = Mode::kPid;  // No line finder: the robot starts on the line.
  g_last_position = 0.0f;
  g_bend_side = 0;
  g_bend_seen_ms = 0;
  g_auton_start_ms = millis();
  g_last_step_ms = 0;
  g_last_print_ms = 0;
  g_last_pid_us = micros();

  while (millis() - g_auton_start_ms < duration_ms) {
    if (should_abort != nullptr && should_abort()) {
      SetDrive(0, 0);
      return false;
    }
    const uint32_t now_ms = millis();
    if (now_ms - g_last_step_ms >= kControlPeriodMs) {
      g_last_step_ms = now_ms;
      ControlStep(now_ms);
    }
    delay(1);
  }
  SetDrive(0, 0);
  Serial.printf("[LF] Followed for %lu ms. Motors stopped.\n",
                static_cast<unsigned long>(duration_ms));
  return true;
}

}  // namespace line_follower
