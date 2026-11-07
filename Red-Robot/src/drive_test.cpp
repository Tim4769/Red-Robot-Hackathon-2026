// Serial test console for the drivetrain. See drive_test.h.
//
// Every test prints [DT] lines saying exactly what it commands, so you can
// match the printout to what the robot did.

#include "drive_test.h"

#include <Arduino.h>
#include <RedRobot.h>
#include <esp_attr.h>
#include <esp_system.h>
#include <stdlib.h>
#include <string.h>

#include "line_follower.h"

namespace drive_test {
namespace {

// ============================ Drive wiring =================================
// Same wiring as line_follower.cpp and RemoteControl.cpp.
constexpr uint8_t kLeftMotor = 0;
constexpr uint8_t kRightMotor = 2;
constexpr bool kInvertLeft = true;   // Left wheel is wired backward.
constexpr bool kInvertRight = false;

constexpr uint32_t kCountdownMs = 3000;  // Before any test that drives.
constexpr uint32_t kStressMarker = 0x5752E55A;

// Survives a reset (not a power cycle): set while the stress test runs, so a
// brownout reset mid-test can be reported after the reboot.
RTC_NOINIT_ATTR uint32_t g_stress_marker;
RTC_NOINIT_ATTR uint32_t g_stress_cycle;

int g_left_trim = 100;   // Percent of commanded speed, set with "trim L R".
int g_right_trim = 100;
bool g_was_connected = false;
char g_line[48];
size_t g_line_len = 0;

// ============================== Helpers ====================================

// Sets wheel speeds in percent (-100..100, + = robot forward), applying trim
// and wiring inversion.
void SetDrive(int left, int right) {
  left = constrain(left * g_left_trim / 100, -100, 100);
  right = constrain(right * g_right_trim / 100, -100, 100);
  RedRobot::WriteDCMotor(kLeftMotor, kInvertLeft ? -left : left);
  RedRobot::WriteDCMotor(kRightMotor, kInvertRight ? -right : right);
}

void Stop() { SetDrive(0, 0); }

// True if the user pressed a key (CR/LF ignored) or remote control took over.
// Consumes the input. Used by every test.
bool ShouldAbort() {
  bool key = false;
  while (Serial.available() > 0) {
    const int c = Serial.read();
    if (c != '\r' && c != '\n') key = true;
  }
  if (key) {
    Stop();
    Serial.println("\n[DT] ABORTED by keypress. Motors stopped.");
    return true;
  }
  if (RedRobot::IsRemoteControlled()) {
    Stop();
    Serial.println("\n[DT] ABORTED: remote control took over.");
    return true;
  }
  return false;
}

// Waits `ms`, checking for abort every 10 ms. Returns false if aborted.
bool Wait(uint32_t ms) {
  const uint32_t start = millis();
  while (millis() - start < ms) {
    if (ShouldAbort()) return false;
    delay(10);
  }
  return true;
}

bool Countdown() {
  Serial.printf("[DT] Driving in %lu s. Any key = abort. Keep the USB cable "
                "slack.\n",
                static_cast<unsigned long>(kCountdownMs / 1000));
  return Wait(kCountdownMs);
}

// Prints `prompt`, then waits (no time limit) for Enter. Typing anything
// else before Enter cancels. Returns true only on a bare Enter. Motors are
// stopped while waiting.
bool WaitForEnter(const char* prompt) {
  Stop();
  Serial.printf("[DT] >>> %s\n", prompt);
  Serial.print("[DT] >>> Press Enter to START (type anything + Enter to "
               "cancel): ");
  bool typed = false;
  while (true) {
    if (RedRobot::IsRemoteControlled()) {
      Serial.println("\n[DT] Cancelled: remote control took over.");
      return false;
    }
    while (Serial.available() > 0) {
      const int c = Serial.read();
      if (c == '\r' || c == '\n') {
        Serial.println();
        delay(20);  // Swallow the other half of CRLF.
        while (Serial.available() > 0) Serial.read();
        if (typed) Serial.println("[DT] Cancelled.");
        return !typed;
      }
      typed = true;
      Serial.print(static_cast<char>(c));
    }
    delay(10);
  }
}

// Drives for `ms` and stops. Returns false if aborted.
bool DriveFor(int left, int right, uint32_t ms) {
  SetDrive(left, right);
  const bool ok = Wait(ms);
  Stop();
  return ok;
}

const char* ResetReasonName(esp_reset_reason_t reason) {
  switch (reason) {
    case ESP_RST_POWERON:
      return "POWER_ON";
    case ESP_RST_SW:
      return "SOFTWARE";
    case ESP_RST_PANIC:
      return "CRASH (panic)";
    case ESP_RST_INT_WDT:
    case ESP_RST_TASK_WDT:
    case ESP_RST_WDT:
      return "WATCHDOG";
    case ESP_RST_BROWNOUT:
      return "BROWNOUT (battery voltage dipped)";
    case ESP_RST_USB:
      return "USB (upload / reset)";
    default:
      return "OTHER";
  }
}

// =============================== Tests =====================================

void RunDeadband(int step, int hold_ms) {
  if (step <= 0) step = 5;
  if (hold_ms <= 0) hold_ms = 700;
  Serial.printf("[DT] Deadband: each wheel alone, 0 -> 100%% in %d%% steps, "
                "%d ms each. Note where it FIRST turns steadily.\n",
                step, hold_ms);
  if (!Countdown()) return;
  for (int wheel = 0; wheel < 2; ++wheel) {
    const char* name = wheel == 0 ? "LEFT" : "RIGHT";
    for (int speed = step; speed <= 100; speed += step) {
      Serial.printf("[DT]   %s wheel %3d%%\n", name, speed);
      SetDrive(wheel == 0 ? speed : 0, wheel == 1 ? speed : 0);
      if (!Wait(hold_ms)) return;
    }
    Stop();
    if (!Wait(800)) return;
  }
  Serial.println("[DT] Done. Set kMinSpeed (line_follower.cpp) a bit above "
                 "the higher of the two starting speeds.");
}

void RunStraight(int speed, int ms) {
  if (speed == 0) speed = 50;
  if (ms <= 0) ms = 3000;
  Serial.printf("[DT] Straight: both wheels %d%% for %d ms (trim L=%d%% "
                "R=%d%%).\n",
                speed, ms, g_left_trim, g_right_trim);
  if (!Countdown()) return;
  if (!DriveFor(speed, speed, ms)) return;
  Serial.println("[DT] Done. Measure distance and sideways drift.");
  Serial.println("[DT]   Curved RIGHT -> left wheel is faster: lower left "
                 "trim, e.g. 'trim 95 100'.");
  Serial.println("[DT]   Curved LEFT  -> right wheel is faster: e.g. "
                 "'trim 100 95'. Then run straight again.");
}

void RunSweep(int ms, int) {
  if (ms <= 0) ms = 1000;
  constexpr int kSpeeds[] = {30, 50, 70, 100};
  Serial.printf("[DT] Speed sweep: forward at 30/50/70/100%% for %d ms each. "
                "It waits for Enter before each speed.\n",
                ms);
  bool first = true;
  for (const int speed : kSpeeds) {
    if (!first) {
      char prompt[96];
      snprintf(prompt, sizeof(prompt),
               "Put the robot back on the START mark, facing the same way, "
               "for %d%%.",
               speed);
      if (!WaitForEnter(prompt)) return;
    }
    first = false;
    if (!Countdown()) return;
    Serial.printf("[DT]   %3d%% for %d ms\n", speed, ms);
    if (!DriveFor(speed, speed, ms)) return;
    Serial.printf("[DT]   Measure distance D (cm) from the start mark: "
                  "speed = D / %.1f s.\n",
                  ms / 1000.0f);
  }
  Serial.println("[DT] Done.");
}

void RunSpin(int speed, int ms) {
  if (speed == 0) speed = 35;  // Matches kCalibrationSpeed in line_follower.
  if (ms <= 0) ms = 2000;
  Serial.printf("[DT] Spin: pivot RIGHT (L=+%d R=-%d) for %d ms.\n", speed,
                speed, ms);
  if (!Countdown()) return;
  if (!DriveFor(speed, -speed, ms)) return;
  Serial.printf("[DT] Done. If it turned D degrees, one full turn takes "
                "about %d*360/D ms at %d%%.\n",
                ms, speed);
  Serial.println("[DT]   At 35% that is kFullRotationMs in line_follower.cpp.");
}

void RunStress(int cycles, int speed) {
  if (cycles <= 0) cycles = 10;
  if (speed == 0) speed = 100;
  Serial.printf("[DT] Stress: %d x (forward %d%% 300 ms, backward 300 ms). "
                "Instant reversals draw peak current.\n",
                cycles, speed);
  if (!Countdown()) return;
  g_stress_marker = kStressMarker;
  bool finished = true;
  for (int c = 1; c <= cycles && finished; ++c) {
    g_stress_cycle = c;
    Serial.printf("[DT]   cycle %d/%d\n", c, cycles);
    SetDrive(speed, speed);
    finished = Wait(300);
    if (finished) {
      SetDrive(-speed, -speed);
      finished = Wait(300);
    }
  }
  Stop();
  g_stress_marker = 0;
  if (finished) {
    Serial.printf("[DT] Done: %d reversals with no reset. If the robot had "
                  "rebooted, this line would not print.\n",
                  cycles);
  }
}

void RunEndurance(int seconds, int speed) {
  if (seconds <= 0) seconds = 60;
  if (speed == 0) speed = 70;
  Serial.printf("[DT] Endurance: forward/back 1 s each at %d%% for %d s. "
                "Watch the battery display; feel motors after.\n",
                speed, seconds);
  if (!Countdown()) return;
  const uint32_t start = millis();
  uint32_t next_report = 10000;
  bool forward = true;
  while (millis() - start < static_cast<uint32_t>(seconds) * 1000) {
    SetDrive(forward ? speed : -speed, forward ? speed : -speed);
    forward = !forward;
    if (!Wait(1000)) return;
    if (millis() - start >= next_report) {
      Serial.printf("[DT]   %lu s\n",
                    static_cast<unsigned long>((millis() - start) / 1000));
      next_report += 10000;
    }
  }
  Stop();
  Serial.println("[DT] Done. Motors hot or slowing down = too much load.");
}

void RunCalibrate(int, int) { line_follower::CalibrateNow(); }

void RunFollow(int seconds, int) {
  if (seconds <= 0) seconds = 10;
  Serial.printf("[DT] Follow: line follower for %d s. Put the robot on the "
                "line.\n",
                seconds);
  if (!Countdown()) return;
  line_follower::FollowFor(static_cast<uint32_t>(seconds) * 1000,
                           &ShouldAbort);
}

// ============================== Menu =======================================

struct DriveTest {
  const char* name;
  const char* args;
  const char* description;  // One line for the menu.
  const char* place;        // Where to put the robot before starting.
  const char* does;         // What the robot will do.
  void (*run)(int, int);
};

constexpr DriveTest kTests[] = {
    {"deadband", "[step% hold_ms]", "Ramp each wheel 0->100%: where it starts",
     "WHEELS OFF THE GROUND (robot on a box), so you can watch each wheel.",
     "Left wheel alone ramps 5%, 10% ... 100% (0.7 s each), then the right "
     "wheel. Note the % where each wheel first turns steadily.",
     RunDeadband},
    {"straight", "[speed ms]", "Both wheels forward (50%, 3 s): check drift",
     "On the FLOOR with ~1.5 m clear ahead. Mark the start with tape and "
     "line the robot up with a straight edge (floor tile line).",
     "Drives straight forward at 50% for 3 s, then stops. Measure distance "
     "and how far it drifted left/right.",
     RunStraight},
    {"sweep", "[ms]", "Forward 30/50/70/100%, 1 s each: speed",
     "On the FLOOR with ~1.5 m clear ahead. Mark a START line with tape.",
     "Drives forward 1 s at 30%, then 50%, 70%, 100%. After each run it "
     "stops and waits for Enter so you can measure and reposition.",
     RunSweep},
    {"spin", "[speed ms]", "Pivot right (35%, 2 s): count degrees",
     "On the FLOOR, ~40 cm clear all around. Note which way it faces (line "
     "it up with a tape mark or tile line).",
     "Spins in place to the RIGHT at 35% for 2 s, then stops. Count how "
     "many degrees it turned.",
     RunSpin},
    {"stress", "[cycles speed]", "Fast forward/back flips: brownout check",
     "WHEELS OFF THE GROUND is safest (on the floor it lurches ~10 cm each "
     "way).",
     "Flips full forward / full backward every 0.3 s, 10 times. If the "
     "battery sags too far the board reboots, and it tells you after.",
     RunStress},
    {"endurance", "[seconds speed]", "Back and forth (60 s, 70%): heat/battery",
     "WHEELS OFF THE GROUND, or on the floor with ~1 m clear front and back.",
     "Drives forward 1 s, backward 1 s at 70% for 60 s, printing every 10 s. "
     "Afterwards, feel the motors and check the battery display.",
     RunEndurance},
    {"calibrate", "", "Line sensor calibration (hand: slide over tape)",
     "Robot ON THE FLOOR next to the black tape line. Motors stay OFF.",
     "For 10 s after you press Enter, SLIDE the robot by hand back and forth "
     "across the tape so every sensor passes over it. Then it prints each "
     "sensor's floor/tape range.",
     RunCalibrate},
    {"follow", "[seconds]", "Line follow (10 s), prints PID status",
     "PLACE THE ROBOT ON THE LINE: sensor bar centered over the tape, robot "
     "facing along the line. Run 'calibrate' first.",
     "Follows the line for 10 s at base speed 45% using PID, printing a "
     "status line every 0.1 s. Any key stops it.",
     RunFollow},
};
constexpr int kNumTests = sizeof(kTests) / sizeof(kTests[0]);

void PrintMenu() {
  Serial.println();
  Serial.println("[DT] ===== Drive train test console =====");
  Serial.println("[DT] Type a number or name + Enter. Optional args after "
                 "it, e.g. \"2 60 3000\".");
  Serial.println("[DT] Any key during a test = ABORT (motors stop).");
  for (int i = 0; i < kNumTests; ++i) {
    Serial.printf("[DT] %2d  %-10s %-16s %s\n", i + 1, kTests[i].name,
                  kTests[i].args, kTests[i].description);
  }
  Serial.printf("[DT] Other: trim L R (now %d %d), stop, menu\n",
                g_left_trim, g_right_trim);
}

void PrintPrompt() { Serial.print("> "); }

void PrintBootInfo() {
  const esp_reset_reason_t reason = esp_reset_reason();
  Serial.printf("[DT] Last reset: %s\n", ResetReasonName(reason));
  if (g_stress_marker == kStressMarker) {
    Serial.printf("[DT] !!! The stress test was interrupted by a reset at "
                  "cycle %lu (reason: %s). Likely a brownout: the motors "
                  "pulled the battery voltage too low.\n",
                  static_cast<unsigned long>(g_stress_cycle),
                  ResetReasonName(reason));
    g_stress_marker = 0;
  }
}

void Execute(char* line) {
  char* save = nullptr;
  const char* cmd = strtok_r(line, " \t", &save);
  if (cmd == nullptr) return;
  const char* a1 = strtok_r(nullptr, " \t", &save);
  const char* a2 = strtok_r(nullptr, " \t", &save);
  const int arg1 = a1 ? atoi(a1) : 0;
  const int arg2 = a2 ? atoi(a2) : 0;

  if (!strcmp(cmd, "menu") || !strcmp(cmd, "m") || !strcmp(cmd, "?") ||
      !strcmp(cmd, "help")) {
    PrintMenu();
    return;
  }
  if (!strcmp(cmd, "stop") || !strcmp(cmd, "x")) {
    Stop();
    Serial.println("[DT] Motors stopped.");
    return;
  }
  if (!strcmp(cmd, "trim")) {
    if (a1 == nullptr || a2 == nullptr) {
      Serial.printf("[DT] trim is L=%d%% R=%d%%. Usage: trim 95 100\n",
                    g_left_trim, g_right_trim);
      return;
    }
    g_left_trim = constrain(arg1, 50, 100);
    g_right_trim = constrain(arg2, 50, 100);
    Serial.printf("[DT] trim set: L=%d%% R=%d%% (until reset).\n",
                  g_left_trim, g_right_trim);
    return;
  }

  int index = -1;
  const int number = atoi(cmd);
  if (number >= 1 && number <= kNumTests) {
    index = number - 1;
  } else {
    for (int i = 0; i < kNumTests; ++i) {
      if (!strcmp(cmd, kTests[i].name)) index = i;
    }
  }
  if (index < 0) {
    Serial.printf("[DT] Unknown command '%s'. Type 'menu'.\n", cmd);
    return;
  }

  const DriveTest& test = kTests[index];
  Serial.printf("[DT] ----- %d %s -----\n", index + 1, test.name);
  Serial.printf("[DT] PLACE THE ROBOT: %s\n", test.place);
  Serial.printf("[DT] WHAT IT DOES:    %s\n", test.does);
  if (a1 != nullptr) {
    Serial.printf("[DT] (custom args: %s %s)\n", a1, a2 != nullptr ? a2 : "");
  }
  // Drop the rest of the Enter key (CR/LF) so it isn't taken as "start".
  delay(20);
  while (Serial.available() > 0) Serial.read();
  if (!WaitForEnter("Place the robot as above.")) return;
  test.run(arg1, arg2);
  Stop();
  Serial.printf("[DT] ----- end of %s -----\n", test.name);
}

}  // namespace

void Update() {
  const bool connected = static_cast<bool>(Serial);
  if (connected && !g_was_connected) {
    PrintBootInfo();
    PrintMenu();
    PrintPrompt();
  }
  g_was_connected = connected;
  if (!connected) return;

  while (Serial.available() > 0) {
    const int c = Serial.read();
    if (c == '\r' || c == '\n') {
      if (g_line_len == 0) continue;
      Serial.println();
      g_line[g_line_len] = '\0';
      g_line_len = 0;
      Execute(g_line);
      PrintPrompt();
    } else if ((c == 8 || c == 127) && g_line_len > 0) {  // Backspace.
      --g_line_len;
      Serial.print("\b \b");
    } else if (c >= 32 && c < 127 && g_line_len < sizeof(g_line) - 1) {
      g_line[g_line_len++] = static_cast<char>(c);
      Serial.print(static_cast<char>(c));  // Echo what you type.
    }
  }
  delay(5);
}

}  // namespace drive_test
