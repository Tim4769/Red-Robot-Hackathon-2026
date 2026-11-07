#include <RedRobot.h>

#include "line_follower.h"

// Club controller (RR_Controller) layout:
//   Left stick   one-stick layout (the default): all driving
//   Right stick  up/down = claw up/down (split layout: left/right = turn)
//                (kDriveLayout = kSplitStick: left Y drives, right X turns)
//   X  line run + turn (test 9)      Y  delayed line run
//      Either one: switches to driver control, runs its routine, then
//      the sticks take over. X or Y again = cancel (-> driver now).
//      Press X twice for driver control without a routine.
//   A  rotate claw (toggle)          B  open / close claw (toggle)
//   D-pad  nothing (driving removed)

// Drive wiring (matches src/line_follower.cpp and the drive test):
// motor port 0 = left wheel, wired backward so it is inverted here;
// motor port 2 = right wheel. Speeds are -100..100, + = robot forward.
namespace {
constexpr uint8_t kLeftMotor = 0;
constexpr uint8_t kRightMotor = 2;
constexpr bool kInvertLeft = true;
constexpr bool kInvertRight = false;

// Drive layout:
//   kOneStick:   left stick does both: up/down = forward/back,
//                left/right = turn (the original arcade drive).
//   kSplitStick: LEFT stick up/down = forward/back, RIGHT stick left/right =
//                turn. (Right stick up/down is still the claw lift.)
enum class DriveLayout { kOneStick, kSplitStick };
constexpr DriveLayout kDriveLayout = DriveLayout::kOneStick;

// Stick sensitivity, in percent of the stick value. 100 = original.
constexpr int kJoystickDrivePercent = 80;  // Forward/back.
constexpr int kJoystickTurnPercent = 40;   // Turning. Was 50.

// =============================== Routines ==================================
// Drive stick is ignored while a routine runs; the claw still works.
//
// X: same as console test 9 (keep in sync with src/drive_test.cpp):
// forward, follow the line, and when it ends: back up, turn right ~120 deg,
// forward, stop.
constexpr int kXStartForwardSpeed = 40;
constexpr uint32_t kXStartForwardMs = 1000;
constexpr uint32_t kXFollowTimeoutMs = 20000;  // Stop if it never ends.
constexpr int kXBackSpeed = 40;
constexpr uint32_t kXBackMs = 500;
constexpr int kXTurnSpeed = 35;
constexpr uint32_t kXTurnMs = 890;             // ~120 deg (timed).
constexpr int kXForwardSpeed = 40;
constexpr uint32_t kXForwardMs = 1000;
//
// Y: wait kYStartDelayMs, drive straight kYForwardMs, then follow the line
// and STOP when it ends.
constexpr uint32_t kYStartDelayMs = 5000;
constexpr int kYForwardSpeed = 40;
constexpr uint32_t kYForwardMs = 1000;
constexpr uint32_t kYFollowTimeoutMs = 15000;  // Stop if the line never ends.

// ============================ Claw mechanism ===============================
// Servo number <-> board connector (from the club's board netlist):
//   servo 0 = J4, servo 1 = J5, servo 2 = J6, servo 3 = J7.
// Wired: up/down on J7 (servo 3), claw on J5 (servo 1), rotation on J6
// (servo 2). All confirmed by the team.

// Right stick Y moves the claw up/down (X is ignored).
//   kLiftUsesMotor = true:  DC motor kLiftMotor runs at the stick's speed,
//                           stick released = stop. (Port 3 is the only free
//                           DC motor port: 0/2 drive, 1 is disabled.)
//   kLiftUsesMotor = false: servo kLiftServo moves SMOOTHLY while the stick
//                           is pushed (a 20 ms loop; speed ~ how far it's
//                           pushed, up to kLiftDegPerSec) and stays put.
constexpr bool kLiftUsesMotor = false;  // Up/down is a servo (J7).
constexpr uint8_t kLiftMotor = 3;
constexpr bool kInvertLift = false;    // Flip if stick-up moves it down.
constexpr int kLiftPercent = 60;       // Max lift speed, % of the stick.
constexpr int kStickDeadzone = 15;     // Ignore small stick wobble.
constexpr uint8_t kLiftServo = 3;      // J7.
constexpr float kLiftDegPerSec = 150.0f;  // Servo speed at full stick.
constexpr uint32_t kLiftUpdateMs = 20;    // Servo update period.
constexpr int kLiftServoMin = 0;
constexpr int kLiftServoMax = 180;

// B toggles the claw between open and closed.
constexpr uint8_t kClawServo = 1;      // J5.
constexpr int kClawOpenAngle = 90;
constexpr int kClawClosedAngle = 20;

// A toggles the claw rotation between two angles.
constexpr uint8_t kRotateServo = 2;    // J6.
constexpr int kRotateAngleA = 0;
constexpr int kRotateAngleB = 90;

bool g_claw_closed = false;   // Assume it starts open.
bool g_rotated = false;       // Assume it starts at kRotateAngleA.
int g_lift_servo_angle = 90;  // Servo mode: current lift angle.
volatile int g_lift_stick = 0;  // Latest right-stick Y (deadzoned, signed).
int g_drive_stick = 0;  // Split layout: latest left-stick Y (forward/back).
int g_turn_stick = 0;   // Split layout: latest right-stick X (turn).
bool g_lift_task_started = false;
enum class Routine { kLineEndTurn, kDelayedLineRun };  // X, Y.
volatile bool g_routine_running = false;  // A routine is active.
volatile bool g_routine_cancel = false;   // X/Y pressed again: stop it.
Routine g_routine = Routine::kLineEndTurn;

void DriveLeft(int speed) {
  speed = constrain(speed, -100, 100);
  RedRobot::WriteDCMotor(kLeftMotor, kInvertLeft ? -speed : speed);
}

void DriveRight(int speed) {
  speed = constrain(speed, -100, 100);
  RedRobot::WriteDCMotor(kRightMotor, kInvertRight ? -speed : speed);
}

// Servo-mode lift: every kLiftUpdateMs, move the servo at a speed
// proportional to the latest stick value, so motion is smooth no matter
// how irregularly the controller sends stick updates.
void LiftTask(void*) {
  float angle = g_lift_servo_angle;
  int written = -1;
  const float dt = kLiftUpdateMs / 1000.0f;
  while (true) {
    const int stick = g_lift_stick;
    if (stick != 0) {
      angle += (stick / 100.0f) * kLiftDegPerSec * dt;
      angle = constrain(angle, static_cast<float>(kLiftServoMin),
                        static_cast<float>(kLiftServoMax));
    }
    const int rounded = static_cast<int>(angle + 0.5f);
    if (stick != 0 && rounded != written) {
      RedRobot::SetServo(kLiftServo, rounded);
      written = rounded;
      g_lift_servo_angle = rounded;
    }
    vTaskDelay(pdMS_TO_TICKS(kLiftUpdateMs));
  }
}

bool RoutineCancelled() { return g_routine_cancel; }

// Waits `ms` in 10 ms steps. Returns false if the routine was cancelled.
bool RoutineWait(uint32_t ms) {
  const uint32_t start = millis();
  while (millis() - start < ms) {
    if (g_routine_cancel) return false;
    vTaskDelay(pdMS_TO_TICKS(10));
  }
  return true;
}

// Drives both wheels for `ms` (or until cancelled), then stops them.
// Returns false if cancelled.
bool RoutineDrive(int left, int right, uint32_t ms) {
  DriveLeft(left);
  DriveRight(right);
  const bool ok = RoutineWait(ms);
  DriveLeft(0);
  DriveRight(0);
  return ok;
}

// X: forward, follow, and at the line end: back, turn right, forward.
void RunLineEndTurn() {
  Serial.printf("[RC] X: forward %lu ms\n",
                static_cast<unsigned long>(kXStartForwardMs));
  if (!RoutineDrive(kXStartForwardSpeed, kXStartForwardSpeed,
                    kXStartForwardMs)) {
    return;
  }
  Serial.println("[RC] X: following the line");
  if (!line_follower::FollowUntilLineEnds(0, kXFollowTimeoutMs,
                                          &RoutineCancelled)) {
    return;  // Timed out or cancelled; it printed why.
  }
  Serial.printf("[RC] X: back %lu ms\n", static_cast<unsigned long>(kXBackMs));
  if (!RoutineDrive(-kXBackSpeed, -kXBackSpeed, kXBackMs)) return;
  Serial.printf("[RC] X: turn right %lu ms (~120 deg)\n",
                static_cast<unsigned long>(kXTurnMs));
  if (!RoutineDrive(kXTurnSpeed, -kXTurnSpeed, kXTurnMs)) return;
  Serial.printf("[RC] X: forward %lu ms\n",
                static_cast<unsigned long>(kXForwardMs));
  RoutineDrive(kXForwardSpeed, kXForwardSpeed, kXForwardMs);
}

// Y: wait, forward, follow, stop at the line end.
void RunDelayedLineRun() {
  Serial.printf("[RC] Y: waiting %lu ms\n",
                static_cast<unsigned long>(kYStartDelayMs));
  if (!RoutineWait(kYStartDelayMs)) return;
  Serial.printf("[RC] Y: forward %lu ms\n",
                static_cast<unsigned long>(kYForwardMs));
  if (!RoutineDrive(kYForwardSpeed, kYForwardSpeed, kYForwardMs)) return;
  Serial.println("[RC] Y: following the line");
  // Prints LINE ENDED / did NOT end, and stops the motors itself.
  line_follower::FollowUntilLineEnds(0, kYFollowTimeoutMs, &RoutineCancelled);
}

// Runs in its own task so the Bluetooth callbacks stay responsive. Driver
// control is already on (set when the routine started), so the sticks take
// over as soon as g_routine_running goes false.
void RoutineTask(void*) {
  const char* name = g_routine == Routine::kLineEndTurn ? "X" : "Y";
  if (g_routine == Routine::kLineEndTurn) {
    RunLineEndTurn();
  } else {
    RunDelayedLineRun();
  }
  DriveLeft(0);
  DriveRight(0);
  Serial.printf("[RC] %s routine %s. Driver control: sticks active.\n", name,
                g_routine_cancel ? "CANCELLED" : "finished");
  g_routine_cancel = false;
  g_routine_running = false;
  vTaskDelete(nullptr);
}

// X/Y pressed: cancel a running routine, or switch to driver control and
// start `routine`.
void StartOrCancelRoutine(Routine routine, const char* button) {
  if (g_routine_running) {
    g_routine_cancel = true;
    Serial.printf("[RC] %s: cancelling the routine\n", button);
    return;
  }
  if (!RedRobot::IsRemoteControlled()) {
    RedRobot::StartRemoteControl();  // Stops the autonomous loop's driving.
  }
  DriveLeft(0);
  DriveRight(0);
  g_routine = routine;
  g_routine_cancel = false;
  g_routine_running = true;
  Serial.printf("[RC] %s: routine started (press X or Y again to cancel)\n",
                button);
  if (xTaskCreate(RoutineTask, "routine", 8192, nullptr, 1, nullptr) !=
      pdPASS) {
    g_routine_running = false;
    Serial.printf("[RC] %s: could not start the routine task\n", button);
  }
}
}  // namespace

namespace Bluefruit
{
  /* DPad_Pressed is called everytime a D-Pad button is pressed or released.
   *
   * Only activates after RedRobot::StartRemoteControl() is called.
   * Button is one of {UP = 0, DOWN = 1, LEFT = 2, RIGHT = 3}
   */
  void DPad_Pressed(button_t Button, bool buttonPressedDown)
  {
    if (!RedRobot::IsRemoteControlled())
      return;

    // values stay the same between calls to DPad_Pressed
    static bool button_pressed[4] = {false, false, false, false};

    button_pressed[Button] = (buttonPressedDown == true);

    // TODO: edit your speed output to the yellow motors below!
    // stop = 0, slow = 50, fast = 100
    // range [-100,100]

    int left_speed = 0;
    int right_speed = 0;

    if (button_pressed[UP])
    { // Forward
      left_speed += 100;
      right_speed += 100;
    }
    if (button_pressed[DOWN])
    { // Backward
      left_speed -= 100;
      right_speed -= 100;
    }
    if (button_pressed[LEFT])
    { // Turn left_speed
      left_speed -= 50;
      right_speed += 50;
    }
    if (button_pressed[RIGHT])
    { // Turn right_speed
      left_speed += 50;
      right_speed -= 50;
    }

    // Clamp and write speeds
    DriveLeft(constrain(left_speed, -100, 100));
    DriveRight(constrain(right_speed, -100, 100));
  }

  void Button1_Pressed(bool buttonPressedDown)
  {
    if (!RedRobot::IsRemoteControlled() && buttonPressedDown)
    {
      RedRobot::StartRemoteControl();

      // start remote controlled with the DC motors off
      DriveLeft(0);
      DriveRight(0);
    }
  }

  void Button2_Pressed(bool buttonPressedDown)
  {
    // Ex: move an arm when the button is pressed
    if (buttonPressedDown)
    {
      RedRobot::SetServo(0, 180);
      RedRobot::SetServo(1, 180);
      RedRobot::SetServo(2, 180);
      RedRobot::SetServo(3, 180);
    }
    else
    {
      RedRobot::SetServo(0, 0);
      RedRobot::SetServo(1, 0);
      RedRobot::SetServo(2, 0);
      RedRobot::SetServo(3, 0);
    }
  }

  void Button3_Pressed(bool buttonPressedDown)
  {
  }

  void Button4_Pressed(bool buttonPressedDown)
  {
  }
}

namespace RR_Controller
{
  // Mixes forward/back and turn stick values into wheel speeds.
  void MixDrive(int drive_stick, int turn_stick)
  {
    if (g_routine_running) return;  // A routine is driving.
    const int drive = drive_stick * kJoystickDrivePercent / 100;
    const int turn = turn_stick * kJoystickTurnPercent / 100;
    int leftMotor = drive + turn;
    int rightMotor = drive - turn;

    // Normalize to [-100, 100]
    int maxMagnitude = max(abs(leftMotor), abs(rightMotor));

    if (maxMagnitude > 100)
    {
      leftMotor = leftMotor * 75 / maxMagnitude;
      rightMotor = rightMotor * 75 / maxMagnitude;
    }
    DriveLeft(leftMotor);
    DriveRight(rightMotor);
  }

  // Left stick: one-stick layout = both axes; split layout = Y only.
  void Joystick1(int8_t x, int8_t y)
  {
    if (kDriveLayout == DriveLayout::kOneStick)
    {
      MixDrive(y, x);
      return;
    }
    g_drive_stick = abs(y) < kStickDeadzone ? 0 : y;
    MixDrive(g_drive_stick, g_turn_stick);
  }

  // Right stick: Y = claw up/down. In the split layout, X = turn.
  void Joystick2(int8_t x, int8_t y)
  {
    if (kDriveLayout == DriveLayout::kSplitStick)
    {
      const int turn = abs(x) < kStickDeadzone ? 0 : x;
      if (turn != g_turn_stick)
      {
        g_turn_stick = turn;
        MixDrive(g_drive_stick, g_turn_stick);
      }
    }
    const int stick = abs(y) < kStickDeadzone ? 0 : y;
    if (kLiftUsesMotor)
    {
      int speed = stick * kLiftPercent / 100;
      if (kInvertLift) speed = -speed;
      RedRobot::WriteDCMotor(kLiftMotor, constrain(speed, -100, 100));
    }
    else
    {
      // Just record the stick; LiftTask moves the servo smoothly.
      g_lift_stick = kInvertLift ? -stick : stick;
      if (!g_lift_task_started)
      {
        g_lift_task_started = true;
        xTaskCreate(LiftTask, "liftServo", 3072, nullptr, 1, nullptr);
      }
    }
  }

  // D-pad: no driving (removed at the team's request). Free for later use.
  void DPad_Pressed(button_t Button, bool buttonPressedDown)
  {
    (void)Button;
    (void)buttonPressedDown;
  }

  // X: line run + turn (console test 9), or cancel a running routine.
  void ButtonX_Pressed(bool buttonPressedDown)
  {
    if (buttonPressedDown) StartOrCancelRoutine(Routine::kLineEndTurn, "X");
  }

  // Y: delayed line run, or cancel a running routine.
  void ButtonY_Pressed(bool buttonPressedDown)
  {
    if (buttonPressedDown)
      StartOrCancelRoutine(Routine::kDelayedLineRun, "Y");
  }

  // B: open / close the claw.
  void ButtonB_Pressed(bool buttonPressedDown)
  {
    if (!buttonPressedDown) return;
    g_claw_closed = !g_claw_closed;
    const int angle = g_claw_closed ? kClawClosedAngle : kClawOpenAngle;
    RedRobot::SetServo(kClawServo, angle);
    Serial.printf("[RC] B: claw %s (servo %u -> %d deg)\n",
                  g_claw_closed ? "CLOSED" : "OPEN", kClawServo, angle);
  }

  // A: rotate the claw between two angles.
  void ButtonA_Pressed(bool buttonPressedDown)
  {
    if (!buttonPressedDown) return;
    g_rotated = !g_rotated;
    const int angle = g_rotated ? kRotateAngleB : kRotateAngleA;
    RedRobot::SetServo(kRotateServo, angle);
    Serial.printf("[RC] A: claw rotated to %d deg (servo %u)\n", angle,
                  kRotateServo);
  }
}
