#include <RedRobot.h>

#include "line_follower.h"

// Club controller (RR_Controller) layout:
//   Left stick   drive (arcade)      Right stick  claw up/down (Y only)
//   X  start driver control          Y  delayed line run (press again
//                                       to cancel), see "Y routine"
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

// Left stick sensitivity (arcade drive), in percent of the stick value.
// 100 = original. Lower = gentler.
constexpr int kJoystickDrivePercent = 80;  // Forward/back (stick Y).
constexpr int kJoystickTurnPercent = 40;   // Left/right (stick X). Was 50.

// ============================== Y routine ==================================
// Y (after X): wait kYStartDelayMs, drive straight kYForwardMs, then follow
// the line and STOP when it ends (like console test 9 without the
// back/turn/forward). Press Y again to cancel. Drive stick and D-pad are
// ignored while it runs; the claw still works.
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
bool g_lift_task_started = false;
volatile bool g_routine_running = false;  // Y routine active.
volatile bool g_routine_cancel = false;   // Y pressed again: stop it.

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

// Runs in its own task so the Bluetooth callbacks stay responsive.
void YRoutineTask(void*) {
  Serial.printf("[RC] Y routine: waiting %lu ms (press Y again to "
                "cancel)\n",
                static_cast<unsigned long>(kYStartDelayMs));
  bool ok = RoutineWait(kYStartDelayMs);
  if (ok) {
    Serial.printf("[RC] Y routine: forward %d%% for %lu ms\n",
                  kYForwardSpeed, static_cast<unsigned long>(kYForwardMs));
    DriveLeft(kYForwardSpeed);
    DriveRight(kYForwardSpeed);
    ok = RoutineWait(kYForwardMs);
    DriveLeft(0);
    DriveRight(0);
  }
  if (ok) {
    Serial.println("[RC] Y routine: following the line");
    // Prints LINE ENDED / did NOT end, and stops the motors itself.
    line_follower::FollowUntilLineEnds(0, kYFollowTimeoutMs,
                                       &RoutineCancelled);
  }
  DriveLeft(0);
  DriveRight(0);
  Serial.println(g_routine_cancel
                     ? "[RC] Y routine: CANCELLED, motors stopped"
                     : "[RC] Y routine: finished, motors stopped");
  g_routine_cancel = false;
  g_routine_running = false;
  vTaskDelete(nullptr);
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
  // Left stick: arcade drive.
  void Joystick1(int8_t x, int8_t y)
  {
    if (g_routine_running) return;  // Y routine is driving.
    const int drive = y * kJoystickDrivePercent / 100;
    const int turn = x * kJoystickTurnPercent / 100;  // Less twitchy turns.
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

  // Right stick: claw up/down. Only Y is used.
  void Joystick2(int8_t x, int8_t y)
  {
    (void)x;
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

  void ButtonX_Pressed(bool buttonPressedDown)
  {
    if (!RedRobot::IsRemoteControlled() && buttonPressedDown)
    {
      RedRobot::StartRemoteControl();

      // start remote controlled with the DC motors off
      DriveLeft(0);
      DriveRight(0);
    }
  }

  // Y: start the delayed line run, or cancel it if it's running.
  void ButtonY_Pressed(bool buttonPressedDown)
  {
    if (!buttonPressedDown) return;
    if (g_routine_running)
    {
      g_routine_cancel = true;
      Serial.println("[RC] Y: cancelling the routine");
      return;
    }
    if (!RedRobot::IsRemoteControlled())
    {
      Serial.println("[RC] Y: press X first (driver control)");
      return;
    }
    g_routine_cancel = false;
    g_routine_running = true;
    if (xTaskCreate(YRoutineTask, "yRoutine", 8192, nullptr, 1, nullptr) !=
        pdPASS)
    {
      g_routine_running = false;
      Serial.println("[RC] Y: could not start the routine task");
    }
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
