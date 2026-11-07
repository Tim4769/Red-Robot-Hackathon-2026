#include <RedRobot.h>

// Club controller (RR_Controller) layout:
//   Left stick   drive (arcade)      Right stick  claw up/down (Y only)
//   X  start driver control          Y  not assigned yet ("delay": TBD)
//   A  rotate claw (toggle)          B  open / close claw (toggle)

// Drive wiring (matches src/line_follower.cpp and the drive test):
// motor port 0 = left wheel, wired backward so it is inverted here;
// motor port 2 = right wheel. Speeds are -100..100, + = robot forward.
namespace {
constexpr uint8_t kLeftMotor = 0;
constexpr uint8_t kRightMotor = 2;
constexpr bool kInvertLeft = true;
constexpr bool kInvertRight = false;

// Joystick turning sensitivity (arcade drive, left stick X), in percent of
// the stick's left/right value. 100 = original (very twitchy), 50 = full
// sideways stick turns half as hard. Forward/back (Y) is unchanged.
constexpr int kJoystickTurnPercent = 50;

// ============================ Claw mechanism ===============================
// CHECK THESE against how the claw is wired, then tune the angles.

// Right stick Y moves the claw up/down (X is ignored).
//   kLiftUsesMotor = true:  DC motor kLiftMotor runs at the stick's speed,
//                           stick released = stop. (Port 3 is the only free
//                           DC motor port: 0/2 drive, 1 is disabled.)
//   kLiftUsesMotor = false: servo kLiftServo moves kLiftServoStep degrees per
//                           stick update while pushed, and stays put.
constexpr bool kLiftUsesMotor = true;
constexpr uint8_t kLiftMotor = 3;
constexpr bool kInvertLift = false;    // Flip if stick-up moves it down.
constexpr int kLiftPercent = 60;       // Max lift speed, % of the stick.
constexpr int kStickDeadzone = 15;     // Ignore small stick wobble.
constexpr uint8_t kLiftServo = 2;
constexpr int kLiftServoStep = 3;      // Degrees per update (servo mode).
constexpr int kLiftServoMin = 0;
constexpr int kLiftServoMax = 180;

// B toggles the claw between open and closed.
constexpr uint8_t kClawServo = 0;
constexpr int kClawOpenAngle = 90;
constexpr int kClawClosedAngle = 20;

// A toggles the claw rotation between two angles.
constexpr uint8_t kRotateServo = 1;
constexpr int kRotateAngleA = 0;
constexpr int kRotateAngleB = 90;

bool g_claw_closed = false;   // Assume it starts open.
bool g_rotated = false;       // Assume it starts at kRotateAngleA.
int g_lift_servo_angle = 90;  // Servo mode: current lift angle.

void DriveLeft(int speed) {
  speed = constrain(speed, -100, 100);
  RedRobot::WriteDCMotor(kLeftMotor, kInvertLeft ? -speed : speed);
}

void DriveRight(int speed) {
  speed = constrain(speed, -100, 100);
  RedRobot::WriteDCMotor(kRightMotor, kInvertRight ? -speed : speed);
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
    const int turn = x * kJoystickTurnPercent / 100;  // Less twitchy turns.
    int leftMotor = y + turn;
    int rightMotor = y - turn;

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
    else if (stick != 0)
    {
      const int step = (stick > 0) == !kInvertLift ? kLiftServoStep
                                                   : -kLiftServoStep;
      g_lift_servo_angle =
          constrain(g_lift_servo_angle + step, kLiftServoMin, kLiftServoMax);
      RedRobot::SetServo(kLiftServo, g_lift_servo_angle);
    }
  }

  /* DPad_Pressed is called everytime a D-Pad button is pressed or released.
   *
   * Only activates after RedRobot::StartRemoteControl() is called.
   * Button is one of {UP = 0, DOWN = 1, LEFT = 2, RIGHT = 3, CENTER = 4}
   */
  void DPad_Pressed(button_t Button, bool buttonPressedDown)
  {
    // values stay the same between calls to DPad_Pressed
    static bool button_pressed[5] = {false, false, false, false, false};

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

  // Y: reserved for the "delay" action (not decided yet).
  void ButtonY_Pressed(bool buttonPressedDown)
  {
    if (buttonPressedDown)
      Serial.println("[RC] Y pressed: not assigned yet");
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
