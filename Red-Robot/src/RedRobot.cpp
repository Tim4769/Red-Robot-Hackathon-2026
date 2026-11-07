#include <Arduino.h>
#include <SPI.h>
#include <stdint.h>
#include <atomic>
#include "driver/mcpwm.h"
#include "freertos/semphr.h"

#include "RedRobot.h"
#include "pinouts.h"
#include "ble.h"

// Changed: was -180..180 deg -> 500..2500 us, so SetServo(0..180) (the
// documented range) only reached center -> one end (~100 deg of real travel)
// and 180 overdrove an HS-311 past its ~2460 us limit. Now 0..180 maps onto
// the HS-311's usable 600..2400 us, i.e. ~0..180 deg of real motion.
#define SERVO_MIN_PULSEWIDTH_US 600
#define SERVO_MAX_PULSEWIDTH_US 2400
#define SERVO_MIN_DEGREE 0
#define SERVO_MAX_DEGREE 180

#define SERVO_LEDC_MAX_DUTY 16383
#define SERVO_PERIOD_US 20000

#define MOTOR_PWM_FREQ 20000

#define VOLTAGE_BUF_SIZE 60

static std::atomic<bool> in_remote_control{false};
static SemaphoreHandle_t actuatorMutex = nullptr;
static TaskHandle_t arduinoLoopTask = nullptr;

extern void AutonomousLoop();

namespace Battery {
  float read() {
    // R26 = 100k, R27 = 33k
    // so Vin = Vout * ((R26+R27)/R27) = Vout * 4.0303
    uint32_t mv = analogReadMilliVolts(ADC1_0);
    float volts = mv / 1000.0; 
    if (volts < 6.0f) digitalWrite(LBAT_LED, HIGH);
    else digitalWrite(LBAT_LED, LOW);
    return volts * 4.0303;
  }
} // namespace Battery

namespace LineSensors {
  const int LINE_SENSOR_PINS[] = { LF0, LF1, LF2, LF3, LF4, LF5 };
  void init() {
    pinMode(LF_LEDCTRL, OUTPUT);
    digitalWrite(LF_LEDCTRL, LOW);
  }

  void read(uint16_t values[6]) {
    digitalWrite(LF_LEDCTRL, HIGH);
    vTaskDelay(pdMS_TO_TICKS(1));
    for (int i = 0; i < 6; ++i) {
      pinMode(LINE_SENSOR_PINS[i], OUTPUT);
      digitalWrite(LINE_SENSOR_PINS[i], HIGH);
      vTaskDelay(pdMS_TO_TICKS(1));
      pinMode(LINE_SENSOR_PINS[i], INPUT);
      unsigned long long start = micros();
      while (digitalRead(LINE_SENSOR_PINS[i]) && micros() - start < 10000) {}
      unsigned long long end = micros();

      uint16_t diff = end - start;

      values[i] = diff;
    }
    digitalWrite(LF_LEDCTRL, LOW);
  }
} // namespace LineSensors

namespace DCMotor {
  static const int dc_en[] = {MOTOR0_EN, MOTOR1_EN, MOTOR2_EN, MOTOR3_EN};
  static const int dc_phase[] = {MOTOR0_PHASE, MOTOR1_PHASE, MOTOR2_PHASE, MOTOR3_PHASE};

  // Each motor gets its own MCPWM timer, output A
  struct McpwmMap {
    mcpwm_unit_t unit;
    mcpwm_timer_t timer;
    mcpwm_io_signals_t signal;
  };

  static const McpwmMap mc[] = {
      {MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM0A},
      {MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM1A},
      {MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM2A},
      {MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM0A},
  };

  enum DCMOTOR {
    MOTOR_0, MOTOR_1, MOTOR_2, MOTOR_3
  };

  void init() {
    Serial.println("[DC_MOTOR] Initializing channels...");

    for (int i = 0; i < 4; i++) {
      // 1. Route the MCPWM output signal to the EN pin
      mcpwm_gpio_init(mc[i].unit, mc[i].signal, dc_en[i]);

      // 2. Configure the timer/operator
      mcpwm_config_t cfg = {};
      cfg.frequency = MOTOR_PWM_FREQ;
      cfg.cmpr_a = 0.0f; // duty in percent
      cfg.cmpr_b = 0.0f;
      cfg.counter_mode = MCPWM_UP_COUNTER;
      cfg.duty_mode = MCPWM_DUTY_MODE_0; // active-high
      mcpwm_init(mc[i].unit, mc[i].timer, &cfg);

      // 3. Configure PHASE pin
      pinMode(dc_phase[i], OUTPUT);
      digitalWrite(dc_phase[i], LOW);

    }
  }

  void write(DCMOTOR d, int16_t speed) {
    int motor = static_cast<int>(d);

    if (motor < 0 || motor >= 4 || motor == 1) {
      Serial.printf("[DC_MOTOR] Error: Invalid motor index %d\n", motor);
      return;
    }
    speed = constrain(speed, -100, 100);

    if (speed == 0) {
      mcpwm_set_duty(mc[motor].unit, mc[motor].timer, MCPWM_OPR_A, 0.0f);
      digitalWrite(dc_phase[motor], LOW);
      #ifdef REDROBOT_DEBUG  // Was unguarded: flooded serial on every stop.
        Serial.printf("[DC_MOTOR] Motor %d STOP (Duty: 0, Phase: LOW)\n", motor);
      #endif
      return;
    }

    // Set PHASE (Direction): LOW = Forward, HIGH = Reverse
    uint8_t phase_state = (speed > 0) ? LOW : HIGH;
    digitalWrite(dc_phase[motor], phase_state);

    // Set ENABLE (PWM duty): MCPWM takes percent directly, so no mapping needed
    float duty = (float)abs(speed);
    mcpwm_set_duty(mc[motor].unit, mc[motor].timer, MCPWM_OPR_A, duty);

    #ifdef REDROBOT_DEBUG
      Serial.printf(
          "[DC_MOTOR] Motor %d | Target Speed: %d | Duty: %d%% | Phase Pin: %s\n",
          motor, speed, (int)abs(speed), phase_state == LOW ? "LOW" : "HIGH");
    #endif
  }
}

namespace Servo {
  static const int servo_pins[] = {SERVO0_PWM, SERVO1_PWM, SERVO2_PWM,
                                  SERVO3_PWM};

  enum SERVO{
    SERVO_0 = 0, SERVO_1 = 1, SERVO_2 = 2, SERVO_3 = 3
  };

  void init() {
    ledcSetClockSource(LEDC_USE_XTAL_CLK);
    Serial.println("[SERVO] Initializing channels...");

    for (int i = 0; i < 4; i++) {
      // (Channel 0-3, 50 Hz, 14-bit resolution)
      ledcAttach(servo_pins[i], 50, 14);
    }
  }

  void set(SERVO s, int16_t angle) {
    angle = constrain(angle, SERVO_MIN_DEGREE, SERVO_MAX_DEGREE);

    uint32_t pulse_us = map(angle, SERVO_MIN_DEGREE, SERVO_MAX_DEGREE,
                            SERVO_MIN_PULSEWIDTH_US, SERVO_MAX_PULSEWIDTH_US);

    uint32_t duty = map(pulse_us, 0, SERVO_PERIOD_US, 0, SERVO_LEDC_MAX_DUTY);

    // Cast enum to integer to get the corresponding channel (0, 1, 2, or 3)
    int idx = static_cast<int>(s);

    // 3. Write the duty cycle to the channel (Core v2 uses channel, not pin)
    ledcWrite(servo_pins[idx], duty);   // pin, not channel
  }
} // namespace Servo

namespace Display {
  namespace {
    /**
     * @brief SPI driver for MAX7219EWG
     *
     * This should be thread safe since only this chip uses the bus
     */
    SPIClass spi(HSPI);

    /**
     * @brief SPI write
     */
    void spi_write(uint8_t address, uint8_t data) {
      // 10MHz clock speed, MSB First, SPI Mode 0
      spi.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE0));
      digitalWrite(BMS_LOAD, LOW);  // Select MAX7219
      spi.transfer(address);        // Transmit address
      spi.transfer(data);           // Transmit data
      digitalWrite(BMS_LOAD, HIGH); // Latch data into MAX7219
      spi.endTransaction();
    }

    /**
     * @brief Draw a single digit (0 to 9) at a position in [3,2,1]
     */
    void show_digit(uint8_t position, uint8_t number, bool decimal_point) {
      if (position >= 1 && position <= 3 && number <= 9) {
        number |= decimal_point ? 0x80 : 0;
        spi_write(position, number);
      }
    }
  } // namespace

  /**
   * @brief Sends packets to configure the display's brightness, number of digits, etc.
   */
  void setup_display() {
    spi_write(0x09, 0x07); // decode mode: enable Code B decode for digits 0-2 to write digits directly
    spi_write(0x0A, 0x0F); // intensity: set to max brightness
    spi_write(0x0B, 0x02); // scan limit: display only 3 digits
    spi_write(0x0C, 0x01); // shutdown: turn on normal operation
    spi_write(0x0F, 0x00); // display Test off
  }

  void show(uint16_t value) {
    // Clamp from 0 to 999
    if (value > 999)
      value = 999;
    uint8_t digit1 = value / 100;
    uint8_t digit2 = (value / 10) % 10;
    uint8_t digit3 = value % 10;
    //setup_display();
    show_digit(1, digit1, true); // decimal point
    show_digit(2, digit2, false);
    show_digit(3, digit3, false);
  }

  void init() {
    pinMode(BMS_LOAD, OUTPUT);
    digitalWrite(BMS_LOAD, HIGH);
    // no MISO since the driver chip doesn't talk back
    spi.begin(BMS_CLK, -1, BMS_SDI, BMS_LOAD);
    setup_display();
    show(0);               // initialize with 0.00
  }
} // namespace Display

void batteryLevelTask(void *) {
  int voltage_buf[VOLTAGE_BUF_SIZE] = {};
  uint16_t index = 0;
  while (true) {
    vTaskDelay(pdMS_TO_TICKS(5));
    voltage_buf[index] =
        static_cast<uint16_t>((Battery::read() * 100.0f) + 0.5f);
    index = (index + 1) % VOLTAGE_BUF_SIZE;

    uint16_t val = 0;
    for (int i = 0; i < VOLTAGE_BUF_SIZE; i++)
      val += voltage_buf[i];
    val /= VOLTAGE_BUF_SIZE;

    Display::show(val);
  }
}

namespace Bluefruit {
  bool Parse(char *buf, size_t len) {
    char c1, c2, c3, c4;
    if (sscanf(buf, "%c%c%c%c", &c1, &c2, &c3, &c4)) {
      #ifdef REDROBOT_DEBUG
        Serial.printf("[Bluefruit] '%c' '%c' '%c' '%c'\n", c1, c2, c3, c4);
      #endif

      if (c1 != '!' || c2 != 'B')
          return false;

      switch (c3) {
      case '1':
        Button1_Pressed((c4 == '1'));
        break;
      case '2':
        Button2_Pressed((c4 == '1'));
        break;
      case '3':
        Button3_Pressed((c4 == '1'));
        break;
      case '4':
        Button4_Pressed((c4 == '1'));
        break;
      case '5':
        DPad_Pressed(UP, (c4 == '1'));
        break;  
      case '6':
        DPad_Pressed(DOWN, (c4 == '1'));
        break;  
      case '7':
        DPad_Pressed(LEFT, (c4 == '1'));
        break;
      case '8':
        DPad_Pressed(RIGHT, (c4 == '1'));
        break;
      default:
          Serial.println("[BLE] Invalid speed command");
          return false;
      }
    } else {
      Serial.println("[BLE] Invalid speed command");
      return false;
    }
    return true;
  }

  void __attribute__((weak)) DPad_Pressed(button_t Button, bool buttonPressedDown) {}
  void __attribute__((weak)) Button1_Pressed(bool buttonPressedDown) {}
  void __attribute__((weak)) Button2_Pressed(bool buttonPressedDown) {}
  void __attribute__((weak)) Button3_Pressed(bool buttonPressedDown) {}
  void __attribute__((weak)) Button4_Pressed(bool buttonPressedDown) {}
}

namespace RR_Controller {
  bool Parse(char *buf, size_t len) {
    char c1, c2, c3, c4;
    if (sscanf(buf, "%c%c%c%c", &c1, &c2, &c3, &c4)) {
      #ifdef REDROBOT_DEBUG
        Serial.printf("[RR_Controller] '%c' '%c' '%c/%d' '%d'\n", c1, c2, c3, c3, c4);
      #endif

      if (c1 != '~')
          return false;

      switch (c2) {
      case 'B':
        switch (c3) {
        case 'X':
          ButtonX_Pressed((c4 == 1));
          break;
        case 'Y':
          ButtonY_Pressed((c4 == 1));
          break;
        case 'B':
          ButtonB_Pressed((c4 == 1));
          break;
        case 'A':
          ButtonA_Pressed((c4 == 1));
          break;
        default:
          Serial.println("[BLE] Invalid speed command");
          return false;
        }
        break;
      case 'D':
        switch (c3) {
        case 'U':
          DPad_Pressed(UP, (c4 == 1));
          break;  
        case 'D':
          DPad_Pressed(DOWN, (c4 == 1));
          break;  
        case 'L':
          DPad_Pressed(LEFT, (c4 == 1));
          break;
        case 'R':
          DPad_Pressed(RIGHT, (c4 == 1));
          break;
        case 'C':
          DPad_Pressed(CENTER, (c4 == 1));
          break;
        default:
            Serial.println("[BLE] Invalid speed command");
            return false;
        }
        break;
      case 'J':
        Joystick1(c3, c4);
        break;
      case 'I':
        Joystick2(c3, c4);
        break;
      default:
        Serial.println("[BLE] Invalid speed command");
        return false;
      }
    } else {
      Serial.println("[BLE] Invalid speed command");
      return false;
    }
    return true;
  }

    void __attribute__((weak)) ButtonX_Pressed(bool buttonPressedDown) {}
    void __attribute__((weak)) ButtonY_Pressed(bool buttonPressedDown) {}
    void __attribute__((weak)) ButtonB_Pressed(bool buttonPressedDown) {}
    void __attribute__((weak)) ButtonA_Pressed(bool buttonPressedDown) {}
    void __attribute__((weak)) DPad_Pressed(button_t Button, bool buttonPressedDown) {}
    void __attribute__((weak)) Joystick1(int8_t x, int8_t y) {}
    void __attribute__((weak)) Joystick2(int8_t x, int8_t y) {}
}

void ParseBluetoothCommands(char *buf, size_t len) {
  #ifdef REDROBOT_DEBUG
    Serial.printf("[Bluetooth] Received: \"%s\"\n", buf);
  #endif
  if (!Bluefruit::Parse(buf, len));
    RR_Controller::Parse(buf, len);
}

namespace RedRobot {
TaskHandle_t batteryTaskHandler = nullptr;

void InitRedRobot() {
  pinMode(MOTOR1_EN, OUTPUT);
  pinMode(LBAT_LED, OUTPUT);
  digitalWrite(MOTOR1_EN, LOW);
  Serial.begin(115200);

  delay(2000);

  Serial.printf("Reset reason: %d\n", (int)esp_reset_reason());

  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  Display::init();
  actuatorMutex = xSemaphoreCreateMutex();
  if (!actuatorMutex)
    Serial.println("[INIT] Failed to create actuator mutex");
  BLE::init();
  Serial.println("[INIT] Creating battery task");

  xTaskCreatePinnedToCore(batteryLevelTask, "batteryLevel", 4096, nullptr, 1, &batteryTaskHandler, 1);
}
void InitDCMotors() {
    DCMotor::init();
}
void InitServo() {
    Servo::init();
}
void InitLineSensors() {
  LineSensors::init();
}
void InitAuton() {
  in_remote_control.store(false);
  Serial.println("[INIT] AutonomousLoop will run from the Arduino loop task");
}

/**
 * @brief Drive a certain motor at a certain speed (-100, 100)
 */
void WriteDCMotor(uint8_t d, int16_t speed) {
    if (actuatorMutex) xSemaphoreTake(actuatorMutex, portMAX_DELAY);
    if (in_remote_control.load() && xTaskGetCurrentTaskHandle() == arduinoLoopTask) {
      if (actuatorMutex) xSemaphoreGive(actuatorMutex);
      return;
    }
    DCMotor::write(DCMotor::DCMOTOR(d), speed);
    if (actuatorMutex) xSemaphoreGive(actuatorMutex);
}

void Brake() {
    for (uint8_t i = 0; i < 4; i++)
      WriteDCMotor(i, 0);
}

void SetServo(uint8_t s, int16_t angle) {
    if (actuatorMutex) xSemaphoreTake(actuatorMutex, portMAX_DELAY);
    if (in_remote_control.load() && xTaskGetCurrentTaskHandle() == arduinoLoopTask) {
      if (actuatorMutex) xSemaphoreGive(actuatorMutex);
      return;
    }
    Servo::set(Servo::SERVO(s), angle);
    if (actuatorMutex) xSemaphoreGive(actuatorMutex);
}

void ReadLineSensor(uint16_t values[6]) {
    LineSensors::read(values);

}

void StartRemoteControl() {
    in_remote_control.store(true);
}

bool IsRemoteControlled() {
    return in_remote_control.load();
}

}

void loop() {
  arduinoLoopTask = xTaskGetCurrentTaskHandle();

  if (!in_remote_control.load())
    AutonomousLoop();

  // Give equal-priority tasks on this core (batteryLevelTask) time.
  taskYIELD();
}
