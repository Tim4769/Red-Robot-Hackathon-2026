/**
 * @file  pinouts.h
 * @brief ESP32 pinout map
 *
 * Nets should match name with schematic
 */

#pragma once

// Motors
#define MOTOR0_PHASE 9
#define MOTOR1_PHASE 10
#define MOTOR2_PHASE 7
#define MOTOR3_PHASE 8

#define MOTOR0_EN 14
#define MOTOR1_EN 43 // TXD0
#define MOTOR2_EN 36
#define MOTOR3_EN 35

// Servo PWM
#define SERVO0_PWM 13
#define SERVO1_PWM 12
#define SERVO2_PWM 47
#define SERVO3_PWM 48

// Buzzer
#define BUZZER 15

// Status LEDs
#define LBAT_LED 16
#define RADIO_LED 21
#define RX_LED 17
#define PROG_LED 18

// Line following
#define LF0 37
#define LF1 38
#define LF2 39
#define LF3 40
#define LF4 41
#define LF5 42
// Fixed: was 43 (that is TXD0 = MOTOR1_EN). The board netlist routes
// LF_LEDCTRL to module pin 36 = RXD0 = GPIO44, so the IR LEDs never turned on.
#define LF_LEDCTRL 44 // RXD0

// BMS
#define BMS_LOAD 4
#define BMS_SDI 5
#define BMS_CLK 6

// Battery voltage divider
#define ADC1_0 1