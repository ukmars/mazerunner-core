#pragma once
#include <Arduino.h>

// NOTE: this is for UKMARSBOT with four-element advanced maze sensor
// and an Arduino nano processor

//***************************************************************************//
// Some physical constants that are likely to be board -specific

// with robot against back wall, how much travel is there to the cell center?
const int BACK_WALL_TO_CENTER = 48;

//***************************************************************************//

// Control loop timing. Pre-calculate to save time in interrupts
const float LOOP_FREQUENCY = 500.0;
const float LOOP_INTERVAL = (1.0 / LOOP_FREQUENCY);

//***************************************************************************//

const uint32_t BAUDRATE = 115200;
const int DEFAULT_DECIMAL_PLACES = 5;

//***************************************************************************//
// set this to zero to disable profile data logging over serial
#define DEBUG_LOGGING 1
// time between logged lined when reporting is enabled (milliseconds)
const int REPORTING_INTERVAL = 10;

//***************************************************************************//
const float MAX_MOTOR_VOLTS = 6.0;

//**** HARDWARE CONFIGURATION ***********************************************//
const uint8_t ENCODER_LEFT_CLK = 2;
const uint8_t ENCODER_RIGHT_CLK = 3;
const uint8_t ENCODER_LEFT_B = 4;
const uint8_t ENCODER_RIGHT_B = 5;
const uint8_t USER_IO = 6;
const uint8_t LED_RIGHT = 6; // an alias for USER_IO
const uint8_t MOTOR_LEFT_DIR = 7;
const uint8_t MOTOR_RIGHT_DIR = 8;
const uint8_t MOTOR_LEFT_PWM = 9;
const uint8_t MOTOR_RIGHT_PWM = 10;
const uint8_t LED_LEFT = 11; // an alias for EMITTER_B
const uint8_t EMITTER_A = 11;
const uint8_t EMITTER_B = 12;
const uint8_t FRONT_EMITTER = 11; // alias for pin 11
const uint8_t SIDE_EMITTER = 12;  // alias for pin 12

const int LED_USER = 6;

// these are the sensor ADC channels in case we have no special use for a given channel
const uint8_t SENSOR_0 = A0;
const uint8_t SENSOR_1 = A1;
const uint8_t SENSOR_2 = A2;
const uint8_t SENSOR_3 = A3;
const uint8_t SENSOR_4 = A4;
const uint8_t SENSOR_5 = A5;
const uint8_t SWITCHES_CHANNEL = A6;
const uint8_t FUNCTION_PIN = A6;
const uint8_t BATTERY_CHANNEL = A7;

// convenient aliases for the basic wall sensor channels
const uint8_t RIGHT_WALL_SENSOR = A0;
const uint8_t FRONT_RIGHT_WALL_SENSOR = A1;
const uint8_t FRONT_LEFT_WALL_SENSOR = A2;
const uint8_t LEFT_WALL_SENSOR = A3;

//***** SENSOR HARDWARE *****************************************************//
// the ADC channels corresponding to the sensor inputs. There are 8 available
// Channels 0..3 are normally used for sensors.
// Channels 4 and 5 are available if you do not want to add an I2C device
// Channel 6 is pre-allocated to the Battery monitor
// Channel 7 is re-allocated to the function switch and button
// ADVANCED SENSOR
#define RFS_CHANNEL 0
#define RSS_CHANNEL 1
#define LSS_CHANNEL 2
#define LFS_CHANNEL 3
// BASIC SENSOR - just repeat the front sensor to make the code cleaner
// #define RFS_CHANNEL 1
// #define RSS_CHANNEL 0
// #define LSS_CHANNEL 2
// #define LFS_CHANNEL 1

// if you have  basic sensor board with a single emitter pin,
// put the same pin number for both entries
// BASIC
// const int EMITTER_FRONT = USER_IO_12;
// const int EMITTER_DIAGONAL = USER_IO_12;
// ADVANCED
const int EMITTER_FRONT = EMITTER_A;
const int EMITTER_DIAGONAL = EMITTER_B;
//***************************************************************************//
// Battery resistor bridge //Derek Hall//
// The battery measurement is performed by first reducing the battery voltage
// with a potential divider formed by two resistors. Here they are named R1 and R2
// though that may not be their designation on the schematics.
//
// Resistor R1 is the high-side resistor and connects to the battery supply
// Resistor R2 is the low-side resistor and connects to ground
// Battery voltage is measured at the junction of these resistors
// The ADC port used for the conversion will have a full scale reading (FSR) that
// depends on the device being used. Typically that will be 1023 for a 10-bit ADC as
// found on an Arduino but it may be 4095 if you have a 12-bit ADC.
// Finally, the ADC converter on your processor will have a reference voltage. On
// the Arduinos for example, this is 5 Volts. Thus, a full scale reading of
// 1023 would represent 5 Volts, 511 would be 2.5Volts and so on.
//
// in this section you can enter the appropriate values for your ADC and potential
// divider setup to ensure that the battery voltage reading performed by the sensors
// is as accurate as possible.
//
// By calculating the battery multiplier here, you can be sure that the actual
// battery voltage calulation is done as efficiently as possible.
// The compiler will do all these calculations so your program does not have to.

const float BATTERY_R1 = 10000.0; // resistor to battery +
const float BATTERY_R2 = 10000.0; // resistor to Gnd
const float BATTERY_DIVIDER_RATIO = BATTERY_R2 / (BATTERY_R1 + BATTERY_R2);
const float ADC_FSR = 1023.0;    // The maximum reading for the ADC
const float ADC_REF_VOLTS = 5.0; // Reference voltage of ADC

const float BATTERY_MULTIPLIER = (ADC_REF_VOLTS / ADC_FSR / BATTERY_DIVIDER_RATIO);

// Finally include the hardware drivers
#include "adc_atmega328.h"
//***************************************************************************//
