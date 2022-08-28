/*
 * File: sensors.cpp
 * Project: mazerunner
 * File Created: Monday, 29th March 2021 11:05:58 pm
 * Author: Peter Harrison
 * -----
 * Last Modified: Friday, 9th April 2021 11:45:39 am
 * Modified By: Peter Harrison
 * -----
 * MIT License
 *
 * Copyright (c) 2021 Peter Harrison
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies
 * of the Software, and to permit persons to whom the Software is furnished to do
 * so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include "sensors.h"
#include "digitalWriteFast.h"
#include "settings.h"
#include <Arduino.h>
#include <util/atomic.h>
#include <wiring_private.h>

/**** Global variables  ****/

volatile float g_battery_voltage;
volatile float g_battery_scale;

/*** wall sensor variables ***/
volatile int g_front_wall_sensor;
volatile int g_left_wall_sensor;
volatile int g_right_wall_sensor;

volatile int g_front_wall_sensor_raw;
volatile int g_left_wall_sensor_raw;
volatile int g_right_wall_sensor_raw;

/*** true if a wall is present ***/
volatile bool g_left_wall_present;
volatile bool g_front_wall_present;
volatile bool g_right_wall_present;

/*** steering variables ***/
bool g_steering_enabled;
volatile float g_cross_track_error;
volatile float g_steering_adjustment;

//***************************************************************************//
/***  Local variables ***/
static float last_steering_error = 0;
static volatile bool s_sensors_enabled = false;
static volatile int adc[6];
static volatile int battery_adc_reading;
static volatile int switches_adc_reading;

//***************************************************************************//

/**
 *  The default for the Arduino is to give a slow ADC clock for maximum
 *  SNR in the results. That typically means a prescale value of 128
 *  for the 16MHz ATMEGA328P running at 16MHz. Conversions then take more
 *  than 100us to complete. In this application, we want to be able to
 *  perform about 16 conversions in around 500us. To do that the prescaler
 *  is reduced to a value of 32. This gives an ADC clock speed of
 *  500kHz and a single conversion in around 26us. SNR is still pretty good
 *  at these speeds:
 *  http://www.openmusiclabs.com/learning/digital/atmega-adc/
 *
 * @brief change the ADC prescaler to give a suitable conversion rate.
 */
void setup_adc() {
  // Change the clock prescaler from 128 to 32 for a 500kHz clock
  bitSet(ADCSRA, ADPS2);
  bitClear(ADCSRA, ADPS1);
  bitSet(ADCSRA, ADPS0);
}

/**
 * The adc_thresholds may beed adjusting for non-standard resistors.
 *
 * @brief  Convert the switch ADC reading into a switch reading.
 * @return integer in range 0..16 or -1 if there is an error
 */
int get_switches() {
  const int adc_thesholds[] = {660, 647, 630, 614, 590, 570, 545, 522, 461, 429, 385, 343, 271, 212, 128, 44, 0};

  if (switches_adc_reading > 800) {
    return 16;
  }
  for (int i = 0; i < 16; i++) {
    if (switches_adc_reading > (adc_thesholds[i] + adc_thesholds[i + 1]) / 2) {
      return i;
    }
  }
  return -1;
}

//***************************************************************************//

/**
 * The steering adjustment is an angular error that is added to the
 * current encoder angle so that the robot can be kept central in
 * a maze cell.
 *
 * A PD controller is used to generate the adjustment and the two constants
 * will need to be adjusted for the best response. You may find that only
 * the P term is needed
 *
 * The steering adjustment is limited to prevent over-correction. You should
 * experiment with that as well.
 *
 * @brief Calculate the steering adjustment from the cross-track error.
 * @param error calculated from wall sensors, Negative if too far right
 * @return steering adjustment in degrees
 */
float calculate_steering_adjustment(float error) {
  // always calculate the adjustment for testing. It may not get used.
  float pTerm = settings.steering_KP * error;
  float dTerm = settings.steering_KD * (error - last_steering_error);
  float adjustment = (pTerm + dTerm) * LOOP_INTERVAL;
  // TODO: are these limits appropriate, or even needed?
  adjustment = constrain(adjustment, -STEERING_ADJUST_LIMIT, STEERING_ADJUST_LIMIT);
  last_steering_error = error;
  return adjustment;
}

void reset_steering() {
  last_steering_error = g_cross_track_error;
  g_steering_adjustment = 0;
}

void enable_steering() {
  reset_steering();
  g_steering_enabled = true;
};

void disable_steering() {
  g_steering_enabled = false;
}

//***************************************************************************//

void enable_sensors() {
  s_sensors_enabled = true;
}

void disable_sensors() {
  s_sensors_enabled = false;
}

//***************************************************************************//

void update_battery_voltage() {
  g_battery_voltage = BATTERY_MULTIPLIER * battery_adc_reading;
  g_battery_scale = 255.0 / g_battery_voltage;
}
/*********************************** Wall tracking **************************/
/***
 * This is for the basic, three detector wall sensor only
 *
 * Note: Runs in the systick interrupt. DO NOT call this directly.
 * @brief update the global wall sensor values.
 * @return robot cross-track-error. Too far left is negative.
 */
float update_wall_sensors() {
  if (not s_sensors_enabled) {
    return 0;
  }
  // they should never be negative
  adc[0] = max(0, adc[0]);
  adc[1] = max(0, adc[1]);
  adc[2] = max(0, adc[2]);
  // keep these values for calibration assistance
  g_right_wall_sensor_raw = adc[0];
  g_front_wall_sensor_raw = adc[1];
  g_left_wall_sensor_raw = adc[2];

  // normalise to a nominal value of 100
  g_right_wall_sensor = (int)(g_right_wall_sensor_raw * settings.right_adjust);
  g_front_wall_sensor = (int)(g_front_wall_sensor_raw * settings.front_adjust);
  g_left_wall_sensor = (int)(g_left_wall_sensor_raw * settings.left_adjust);

  // set the wall detection flags
  g_left_wall_present = g_left_wall_sensor > settings.left_threshold;
  g_right_wall_present = g_right_wall_sensor > settings.right_threshold;
  g_front_wall_present = g_front_wall_sensor > settings.front_threshold;

  // calculate the alignment errors - too far left is negative
  float error = 0;
  float right_error = settings.right_nominal - g_right_wall_sensor;
  float left_error = settings.left_nominal - g_left_wall_sensor;
  if (g_left_wall_present && g_right_wall_present) {
    error = left_error - right_error;
  } else if (g_left_wall_present) {
    error = 2.0 * left_error;
  } else if (g_right_wall_present) {
    error = -2.0 * right_error;
  }
  // the side sensors are not reliable close to a wall ahead.
  // TODO: The magic number 100 may need adjusting
  if (g_front_wall_sensor > 100) {
    error = 0;
  }
  return error;
}

//***************************************************************************//

/***
 * NOTE: Manual analogue conversions
 * All eight available ADC channels are aatomatically converted
 * by the sensor interrupt. Attempting to performa a manual ADC
 * conversion with the Arduino AnalogueIn() function will disrupt
 * that process so avoid doing that.
 */

static const uint8_t ADC_REF = DEFAULT;

static void start_adc(uint8_t pin) {
  if (pin >= 14)
    pin -= 14; // allow for channel or pin numbers
               // set the analog reference (high two bits of ADMUX) and select the
               // channel (low 4 bits).  Result is right-adjusted
  ADMUX = (ADC_REF << 6) | (pin & 0x07);
  // start the conversion
  sbi(ADCSRA, ADSC);
}

static int get_adc_result() {
  // ADSC is cleared when the conversion finishes
  // while (bit_is_set(ADCSRA, ADSC));

  // we have to read ADCL first; doing so locks both ADCL
  // and ADCH until ADCH is read.  reading ADCL second would
  // cause the results of each conversion to be discarded,
  // as ADCL and ADCH would be locked when it completed.
  uint8_t low = ADCL;
  uint8_t high = ADCH;

  // combine the two bytes
  return (high << 8) | low;
}

static uint8_t sensor_phase = 0;

void start_sensor_cycle() {
  sensor_phase = 0;     // sync up the start of the sensor sequence
  bitSet(ADCSRA, ADIE); // enable the ADC interrupt
  start_adc(0);         // begin a conversion to get things started
}

/** @brief Sample all the sensor channels with and without the emitter on
 *
 * At the end of the 500Hz systick interrupt, the ADC interrupt is enabled
 * and a conversion started. After each ADC conversion the interrupt gets
 * generated and this ISR is called. The eight channels are read in turn with
 * the sensor emitter(s) off.
 * At the end of that sequence, the emiter(s) get turned on and a dummy ADC
 * conversion is started to provide a delay while the sensors respond.
 * After that, all channels are read again to get the lit values.
 * After all the channels have been read twice, the ADC interrupt is disabbled
 * and the sensors are idle until triggered again.
 *
 * The ADC service runs all th etime even with the sensors 'disabled'. In this
 * software, 'enabled' only means that the emitters are turned on in the second
 * phase. Without that, you might expect the sensor readings to be zero.
 *
 * Timing tests indicate that the sensor ISR consumes no more that 5% of the
 * available system bandwidth.
 *
 * There are actually 16 available channels and channel 8 is the internal
 * temperature sensor. Channel 15 is Gnd. If appropriate, a read of channel
 * 15 can be used to zero the ADC sample and hold capacitor.
 *
 * NOTE: All the channels are read even though only 5 are used for the maze
 * robot. This gives worst-case timing so there are no surprises if more
 * sensors are added.
 * If different types of sensor are used or the I2C is needed, there
 * will need to be changes here.
 */
ISR(ADC_vect) {
  // digitalWriteFast(13, 1);
  switch (sensor_phase) {
    case 0:
      // always start conversions as soon as  possible so they get a
      // full 50us to convert
      start_adc(BATTERY_VOLTS);
      break;
    case 1:
      battery_adc_reading = get_adc_result();
      start_adc(FUNCTION_PIN);
      break;
    case 2:
      switches_adc_reading = get_adc_result();
      start_adc(RIGHT_WALL_SENSOR);
      break;
    case 3:
      adc[0] = get_adc_result();
      start_adc(FRONT_WALL_SENSOR);
      break;
    case 4:
      adc[1] = get_adc_result();
      start_adc(LEFT_WALL_SENSOR);
      break;
    case 5:
      adc[2] = get_adc_result();
      start_adc(A3);
      break;
    case 6:
      adc[3] = get_adc_result();
      start_adc(A4);
      break;
    case 7:
      adc[4] = get_adc_result();
      start_adc(A5);
      break;
    case 8:
      adc[5] = get_adc_result();
      if (s_sensors_enabled) {
        // got all the dark ones so light them up
        digitalWriteFast(EMITTER, 1);
      }
      start_adc(A7); // dummy read of the battery to provide delay
      // wait at least one cycle for the detectors to respond
      break;
    case 9:
      start_adc(RIGHT_WALL_SENSOR);
      break;
    case 10:
      adc[0] = get_adc_result() - adc[0];
      start_adc(FRONT_WALL_SENSOR);
      break;
    case 11:
      adc[1] = get_adc_result() - adc[1];
      start_adc(LEFT_WALL_SENSOR);
      break;
    case 12:
      adc[2] = get_adc_result() - adc[2];
      start_adc(A3);
      break;
    case 13:
      adc[3] = get_adc_result() - adc[3];
      start_adc(A4);
      break;
    case 14:
      adc[4] = get_adc_result() - adc[4];
      start_adc(A5);
      break;
    case 15:
      adc[5] = get_adc_result() - adc[5];
      digitalWriteFast(EMITTER, 0);
      bitClear(ADCSRA, ADIE); // turn off the interrupt
      break;
    default:
      break;
  }
  sensor_phase++;
  // digitalWriteFast(13, 0);
}
