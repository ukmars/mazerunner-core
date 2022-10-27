/******************************************************************************
 * Project: mazerunner-core                                                   *
 * File:    sensors.h                                                         *
 * File Created: Wednesday, 26th October 2022 12:11:36 am                     *
 * Author: Peter Harrison                                                     *
 * -----                                                                      *
 * Last Modified: Thursday, 27th October 2022 11:08:21 pm                     *
 * -----                                                                      *
 * Copyright 2022 - 2022 Peter Harrison, Micromouseonline                     *
 * -----                                                                      *
 * Licence:                                                                   *
 *     Use of this source code is governed by an MIT-style                    *
 *     license that can be found in the LICENSE file or at                    *
 *     https://opensource.org/licenses/MIT.                                   *
 ******************************************************************************/

#ifndef SENSORS_H
#define SENSORS_H

#include "../config.h"
#include "adc.h"
#include "digitalWriteFast.h"
#include <Arduino.h>
#include <src/atomic.h>
#include <wiring_private.h>

enum {
  STEER_NORMAL,
  STEER_LEFT_WALL,
  STEER_RIGHT_WALL,
  STEERING_OFF,
};

// used in the wait_for_user_start_function to indicate whih sensor was occluded
const uint8_t NO_START = 0;
const uint8_t LEFT_START = 1;
const uint8_t RIGHT_START = 2;

//***************************************************************************//
struct SensorChannel {
  int raw;   // whatever the ADC gives us
  int value; // normalised to 100 at reference position
};

class Sensors;
extern Sensors sensors;

class Sensors {

public:
  /*** wall sensor variables ***/

  volatile SensorChannel lfs;
  volatile SensorChannel lss;
  volatile SensorChannel rss;
  volatile SensorChannel rfs;

  volatile bool see_front_wall;
  volatile bool see_left_wall;
  volatile bool see_right_wall;

  volatile int m_front_sum;
  volatile int m_front_diff;

  /*** steering variables ***/
  uint8_t g_steering_mode = STEER_NORMAL;

  int get_front_sum() { return int(m_front_sum); };
  int get_front_diff() { return int(m_front_diff); };
  float get_steering_feedback() { return m_steering_adjustment; }
  float get_cross_track_error() { return m_cross_track_error; };
  float get_battery_comp() { return m_battery_compensation; };

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
  float calculate_steering_adjustment() {
    // always calculate the adjustment for testing. It may not get used.
    float pTerm = STEERING_KP * m_cross_track_error;
    float dTerm = STEERING_KD * (m_cross_track_error - last_steering_error);
    float adjustment = (pTerm + dTerm) * LOOP_INTERVAL;
    // TODO: are these limits appropriate, or even needed?
    adjustment = constrain(adjustment, -STEERING_ADJUST_LIMIT, STEERING_ADJUST_LIMIT);
    last_steering_error = m_cross_track_error;
    m_steering_adjustment = adjustment;
    return adjustment;
  }

  void set_steering_mode(uint8_t mode) {
    last_steering_error = m_cross_track_error;
    m_steering_adjustment = 0;
    g_steering_mode = mode;
  }

  //***************************************************************************//

  void enable() {
    adc.enable_emitters();
    m_enabled = true;
  }

  void disable() {
    adc.disable_emitters();
    m_enabled = false;
  }

  //***************************************************************************//

  void update_battery_voltage() {
    m_battery_volts = BATTERY_MULTIPLIER * m_battery_adc;
    m_battery_compensation = 255.0 / m_battery_volts;
  }

  /*********************************** Wall tracking **************************/
  // calculate the alignment errors - too far left is negative

  /***
   * Note: Runs from the systick interrupt. DO NOT call this directly.
   * @brief update the global wall sensor values.
   * @return robot cross-track-error. Too far left is negative.
   */
  void update() {
    m_battery_adc = adc[BATTERY_CHANNEL];

    update_battery_voltage();
    if (not m_enabled) {
      // NOTE: No values will be updated although the ADC is working
      m_cross_track_error = 0;
      m_steering_adjustment = 0;
      return;
    }

    // this should be the only place that the aactual ADC channels are referenced
    // if there is only a single front sensor (Basic sensor board) then the value is
    // just used twice
    // keep these values for calibration assistance
    // they should never be negative
    rfs.raw = max(0, adc[RFS_CHANNEL]);
    rss.raw = max(0, adc[RSS_CHANNEL]);
    lss.raw = max(0, adc[LSS_CHANNEL]);
    lfs.raw = max(0, adc[LFS_CHANNEL]);

    // normalise to a nominal value of 100
    rfs.value = (int)(rfs.raw * FRONT_RIGHT_SCALE);
    rss.value = (int)(rss.raw * RIGHT_SCALE);
    lss.value = (int)(lss.raw * LEFT_SCALE);
    lfs.value = (int)(lfs.raw * FRONT_LEFT_SCALE);

    // set the wall detection flags
    see_left_wall = lss.value > LEFT_THRESHOLD;
    see_right_wall = rss.value > RIGHT_THRESHOLD;
    m_front_sum = lfs.value + rfs.value;
    m_front_diff = lfs.value - rfs.value;
    see_front_wall = m_front_sum > FRONT_THRESHOLD;

    // calculate the alignment errors - too far left is negative
    int error = 0;
    int right_error = SIDE_NOMINAL - rss.value;
    int left_error = SIDE_NOMINAL - lss.value;
    if (g_steering_mode == STEER_NORMAL) {
      if (sensors.see_left_wall && sensors.see_right_wall) {
        error = left_error - right_error;
      } else if (sensors.see_left_wall) {
        error = 2 * left_error;
      } else if (sensors.see_right_wall) {
        error = -2 * right_error;
      }
    } else if (g_steering_mode == STEER_LEFT_WALL) {
      error = 2 * left_error;
    } else if (g_steering_mode == STEER_RIGHT_WALL) {
      error = -2 * right_error;
    }

    // the side sensors are not reliable close to a wall ahead.
    // TODO: The magic number 100 may need adjusting
    if (m_front_sum > 100) {
      error = 0;
    }
    m_cross_track_error = error;
    calculate_steering_adjustment();
  }

  //***************************************************************************//

  bool occluded_left() {
    return lfs.raw > 100 && sensors.rfs.raw < 100;
  }

  bool occluded_right() {
    return lfs.raw < 100 && sensors.rfs.raw > 100;
  }

  uint8_t wait_for_user_start() {
    digitalWrite(LED_LEFT, 1);
    enable();
    int count = 0;
    uint8_t choice = NO_START;
    while (choice == NO_START) {
      count = 0;
      while (occluded_left()) {
        count++;
        delay(20);
      }
      if (count > 5) {
        choice = LEFT_START;
        break;
      }
      count = 0;
      while (occluded_right()) {
        count++;
        delay(20);
      }
      if (count > 5) {
        choice = RIGHT_START;
        break;
      }
    }
    disable();
    digitalWrite(LED_LEFT, 0);
    delay(250);
    return choice;
  }

private:
  float last_steering_error = 0;
  volatile bool m_enabled = false;
  volatile int m_battery_adc;
  volatile float m_battery_volts;
  volatile float m_battery_compensation;
  volatile float m_cross_track_error;
  volatile float m_steering_adjustment;
};

#endif