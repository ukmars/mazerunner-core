/*
 * File: sensors.h
 * Project: mazerunner
 * File Created: Monday, 29th March 2021 11:05:58 pm
 * Author: Peter Harrison
 * -----
 * Last Modified: Friday, 9th April 2021 11:45:23 am
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

#ifndef SENSORS_H
#define SENSORS_H

#include <Arduino.h>
#include <util/atomic.h>

struct WallSensor {
  int raw;       // whatever the ADC gives us
  int value;     // normalised to 100 at reference position
  bool has_wall; // true if a wall is present
};

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
extern volatile float g_battery_voltage;
extern volatile float g_battery_scale; // adjusts PWM for voltage changes
//***************************************************************************//

extern volatile int g_ws_lfs;
extern volatile int g_ws_lss;
extern volatile int g_ws_rss;
extern volatile int g_ws_rfs;

/*** wall sensor variables ***/
extern volatile int g_lfs;
extern volatile int g_lss;
extern volatile int g_rss;
extern volatile int g_rfs;

extern volatile int g_front_sum;

/*** These are the values before normalisation */
extern volatile int g_lfs_raw;
extern volatile int g_lss_raw;
extern volatile int g_rss_raw;
extern volatile int g_rfs_raw;

// true if a wall is present
extern volatile bool g_front_has_wall;

extern volatile bool g_lfs_has_wall;
extern volatile bool g_lss_has_wall;
extern volatile bool g_rss_has_wall;
extern volatile bool g_rfs_has_wall;

/*** steering variables ***/
extern uint8_t g_steering_mode;
extern bool g_steering_enabled;
extern volatile float g_cross_track_error;
extern volatile float g_steering_adjustment;

//***************************************************************************//
void setup_adc();
void enable_sensors();
void disable_sensors();

void update_battery_voltage();
float update_wall_sensors();

void start_sensor_cycle();

void set_steering_mode(uint8_t mode);
float calculate_steering_adjustment(float error);

int get_switches();

// TODO - make these NOT inline and move to UI
inline bool button_pressed() {
  return get_switches() == 16;
}

inline void wait_for_button_press() {
  while (not(button_pressed())) {
    delay(10);
  };
}

inline void wait_for_button_release() {
  while (button_pressed()) {
    delay(10);
  };
}

inline void wait_for_button_click() {
  wait_for_button_press();
  wait_for_button_release();
  delay(250);
}

inline bool occluded_left() {
  return g_lfs_raw > 100 && g_rfs_raw < 100;
}

inline bool occluded_right() {
  return g_lfs_raw < 100 && g_rfs_raw > 100;
}

inline uint8_t wait_for_user_start() {
  enable_sensors();
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
  disable_sensors();
  delay(250);
  return choice;
}
#endif
