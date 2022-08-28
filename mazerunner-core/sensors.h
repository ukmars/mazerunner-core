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
//***************************************************************************//
extern volatile float g_battery_voltage;
extern volatile float g_battery_scale; // adjusts PWM for voltage changes
//***************************************************************************//

/*** wall sensor variables ***/
extern volatile int g_front_wall_sensor;
extern volatile int g_left_wall_sensor;
extern volatile int g_right_wall_sensor;

/*** These are the values before normalisation */
extern volatile int g_front_wall_sensor_raw;
extern volatile int g_left_wall_sensor_raw;
extern volatile int g_right_wall_sensor_raw;

// true if a wall is present
extern volatile bool g_left_wall_present;
extern volatile bool g_front_wall_present;
extern volatile bool g_right_wall_present;

/*** steering variables ***/
extern bool g_steering_enabled;
extern volatile float g_cross_track_error;
extern volatile float g_steering_adjustment;

inline int get_left_sensor() {
  int value;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    value = g_left_wall_sensor;
  }
  return value;
}

inline int get_front_sensor() {
  int value;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    value = g_front_wall_sensor;
  }
  return value;
}

inline int get_right_sensor() {
  int value;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    value = g_right_wall_sensor;
  }
  return value;
}

//***************************************************************************//
void setup_adc();
void enable_sensors();
void disable_sensors();

void update_battery_voltage();
float update_wall_sensors();

void start_sensor_cycle();

void reset_steering();
void enable_steering();
void disable_steering();
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

inline void wait_for_front_sensor() {
  enable_sensors();
  while (g_front_wall_sensor < 250) {
    delay(10);
  }
  while (g_front_wall_sensor > 200) {
    delay(10);
  }
  disable_sensors();
  delay(500);
}

#endif
