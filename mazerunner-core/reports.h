/*
 * File: logger.h
 * Project: mazerunner
 * File Created: Tuesday, 23rd March 2021 10:18:19 pm
 * Author: Peter Harrison
 * -----
 * Last Modified: Wednesday, 14th April 2021 4:36:51 pm
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

#ifndef LOGGER_H
#define LOGGER_H

#include "reports.h"
#include "src/encoders.h"
#include "src/motors.h"
#include "src/profile.h"
#include "src/sensors.h"
#include "src/utils.h"
#include <Arduino.h>

class Reporter;
extern Reporter reporter;
class Reporter {

  uint32_t s_start_time;
  uint32_t s_report_time;
  uint32_t s_report_interval = REPORTING_INTERVAL;

public:
  // note that the Serial device has a 64 character buffer and, at 115200 baud
  // 64 characters will take about 6ms to go out over the wire.
  void report_profile_header() {
#if DEBUG_LOGGING == LOGGING_ON
    Serial.println(F("time robotPos robotAngle fwdPos  fwdSpeed rotpos rotSpeed fwdVolts rotVolts"));
    s_start_time = millis();
    s_report_time = s_start_time;
#endif
  }

  void report_profile() {
#if DEBUG_LOGGING == LOGGING_ON
    if (millis() >= s_report_time) {
      s_report_time += s_report_interval;
      Serial.print(millis() - s_start_time);
      Serial.print(' ');
      Serial.print(encoders.robot_distance());
      Serial.print(' ');
      Serial.print(encoders.robot_angle());
      Serial.print(' ');
      Serial.print(forward.position());
      Serial.print(' ');
      Serial.print(forward.speed());
      Serial.print(' ');
      Serial.print(rotation.position());
      Serial.print(' ');
      Serial.print(rotation.speed());
      Serial.print(' ');
      Serial.print(50 * (motors.get_right_motor_volts() + motors.get_left_motor_volts()));
      Serial.print(' ');
      Serial.print(50 * (motors.get_right_motor_volts() - motors.get_left_motor_volts()));
      Serial.println();
    }
#else
    delay(2);
#endif
  }

  //***************************************************************************//

  void report_sensor_track_header() {
#if DEBUG_LOGGING == LOGGING_ON
    Serial.println(F("time pos angle left right front error adjustment"));
    s_start_time = millis();
    s_report_time = s_start_time;
#endif
  }

  void report_sensor_track(bool use_raw = false) {
#if DEBUG_LOGGING == LOGGING_ON
    if (millis() >= s_report_time) {
      s_report_time += s_report_interval;
      Serial.print(millis() - s_start_time);
      Serial.print(' ');
      Serial.print(encoders.robot_distance());
      Serial.print(' ');
      Serial.print(encoders.robot_angle());
      if (use_raw) {
        Serial.print(' ');
        Serial.print(sensors.lss.raw);
        Serial.print(' ');
        Serial.print(sensors.rss.raw);
        Serial.print(' ');
        Serial.print(sensors.lfs.raw);
        Serial.print(' ');
        Serial.print(sensors.rfs.raw);
      } else {
        Serial.print(' ');
        Serial.print(sensors.lss.value);
        Serial.print(' ');
        Serial.print(sensors.rss.value);
        Serial.print(' ');
        Serial.print(sensors.lfs.value);
        Serial.print(' ');
        Serial.print(sensors.rfs.value);
      }
      Serial.print(' ');
      Serial.print(sensors.get_cross_track_error());
      Serial.print(' ');
      Serial.print(sensors.get_steering_feedback());
      Serial.println();
    }
#else
    delay(2);
#endif
  }

  void report_front_sensor_track_header() {
#if DEBUG_LOGGING == LOGGING_ON
    Serial.println(F("time pos front_normal front_raw"));
    s_start_time = millis();
    s_report_time = s_start_time;
#endif
  }

  void report_front_sensor_track() {
#if DEBUG_LOGGING == LOGGING_ON
    if (millis() >= s_report_time) {
      s_report_time += s_report_interval;
      Serial.print(millis() - s_start_time);
      Serial.print(' ');
      Serial.print(fabsf(encoders.robot_distance()));
      Serial.print(' ');
      Serial.print(sensors.lfs.value);
      Serial.print(' ');
      Serial.print(sensors.lfs.raw);
      Serial.println();
    }
#else
    delay(2);
#endif
  }

  //***************************************************************************//

  void report_wall_sensors() {
    Serial.print('\n');
    Serial.print('R');
    Serial.print(' ');
    print_justified(sensors.lfs.raw, 7);
    print_justified(sensors.lss.raw, 7);
    print_justified(sensors.rss.raw, 7);
    print_justified(sensors.rfs.raw, 7);
    Serial.print(' ');
    Serial.print('-');
    Serial.print('>');
    print_justified(sensors.lfs.value, 7);
    print_justified(sensors.lss.value, 7);
    print_justified(sensors.rss.value, 7);
    print_justified(sensors.rfs.value, 7);
    Serial.print(' ');
  }

  //***************************************************************************//

  void report_sensor_calibration() {
    Serial.println(F("   lf_raw ls_raw rs_raw rf_raw |  lf_cal ls_cal rs_cal rf_cal"));
    sensors.enable();
    s_start_time = millis();
    s_report_time = s_start_time;
    while (not sensors.button_pressed()) {
      if (millis() >= s_report_time) {
        s_report_time += 100;
        report_wall_sensors();
      }
    }
    Serial.println();
    sensors.wait_for_button_release();
    delay(200);
    sensors.disable();
  }
};
/**
 * The profile reporter will send out a table of space separated
 * data so that the results can be saved to a file or imported to
 * a spreadsheet or other analyss software.
 *
 * Always send the header first because that restarts the elapsed
 * time count.
 *
 * The data includes
 *   time        - in milliseconds since the header was sent
 *   robotPos    - position in mm as reported by the encoders
 *   robotAngle  - angle in degrees as reported by the encoders
 *   fwdPos      - forward profiler setpoint in mm
 *   fwdSpeed    - forward profiler current speed in mm/s
 *   rotpos      - rotation profiler setpoint in deg
 *   rotSpeed    - rotation profiler current speed in deg/s
 *   fwdVolts    - voltage sent to the motors for forward control
 *   rotVolts    - voltage sent to the motors for rotation control
 *
 */
#endif