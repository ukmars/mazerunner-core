/******************************************************************************
 * Project: mazerunner-core                                                   * 
 * File:    reports.h                                                         * 
 * File Created: Saturday, 10th September 2022 2:18:37 pm                     * 
 * Author: Peter Harrison                                                     * 
 * -----                                                                      * 
 * Last Modified: Wednesday, 26th October 2022 11:49:54 pm                    * 
 * -----                                                                      * 
 * Copyright 2022 - 2022 Peter Harrison, Micromouseonline                     * 
 * -----                                                                      * 
 * Licence:                                                                   * 
 *     Use of this source code is governed by an MIT-style                    * 
 *     license that can be found in the LICENSE file or at                    * 
 *     https://opensource.org/licenses/MIT.                                   * 
 ******************************************************************************/

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
   *   fwdmVolts    - voltage sent to the motors for forward control
   *   rotmVolts    - voltage sent to the motors for rotation control
   *
   */
  void report_profile_header() {
    Serial.println(F("time robotPos robotAngle fwdPos  fwdSpeed rotpos rotSpeed fwdmVolts rotmVolts"));
    s_start_time = millis();
    s_report_time = s_start_time;
  }

  void report_profile() {
    if (millis() >= s_report_time) {
      s_report_time += s_report_interval;
      print_justified(int(millis() - s_start_time), 6);
      print_justified(int(encoders.robot_distance()), 6);
      print_justified(int(encoders.robot_angle()), 6);
      print_justified(int(forward.position()), 6);
      print_justified(int(forward.speed()), 6);
      print_justified(int(rotation.position()), 6);
      print_justified(int(rotation.speed()), 6);
      print_justified(motors.get_fwd_millivolts(), 6);
      print_justified(motors.get_rot_millivolts(), 6);
      Serial.println();
    }
  }

  //***************************************************************************//

  /***
   * The full sensor track report can be used to help calibrate the sensors and
   * check the operation of the steering.
   *
   * You could also use it to examine the sensor readings as walls are detected or lost.
   *
   * To avoid sending more data that is needed, the default is to send only the
   * normalised sensor readings. Call with the argument set true to see raw sensor
   * readings for calibration checks.
   *
   */
  void report_sensor_track_header() {
    Serial.println(F("time pos angle left right front error adjustment"));
    s_start_time = millis();
    s_report_time = s_start_time;
  }

  void report_sensor_track(bool use_raw = false) {
    if (millis() >= s_report_time) {
      s_report_time += s_report_interval;
      print_justified(int(millis() - s_start_time), 6);
      print_justified(int(encoders.robot_distance()), 6);
      print_justified(int(encoders.robot_angle()), 6);
      if (use_raw) {
        print_justified(sensors.lfs.raw, 6);
        print_justified(sensors.lss.raw, 6);
        print_justified(sensors.rss.raw, 6);
        print_justified(sensors.rfs.raw, 6);
      } else {
        print_justified(sensors.lfs.value, 6);
        print_justified(sensors.lss.value, 6);
        print_justified(sensors.rss.value, 6);
        print_justified(sensors.rfs.value, 6);
      }
      Serial.print(' ');
      Serial.print(sensors.get_cross_track_error());
      Serial.print(' ');
      Serial.print(sensors.get_steering_feedback());
      Serial.println();
    }
  }

  //***************************************************************************//

  /***
   * This specialised report is meant to be used when calibrating the forward
   * sensors. The robot would be placed up agains a wall ahead and commanded to
   * back up slowly for some fixed distance - 180mm seems appropriate. As it
   * backs up the sensor readings are transmitted so that you can build a table
   * of expected sensor readings against distance from the front wall. This
   * will be useful in setting up things like turn thresholds and to allow the
   * robot to adjust its position in a maze cell.
   *
   * In normal operation, UKMARSBOT has its front edge 22 mm from the wall ahead.
   *
   */
  void front_sensor_track_header() {
    Serial.println(F("dist front_sum front_diff"));
  }

  void front_sensor_track() {
    if (millis() >= s_report_time) {
      print_justified(int(encoders.robot_distance()), 7);
      print_justified(sensors.get_front_sum(), 7);
      print_justified(sensors.get_front_diff(), 7);
      Serial.println();
    }
  }

  //***************************************************************************//

  /***
   * Mostly, you are likely to want to just stream out the current sensor readings
   * to a terminal while you check their values.
   *
   * This report does that, showing the raw and normalised values as well as the
   * sum and difference of the front sensors.
   *
   * This is possibly the most useful streamed information because you can use it
   * with the robot in a maze to get basic calibration values.
   *
   *
   */
  void wall_sensor_header() {
    Serial.println(F("   lf_   ls_   rs_   rf_   lfs   lss   rss   rfs   sum  diff"));
  }

  void show_wall_sensors() {
    print_justified(sensors.lfs.raw, 6);
    print_justified(sensors.lss.raw, 6);
    print_justified(sensors.rss.raw, 6);
    print_justified(sensors.rfs.raw, 6);
    print_justified(sensors.lfs.value, 6);
    print_justified(sensors.lss.value, 6);
    print_justified(sensors.rss.value, 6);
    print_justified(sensors.rfs.value, 6);
    print_justified(sensors.get_front_sum(), 6);
    print_justified(sensors.get_front_diff(), 6);
    Serial.println();
  }

  //***************************************************************************//
};

#endif