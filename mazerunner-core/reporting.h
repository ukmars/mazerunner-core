/******************************************************************************
 * Project: mazerunner-core                                                   *
 * -----                                                                      *
 * Copyright 2022 - 2023 Peter Harrison, Micromouseonline                     *
 * -----                                                                      *
 * Licence:                                                                   *
 *     Use of this source code is governed by an MIT-style                    *
 *     license that can be found in the LICENSE file or at                    *
 *     https://opensource.org/licenses/MIT.                                   *
 ******************************************************************************/

#ifndef LOGGER_H
#define LOGGER_H

#include <Arduino.h>
#include "encoders.h"
#include "motion.h"
#include "motors.h"
#include "profile.h"
#include "sensors.h"

/***
 * While there is no absolute requirement for reporting in a micromouse or
 * similar robot, it can be of enormous help when setting up and debugging
 * the behaviour of the robot.
 *
 * The Reporter class and its supporting functions provide a number of pre-
 * written reports over the Serial device that let you observe things like
 * controller state and sensor readings. Many of these will be used a lot
 * during development and you can use any of them as a template for custom
 * reports.
 *
 * Although the Serial device on an Arduino normally requires the USB connection
 * UKMARSBOT has a connector specially intended for a cheap Bluetooth
 * module like the HC-05. This can be left plugged in all the time so that
 * reporting can be done while running. You could also connect a serial
 * logger to this port and record data throughout a contest run.
 *
 * The BT connector also makes it possible to use the interactive Command
 * Line Interface while the robot is running.
 *
 */
const char hdg_letters[] = "FRAL";
const char dirLetters[] = "NESW";

// simple formatting functions for printing maze costs
inline void print_hex_2(unsigned char value) {
  if (value < 16) {
    Serial.print('0');
  }
  Serial.print(value, HEX);
}

inline void print_justified(int32_t value, int width) {
  int v = value;
  int w = width;
  w--;
  if (v < 0) {
    w--;
  }
  while (v /= 10) {
    w--;
  }
  while (w > 0) {
    Serial.write(' ');
    --w;
  }
  Serial.print(value);
}

inline void print_justified(int value, int width) {
  print_justified(int32_t(value), width);
}

//***************************************************************************//

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
      print_justified(int(motion.position()), 6);
      print_justified(int(motion.velocity()), 6);
      print_justified(int(motion.angle()), 6);
      print_justified(int(motion.omega()), 6);
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
    Serial.println(F(" time pos angle lfs lss rss rfs cte steer"));
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

  void report_radial_track_header() {
    Serial.println(F(" angle lfs lss rss rfs cte steer"));
    s_start_time = millis();
    s_report_time = s_start_time;
  }
  void report_radial_track(bool use_raw = false) {
    static int recorded_angle = INT16_MAX;
    int this_angle = (int)encoders.robot_angle();
    if (recorded_angle != this_angle) {
      recorded_angle = this_angle;
      print_justified(recorded_angle, 6);
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

  void front_sensor_track_header() { Serial.println(F(" dist front_sum front_diff, distance")); }

  void front_sensor_track() {
    static int position = INT16_MAX;
    if (position != (int)encoders.robot_distance()) {
      position = encoders.robot_distance();
      print_justified(position, 7);
      print_justified(sensors.get_front_sum(), 7);
      print_justified(sensors.get_front_diff(), 7);
      print_justified((int)sensors.get_distance(sensors.get_front_sum(), FRONT_LINEAR_CONSTANT), 7);
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
    Serial.println(F("|           RAW            |          NORMALISED       |             |            |"));
    Serial.println(F("|   lf_   ls_   rs_   rf_  |    lfs   lss   rss   rfs  |   sum diff  | front_dist |"));
  }

  void print_wall_sensors() {
    Serial.print(F("|"));
    print_justified(sensors.lfs.raw, 6);
    print_justified(sensors.lss.raw, 6);
    print_justified(sensors.rss.raw, 6);
    print_justified(sensors.rfs.raw, 6);
    Serial.print(F("  | "));
    print_justified(sensors.lfs.value, 6);
    print_justified(sensors.lss.value, 6);
    print_justified(sensors.rss.value, 6);
    print_justified(sensors.rfs.value, 6);
    Serial.print(F("  | "));
    print_justified(sensors.get_front_sum(), 5);
    print_justified(sensors.get_front_diff(), 5);
    Serial.print(F("  | "));
    print_justified((int)sensors.get_distance(sensors.get_front_sum(), FRONT_LINEAR_CONSTANT), 6);
    Serial.print(F("     | "));
    Serial.println();
  }

  // void

  //***************************************************************************//
  void print_walls() {
    if (sensors.see_left_wall) {
      Serial.print('L');
    } else {
      Serial.print('-');
    }
    if (sensors.see_front_wall) {
      Serial.print('F');
    } else {
      Serial.print('-');
    }
    if (sensors.see_right_wall) {
      Serial.print('R');
    } else {
      Serial.print('-');
    }
  }

  //***************************************************************************//
  /// @private  don't  show this in doxygen output
  void log_status(char action, uint8_t location, uint8_t heading) {
    Serial.print('{');
    Serial.print(action);
    Serial.print(' ');
    print_hex_2(location);
    Serial.print(' ');
    Serial.print(dirLetters[heading]);
    print_justified(sensors.get_front_sum(), 4);
    Serial.print('@');
    print_justified((int)motion.position(), 4);
    Serial.print(' ');
    // print_walls();
    Serial.print('}');
    Serial.print(' ');
  }
  //***************************************************************************//
  void show_adc() {
    while (true) {
      sensors.enable();
      for (int i = 0; i < 4; i++) {
        print_justified(adc.get_raw(i), 5);
        Serial.print(' ');
      }
      for (int i = 6; i < 8; i++) {
        print_justified(adc.get_dark(i), 5);
        Serial.print(' ');
      }
      Serial.println();
      delay(50);
    }
    sensors.disable();
  }
};

#endif