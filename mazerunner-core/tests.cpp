/*
 * File: tests.cpp
 * Project: mazerunner
 * File Created: Tuesday, 16th March 2021 10:17:18 pm
 * Author: Peter Harrison
 * -----
 * Last Modified: Wednesday, 14th April 2021 12:59:27 pm
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

#include "tests.h"
#include "encoders.h"
#include "motion.h"
#include "motors.h"
#include "mouse.h"
#include "profile.h"
#include "reports.h"
#include "sensors.h"

//***************************************************************************//

/** TEST 5
 * Used to  calibrate the encoder counts per meter for each wheel.
 *
 * With the robot on the ground, start the test and push the robot in as
 * straight a line as possible over a known distance. 1000mm is best since
 * the encoder calibrations are expressed in counts per meter.
 *
 * Reports left count, right count, distance (mm) and angle (deg)
 *
 * At the end of the move, record the left and right encoder counts and
 * enter them into the configuration settings in config.h
 *
 * The values are likely to be different because the wheels will have
 * slightly different diameters. If you estimate the values in some other
 * way, and use the same value for both wheels, the robot is likely to
 * move in a slight curve instead of a straight line. A later test will let
 * you fine-tune these calibration value to get better straight line motion.
 *
 * Press the function button when done.
 *
 * @brief wheel encoder calibration
 */
void test_calibrate_encoders() {
  reset_drive_system();
  report_encoder_header();
  while (not button_pressed()) {
    report_encoders();
    delay(50);
  }
  report_pose();
}

//***************************************************************************//
/** TEST 8
 * This test wil use the rotation profiler to perform an in-place turn of
 * an integer multiple of 360 degrees. You can use the test to calibrate
 * the MOUSE_RADIUS config setting in the file config.h.
 *
 * There is no point in adjusting MOUSE_RADIUS until you have adjusted
 * the left and right wheel encoder calibration.
 *
 * Test in both left and right directions and adjust the MOUSE_RADIUS to
 * get a reasonable average turn accuracy. The stock motors have a lot of
 * backash so this is never going to be high precision but you should be
 * able to get to +/- a degree or two.
 *
 * Maxumum angular velocity here should not exceed 1000 deg/s or the robot
 * is likely to begin to wander about because the centre of mass is not
 * over the centre of rotation.
 *
 * You can experiment by using the robot_angle instead of the
 * rotation.position() function to get the current angle. The robot_angle
 * is measured from the encoders while rotation.position() is the set
 * value from the profiler. There is no 'correct' way to do this but,
 * if you want repeatable results, always use the same technique.
 *
 * If the robot physical turn angle is less than expected, increase the
 * MOUSE_RADIUS.
 *
 * @brief perform n * 360 degree turn-in-place
 */
void test_spin_turn(float angle) {
  float max_speed = 720.0;     // deg/s
  float acceleration = 4320.0; // deg/s/s
  report_profile_header();
  reset_drive_system();
  enable_motor_controllers();
  spin_turn(angle, max_speed, acceleration);
  reset_drive_system();
}

//***************************************************************************//
/** TEST 9
 *
 * Perform a straight-line movement
 *
 * Two segments are used to illustrate how movement profiles can be
 * concatenated.
 *
 * You can use this test to adjust the encoder calibration so that your
 * robot drives as straight as possible for the correct distance.
 *
 * @brief perform 1000mm forward or reverse move
 */
void test_fwd_move() {
  float distance_a = 3 * FULL_CELL;         // mm
  float distance_b = FULL_CELL + HALF_CELL; // mm
  float max_speed_a = 800.0;                // mm/s
  float common_speed = 300.0;               // mm/s
  float max_speed_b = 500.0;                // mm/s
  float acceleration_a = 2000.0;            // mm/s/s
  float acceleration_b = 1000.0;            // mm/s/s
  reset_drive_system();
  enable_motor_controllers();
  report_profile_header();
  forward.start(distance_a, max_speed_a, common_speed, acceleration_a);
  while (not forward.is_finished()) {
    report_profile();
  }
  forward.start(distance_b, max_speed_b, 0, acceleration_b);
  while (not forward.is_finished()) {
    report_profile();
  }
  reset_drive_system();
}

//***************************************************************************//

//***************************************************************************//

/** TEST 11
 *
 * Illustrates how to combine forward motion with rotation to get a smooth,
 * integrated turn.
 *
 * All the parameters in the call to rotation.start() interact with the
 * forward speed to determine the turn radius
 *
 * @brief move, smooth turn, move sequence
 */
void test_search_turn(float angle) {
  float turn_speed = 300;
  reset_drive_system();
  enable_motor_controllers();
  report_profile_header();
  // it takes only 45mm to get up to speed
  forward.start(300, 800, turn_speed, 1500);
  while (not forward.is_finished()) {
    report_profile();
  }
  rotation.start(angle, 300, 0, 2000);
  while (not rotation.is_finished()) {
    report_profile();
  }
  forward.start(300, 800, 0, 1000);
  while (not forward.is_finished()) {
    report_profile();
  }
  reset_drive_system();
}

//***************************************************************************//

//***************************************************************************//

//***************************************************************************//

//***************************************************************************//

//***************************************************************************//
/**
 * By turning in place through 360 degrees, it should be possible to get a
 * sensor calibration for all sensors?
 *
 * At the least, it will tell you about the range of values reported and help
 * with alignment, You should be able to see clear maxima 180 degrees apart as
 * well as the left and right values crossing when the robot is parallel to
 * walls either side.
 *
 * Use either the normal report_sensor_track() for the normalised readings
 * or report_sensor_track_raw() for the readings straight off the sensor.
 *
 * Sensor sensitivity should be set so that the peaks from raw readings do
 * not exceed about 700-800 so that there is enough headroom to cope with
 * high ambient light levels.
 *
 * @brief turn in place while streaming sensors
 */

void test_sensor_spin_calibrate() {
  enable_sensors();
  delay(100);
  reset_drive_system();
  enable_motor_controllers();
  disable_steering();
  report_sensor_track_header();
  rotation.start(360, 180, 0, 1800);
  while (not rotation.is_finished()) {
    report_sensor_track_raw();
  }
  reset_drive_system();
  disable_sensors();
  delay(100);
}

//***************************************************************************//
/**
 * Edge detection test displays the position at which an edge is found when
 * the robot is travelling down a straight.
 *
 * Start with the robot backed up to a wall.
 * Runs forward for 150mm and records the robot position when the trailing
 * edge of the adjacent wall(s) is found.
 *
 * The value is only recorded to the nearest millimeter to avoid any
 * suggestion of better accuracy than that being available.
 *
 * Note that UKMARSBOT, with its back to a wall, has its wheels 43mm from
 * the cell boundary.
 *
 * This value can be used to permit forward error correction of the robot
 * position while exploring.
 *
 * @brief find sensor wall edge detection positions
 */

void test_edge_detection() {
  bool left_edge_found = false;
  bool right_edge_found = false;
  int left_edge_position = 0;
  int right_edge_position = 0;
  int left_max = 0;
  int right_max = 0;
  enable_sensors();
  delay(100);
  reset_drive_system();
  enable_motor_controllers();
  disable_steering();
  Serial.println(F("Edge positions:"));
  forward.start(FULL_CELL - 30.0, 100, 0, 1000);
  while (not forward.is_finished()) {
    if (g_left_wall_sensor > left_max) {
      left_max = g_left_wall_sensor;
    }

    if (g_right_wall_sensor > right_max) {
      right_max = g_right_wall_sensor;
    }

    if (not left_edge_found) {
      if (g_left_wall_sensor < left_max / 2) {
        left_edge_position = int(0.5 + forward.position());
        left_edge_found = true;
      }
    }
    if (not right_edge_found) {
      if (g_right_wall_sensor < right_max / 2) {
        right_edge_position = int(0.5 + forward.position());
        right_edge_found = true;
      }
    }
    delay(5);
  }
  Serial.print(F("Left: "));
  if (left_edge_found) {
    Serial.print(BACK_WALL_TO_CENTER + left_edge_position);
  } else {
    Serial.print('-');
  }

  Serial.print(F("  Right: "));
  if (right_edge_found) {
    Serial.print(BACK_WALL_TO_CENTER + right_edge_position);
  } else {
    Serial.print('-');
  }
  Serial.println();

  reset_drive_system();
  disable_sensors();
  delay(100);
}
//***************************************************************************//
/** Test runner
 *
 * Runs one of 16 different test routines depending on the settings fthe DIP
 * switches.
 *
 * Custom tests should leave the robot inert. That is, sensors off with drive
 * system reset and shut down.
 *
 * @brief Uses the DIP switches to decide which test to run
 */
void run_test(int test, float arg) {
  switch (test) {
    case 0:
      // ui
      Serial.println(F("OK"));
      break;
    case 1:
      report_sensor_calibration();
      break;
    case 2:
      break;
    case 3:
      break;
    case 4:
      settings = defaults;
      Serial.println(F("OK - Settings cleared to defaults"));
      break;
    case 5:
      test_calibrate_encoders();
      break;
    case 6:
      break;
    case 7:
      break;
    case 8:
      test_spin_turn(360);
      break;
    case 9:
      test_fwd_move();
      break;
    case 10:
      break;
    case 11:
      test_search_turn(90);
      break;
    case 12:
      break;
    case 13:
      break;
    case 14:
      break;
    case 15:
      break;
    case (20):
      test_edge_detection();
      break;
    case (21):
      test_sensor_spin_calibrate();
      break;
    default:
      disable_sensors();
      reset_drive_system();
      break;
  }
}
