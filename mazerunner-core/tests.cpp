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
/** TEST 6/7
 * This test will set the appropriate motion profiler into CONSTANT mod.
 * In that state, you are able to set the speed directly. The test will
 * then generate a cyclic series of speeds and report the actual motion
 * of the robot over a period of 2 seconds.
 *
 * For each kind of motion you can tune the relevant controller constanst
 * to get a smooth and accurate response. You are looking for good
 * tracking of the commanded speed though there will be some delay. The
 * delay should be constant.
 *
 * A well tuned system will have a motor drive voltage that is not too
 * large and does not have large amplitude swings. There will always
 * be some noise in the drive voltage because the encoders have low
 * resolution and the D term does not cope well with that.
 *
 * The controller constants are defined in the config.h file
 *
 * @brief Exercise the motor controllers for tuning of KP and KD
 */
void test_controller_tuning(Profile &profile) {
  reset_drive_system();
  uint32_t duration = 2000;       // milliseconds
  uint32_t period = duration / 2; // 2 cycles
  float max_speed = 800;          // mm/s or deg/s
  enable_motor_controllers();
  profile.set_state(CS_IDLE); // allows dorect setting of speed
  uint32_t start_time = millis();
  uint32_t end_time = start_time + duration;
  report_profile_header();
  while (not button_pressed() && (millis() < end_time)) {
    uint32_t time = millis() - start_time;
    float sinus = sin(2 * PI * time / period); // base pattern
    float speed;                               // degrees per second
    // speed = max_speed * (sinus);                // sinusoid
    // speed = sinus > 0 ? max_speed : -max_speed; // square wave
    speed = (2 * max_speed / PI) * asin(sinus); // triangle
    // speed = max_speed;                          // constant speed
    profile.set_speed(speed);
    report_profile();
  }
  Serial.println();
  reset_drive_system();
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

/** TEST 10
 *
 * @brief move forward n cells, about face, return
 */

void test_sprint_and_return() {
  float distance = 3 * FULL_CELL; // mm
  float max_speed = 1200.0;       // mm/s
  float acceleration = 2000.0;    // mm/s/s
  reset_drive_system();
  enable_motor_controllers();
  report_profile_header();
  forward.start(distance, max_speed, 0, acceleration);
  while (not forward.is_finished()) {
    report_profile();
  }
  turn(-180, 720, 1080);
  forward.start(distance, max_speed, 0, acceleration);
  while (not forward.is_finished()) {
    report_profile();
  }
  reset_drive_system();
}

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
void test_smooth_turn(float angle) {
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

/** TEST 12
 *
 * Profiles finish when the specified command is complete. The motion will,
 * however, continue if the speed is not zero. During that time, the position
 * counter continues to increment.
 *
 * Here a move is started which leaves the robot still moving forwards when it
 * finishes.
 *
 * The robot continues to move for a short time.
 *
 * Then a second move is started with the intention of stopping the robot at a
 * fixed distance from the original move start. This second move fixes the
 * speed at the current value and uses the current acceleration.
 *
 * Experiment with the delay in the middle. You should find that the robot will
 * always stop at the same point even with different delays.
 *
 * Clearly, you could wait so long that it is no longer possible to come to a
 * halt in time.
 *
 * No error checking is done.
 *
 * In motion.cpp, there is a utility function that performs this task.
 *
 * @brief Illustrates stopping at a fixed distance;
 */
void test_stop_at() {
  float initial_distance = 300;
  float steady_speed = 300;
  float final_position = 800;
  float max_speed = 800;
  float acceleration = 1800;
  reset_drive_system();
  enable_motor_controllers();
  report_profile_header();
  forward.start(initial_distance, max_speed, steady_speed, acceleration);
  while (not forward.is_finished()) {
    report_profile();
  }
  uint32_t delay_end = millis() + 100;
  while (millis() < delay_end) {
    report_profile();
  }
  float remaining = final_position - forward.position();
  forward.start(remaining, forward.speed(), 0, forward.acceleration());
  while (not forward.is_finished()) {
    report_profile();
  }
  reset_drive_system();
}
//***************************************************************************//

/** TEST 13
 *
 * Once test 10 (sprint_and_return) are running successfully, it is time to get
 * the steering controls working. This test does the same forward-180-back run
 * that is used in test 10 but has the steering enabled.
 *
 * You will first need to set up the basic sensor reference values as described
 * in the README file.
 *
 * Once that is done, the robot is placed between parallel walls running for
 * as many cells as possible. When the test is started, the robot will run
 * forwards for the specified number of cells turn around and come back.
 *
 * While travelling (including the turn) the sensor values will be streamed
 * over the Serial device so that you can record values using BlueTooth for
 * later analysis.
 *
 * To tune the steering response, you can adjust the settings STEERING_KP
 * and STEERING_KD in config.h. Steering behaviour is achieved by using the
 * sensor cross-track-error to calculate an error angle. This error angle is
 * fed back into the controllers along with the angle obtained from the
 * encoders. The magnitude of the error is limited to the values given in
 * STEERING_ADJUST_LIMIT.
 *
 * It is possible that you will get adequate steering behaviour with only
 * proportional control (STEERING_KD = 0).
 *
 * You are looking for an smooth correction to initial errors in either
 * heading or offset. There should be no oscillation or weaving.
 *
 * Initial setup is done at a constant speed of 800mm/s.
 *
 * @brief run between walls to tune steering behaviour.
 */
void test_sprint_with_steering() {
  // sensor calibration
  float distance = 5 * FULL_CELL; // mm
  float max_speed = 800.0;        // mm/s
  float acceleration = 2000.0;    // mm/s/s
  enable_sensors();
  reset_drive_system();
  enable_steering();
  enable_motor_controllers();
  report_sensor_track_header();
  forward.start(distance, max_speed, 0, acceleration);
  while (not forward.is_finished()) {
    report_sensor_track();
  }
  disable_steering();
  rotation.reset();
  rotation.start(180, 720, 0, 2000);
  while (not rotation.is_finished()) {
    report_sensor_track();
  }
  enable_steering();
  forward.start(distance, max_speed, 0, acceleration);
  while (not forward.is_finished()) {
    report_sensor_track();
  }
  reset_drive_system();
  disable_sensors();
  disable_steering();
}
//***************************************************************************//

/** TEST 14
 *
 *  steering lock test.
 *
 * Place the robot next to a wall or between two walls. It should 'lock' into
 * position so that the steering error is zero.
 *
 * Move the wall(s) and the mouse should track
 *
 * @brief steering tracking test
 */
void test_steering_lock() {
  enable_sensors();
  enable_motor_controllers();
  enable_steering();
  report_sensor_track_header();
  while (not button_pressed()) {
    report_sensor_track();
  }
  wait_for_button_release();
  reset_drive_system();
  disable_sensors();
  delay(100);
}
//***************************************************************************//

/** TEST 15
 *
 *
 */
void test_feedforward(float volts) {
  Serial.print("feedforward test ");
  Serial.println(volts);
  wait_for_button_click();
  reset_drive_system();
  report_reset(2);
  report_encoder_header();
  volts = -volts;
  set_left_motor_volts(volts);
  set_right_motor_volts(volts);
  uint32_t start_time = millis();

  while (millis() - start_time < 1000) {
    report_encoders();
  }
  stop_motors();
}

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
      load_settings_from_eeprom();
      Serial.println(F("OK - Settings read from EEPROM, changes lost"));
      break;
    case 3:
      save_settings_to_eeprom();
      Serial.println(F("OK - Settings written to EEPROM"));
      break;
    case 4:
      settings = defaults;
      Serial.println(F("OK - Settings cleared to defaults"));
      break;
    case 5:
      test_calibrate_encoders();
      break;
    case 6:
      test_controller_tuning(rotation);
      break;
    case 7:
      test_controller_tuning(forward);
      break;
    case 8:
      test_spin_turn(360);
      break;
    case 9:
      test_fwd_move();
      break;
    case 10:
      test_sprint_and_return();
      break;
    case 11:
      test_smooth_turn(90);
      break;
    case 12:
      test_stop_at();
      break;
    case 13:
      test_sprint_with_steering();
      break;
    case 14:
      test_steering_lock();
      break;
    case 15:
      test_feedforward(arg); // arg is the voltage required of the motors
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
