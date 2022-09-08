/*
 * File: mazerunner.ino
 * Project: mazerunner
 * File Created: Monday, 5th April 2021 8:38:15 am
 * Author: Peter Harrison
 * -----
 * Last Modified: Thursday, 8th April 2021 8:38:41 am
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
#include "config.h"
#include "reports.h"
#include "src/encoders.h"
#include "src/maze.h"
#include "src/motion.h"
#include "src/motors.h"
#include "src/sensors.h"
#include "src/systick.h"
#include "tests.h"
#include "ui.h"
#include "user.h"
#include <Arduino.h>

// Global objects
Systick systick;
Encoders encoders;
Sensors sensors;
Motion motion;
Motors motors;
Profile forward;
Profile rotation;
Maze maze PERSISTENT;
// these are maintained only for logging
float g_left_motor_volts;
float g_right_motor_volts;

void setup() {
  Serial.begin(BAUDRATE);
  systick.begin();
  pinMode(USER_IO_6, OUTPUT);
  pinMode(EMITTER_A, OUTPUT);
  pinMode(EMITTER_B, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  sensors.enable_sensors();
  maze.initialise_maze();
  motors.setup_motors();
  encoders.setup_encoders();
  sensors.setup_adc();
  Serial.println();
  sensors.disable_sensors();
  Serial.println(F("RDY"));
}

void loop() {

  if (Serial.available()) {
    cli_run();
  }
  if (sensors.button_pressed()) {
    sensors.wait_for_button_release();
    int function = sensors.get_switches();
    if (function > 1) {
      sensors.wait_for_user_start(); // cover front sensor with hand to start
    }
    run_mouse(function);
  }
}