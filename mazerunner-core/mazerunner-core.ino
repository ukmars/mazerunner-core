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

#include "encoders.h"
#include "maze.h"
#include "motors.h"
#include "reports.h"
#include "sensors.h"
#include "settings.h"
#include "stopwatch.h"
#include "systick.h"
#include "tests.h"
#include "ui.h"
#include "user.h"
#include <Arduino.h>

void setup() {
  Serial.begin(BAUDRATE);
  load_settings_from_eeprom();
#if ALWAYS_USE_DEFAULT_SETTINGS
  // used during development to make sure compiled-in defaults are used
  restore_default_settings();
#endif
  setup_systick();
  pinMode(USER_IO, OUTPUT);
  pinMode(EMITTER_A, OUTPUT);
  pinMode(EMITTER_B, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  enable_sensors();
  initialise_maze(emptyMaze);
  setup_motors();
  setup_encoders();
  setup_adc();
  Serial.println();
  disable_sensors();
  Serial.println(F("RDY"));
}

void loop() {
  if (Serial.available()) {
    cli_run();
  }
  if (button_pressed()) {
    wait_for_button_release();
    int function = get_switches();
    if (function > 1) {
      wait_for_front_sensor(); // cover front sensor with hand to start
    }
    if (USER_MODE) {
      run_mouse(function);
    } else {
      run_test(function);
    }
  }
}