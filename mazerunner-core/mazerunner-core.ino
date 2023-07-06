/******************************************************************************
 * Project: mazerunner-core                                                   *
 * File:    mazerunner-core.ino                                               *
 * File Created: Wednesday, 26th October 2022 10:56:33 pm                     *
 * Author: Peter Harrison                                                     *
 * -----                                                                      *
 * Last Modified: Sunday, 27th November 2022 11:13:57 pm                      *
 * -----                                                                      *
 * Copyright 2022 - 2022 Peter Harrison, Micromouseonline                     *
 * -----                                                                      *
 * Licence:                                                                   *
 *     Use of this source code is governed by an MIT-style                    *
 *     license that can be found in the LICENSE file or at                    *
 *     https://opensource.org/licenses/MIT.                                   *
 ******************************************************************************/

#include "adc.h"
#include "cli.h"
#include "config.h"
#include "encoders.h"
#include "list.h"
#include "maze.h"
#include "motion.h"
#include "motors.h"
#include "mouse.h"
#include "reports.h"
#include "sensors.h"
#include "serial.h"
#include "switches.h"
#include "systick.h"
#include <Arduino.h>

// Global objects
Systick systick;

Switches switches(SWITCHES_CHANNEL);
Encoders encoders;
Sensors sensors;
Motion motion;
Motors motors;
Profile forward;
Profile rotation;
Maze maze PERSISTENT;
Mouse mouse;
CommandLineInterface cli;
Reporter reporter;

void setup() {

  console.begin(BAUDRATE);

  // redirectPrintf(); // uncomment to send printf output to console
  console.println((const __FlashStringHelper *)board_name);
  // set up the emitters and groups BEFORE starting the adc
  // group the front sensors
  adc.add_channel_to_group(0, 0);
  adc.add_channel_to_group(3, 0);
  adc.set_emitter_for_group(EMITTER_FRONT, 0);
  // group the side sensors
  adc.add_channel_to_group(1, 1);
  adc.add_channel_to_group(2, 1);
  adc.set_emitter_for_group(EMITTER_DIAGONAL, 1);
  // now configure the hardware
  adc.begin();
  pinMode(LED_USER, OUTPUT);
  digitalWrite(LED_USER, 0);
  pinMode(LED_BUILTIN, OUTPUT);
  motors.setup();
  encoders.setup();
  systick.begin();

  if (switches.button_pressed()) {
    maze.initialise_maze();
    for (int i = 0; i < 4; i++) {
      digitalWrite(LED_BUILTIN, 1);
      delay(50);
      digitalWrite(LED_BUILTIN, 0);
      delay(50);
    }
    console.println(F("Maze cleared"));
    switches.wait_for_button_release();
  }

  sensors.disable();
  console.println(F("RDY"));
}

void loop() {
  if (cli.read_serial() > 0) {
    cli.interpret_line();
  }
  if (switches.button_pressed()) {
    switches.wait_for_button_release();
    mouse.execute_cmd(switches.read());
  }
}
