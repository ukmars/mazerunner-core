/******************************************************************************
 * Project: mazerunner-core                                                   *
 * File:    mazerunner-core.ino                                               *
 * File Created: Wednesday, 26th October 2022 10:56:33 pm                     *
 * Author: Peter Harrison                                                     *
 * -----                                                                      *
 * Last Modified: Saturday, 26th November 2022 11:06:37 pm                    *
 * -----                                                                      *
 * Copyright 2022 - 2022 Peter Harrison, Micromouseonline                     *
 * -----                                                                      *
 * Licence:                                                                   *
 *     Use of this source code is governed by an MIT-style                    *
 *     license that can be found in the LICENSE file or at                    *
 *     https://opensource.org/licenses/MIT.                                   *
 ******************************************************************************/

#include "cli.h"
#include "config.h"
#include "maze.h"
#include "mouse.h"
#include "reports.h"
#include "src/adc.h"
#include "src/encoders.h"
#include "src/list.h"
#include "src/mazeprinter.h"
#include "src/motion.h"
#include "src/motors.h"
#include "src/sensors.h"
#include "src/serial.h"
#include "src/switches.h"
#include "src/systick.h"
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
  maze.initialise_maze();
  maze.flood_maze(0x77);
  MazePrinter::print_maze(maze, COSTS);
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
