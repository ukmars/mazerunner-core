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
#include "battery.h"
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
AnalogueConverter adc;
Battery battery(BATTERY_CHANNEL);
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
FILE serial_stdout;
HardwareSerial &console = Serial;

ISR(ADC_vect) {
  adc.isr();
}

// Systick systick;

ISR(TIMER2_COMPA_vect, ISR_NOBLOCK) {
  systick.update();
}

void setup() {

  console.begin(BAUDRATE);
  // redirectPrintf(); // send printf output to console (uses 20 bytes RAM)
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
  sensors.enable();
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
