/******************************************************************************
 * Project: mazerunner-core                                                   *
 * File:    mazerunner-core.ino                                               *
 * File Created: Wednesday, 26th October 2022 10:56:33 pm                     *
 * Author: Peter Harrison                                                     *
 * -----                                                                      *
 * Last Modified: Thursday, 27th October 2022 4:34:04 pm                      *
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
#include "src/motion.h"
#include "src/motors.h"
#include "src/sensors.h"
#include "src/switches.h"
#include "src/systick.h"
#include <Arduino.h>

#ifdef ARDUINO_ARCH_MEGAAVR
#warning this is an Arduino Nano Every ATMEGA4809?
#endif
#ifdef ARDUINO_ARCH_NRF52840
#warning this is an Arduino Nano 33 something with the NRF52840 processor?
#endif
#ifdef ARDUINO_ARCH_AVR
#warning this is an Arduino Nano or similar with the ATMega328P processor?
#if defined(__AVR_ATmega328P__)
#warning ATMEGA328P
#endif
#if defined(__AVR_ATmega328__)
#warning ATMEGA328
#endif

#endif

// Global objects
Systick systick;
adc_atmega328 adc; // TODO: this needs to get defineed in a hardware file not here

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
  Serial.begin(BAUDRATE);
  // group the front sensors
  adc.add_channel_to_group(0, 0);
  adc.add_channel_to_group(3, 0);
  adc.set_emitter_for_group(EMITTER_FRONT, 0);
  // group the side sensors
  adc.add_channel_to_group(1, 1);
  adc.add_channel_to_group(2, 1);
  adc.set_emitter_for_group(EMITTER_DIAGONAL, 1);

  adc.begin();

  systick.begin();
  pinMode(USER_IO_6, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  motors.setup();
  encoders.setup();
  Serial.print('-');
  if (switches.button_pressed()) {
    maze.initialise_maze();
    for (int i = 0; i < 4; i++) {
      digitalWrite(LED_BUILTIN, 1);
      delay(50);
      digitalWrite(LED_BUILTIN, 0);
      delay(50);
    }
    Serial.println(F("Maze cleared"));
    switches.wait_for_button_release();
  }

  sensors.disable();
  Serial.println(F("RDY"));
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
