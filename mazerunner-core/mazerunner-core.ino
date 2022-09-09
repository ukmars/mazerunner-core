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
#include "maze.h"
#include "mouse.h"
#include "reports.h"
#include "src/encoders.h"
#include "src/motion.h"
#include "src/motors.h"
#include "src/sensors.h"
#include "src/systick.h"
#include "ui.h"
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
Mouse mouse;
UI ui;
Reporter reporter;

void setup() {
  Serial.begin(BAUDRATE);
  systick.begin();
  pinMode(USER_IO_6, OUTPUT);
  pinMode(EMITTER_A, OUTPUT);
  pinMode(EMITTER_B, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  encoders.setup();
  motors.setup_motors();
  encoders.setup_encoders();
  sensors.setup_adc();
  Serial.println();
  sensors.disable();
  Serial.println(F("RDY"));
}

void loop() {
  if (ui.read_line() > 0) {
    ui.interpret_line();
  }
  if (sensors.button_pressed()) {
    sensors.wait_for_button_release();
    mouse.execute_cmd(sensors.get_switches(), Args{0});
  }
}

/**
 * Measurements indicate that even at 1500mm/s thetotal load due to
 * the encoder interrupts is less than 3% of the available bandwidth.
 */

// INT0 will respond to the XOR-ed pulse train from the leftencoder
// runs in constant time of around 3us per interrupt.
// would be faster with direct port access
ISR(INT0_vect) {
  encoders.update_left();
}

// INT1 will respond to the XOR-ed pulse train from the right encoder
// runs in constant time of around 3us per interrupt.
// would be faster with direct port access
ISR(INT1_vect) {
  encoders.update_right();
}

ISR(TIMER2_COMPA_vect, ISR_NOBLOCK) {
  systick.update();
}

ISR(ADC_vect) {
  sensors.update_channel();
}