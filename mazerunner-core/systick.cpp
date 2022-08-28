/*
 * File: systick.cpp
 * Project: vw-control
 * File Created: Monday, 29th March 2021 11:34:26 pm
 * Author: Peter Harrison
 * -----
 * Last Modified: Monday, 5th April 2021 12:05:59 am
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

#include "systick.h"
#include "encoders.h"
#include "motors.h"
#include "profile.h"
#include "sensors.h"
#include <Arduino.h>

void setup_systick() {
  bitClear(TCCR2A, WGM20);
  bitSet(TCCR2A, WGM21);
  bitClear(TCCR2B, WGM22);
  // set divisor to 128 => 125kHz
  bitSet(TCCR2B, CS22);
  bitClear(TCCR2B, CS21);
  bitSet(TCCR2B, CS20);
  OCR2A = 249; // (16000000/128/500)-1 => 500Hz
  bitSet(TIMSK2, OCIE2A);
}

/***
 * This is the SYSTICK ISR. It runs at 500Hz by default.
 *
 * All the time-critical control functions happen in here.
 *
 * interrupts are enabled at the start of the ISR so that encoder
 * counts are not lost.
 *
 * The last thing it does is to start the sensor reads so that they
 * will be ready to use next time around.
 *
 * Timing tests indicate that, with the robot at rest, the systick ISR
 * consumes about 10% of the available system bandwidth.
 *
 * With just a single profile active and moving, that increases to nearly 30%.
 * Two such active profiles increases it to about 35-40%.
 *
 * The reasin that two prifiles does not take up twice as much time is that
 * an active profile has a processing overhead even if there is no motion.
 *
 * Most of the load is due to that overhead. While the profile generates actual
 * motion, there is an additional load.
 *
 *
 */
ISR(TIMER2_COMPA_vect, ISR_NOBLOCK) {
  // TODO: make sure all variables are interrupt-safe if they are used outside IRQs
  // grab the encoder values first because they will continue to change
  update_encoders();
  update_battery_voltage();
  forward.update();
  rotation.update();
  g_cross_track_error = update_wall_sensors();
  g_steering_adjustment = calculate_steering_adjustment(g_cross_track_error);
  update_motor_controllers(g_steering_adjustment);
  start_sensor_cycle();
  // NOTE: no code should follow this line;
}
