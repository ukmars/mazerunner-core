/******************************************************************************
 * Project: mazerunner-core                                                   *
 * -----                                                                      *
 * Copyright 2022 - 2023 Peter Harrison, Micromouseonline                     *
 * -----                                                                      *
 * Licence:                                                                   *
 *     Use of this source code is governed by an MIT-style                    *
 *     license that can be found in the LICENSE file or at                    *
 *     https://opensource.org/licenses/MIT.                                   *
 ******************************************************************************/

#pragma once

#include <Arduino.h>

/***
 * Provide a way to talk to the serial port without knowing exactly
 * what it is called.
 *
 * While most Arduino implementations provide a Serial object, that
 * is not always the case though it is hard to see why.
 *
 * Also, some targets are able to provide multiple serial ports
 * like Serial1, Serial2,...
 *
 * Here the basic default serial device is referred to everywhere as
 * console and the little bit of conditional compilation below
 * lets you decide which actual serial device it refers to.
 *
 * TODO: Probably better to make the decision in the board config file
 *
 *
 */
extern HardwareSerial &console;

/***
 * A little-known provision of the compiler lets you
 * configure the standard printf() function to print directly
 * to a serial device. There is a cost overhead if you are not
 * using printf() or sprintf() elsewhere but it is a great convenience
 * if you want formatted printing to the serial port.
 *
 * To use this facility add a call to redirectPrintf() early in the
 * setup() function of your code.
 *
 */

extern FILE serial_stdout;
extern void redirectPrintf();
// Function that printf and related will use to print
inline int serial_putchar(char c, FILE *f) {
  return console.write(c) == 1 ? 0 : 1;
}

inline void redirectPrintf() {
  // Redirect stdout so that we can use printf() with the console
  fdev_setup_stream(&serial_stdout, serial_putchar, NULL, _FDEV_SETUP_WRITE);
  stdout = &serial_stdout;
}
