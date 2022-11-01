/******************************************************************************
 * Project: mazerunner-core                                                   *
 * File:    serial.cpp                                                        *
 * File Created: Monday, 31st October 2022 12:42:39 pm                        *
 * Author: Peter Harrison                                                     *
 * -----                                                                      *
 * Last Modified: Tuesday, 1st November 2022 11:22:23 am                      * 
 * -----                                                                      *
 * Copyright 2022 - 2022 Peter Harrison, Micromouseonline                     *
 * -----                                                                      *
 * Licence:                                                                   *
 *     Use of this source code is governed by an MIT-style                    *
 *     license that can be found in the LICENSE file or at                    *
 *     https://opensource.org/licenses/MIT.                                   *
 ******************************************************************************/

#include "serial.h"
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
#if defined(ARDUINO_AVR_NANO) || defined(ARDUINO_AVR_NANO_EVERY)
HardwareSerial &console = Serial;
#elif defined(ARDUINO_AVR_LEONARDO)
/***
 * On the Leonardo platform, Serial is a virtual com port and so
 * useless without a USB connection.
 *
 * Here we ensure that a 'proper' serial connection is made with the
 * Arduino TX and RX pins.
 */
HardwareSerial &console = Serial1;
#elif defined(ARDUINO_ARDUINO_NANO33BLE)
HardwareSerial &console = Serial1;
#else
#error Unsupported hardware
#endif

/***
 * The code also has a NULL serial device. You can use this exactly 
 * like any other serial device but all the calls immediately return 
 * without actually sending any data anywhere.
 * 
 * This is useful for when you want to send debug messages. 
 * 
 * Here a device called debug is connected either to the console or
 * to a null serial device. You can then safely write lines like
 * 
 *   debug.println(F("Starting in safe mode");
 * 
 * and leave them in the code. If the macro USE_DEBUG is not
 * defined, then the statements do nothing.
 * 
 * You might consider having a second serial device connected to a 
 * Bluetooth module and it would then be easy to send some stuff
 * there and other stuff elsewhere. Another option might be to 
 * implement a version of the serial device (it is a descendant of 
 * Stream) that just logs messages to an SD card or flash device.
*/

SerialNull nullConsole;

#ifdef USE_DEBUG
Stream &debug = console;
#else
Stream &debug = nullConsole;
#endif


/***
 * Finally, a little-known provision of the compiler lets you 
 * configure the standard printf() function to print directly
 * to a serial device. There is a cost overhead if you are not 
 * using printf() or sprintf() elsewhere but it is a great convenience
 * if you want formatted printing to the serial port.
 * 
 * To use this facility add a call to redirectPrintf() early in the 
 * setup() function of your code.
 * 
 * TODO: this does not work with the NRF52 compiler. 
*/

#define USE_PRINTF 1
#if defined(USE_PRINTF) && !defined(ARDUINO_ARCH_NRF52840) 
// Function that printf and related will use to print
int serial_putchar(char c, FILE *f) {
  if (c == '\n') {
    // TODO do we need to add carriage returns? I think not.
    console.write('\r');
  }
  return console.write(c) == 1 ? 0 : 1;
}

FILE serial_stdout;
void redirectPrintf() {
  // Redirect stdout so that we can use printf() with the console
  fdev_setup_stream(&serial_stdout, serial_putchar, NULL, _FDEV_SETUP_WRITE);
  stdout = &serial_stdout;
}
#else
void redirectPrintf(){};
#endif
