/******************************************************************************
 * Project: mazerunner-core                                                   *
 * File:    serialnull.h                                                      *
 * File Created: Monday, 31st October 2022 12:46:17 pm                        *
 * Author: Peter Harrison                                                     *
 * -----                                                                      *
 * Last Modified: Monday, 31st October 2022 10:13:20 pm                       * 
 * -----                                                                      *
 * Copyright 2022 - 2022 Peter Harrison, Micromouseonline                     *
 * -----                                                                      *
 * Licence:                                                                   *
 *     Use of this source code is governed by an MIT-style                    *
 *     license that can be found in the LICENSE file or at                    *
 *     https://opensource.org/licenses/MIT.                                   *
 ******************************************************************************/
#pragma once

#include "Arduino.h"

class NullSerial : public Stream {
private:
public:
  // public methods
  NullSerial() {}
  ~NullSerial() {}

  void begin(long speed) {
    (void)speed;
  }

  // These need to be implemented for a concrete
  // class because Stream is pure virtual

  /***
   * Disables serial communication, allowing the RX and TX pins to be
   * used for general input and output.
   * To re-enable serial communication, call Serial.begin().
   */
  virtual void end() {}

  /***
   * Returns the next byte (character) of incoming serial data without
   * removing it from the internal serial buffer. That is, successive
   * calls to peek() will return the same character, as will the next
   * call to read().
   * Serial.peek() inherits from the Stream utility class.
   *
   * RETURN: The first byte of incoming serial data available
   *         (or -1 if no data is available). Data type: int.
   */
  virtual int peek() {
    return -1;
  }

  /***
   * Writes binary data to the serial port. This data is sent as a byte
   * or series of bytes; to send the characters representing the digits
   * of a number use the print() function instead.
   *
   * RETURN: will return the number of bytes written, though reading that
   *         number is optional. Data type: size_t.
   */
  virtual size_t write(unsigned char byte) {
    (void)byte;
    return 0;
  }
  size_t write(const char *str) {
    (void)str;
    return 0;
  }
  size_t write(const uint8_t *buffer, size_t size) {
    (void)buffer;
    (void)size;
    return 0;
  }

  /***
   * Reads incoming serial data - probably from a buffer
   *
   * RETURN: first byte of incoming data or -1 if none is available
   *         (EOF is probably defined as -1 in many implementations)
   */
  virtual int read() {
    return -1;
  }

  /***
   * Get the number of bytes (characters) available for reading from
   * the serial port. This is data that’s already arrived and stored
   * in the serial receive buffer (which holds 64 bytes).
   *
   * Serial.available() inherits from the Stream utility class.
   *
   * RETURN: the number of bytes in the buffer.
   */
  virtual int available() {
    return 0;
  }

  /***
   * Waits for the transmission of outgoing serial data to complete.
   * (Prior to Arduino 1.0, this instead removed any buffered
   * incoming serial data.)
   *
   * flush() inherits from the Stream utility class.
   *
   * RETURN: nothing
   */
  virtual void flush() {}

  // public only for easy access by interrupt handlers
  void handle_interrupt();
};