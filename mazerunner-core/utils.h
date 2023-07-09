/******************************************************************************
 * Project: mazerunner-core                                                   *
 * File:    utils.h                                                           *
 * File Created: Friday, 9th September 2022 2:00:47 pm                        *
 * Author: Peter Harrison                                                     *
 * -----                                                                      *
 * Last Modified: Monday, 31st October 2022 10:29:27 pm                       *
 * -----                                                                      *
 * Copyright 2022 - 2022 Peter Harrison, Micromouseonline                     *
 * -----                                                                      *
 * Licence:                                                                   *
 *     Use of this source code is governed by an MIT-style                    *
 *     license that can be found in the LICENSE file or at                    *
 *     https://opensource.org/licenses/MIT.                                   *
 ******************************************************************************/

#ifndef UTILS_H
#define UTILS_H

#include "Arduino.h"
#include "serial.h"

const int MAX_ARGC = 16;
#define MAX_DIGITS 8

struct Args {
  char *argv[MAX_ARGC];
  int argc;
};

// simple formatting functions for printing maze costs
inline void print_hex_2(unsigned char value) {
  if (value < 16) {
    console.print('0');
  }
  console.print(value, HEX);
}

inline void print_justified(int32_t value, int width) {
  int v = value;
  int w = width;
  w--;
  if (v < 0) {
    w--;
  }
  while (v /= 10) {
    w--;
  }
  while (w > 0) {
    console.write(' ');
    --w;
  }
  console.print(value);
}

inline void print_justified(int value, int width) {
  print_justified(int32_t(value), width);
}

/***
 * Scan a character array for an integer.
 * Begin scn at line[pos]
 * Assumes no leading spaces.
 * Stops at first non-digit.
 * MODIFIES pos so that it points to the first non-digit
 * MODIFIES value ONLY IF a valid integer is converted
 * RETURNS  boolean status indicating success or error
 *
 * optimisations are possible but may not be worth the effort
 */
inline uint8_t read_integer(const char *line, int &value) {
  char *ptr = (char *)line;
  char c = *ptr++;
  bool is_minus = false;
  uint8_t digits = 0;
  if (c == '-') {
    is_minus = true;
    c = *ptr++;
  }
  int32_t number = 0;
  while (c >= '0' and c <= '9') {
    if (digits++ < MAX_DIGITS) {
      number = 10 * number + (c - '0');
    }
    c = *ptr++;
  }
  if (digits > 0) {
    value = is_minus ? -number : number;
  }
  return digits;
}

/***
 * Scan a character array for a float.
 * This is a much simplified and limited version of the library function atof()
 * It will not convert exponents and has a limited range of valid values.
 * They should be more than adequate for the robot parameters however.
 * Begin scan at line[pos]
 * Assumes no leading spaces.
 * Only scans MAX_DIGITS characters
 * Stops at first non-digit, or decimal point.
 * MODIFIES pos so that it points to the first character after the number
 * MODIFIES value ONLY IF a valid float is converted
 * RETURNS  boolean status indicating success or error
 *
 * optimisations are possible but may not be worth the effort
 */
uint8_t read_float(const char *line, float &value) {

  char *ptr = (char *)line;
  char c = *ptr++;
  uint8_t digits = 0;

  bool is_minus = false;
  if (c == '-') {
    is_minus = true;
    c = *ptr++;
  }

  uint32_t a = 0.0;
  int exponent = 0;
  while (c >= '0' and c <= '9') {
    if (digits++ < MAX_DIGITS) {
      a = a * 10 + (c - '0');
    }
    c = *ptr++;
  };
  if (c == '.') {
    c = *ptr++;
    while (c >= '0' and c <= '9') {
      if (digits++ < MAX_DIGITS) {
        a = a * 10 + (c - '0');
        exponent = exponent - 1;
      }
      c = *ptr++;
    }
  }
  float b = a;
  while (exponent < 0) {
    b *= 0.1;
    exponent++;
  }
  if (digits > 0) {
    value = is_minus ? -b : b;
  }
  return digits;
}

// Reset
#define VT_TERMINAL_RESET F("\ec") // Wait >10ms after this command before sending any other commands

// Erasing
#define VT_CLR_LINE_AFTER_CURSOR F("\e[K")
#define VT_CLR_LINE_TO_CURSOR F("\e[1K")
#define VT_CLR_LINE F("\e[2K")
#define VT_CLR_SCREEN F("\e[2J")
#define VT_CLR_ALL F("\e[1;1H\e[2J")

// Cursor
#define VT_CURSOR_OFF F("\e[?25l")
#define VT_CURSOR_ON F("\e[?25h")
#define VT_CURSOR_HOME F("\e[H")
#define VT_CURSOR_SAVE F("\e7")
#define VT_CURSOR_RESTORE F("\e8")
#define VT_CURSOR_UP F("\e[A")
#define VT_CURSOR_DOWN F("\e[B")
#define VT_CURSOR_RIGHT F("\e[C")
#define VT_CURSOR_LEFT F("\e[D")

// Text
#define VT_TEXT_DEFAULT F("\e[0m")
#define VT_TEXT_BRIGHT F("\e[1m")
#define VT_TEXT_DIM F("\e[2m")
#define VT_TEXT_UNDERSCORE F("\e[4m")
#define VT_TEXT_BLINK F("\e[5m")
#define VT_TEXT_REVERSE F("\e[7m")
#define VT_TEXT_HIDDEN F("\e[8m")

// Text colors
#define VT_FOREGROUND_BLACK F("\e[30m")
#define VT_FOREGROUND_RED F("\e[31m")
#define VT_FOREGROUND_GREEN F("\e[32m")
#define VT_FOREGROUND_YELLOW F("\e[33m")
#define VT_FOREGROUND_BLUE F("\e[34m")
#define VT_FOREGROUND_MAGNETA F("\e[35m")
#define VT_FOREGROUND_CYAN F("\e[36m")
#define VT_FOREGROUND_WHITE F("\e[37m")
#define VT_FOREGROUND_DEFAULT F("\e[39m")

// Background colors
#define VT_BACKGROUND_BLACK F("\e[40m")
#define VT_BACKGROUND_RED F("\e[41m")
#define VT_BACKGROUND_GREEN F("\e[42m")
#define VT_BACKGROUND_YELLOW F("\e[43m")
#define VT_BACKGROUND_BLUE F("\e[44m")
#define VT_BACKGROUND_MAGNETA F("\e[45m")
#define VT_BACKGROUND_CYAN F("\e[46m")
#define VT_BACKGROUND_WHITE F("\e[47m")
#define VT_BACKGROUND_DEFAULT F("\e[49m")

inline void vt100_cls() {
#ifdef USE_VT100
  console.print(VT_CLR_SCREEN);
  console.print(VT_CURSOR_HOME);
#endif
}

inline void vt100_red() {
#ifdef USE_VT100
  console.print(VT_FOREGROUND_RED);
#endif
}

inline void vt100_default() {
#ifdef USE_VT100
  console.print(VT_FOREGROUND_DEFAULT);
#endif
}

inline void vt100_cursor_off() {
#ifdef USE_VT100
  console.print(VT_CURSOR_OFF);
#endif
}

inline void vt100_cursor_on() {
#ifdef USE_VT100
  console.print(VT_CURSOR_ON);
#endif
}

#endif