/*
 * File: ui.h
 * Project: mazerunner
 * File Created: Sunday, 28th March 2021 2:44:57 pm
 * Author: Peter Harrison
 * -----
 * Last Modified: Monday, 5th April 2021 2:59:37 pm
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

#ifndef UI_H_
#define UI_H_

#include "config.h"
#include "maze.h"
#include "reports.h"
#include "src/digitalWriteFast.h"
#include "src/sensors.h"
#include "user.h"
#include <Arduino.h>
#include <stdint.h>

#define MAX_DIGITS 8

const int MAX_ARGC = 16;
struct Args {
  const char *argv[MAX_ARGC];
  int argc;
};

const int INPUT_BUFFER_SIZE = 32;

// These are the error codes produced by commands to pass into interpreter error.
enum {
  T_SILENT_ERROR = -1, // Special error code designed for ?, h and unimplemented commands that report their own errors.
  T_OK = 0,            // Normal 'no error' return code.
  T_OUT_OF_RANGE = 1,
  T_READ_NOT_SUPPORTED = 2,
  T_LINE_TOO_LONG = 3,
  T_UNKNOWN_COMMAND = 4,
  T_UNEXPECTED_TOKEN = 5
};

class UI {

  public:
  char s_input_line[INPUT_BUFFER_SIZE];
  uint8_t s_index = 0;

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
  uint8_t read_integer(const char *line, int &value) {
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
  //***************************************************************************//

  int cli_run_user(const Args args) {
    if (args.argc < 2) {
      run_mouse(sensors.get_switches());
      return T_OK;
    }
    int test_number = -1;
    read_integer(args.argv[1], test_number);
    if (test_number < 0) {
      return T_UNEXPECTED_TOKEN;
    }
    run_mouse(test_number);
    return T_OK;
  }

  void cli_clear_input() {
    s_index = 0;
    s_input_line[s_index] = 0;
  }

  int cli_read_line() {
    while (Serial.available()) {
      char c = Serial.read();
      // TODO : add single character priority commands like Abort
      if (c == '\n') {
        Serial.println();
        return 1;
      } else if (c == 8) {
        if (s_index > 0) {
          s_input_line[s_index] = 0;
          s_index--;
          Serial.print(c);
          Serial.print(' ');
          Serial.print(c);
        }
      } else if (isPrintable(c)) {
        c = toupper(c);
        Serial.print(c);
        if (s_index < INPUT_BUFFER_SIZE - 1) {
          s_input_line[s_index++] = c;
          s_input_line[s_index] = 0;
        }
      } else {
        // drop the character silently
      }
    }
    return 0;
  }

  Args cli_split_line() {
    Args args = {0};
    char *line = s_input_line;
    char *token;
    // special case for the single character settings commands
    if (s_input_line[0] == '$') {
      args.argv[args.argc] = "$";
      args.argc++;
      line++;
    }
    for (token = strtok(line, " ,="); token != NULL; token = strtok(NULL, " ,=")) {
      args.argv[args.argc] = token;
      args.argc++;
      if (args.argc == MAX_ARGC)
        break;
    }
    return args;
  }

  void cli_prompt() {
    Serial.print('\n');
    Serial.print('>');
    Serial.print(' ');
  }

  void cli_help() {
    Serial.println(F("$   : settings"));
    Serial.println(F("W   : display maze walls"));
    Serial.println(F("X   : reset maze"));
    Serial.println(F("R   : display maze with directions"));
    Serial.println(F("S   : show sensor readings"));
    Serial.println(F("U n : Run user function n"));
    Serial.println(F("       0 = ---"));
    Serial.println(F("       1 = "));
    Serial.println(F("       2 = "));
    Serial.println(F("       3 = "));
    Serial.println(F("       4 = "));
    Serial.println(F("       5 = "));
    Serial.println(F("       6 = "));
    Serial.println(F("       7 = "));
    Serial.println(F("       8 = "));
    Serial.println(F("       9 = "));
    Serial.println(F("      10 = "));
    Serial.println(F("      11 = "));
    Serial.println(F("      12 = "));
    Serial.println(F("      13 = "));
    Serial.println(F("      14 = "));
    Serial.println(F("      15 = "));
  }

  void cli_interpret(const Args &args) {
    if (strlen(args.argv[0]) == 1) {
      // These are all single-character commands
      char c = args.argv[0][0]; //  first character of first token
      switch (c) {
        case '?':
          cli_help();
          break;
        case '$':
          break;
        case 'W':
          print_maze_plain();
          break;
        case 'X':
          Serial.println(F("Reset Maze"));
          maze.initialise_maze();
          break;
        case 'R':
          print_maze_with_directions();
          break;
        case 'S':
          sensors.enable_sensors();
          delay(10);
          report_wall_sensors();
          sensors.disable_sensors();
          break;
        case 'U':
          cli_run_user(args);
          break;
        default:
          break;
      }
      return;
    }
    // parse multi-character commands here
    // TODO - remove these debugging lines
    for (int i = 0; i < args.argc; i++) {
      Serial.println(args.argv[i]);
    }
  }

  void cli_run() {
    if (cli_read_line() > 0) {
      Args args = cli_split_line();
      cli_interpret(args);
      cli_clear_input();
      cli_prompt();
    }
  }

  /***
   * just sit in a loop, flashing lights waiting for the button to be pressed
   */
  void panic() {
    while (!sensors.button_pressed()) {
      digitalWriteFast(LED_BUILTIN, 1);
      delay(100);
      digitalWriteFast(LED_BUILTIN, 0);
      delay(100);
    }
    sensors.wait_for_button_release();
    digitalWriteFast(LED_BUILTIN, 0);
  }
};
#endif /* UI_H_ */
