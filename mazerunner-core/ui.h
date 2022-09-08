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
#include "mouse.h"
#include "reports.h"
#include "src/digitalWriteFast.h"
#include "src/sensors.h"
#include "src/utils.h"
#include <Arduino.h>
#include <stdint.h>

// #define MAX_DIGITS 8
const int INPUT_BUFFER_SIZE = 32;

class UI {

  public:
  void interpret_line() {
    Args args = get_tokens();
    switch (args.argc) {
      case 0:
        break;
      case 1:
        if (strlen(args.argv[0]) == 1) {
          run_short_cmd(args);
        }
        break;
      default:
        run_long_cmd(args);
        break;
    }
    clear_input();
    prompt();
  }

  /***
   * Run a complex command. These all start with a string and have
   * arguments. The command string can be a single letter.
   *
   * The arguments will be passed on to the robot.
   *
   * e.g. 'F 4 500 1000 3000' might mean:
   *  - Run function 4 (as if the function switches had been set)
   *  - pass integer arguments 500,1000,3000
   *
   * Most simply, just send 'F n' where n is the function switch value
   *
   */
  void run_long_cmd(const Args args) {
    int function = -1;
    int digits = read_integer(args.argv[1], function);
    if (digits > 0) {
      mouse.execute_cmd(function, args);
    }
  }

  /***
   * Simple commands represented by a single character
   *
   */
  void run_short_cmd(const Args &args) {
    // These are all the single-character commands
    char c = args.argv[0][0];
    switch (c) {
      case '?':
        help();
        break;
      case 'W':
        maze.print_maze_plain();
        break;
      case 'X':
        Serial.println(F("Reset Maze"));
        maze.initialise_maze();
        break;
      case 'R':
        maze.print_maze_with_directions();
        break;
      case 'S':
        sensors.enable_sensors();
        delay(10);
        reporter.report_wall_sensors();
        sensors.disable_sensors();
        break;
      default:
        break;
    }
  }

  void clear_input() {
    m_index = 0;
    m_buffer[m_index] = 0;
  }

  /***
   * Read characters from the serial port into the buffer.
   * return 1 if there is a complete line avaialble
   * return 0 if not
   *
   */
  int read_line() {
    while (Serial.available()) {
      char c = Serial.read();
      // TODO : add single character priority commands like Abort
      if (c == '\n') {
        Serial.println();
        return 1;
      } else if (c == 8) {
        if (m_index > 0) {
          m_buffer[m_index] = 0;
          m_index--;
          Serial.print(c);
          Serial.print(' ');
          Serial.print(c);
        }
      } else if (isPrintable(c)) {
        c = toupper(c);
        Serial.print(c);
        if (m_index < INPUT_BUFFER_SIZE - 1) {
          m_buffer[m_index++] = c;
          m_buffer[m_index] = 0;
        }
      } else {
        // drop the character silently
      }
    }
    return 0;
  }

  Args get_tokens() {
    Args args = {0};
    char *line = m_buffer;
    char *token;
    // special case for the single character settings commands
    if (m_buffer[0] == '$') {
      args.argv[args.argc] = (char *)"$";
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

  void prompt() {
    Serial.print('\n');
    Serial.print('>');
    Serial.print(' ');
  }

  void help() {
    Serial.println(F("W   : display maze walls"));
    Serial.println(F("X   : reset maze"));
    Serial.println(F("R   : display maze with directions"));
    Serial.println(F("S   : show sensor readings"));
    Serial.println(F("F n : Run user function n"));
    Serial.println(F("       0 = ---"));
    Serial.println(F("       1 = Sensor Calibration"));
    Serial.println(F("       2 = Search to the goal and back"));
    Serial.println(F("       3 = Follow a wall to the goal"));
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

  private:
  char m_buffer[INPUT_BUFFER_SIZE];
  uint8_t m_index = 0;
};

extern UI ui;

#endif /* UI_H_ */
