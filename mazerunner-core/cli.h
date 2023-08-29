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

#ifndef CLI_H_
#define CLI_H_

#include <Arduino.h>
#include <stdint.h>
#include "config.h"
#include "maze.h"
#include "mouse.h"
#include "reporting.h"
#include "sensors.h"

const int MAX_ARGC = 16;
#define MAX_DIGITS 8

/***
 * The Args class is little more than an array of pointers to strings and a counter.
 * Input lines can be parsed into tokens and each token ha its location stored in
 * the array.
 *
 * During command line processing, a temporary instance of Args will be created
 * on the stack and a reference to that passed around to any methds that may
 * need to see the contents. Control eventually returns to the function that
 * created the instance and when that terminates, the information is lost. This
 * means that memory is only used when needed but bear in mind that the instance
 * uses an extra 34 bytes on the stack.
 *
 * For convenience when debugging, there is a method that displays all the tokens
 * on one line.
 *
 */
struct Args {
  char *argv[MAX_ARGC];
  int argc = 0;
  void print() const {
    for (int i = 0; i < argc; i++) {
      Serial.print(argv[i]);
      Serial.print(' ');
    }
    Serial.println();
  }
};

/***
 * Scan a character array for an integer.
 * Begin scan at line[pos]
 * Assumes no leading spaces.
 * Stops at first non-digit.
 * MODIFIES pos so that it points to the first non-digit
 * MODIFIES value ONLY IF a valid integer is converted
 * RETURNS  the number of digits found and converted
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
 * RETURNS  the number of digits found an converted
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

// #define MAX_DIGITS 8
const int INPUT_BUFFER_SIZE = 32;

class CommandLineInterface {
 public:
  /***
   * Read characters from the serial port into the buffer.
   * return true if there is a complete line available
   * return false if not.
   *
   * Input is echoed back through the serial port and can be
   * edited by the user using the backspace key. Accepted
   * characters are converted to upper case for convenience.
   *
   * Lines are terminated with a LINEFEED character which is
   * echoed but not placed in the buffer.
   *
   * All printiable characters are placed in a buffer with a
   * maximum length of just 32 characters. You could make this
   * longer but there should be little practical need.
   *
   * All other characters are ignored.
   *
   */
  const char BACKSPACE = 0x08;
  bool read_serial() {
    while (Serial.available()) {
      char c = Serial.read();
      if (c == '\r') {
        Serial.println();
        return true;
      } else if (c == BACKSPACE) {
        if (m_index > 0) {
          m_buffer[m_index] = 0;
          m_index--;
          Serial.print(c);  // backspace only moves the cursor
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
    return false;
  }

  /***
   * Input lines are parsed into a number of space-separated
   * tokens which are stored in an argument structure, Args.
   *
   * After tokenising, the arguments are examined and processed.
   *
   * Single character commands are handled separately for simplicity.
   *
   * Commands that consist of more than one token have their own
   * handler which executes a function that is passed a reference to
   * the list of tokens as an argument.
   *
   * Once a command line has been dealt with, the input buffer is
   * cleared. That means that new characters that arrive while a
   * function is executing will be lost. A side effect of that
   * is that commands cannot be aborted over the serial link.
   *
   * NOTES:
   *    - serial input is dealt with by polling so you must
   *      frequently check for new input in the main program loop.
   *    - tokenising modifies the input buffer so no extra storage space
   *      is used.
   *
   */
  void interpret_line() {
    Args args;
    if (get_tokens(args) > 0) {
      if (strlen(args.argv[0]) == 1) {
        run_short_cmd(args);
      } else {
        run_long_cmd(args);
      }
    }
    clear_input();
    prompt();
  }

  /***
   * Tokenising is a process where the input buffer is examined
   * for separators that divide one string from another. Here
   * the separator is a space character, a comma or an '=' sign.
   * consecutive separators are treated as one.
   *
   * The start of each separate string is recorded in an array
   * in the Args structure and a null character is placed at its
   * end. In this way, the original string gets to look like a
   * sequence of shorter strings - the tokens. The argv array
   * is a list of where each starts and you can think if it as
   * aan array of strings. The argc element keeps count of how
   * many tokens were found.
   *
   * TODO: this creates an Args instance on the stack then
   * copies it back to the caller which seems wasteful as the stack
   * will grow by two times the size of Args. There should be a
   * better way. Perhaps the caller could create the instance and
   * pass a reference to it.
   *
   *
   */
  int get_tokens(Args &args) {
    char *line = m_buffer;
    char *token;
    args.argc = 0;  // just to be safe
    for (token = strtok(line, " ,="); token != NULL; token = strtok(NULL, " ,=")) {
      args.argv[args.argc] = token;
      args.argc++;
      if (args.argc >= MAX_ARGC)
        break;
    }
    return args.argc;
  }

  /***
   * Run a complex command. These all start with a string with more than
   * one character in ti and have optional arguments.
   *
   * The arguments will be passed on to the robot.
   *
   * e.g. 'TURN 4 500 1000 3000' might mean:
   *  - perform a turn test
   *  - pass integer arguments 500,1000,3000
   *
   */
  void run_long_cmd(const Args args) {
    if (strcmp("HELP", args.argv[0]) == 0) {
      help();
    } else if (strcmp("SEARCH", args.argv[0]) == 0) {
      int x = 0;
      int y = 0;
      if (!read_integer(args.argv[1], x)) {
        x = 7;
      };

      if (!read_integer(args.argv[2], y)) {
        y = 7;
      };
      mouse.search_to(Location(x, y));
    }
  }

  /***
   * Simple commands represented by a single character
   *
   */
  void run_short_cmd(const Args &args) {
    char c = args.argv[0][0];
    switch (c) {
      case '?':
        help();
        break;
      case 'X':
        Serial.println(F("Reset Maze"));
        maze.initialise();
        break;
      case 'W':
        reporter.print_maze(PLAIN);
        break;
      case 'C':
        reporter.print_maze(COSTS);
        break;
      case 'D':
        reporter.print_maze(DIRS);
        break;
      case 'B':
        Serial.print(F("Battery: "));
        Serial.print(battery.voltage(), 2);
        Serial.print(F(" Volts\n"));
        break;
      case 'S':
        sensors.enable();
        delay(10);
        reporter.print_wall_sensors();
        sensors.disable();
        break;
      case 'F': {
        // simulate the function switches
        int function = -1;
        int digits = read_integer(args.argv[1], function);
        if (digits) {
          run_function(function);
        }
      }
      default:
        break;
    }
  }

  /**
   * These are the actions associated with the function switches on UKMARSBOT
   *
   * The switches are set to one of 16 combinations. Combination 0 (zero) is
   * ignored so that there is a 'safe' state where the robot will not do anything
   * unexpected.
   *
   * The function can handle any number of additional functions if called as a
   * result of a command line input like 'F 45'. Otherwise, a command like 'F 1'
   * is equivalent to selecting 1 on te function switches and then pressing
   * the 'start' button.
   *
   */
  void run_function(int cmd) {
    if (cmd == 0) {
      return;
    }
    switch (cmd) {
      case 1:
        mouse.show_sensor_calibration();
        break;
      case 2:
        mouse.search_maze();
        break;
      case 3: {
        mouse.follow_to(maze.goal());
      } break;
      case 4:
        mouse.test_SS90E();
        break;
      case 5:
        // mouse.test_SS90F(); // not implemented
        break;
      case 6:
        mouse.conf_edge_detection();
        break;
      case 7:
        mouse.conf_sensor_spin_calibrate();
        break;
      case 8:
        mouse.conf_log_front_sensor();
        break;
      default:
        // just to be safe...
        sensors.disable();
        motion.reset_drive_system();
        sensors.set_steering_mode(STEERING_OFF);
        break;
    }
  }

  void clear_input() {
    m_index = 0;
    m_buffer[m_index] = 0;
  }

  void prompt() {
    Serial.println();
    Serial.print('>');
    Serial.print(' ');
  }

  /***
   * You may add a help text here but remember to keep it in
   * sync with what the robot acually does.
   *
   */
  void help() {
    Serial.println(F("?   : this text"));
    Serial.println(F("X   : reset maze"));
    Serial.println(F("W   : display maze walls"));
    Serial.println(F("C   : display maze costs"));
    Serial.println(F("D   : display maze with directions"));
    Serial.println(F("B   : show battery voltage"));
    Serial.println(F("S   : show sensor readings"));
    Serial.println(F("F n : Run user function n"));
    Serial.println(F("       0 = ---"));
    Serial.println(F("       1 = Sensor Static Calibration"));
    Serial.println(F("       2 = Search to the goal and back"));
    Serial.println(F("       3 = Follow a wall to the goal"));
    Serial.println(F("       4 = Test SS90E Turn"));
    Serial.println(F("       5 = Test SS90F Turn"));
    Serial.println(F("       6 = Test Edge Detect Position"));
    Serial.println(F("       7 = Sensor Spin Calibration"));
    Serial.println(F("       8 = "));
    Serial.println(F("       9 = "));
    Serial.println(F("      10 = "));
    Serial.println(F("      11 = "));
    Serial.println(F("      12 = "));
    Serial.println(F("      13 = "));
    Serial.println(F("      14 = "));
    Serial.println(F("      15 = "));
    Serial.println(F("SEARCH x y : search to location (x,y)"));
    Serial.println(F("HELP       : this text"));
  }

 private:
  char m_buffer[INPUT_BUFFER_SIZE];
  uint8_t m_index = 0;
};

extern CommandLineInterface cli;

#endif /* UI_H_ */
