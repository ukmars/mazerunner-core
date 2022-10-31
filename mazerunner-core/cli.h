/******************************************************************************
 * Project: mazerunner-core                                                   *
 * File:    cli.h                                                             *
 * File Created: Saturday, 10th September 2022 10:58:08 pm                    *
 * Author: Peter Harrison                                                     *
 * -----                                                                      *
 * Last Modified: Monday, 31st October 2022 4:29:38 pm                        *
 * -----                                                                      *
 * Copyright 2022 - 2022 Peter Harrison, Micromouseonline                     *
 * -----                                                                      *
 * Licence:                                                                   *
 *     Use of this source code is governed by an MIT-style                    *
 *     license that can be found in the LICENSE file or at                    *
 *     https://opensource.org/licenses/MIT.                                   *
 ******************************************************************************/

#ifndef CLI_H_
#define CLI_H_

#include "config.h"
#include "maze.h"
#include "mouse.h"
#include "reports.h"
#include "src/sensors.h"
#include "src/serial.h"
#include "src/utils.h"
#include <Arduino.h>
#include <stdint.h>

// #define MAX_DIGITS 8
const int INPUT_BUFFER_SIZE = 32;

class CommandLineInterface {

public:
  /***
   * Read characters from the serial port into the buffer.
   * return 1 if there is a complete line avaialble
   * return 0 if not.
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
  int read_serial() {
    while (console.available()) {
      char c = console.read();
      if (c == '\n') {
        console.println();
        return 1;
      } else if (c == BACKSPACE) {
        if (m_index > 0) {
          m_buffer[m_index] = 0;
          m_index--;
          console.print(c); // backspace only moves the cursor
          console.print(' ');
          console.print(c);
        }
      } else if (isPrintable(c)) {
        c = toupper(c);
        console.print(c);
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

  /***
   * Input lines are parsed into a number of space-separated
   * tokens which are stored in an argument structure, Args.
   *
   * This structure has an integer count of the number of tokens
   * and an array of the token values as strings.
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
   * cleared, that means that new characters that arrive while a
   * function is executing will be lost. A side effect of that
   * is that commands cannot be aborted over the serial link.
   *
   * NOTES:
   *    - serial input is dealt with by polling so you must
   *      frequently check for new input in the main program loop.
   *    - tokenising uses the input buffer so no extra storage space
   *      is used.
   *
   */
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
   * If you wanted to list all the tokens after processing, you
   * would just use the code:
   *
   *      for (int i = 0; i < args.argc; i++) {
   *        console.println(args.argv[i]);
   *      }
   *
   */
  Args get_tokens() {
    Args args = {0};
    char *line = m_buffer;
    char *token;
    for (token = strtok(line, " ,="); token != NULL; token = strtok(NULL, " ,=")) {
      args.argv[args.argc] = token;
      args.argc++;
      if (args.argc == MAX_ARGC)
        break;
    }
    // for (int i = 0; i < args.argc; i++) {
    //   console.println(args.argv[i]);
    // }
    return args;
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
        maze.print_plain();
        break;
      case 'X':
        console.println(F("Reset Maze"));
        maze.initialise_maze();
        break;
      case 'C':
        maze.print_with_costs();
        break;
      case 'R':
        maze.print_with_directions();
        break;
      case 'S':
        sensors.enable();
        delay(10);
        reporter.show_wall_sensors();
        sensors.disable();
        break;
      default:
        break;
    }
  }

  void clear_input() {
    m_index = 0;
    m_buffer[m_index] = 0;
  }

  void prompt() {
    console.print('\n');
    console.print('>');
    console.print(' ');
  }

  /***
   * You may add a help text here but remember to keep it in
   * sync with what the robot acually does.
   *
   */
  void help() {
    console.println(F("W   : display maze walls"));
    console.println(F("X   : reset maze"));
    console.println(F("R   : display maze with directions"));
    console.println(F("S   : show sensor readings"));
    console.println(F("F n : Run user function n"));
    console.println(F("       0 = ---"));
    console.println(F("       1 = Sensor Calibration"));
    console.println(F("       2 = Search to the goal and back"));
    console.println(F("       3 = Follow a wall to the goal"));
    console.println(F("       4 = "));
    console.println(F("       5 = "));
    console.println(F("       6 = "));
    console.println(F("       7 = "));
    console.println(F("       8 = "));
    console.println(F("       9 = "));
    console.println(F("      10 = "));
    console.println(F("      11 = "));
    console.println(F("      12 = "));
    console.println(F("      13 = "));
    console.println(F("      14 = "));
    console.println(F("      15 = "));
  }

private:
  char m_buffer[INPUT_BUFFER_SIZE];
  uint8_t m_index = 0;
};

extern CommandLineInterface cli;

#endif /* UI_H_ */
