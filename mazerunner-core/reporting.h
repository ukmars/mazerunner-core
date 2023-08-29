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

#ifndef REPORTER_H
#define REPORTER_H

#include <Arduino.h>
#include "encoders.h"
#include "maze.h"
#include "motion.h"
#include "motors.h"
#include "profile.h"
#include "sensors.h"

/***
 * While there is no absolute requirement for reporting in a micromouse or
 * similar robot, it can be of enormous help when setting up and debugging
 * the behaviour of the robot.
 *
 * The Reporter class and its supporting functions provide a number of pre-
 * written reports that let you observe things like controller state and
 * sensor readings. Many of these will be used a lot during development and
 * you can use any of them as a template for custom reports.
 *
 * Most commonly the only output device will be the normal Arduino Serial
 * device. However, if you have a second serial port or some other device such
 * as a serial logger you can divert reporter output to that. The method
 * set_printer() will change the destination for _all_ reporter output. Any
 * interaction on the command line interface will not be affected except, of
 * course a cli command that fires off a report may not see the output if you
 * have changed the reporter print device. You can switch devices as often
 * as you like.
 *
 * I would suggest that any interactive use might leave the reporter output
 * destination as Serial and you could switch it to a logger, for example,
 * at the start of a maze run.
 *
 * Although the Serial device on an Arduino normally requires the USB connection
 * UKMARSBOT has a connector specially intended for a cheap Bluetooth
 * module like the HC-05. This can be left plugged in all the time so that
 * reporting can be done while running. You could also connect a serial
 * logger to this port and record data throughout a contest run.
 *
 * Note that the Serial device has a 64 character buffer and, at 115200 baud
 * 64 characters will take about 6ms to go out over the wire.
 *
 * The BT connector also makes it possible to use the interactive Command
 * Line Interface while the robot is running.
 *
 */
const char dir_letters[] = "FRAL";
const char hdg_letters[] = "NESW";

//***************************************************************************//

/*
// For future expansion, suppose you have a list of operations
// as a set of defines like this:

#define OP_START_MOVE 0x05
#define OP_MAKE_MOVE 0x06
#define OP_STOP 0x07

// you might want to print their names not their values for
// debugging purposes. Use the stringify operator, #, like this

#define case_print(x)      \
  case (x):                \
    Serial.println(F(#x)); \
    break;

// and use a case statement to display the output
// remmeber that each identifier string ends up in flash

void print_name(int operation) {
  switch (operation) {
    case_print(OP_START_MOVE);
    case_print(OP_MAKE_MOVE);
    case_print(OP_STOP);
    default:
      Serial.println(F("UNKNOWN"));
  }
}
*/
//***************************************************************************//

enum MazeView { PLAIN, COSTS, DIRS };
//***************************************************************************//

static Stream& printer = Serial;

class Reporter;
extern Reporter reporter;
class Reporter {
  uint32_t s_start_time;
  uint32_t s_report_time;
  uint32_t s_report_interval = REPORTING_INTERVAL;

 public:
  void set_printer(Stream& stream) {
    printer = stream;
  }

  // simple formatting methods for printing maze costs
  void print_hex_2(unsigned char value) {
    if (value < 16) {
      printer.print('0');
    }
    printer.print(value, HEX);
  }

  void print_justified(int32_t value, int width) {
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
      printer.write(' ');
      --w;
    }
    printer.print(value);
  }

  /**
   * The profile reporter will send out a table of space separated
   * data so that the results can be saved to a file or imported to
   * a spreadsheet or other analyss software.
   *
   * Always send the header first because that restarts the elapsed
   * time count.
   *
   * The data includes
   *   time        - in milliseconds since the header was sent
   *   robotPos    - position in mm as reported by the encoders
   *   robotAngle  - angle in degrees as reported by the encoders
   *   fwdPos      - forward profiler setpoint in mm
   *   fwdSpeed    - forward profiler current speed in mm/s
   *   rotpos      - rotation profiler setpoint in deg
   *   rotSpeed    - rotation profiler current speed in deg/s
   *   fwdmVolts    - voltage sent to the motors for forward control
   *   rotmVolts    - voltage sent to the motors for rotation control
   *
   */
  void report_profile_header() {
    printer.println(F("time robotPos robotAngle fwdPos  fwdSpeed rotpos rotSpeed fwdmVolts rotmVolts"));
    s_start_time = millis();
    s_report_time = s_start_time;
  }

  void report_profile() {
    if (millis() >= s_report_time) {
      s_report_time += s_report_interval;
      print_justified(int(encoders.robot_distance()), 6);
      print_justified(int(encoders.robot_angle()), 6);
      print_justified(int(motion.position()), 6);
      print_justified(int(motion.velocity()), 6);
      print_justified(int(motion.angle()), 6);
      print_justified(int(motion.omega()), 6);
      print_justified(motors.get_fwd_millivolts(), 6);
      print_justified(motors.get_rot_millivolts(), 6);
      printer.println();
    }
  }

  //***************************************************************************//

  /***
   * The full sensor track report can be used to help calibrate the sensors and
   * check the operation of the steering.
   *
   * You could also use it to examine the sensor readings as walls are detected or lost.
   *
   * To avoid sending more data that is needed, the default is to send only the
   * normalised sensor readings. Call with the argument set true to see raw sensor
   * readings for calibration checks.
   *
   */
  void report_sensor_track_header() {
    printer.println(F(" time pos angle lfs lss rss rfs cte steer"));
    s_start_time = millis();
    s_report_time = s_start_time;
  }

  void report_sensor_track(bool use_raw = false) {
    if (millis() >= s_report_time) {
      s_report_time += s_report_interval;
      print_justified(int(millis() - s_start_time), 6);
      print_justified(int(encoders.robot_distance()), 6);
      print_justified(int(encoders.robot_angle()), 6);
      if (use_raw) {
        print_justified(sensors.lfs.raw, 6);
        print_justified(sensors.lss.raw, 6);
        print_justified(sensors.rss.raw, 6);
        print_justified(sensors.rfs.raw, 6);
      } else {
        print_justified(sensors.lfs.value, 6);
        print_justified(sensors.lss.value, 6);
        print_justified(sensors.rss.value, 6);
        print_justified(sensors.rfs.value, 6);
      }
      printer.print(' ');
      printer.print(sensors.get_cross_track_error());
      printer.print(' ');
      printer.print(sensors.get_steering_feedback());
      printer.println();
    }
  }

  void report_radial_track_header() {
    printer.println(F(" angle lfs lss rss rfs cte steer"));
    s_start_time = millis();
    s_report_time = s_start_time;
  }
  void report_radial_track(bool use_raw = false) {
    static int recorded_angle = INT16_MAX;
    int this_angle = (int)encoders.robot_angle();
    if (recorded_angle != this_angle) {
      recorded_angle = this_angle;
      print_justified(recorded_angle, 6);
      if (use_raw) {
        print_justified(sensors.lfs.raw, 6);
        print_justified(sensors.lss.raw, 6);
        print_justified(sensors.rss.raw, 6);
        print_justified(sensors.rfs.raw, 6);
      } else {
        print_justified(sensors.lfs.value, 6);
        print_justified(sensors.lss.value, 6);
        print_justified(sensors.rss.value, 6);
        print_justified(sensors.rfs.value, 6);
      }
      printer.print(' ');
      printer.print(sensors.get_cross_track_error());
      printer.print(' ');
      printer.print(sensors.get_steering_feedback());
      printer.println();
    }
  }

  //***************************************************************************//

  /***
   * This specialised report is meant to be used when calibrating the forward
   * sensors. The robot would be placed up agains a wall ahead and commanded to
   * back up slowly for some fixed distance - 180mm seems appropriate. As it
   * backs up the sensor readings are transmitted so that you can build a table
   * of expected sensor readings against distance from the front wall. This
   * will be useful in setting up things like turn thresholds and to allow the
   * robot to adjust its position in a maze cell.
   *
   * In normal operation, UKMARSBOT has its front edge 22 mm from the wall ahead.
   *
   */

  void front_sensor_track_header() {
    printer.println(F(" dist front_sum front_diff, distance"));
  }

  void front_sensor_track() {
    static int position = INT16_MAX;
    if (position != (int)encoders.robot_distance()) {
      position = encoders.robot_distance();
      print_justified(position, 7);
      print_justified(sensors.get_front_sum(), 7);
      print_justified(sensors.get_front_diff(), 7);
      print_justified((int)sensors.get_distance(sensors.get_front_sum(), FRONT_LINEAR_CONSTANT), 7);
      printer.println();
    }
  }

  //***************************************************************************//

  /***
   * Mostly, you are likely to want to just stream out the current sensor readings
   * to a terminal while you check their values.
   *
   * This report does that, showing the raw and normalised values as well as the
   * sum and difference of the front sensors.
   *
   * This is possibly the most useful streamed information because you can use it
   * with the robot in a maze to get basic calibration values.
   *
   *
   */
  void wall_sensor_header() {
    printer.println(F("|           RAW            |          NORMALISED       |             |            |"));
    printer.println(F("|   lf_   ls_   rs_   rf_  |    lfs   lss   rss   rfs  |   sum diff  | front_dist |"));
  }

  void print_wall_sensors() {
    printer.print(F("|"));
    print_justified(sensors.lfs.raw, 6);
    print_justified(sensors.lss.raw, 6);
    print_justified(sensors.rss.raw, 6);
    print_justified(sensors.rfs.raw, 6);
    printer.print(F("  | "));
    print_justified(sensors.lfs.value, 6);
    print_justified(sensors.lss.value, 6);
    print_justified(sensors.rss.value, 6);
    print_justified(sensors.rfs.value, 6);
    printer.print(F("  | "));
    print_justified(sensors.get_front_sum(), 5);
    print_justified(sensors.get_front_diff(), 5);
    printer.print(F("  | "));
    print_justified((int)sensors.get_distance(sensors.get_front_sum(), FRONT_LINEAR_CONSTANT), 6);
    printer.print(F("     | "));
    printer.println();
  }

  // void

  //***************************************************************************//
  void print_walls() {
    if (sensors.see_left_wall) {
      printer.print('L');
    } else {
      printer.print('-');
    }
    if (sensors.see_front_wall) {
      printer.print('F');
    } else {
      printer.print('-');
    }
    if (sensors.see_right_wall) {
      printer.print('R');
    } else {
      printer.print('-');
    }
  }

  //***************************************************************************//
  /**
   * log_action_status outputs a formatted block of test to the serial device.
   * You could connect the Serial device to a BT radio or datalogger if you want.
   *
   * A typical block might look like this:
   *
   * {F [3,4]  N   227@ 175 }
   *  - ----- ---  ---  ---
   *  |   |    |    |     `---  Robot Position (mm)
   *  |   |    |     `--------  Front Sensor Sum
   *  |   |     `-------------  Heading
   *  |    `------------------  Current Cell
   *   ` ---------------------  Action
   *
   * During the search, several such blocks will be generated in each cell
   *
   * TODO: consider a structure that can be populated during the search loop
   *       and displayed in one line without the repetition of multiple blocks
   */
  /// @private  don't  show this in doxygen output
  //
  void log_action_status(char action, char note, Location location, Heading heading) {
    printer.print('{');
    printer.print(action);
    printer.print(note);
    printer.print('[');
    printer.print(location.x);
    printer.print(',');
    printer.print(location.y);
    printer.print(']');
    printer.print(' ');
    if (heading < HEADING_COUNT) {
      printer.print(hdg_letters[heading]);
    } else {
      printer.print('!');
    }
    print_justified(sensors.get_front_sum(), 4);
    printer.print('@');
    print_justified((int)motion.position(), 4);
    printer.print(' ');
    // print_walls();
    printer.print('}');
    printer.print(' ');
  }

  //***************************************************************************//
  void show_adc() {
    while (true) {
      sensors.enable();
      for (int i = 0; i < 4; i++) {
        print_justified(adc.get_raw(i), 5);
        printer.print(' ');
      }
      for (int i = 6; i < 8; i++) {
        print_justified(adc.get_dark(i), 5);
        printer.print(' ');
      }
      printer.println();
      delay(50);
    }
    sensors.disable();
  }

  //***************************************************************************//
  /**
   * Maze printing.
   *
   */
#define POST 'o'
#define ERR '?'
#define GAP F("   ")
#define H_WALL F("---")
#define H_EXIT F("   ")
#define H_UNKN F("···")
#define H_VIRT F("###")
#define V_WALL '|'
#define V_EXIT ' '
#define V_UNKN ':'
#define V_VIRT '#'

  void print_h_wall(uint8_t state) {
    if (state == EXIT) {
      printer.print(H_EXIT);
    } else if (state == WALL) {
      printer.print(H_WALL);
    } else if (state == VIRTUAL) {
      printer.print(H_VIRT);
    } else {
      printer.print(H_UNKN);
    }
  }
  void printNorthWalls(int y) {
    for (int x = 0; x < MAZE_WIDTH; x++) {
      printer.print(POST);
      WallInfo walls = maze.walls(Location(x, y));
      print_h_wall(walls.north & maze.get_mask());
    }
    printer.println(POST);
  }

  void printSouthWalls(int y) {
    for (int x = 0; x < MAZE_WIDTH; x++) {
      printer.print(POST);
      WallInfo walls = maze.walls(Location(x, y));
      print_h_wall(walls.south & maze.get_mask());
    }
    printer.println(POST);
  }

  void print_maze(int style = PLAIN) {
    const char dirChars[] = "^>v<* ";
    maze.flood(maze.goal());

    for (int y = MAZE_HEIGHT - 1; y >= 0; y--) {
      printNorthWalls(y);
      for (int x = 0; x < MAZE_WIDTH; x++) {
        Location location(x, y);
        WallInfo walls = maze.walls(location);
        uint8_t state = walls.west & maze.get_mask();
        if (state == EXIT) {
          printer.print(V_EXIT);
        } else if (state == WALL) {
          printer.print(V_WALL);
        } else if (state == VIRTUAL) {
          printer.print(V_VIRT);
        } else {
          printer.print(V_UNKN);
        }
        if (style == COSTS) {
          print_justified((int)maze.cost(location), 3);
        } else if (style == DIRS) {
          unsigned char direction = maze.heading_to_smallest(location, NORTH);
          if (location == maze.goal()) {
            direction = DIRECTION_COUNT;
          }
          char arrow = ' ';
          if (direction != BLOCKED) {
            arrow = dirChars[direction];
          }
          printer.print(' ');
          printer.print(arrow);
          printer.print(' ');
        } else {
          printer.print(GAP);
        }
      }
      printer.println(V_WALL);
    }
    printSouthWalls(0);
    printer.println();
  }
};

#endif