/******************************************************************************
 * Project: mazerunner-core                                                   *
 * File:    mouse.h                                                           *
 * File Created: Saturday, 10th September 2022 11:24:12 pm                    *
 * Author: Peter Harrison                                                     *
 * -----                                                                      *
 * Last Modified: Wednesday, 2nd November 2022 11:22:21 pm                    *
 * -----                                                                      *
 * Copyright 2022 - 2022 Peter Harrison, Micromouseonline                     *
 * -----                                                                      *
 * Licence:                                                                   *
 *     Use of this source code is governed by an MIT-style                    *
 *     license that can be found in the LICENSE file or at                    *
 *     https://opensource.org/licenses/MIT.                                   *
 ******************************************************************************/

#ifndef MOUSE_H
#define MOUSE_H
#include "Arduino.h"
#include "config.h"
#include "maze.h"
#include "reports.h"
#include "src/encoders.h"
#include "src/motion.h"
#include "src/motors.h"
#include "src/profile.h"
#include "src/sensors.h"
#include "src/serial.h"
#include "src/switches.h"
#include "src/utils.h"

enum MouseState {
  FRESH_START,
  SEARCHING,
  INPLACE_RUN,
  SMOOTH_RUN,
  FINISHED
};

enum TurnType {
  SS90EL = 0,
  SS90ER = 1,
  SS90L = 3,
  SS90R = 3,
};

class Mouse;
extern Mouse mouse;

class Mouse {
  unsigned char heading;
  unsigned char location;
  bool leftWall = false;
  bool frontWall = false;
  bool rightWall = false;
  bool handStart = false;

public:
  Mouse() {
    init();
  }

  void init() {
    handStart = false;
    sensors.set_steering_mode(STEERING_OFF);
    location = 0;
    heading = NORTH;
  }

  void execute_cmd(int cmd) {
    execute_cmd(cmd, Args{0});
  }

  void execute_cmd(int cmd, const Args &args) {
    if (cmd == 0) {
      return;
    }
    sensors.wait_for_user_start(); // cover front sensor with hand to start
    switch (cmd) {
      case 1:
        show_sensor_calibration();
        break;
      case 2:
        search_maze();
        break;
      case 3:
        follow_to(maze.maze_goal());
        break;
      case 4:
        test_SS90E();
        break;
      case 5:
        test_edge_detection();
        break;
      case 6:
        test_sensor_spin_calibrate();
        break;
      default:
        // just to be safe...
        sensors.disable();
        motion.reset_drive_system();
        break;
    }
  }

  /**
   * change the mouse heading but do not physically turn
   */

  void set_heading(unsigned char new_heading) {
    heading = new_heading;
  }

  //***************************************************************************//
  /**
   * Used to bring the mouse to a halt, centered in a cell.
   *
   * If there is a wall ahead, it will use that for a reference to make sure it
   * is well positioned.
   *
   * TODO: the critical values are robot-dependent.
   *
   * TODO: need a function just to adjust forward position
   */
  static void stopAndAdjust() {
    float remaining = (FULL_CELL + HALF_CELL) - forward.position();
    sensors.set_steering_mode(STEERING_OFF);
    forward.start(remaining, forward.speed(), 0, forward.acceleration());
    while (not forward.is_finished()) {
      if (sensors.get_front_sum() > (FRONT_REFERENCE - 150)) {
        break;
      }
      delay(2);
    }
    if (sensors.see_front_wall) {
      while (sensors.get_front_sum() < FRONT_REFERENCE) {
        forward.start(10, 50, 0, 1000);
        delay(2);
      }
    }
  }

  /**
   * These convenience functions will bring the robot to a halt
   * before actually turning.
   */

  void turn_IP180() {
    static int direction = 1;
    direction *= -1; // alternate direction each time it is called
    motion.spin_turn(direction * 180, OMEGA_SPIN_TURN, ALPHA_SPIN_TURN);
  }

  void turn_IP90R() {
    motion.spin_turn(-90, OMEGA_SPIN_TURN, ALPHA_SPIN_TURN);
  }

  void turn_IP90L() {
    motion.spin_turn(90, OMEGA_SPIN_TURN, ALPHA_SPIN_TURN);
  }

  //***************************************************************************//

  /** Search turns
   *
   * These turns assume that the robot is crossing the cell boundary but is still
   * short of the start position of the turn.
   *
   * The turn will be a smooth, coordinated turn that should finish 10mm short of
   * the next cell boundary.
   *
   * Does NOT update the mouse heading but it should
   *
   * TODO: There is only just enough space to get down to turn speed. Increase turn speed to 350?
   *
   */

  void turn_smooth(int turn_id) {
    bool triggered = false;
    sensors.set_steering_mode(STEERING_OFF);
    forward.set_target_speed(SEARCH_TURN_SPEED);
    TurnParameters params = turn_params[turn_id];

    float trigger = params.trigger;
    if (sensors.see_left_wall) {
      trigger += 10; // MAGIC number
    }
    if (sensors.see_right_wall) {
      trigger += 6; // MAGIC number
    }

    float turn_point = FULL_CELL + params.run_in;
    while (forward.position() < turn_point) {
      if (sensors.get_front_sum() > trigger) {
        forward.set_state(PS_FINISHED);
        triggered = true;
        break;
      }
    }
    if (triggered) {
      reporter.log_status('S', location, heading); // the sensors triggered the turn
    } else {
      reporter.log_status('D', location, heading); // the position triggered the turn
    }
    rotation.start(params.angle, params.omega, 0, params.alpha);
    rotation.wait_until_finished();
    forward.start(params.run_out, forward.speed(), SEARCH_SPEED, SEARCH_ACCELERATION);
    forward.wait_until_finished();
    forward.set_position(SENSING_POSITION);
  }

  /**
   * As with all the search turns, this command will be called after the robot has
   * reached the search decision point and decided its next move. It is not known
   * how long that takes or what the exact position will be.
   *
   * Turning around is always going to be an in-place operation so it is important
   * that the robot is stationary and as well centred as possible.
   *
   * It only takes 27mm of travel to come to a halt from normal search speed.
   *
   *
   */
  void turn_around() {
    bool has_wall = frontWall;
    sensors.set_steering_mode(STEERING_OFF);
    reporter.log_status('A', location, heading);
    float remaining = (FULL_CELL + HALF_CELL) - forward.position();
    forward.start(remaining, forward.speed(), 30, forward.acceleration());
    if (has_wall) {
      while (sensors.get_front_sum() < FRONT_REFERENCE) {
        delay(2);
      }
    } else {
      forward.wait_until_finished();
    }
    // Be sure robot has come to a halt.
    forward.stop();
    motion.spin_turn(-180, OMEGA_SPIN_TURN, ALPHA_SPIN_TURN);
    // move from cell center to the sensing point and do not stop
    // TODO: should this be here or in the caller?
    forward.start(SENSING_POSITION - HALF_CELL, SEARCH_SPEED, SEARCH_SPEED, SEARCH_ACCELERATION);
    forward.wait_until_finished();
    forward.set_position(SENSING_POSITION);
  }

  void end_run() {
    bool has_wall = frontWall;
    sensors.set_steering_mode(STEERING_OFF);
    reporter.log_status('T', location, heading);
    float remaining = (FULL_CELL + HALF_CELL) - forward.position();
    forward.start(remaining, forward.speed(), 30, forward.acceleration());
    if (has_wall) {
      while (sensors.get_front_sum() < 850) {
        delay(2);
      }
    } else {
      forward.wait_until_finished();
    }
    reporter.log_status('x', location, heading);
    // Be sure robot has come to a halt.
    forward.stop();
    motion.spin_turn(-180, OMEGA_SPIN_TURN, ALPHA_SPIN_TURN);
  }

  //***************************************************************************//

  void check_the_walls() {
    rightWall = (sensors.see_right_wall);
    leftWall = (sensors.see_left_wall);
    frontWall = (sensors.see_front_wall);
  }

  /***
   * A simple wall follower that knows where it is
   * It will follow the left wall until it reaches the supplied taget
   * cell.
   */
  void follow_to(unsigned char target) {
    console.println(F("Follow TO"));
    handStart = true;
    location = 0;
    heading = NORTH;
    maze.initialise_maze();
    maze.flood_maze(maze.maze_goal());
    // wait_for_user_start();
    delay(1000);
    sensors.enable();
    motion.reset_drive_system();
    forward.start(BACK_WALL_TO_CENTER, SEARCH_SPEED, SEARCH_SPEED, SEARCH_ACCELERATION);
    forward.wait_until_finished();
    forward.set_position(HALF_CELL);
    console.println(F("Off we go..."));
    motion.wait_until_position(FULL_CELL - 10);
    // at the start of this loop we are always at the sensing point
    while (location != target) {
      if (switches.button_pressed()) {
        break;
      }
      console.println();
      reporter.log_status('-', location, heading);
      sensors.set_steering_mode(STEER_NORMAL);
      location = maze.neighbour(location, heading);
      check_the_walls();
      update_map();
      maze.flood_maze(maze.maze_goal());
      unsigned char newHeading = maze.direction_to_smallest(location, heading);
      unsigned char hdgChange = (newHeading - heading) & 0x3;
      console.print(hdgChange);
      console.write(' ');
      console.write('|');
      console.write(' ');
      reporter.log_status('.', location, heading);
      if (location == target) {
        end_run();
      } else if (!leftWall) {
        turn_smooth(SS90EL);
        heading = (heading + 3) & 0x03;
        reporter.log_status('x', location, heading);
      } else if (!frontWall) {
        forward.adjust_position(-FULL_CELL);
        reporter.log_status('F', location, heading);
        motion.wait_until_position(FULL_CELL - 10.0);
        reporter.log_status('x', location, heading);
      } else if (!rightWall) {
        turn_smooth(SS90ER);
        heading = (heading + 1) & 0x03;
        reporter.log_status('x', location, heading);
      } else {
        turn_around();
        heading = (heading + 2) & 0x03;
        reporter.log_status('x', location, heading);
      }
    }
    console.println();
    console.println(F("Arrived!  "));
    for (int i = 0; i < 4; i++) {
      sensors.disable();
      delay(250);
      sensors.enable();
      delay(250);
    }
    sensors.disable();

    motion.reset_drive_system();
  }

  /***
   * The mouse is assumed to be centrally placed in a cell and may be
   * stationary. The current location is known and need not be any cell
   * in particular.
   *
   * The walls for the current location are assumed to be correct in
   * the map.
   *
   * On execution, the mouse will search the maze until it reaches the
   * given target.
   *
   * The maze is mapped as each cell is entered. Mapping happens even in
   * cells that have already been visited. Walls are only ever added, not
   * removed.
   *
   * It is possible for the mapping process to make the mouse think it
   * is walled in with no route to the target.
   *
   * Returns  0  if the search is successful
   *         -1 if the maze has no route to the target.
   */
  int search_to(unsigned char target) {

    maze.flood_maze(target);
    delay(1000);
    sensors.enable();
    motion.reset_drive_system();
    if (not handStart) {
      // back up to the wall behind
      forward.start(-60, 120, 0, 1000);
      forward.wait_until_finished();
    }
    forward.start(BACK_WALL_TO_CENTER, SEARCH_SPEED, SEARCH_SPEED, SEARCH_ACCELERATION);
    forward.wait_until_finished();
    forward.set_position(HALF_CELL);
    console.println(F("Off we go..."));
    motion.wait_until_position(FULL_CELL - 10);
    // TODO. the robot needs to start each iteration at the sensing point
    while (location != target) {
      if (switches.button_pressed()) {
        break;
      }
      console.println();
      reporter.log_status('-', location, heading);
      sensors.set_steering_mode(STEER_NORMAL);
      location = maze.neighbour(location, heading);
      check_the_walls();
      update_map();
      maze.flood_maze(target);
      unsigned char newHeading = maze.direction_to_smallest(location, heading);
      unsigned char hdgChange = (newHeading - heading) & 0x3;
      console.print(hdg_letters[hdgChange]);
      console.write(' ');
      if (location == target) {
        end_run();
        heading = (heading + 2) & 0x03;
      } else {

        switch (hdgChange) {
          case 0: // ahead
            forward.adjust_position(-FULL_CELL);
            reporter.log_status('x', location, heading);
            motion.wait_until_position(FULL_CELL - 10);
            break;
          case 1: // right
            turn_smooth(SS90ER);
            reporter.log_status('x', location, heading);
            heading = (heading + 1) & 0x03;
            break;
          case 2: // behind
            turn_around();
            reporter.log_status('x', location, heading);
            heading = (heading + 2) & 0x03;
            break;
          case 3: // left
            turn_smooth(SS90EL);
            reporter.log_status('x', location, heading);
            heading = (heading + 3) & 0x03;
            break;
        }
      }
    }
    sensors.disable();
    console.println();
    console.println(F("Arrived!  "));
    for (int i = 0; i < 4; i++) {
      digitalWrite(LED_LEFT, 1);
      delay(250);
      digitalWrite(LED_LEFT, 0);
      delay(250);
    }

    motion.reset_drive_system();
    return 0;
  }

  void turn_to_face(unsigned char newHeading) {
    unsigned char hdgChange = (newHeading - heading) & 0x3;
    switch (hdgChange) {
      case 0: // already got it
        break;
      case 1: // right
        turn_IP90R();
        break;
      case 2: // behind
        turn_IP180();
        break;
      case 3: // left
        turn_IP90L();
        break;
    }
    heading = newHeading;
  }

  void update_map() {
    switch (heading) {
      case NORTH:
        if (frontWall) {
          maze.set_wall_present(location, NORTH);
        }
        if (rightWall) {
          maze.set_wall_present(location, EAST);
        }
        if (leftWall) {
          maze.set_wall_present(location, WEST);
        }
        break;
      case EAST:
        if (frontWall) {
          maze.set_wall_present(location, EAST);
        }
        if (rightWall) {
          maze.set_wall_present(location, SOUTH);
        }
        if (leftWall) {
          maze.set_wall_present(location, NORTH);
        }
        break;
      case SOUTH:
        if (frontWall) {
          maze.set_wall_present(location, SOUTH);
        }
        if (rightWall) {
          maze.set_wall_present(location, WEST);
        }
        if (leftWall) {
          maze.set_wall_present(location, EAST);
        }
        break;
      case WEST:
        if (frontWall) {
          maze.set_wall_present(location, WEST);
        }
        if (rightWall) {
          maze.set_wall_present(location, NORTH);
        }
        if (leftWall) {
          maze.set_wall_present(location, SOUTH);
        }
        break;
      default:
        // This is an error. We should handle it.
        break;
    }
    maze.mark_cell_visited(location);
  }

  /***
   * The mouse is expected to be in the start cell heading NORTH
   * The maze may, or may not, have been searched.
   * There may, or may not, be a solution.
   *
   * This simple searcher will just search to goal, turn around and
   * search back to the start. At that point there will be a route
   * but it is unlikely to be optimal.
   *
   * the mouse can run this route by creating a path that does not
   * pass through unvisited cells.
   *
   * A better searcher will continue until a path generated through all
   * cells, regardless of visited state, does not pass through any
   * unvisited cells.
   *
   * The return value is not currently used but could indicate whether
   * the maze is 'solved'. That is, whether there is any need to search
   * further.
   *
   */
  int search_maze() {
    sensors.wait_for_user_start();
    console.println(F("Search TO"));
    handStart = true;
    location = START;
    heading = NORTH;
    search_to(maze.maze_goal());
    handStart = false;
    search_to(START);
    motors.stop();
    return 0;
  }

  //***************************************************************************//
  //************  BELOW HERE ARE VARIOUS TEST FUNCTIONS ***********************//
  //********** THEY ARE NOT ESSENTIAL TO THE BUSINESS OF **********************//
  //******** SOLVING THE MAZE BUT THEY MAY HELP WITH SETUP ********************//
  //***************************************************************************//

  /***
   * just sit in a loop, flashing lights waiting for the button to be pressed
   */
  void panic() {
    while (!switches.button_pressed()) {
      digitalWriteFast(LED_BUILTIN, 1);
      delay(100);
      digitalWriteFast(LED_BUILTIN, 0);
      delay(100);
    }
    switches.wait_for_button_release();
    digitalWriteFast(LED_BUILTIN, 0);
  }

  /***
   * You may want to log the front sensor readings as a function of distance
   * from the wall. This function does that. Place the robot hard up against
   * a wall ahead and run the command. You will get a table of values for
   * the sensors as a function of distance.
   *
   */
  void user_log_front_sensor() {
    sensors.enable();
    motion.reset_drive_system();
    reporter.front_sensor_track_header();
    forward.start(-200, 100, 0, 500);
    while (not forward.is_finished()) {
      reporter.front_sensor_track();
    }
    motion.reset_drive_system();
    sensors.disable();
  }

  /**
   * By turning in place through 360 degrees, it should be possible to get a
   * sensor calibration for all sensors?
   *
   * At the least, it will tell you about the range of values reported and help
   * with alignment, You should be able to see clear maxima 180 degrees apart as
   * well as the left and right values crossing when the robot is parallel to
   * walls either side.
   *
   * Use either the normal report_sensor_track() for the normalised readings
   * or report_sensor_track_raw() for the readings straight off the sensor.
   *
   * Sensor sensitivity should be set so that the peaks from raw readings do
   * not exceed about 700-800 so that there is enough headroom to cope with
   * high ambient light levels.
   *
   * @brief turn in place while streaming sensors
   */

  void test_sensor_spin_calibrate() {
    sensors.enable();
    delay(100);
    motion.reset_drive_system();
    sensors.set_steering_mode(STEERING_OFF);
    reporter.report_sensor_track_header();
    rotation.start(360, 180, 0, 1800);
    while (not rotation.is_finished()) {
      // reporter.report_sensor_track(true);
      reporter.show_wall_sensors();
    }
    motion.reset_drive_system();
    sensors.disable();
    delay(100);
  }

  /**
   * Edge detection test displays the position at which an edge is found when
   * the robot is travelling down a straight.
   *
   * Start with the robot backed up to a wall.
   * Runs forward for 150mm and records the robot position when the trailing
   * edge of the adjacent wall(s) is found.
   *
   * The value is only recorded to the nearest millimeter to avoid any
   * suggestion of better accuracy than that being available.
   *
   * Note that UKMARSBOT, with its back to a wall, has its wheels 43mm from
   * the cell boundary.
   *
   * This value can be used to permit forward error correction of the robot
   * position while exploring.
   *
   * @brief find sensor wall edge detection positions
   */

  void test_edge_detection() {
    bool left_edge_found = false;
    bool right_edge_found = false;
    int left_edge_position = 0;
    int right_edge_position = 0;
    int left_max = 0;
    int right_max = 0;
    sensors.enable();
    delay(100);
    motion.reset_drive_system();
    sensors.set_steering_mode(STEERING_OFF);
    console.println(F("Edge positions:"));
    forward.start(FULL_CELL - 30.0, 100, 0, 1000);
    while (not forward.is_finished()) {
      if (sensors.lss.value > left_max) {
        left_max = sensors.lss.value;
      }

      if (sensors.rss.value > right_max) {
        right_max = sensors.rss.value;
      }

      if (not left_edge_found) {
        if (sensors.lss.value < left_max / 2) {
          left_edge_position = int(0.5 + forward.position());
          left_edge_found = true;
        }
      }
      if (not right_edge_found) {
        if (sensors.rss.value < right_max / 2) {
          right_edge_position = int(0.5 + forward.position());
          right_edge_found = true;
        }
      }
      delay(5);
    }
    console.print(F("Left: "));
    if (left_edge_found) {
      console.print(BACK_WALL_TO_CENTER + left_edge_position);
    } else {
      console.print('-');
    }

    console.print(F("  Right: "));
    if (right_edge_found) {
      console.print(BACK_WALL_TO_CENTER + right_edge_position);
    } else {
      console.print('-');
    }
    console.println();

    motion.reset_drive_system();
    sensors.disable();
    delay(100);
  }

  /***
   * A basic function to let you test the configuration of the SS90Ex turns.
   *
   * These are the turns used during the search of the maze and need to be
   * accurate and repeatable.
   *
   * You may need to spend some time with this function to get the turns
   * just right.
   *
   * NOTE: that the turn parameters are stored in the robot config file
   * NOTE: that the left and right turns are likely to be different.
   *
   */
  void test_SS90E() {
    // note that changes to the speeds are likely to affect
    // the other turn parameters
    uint8_t side = sensors.wait_for_user_start();
    motion.reset_drive_system();
    sensors.set_steering_mode(STEERING_OFF);
    // move to the boundary with the next cell
    float distance = BACK_WALL_TO_CENTER + HALF_CELL;
    forward.start(distance, SEARCH_TURN_SPEED, SEARCH_TURN_SPEED, SEARCH_ACCELERATION);
    forward.wait_until_finished();
    forward.set_position(FULL_CELL);

    if (side == RIGHT_START) {
      turn_smooth(SS90ER);
    } else {
      turn_smooth(SS90EL);
    }
    // after the turn, estimate the angle error by looking for
    // changes in the side sensor readings
    int sensor_left = sensors.lss.value;
    int sensor_right = sensors.rss.value;
    // move two cells. The resting position of the mouse have the
    // same offset as the turn ending
    forward.start(2 * FULL_CELL, SEARCH_TURN_SPEED, 0, SEARCH_ACCELERATION);
    forward.wait_until_finished();
    sensor_left -= sensors.lss.value;
    sensor_right -= sensors.rss.value;
    print_justified(sensor_left, 5);
    print_justified(sensor_right, 5);
    motion.reset_drive_system();
  }

  /***
   * loop until the user button is pressed while
   * pumping out sensor readings. The first four numbers are
   * the raw readings, the next four are normalised then there
   * are two values for the sum and difference of the front sensors
   *
   * The advanced user might use this a s a start for auto calibration
   */
  void show_sensor_calibration() {
    reporter.wall_sensor_header();
    sensors.enable();
    while (not switches.button_pressed()) {
      reporter.show_wall_sensors();
    }
    switches.wait_for_button_release();
    console.println();
    delay(200);
    sensors.disable();
  }
};

#endif // MOUSE_H