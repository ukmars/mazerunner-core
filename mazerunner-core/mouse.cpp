/*
 * File: mouse.cpp
 * Project: mazerunner
 * File Created: Friday, 23rd April 2021 9:09:10 am
 * Author: Peter Harrison
 * -----
 * Last Modified: Thursday, 6th May 2021 9:20:34 am
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
#include "mouse.h"
#include "Arduino.h"
#include "config.h"
#include "reports.h"
#include "src/encoders.h"
#include "src/maze.h"
#include "src/motion.h"
#include "src/motors.h"
#include "src/profile.h"
#include "src/sensors.h"
#include "ui.h"

Mouse mouse;

char path[128];
char commands[128];
char p_mouse_state __attribute__((section(".noinit")));

static char dirLetters[] = "NESW";

void print_walls() {
  if (g_lss_has_wall) {
    Serial.print('L');
  } else {
    Serial.print('-');
  }
  if (g_front_has_wall) {
    Serial.print('F');
  } else {
    Serial.print('-');
  }
  if (g_rss_has_wall) {
    Serial.print('R');
  } else {
    Serial.print('-');
  }
}
//***************************************************************************//
/**
 * Used to bring the mouse to a halt, centred in a cell.
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
  set_steering_mode(STEERING_OFF);
  forward.start(remaining, forward.speed(), 0, forward.acceleration());
  while (not forward.is_finished()) {
    if (g_front_sum > (FRONT_REFERENCE - 150)) {
      break;
    }
    delay(2);
  }
  if (g_front_has_wall) {
    while (g_front_sum < FRONT_REFERENCE) {
      forward.start(10, 50, 0, 1000);
      delay(2);
    }
  }
}

/**
 * These convenience functions only perform the turn
 */

void turnIP180() {
  static int direction = 1;
  direction *= -1; // alternate direction each time it is called
  motion.spin_turn(direction * 180, OMEGA_MAX_SPIN_TURN, ALPHA_SPIN_TURN);
}

void turn_IP90R() {
  motion.spin_turn(-90, OMEGA_MAX_SPIN_TURN, ALPHA_SPIN_TURN);
}

void turn_IP90L() {
  motion.spin_turn(90, OMEGA_MAX_SPIN_TURN, ALPHA_SPIN_TURN);
}

//***************************************************************************//

void Mouse::end_run() {
  bool has_wall = frontWall;
  set_steering_mode(STEERING_OFF);
  log_status('T');
  float remaining = (FULL_CELL + HALF_CELL) - forward.position();
  forward.start(remaining, forward.speed(), 30, forward.acceleration());
  if (has_wall) {
    while (g_front_sum < 850) {
      delay(2);
    }
  } else {
    while (not forward.is_finished()) {
      delay(2);
    }
  }
  log_status('x');
  // Be sure robot has come to a halt.
  forward.stop();
  motion.spin_turn(-180, OMEGA_MAX_SPIN_TURN, ALPHA_SPIN_TURN);
}

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

void Mouse::turn_smooth(int turn_id) {
  bool triggered = false;
  set_steering_mode(STEERING_OFF);
  forward.set_target_speed(DEFAULT_TURN_SPEED);

  float trigger = turn_params[turn_id].trigger;
  if (g_lss_has_wall) {
    trigger += 10;
  }
  if (g_rss_has_wall) {
    trigger += 6;
  }

  float turn_point = FULL_CELL + turn_params[turn_id].run_in;
  while (forward.position() < turn_point) {
    if (g_front_sum > trigger) {
      forward.set_state(CS_FINISHED);
      triggered = true;
      break;
    }
  }
  if (triggered) {
    log_status('S');
  } else {
    log_status('D');
  }
  rotation.start(turn_params[turn_id].angle, turn_params[turn_id].omega, 0, turn_params[turn_id].alpha);
  while (not rotation.is_finished()) {
    delay(2);
  }
  forward.start(turn_params[turn_id].run_out, forward.speed(), SPEEDMAX_EXPLORE, SEARCH_ACCELERATION);
  while (not forward.is_finished()) {
    delay(2);
  }
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
void Mouse::turn_around() {
  bool has_wall = frontWall;
  set_steering_mode(STEERING_OFF);
  log_status('A');
  float remaining = (FULL_CELL + HALF_CELL) - forward.position();
  forward.start(remaining, forward.speed(), 30, forward.acceleration());
  if (has_wall) {
    while (g_front_sum < FRONT_REFERENCE) {
      delay(2);
    }
  } else {
    while (not forward.is_finished()) {
      delay(2);
    }
  }
  // Be sure robot has come to a halt.
  forward.stop();
  motion.spin_turn(-180, OMEGA_MAX_SPIN_TURN, ALPHA_SPIN_TURN);
  forward.start(HALF_CELL - 10.0, SPEEDMAX_EXPLORE, SPEEDMAX_EXPLORE, SEARCH_ACCELERATION);
  while (not forward.is_finished()) {
    delay(2);
  }
  forward.set_position(FULL_CELL - 10.0);
}

//***************************************************************************//

Mouse::Mouse() {
  init();
}

void Mouse::init() {
  handStart = false;
  set_steering_mode(STEERING_OFF);
  location = 0;
  heading = NORTH;
  p_mouse_state = SEARCHING;
}

void Mouse::update_sensors() {
  rightWall = (g_rss_has_wall);
  leftWall = (g_lss_has_wall);
  frontWall = (g_front_has_wall);
}

void Mouse::log_status(char action) {
  Serial.print('{');
  Serial.print(action);
  Serial.print(' ');
  print_hex_2(location);
  Serial.print(' ');
  Serial.print(dirLetters[heading]);
  print_justified(g_front_sum, 4);
  Serial.print('@');
  print_justified((int)forward.position(), 4);
  Serial.print(' ');
  print_walls();
  Serial.print('}');
  Serial.print(' ');
}

void Mouse::follow_to(unsigned char target) {
  Serial.println("Follow TO");
  handStart = true;
  location = 0;
  heading = NORTH;
  initialise_maze();
  flood_maze(maze_goal());
  // wait_for_user_start();
  delay(1000);
  enable_sensors();
  motion.reset_drive_system();
  enable_motor_controllers();
  forward.start(BACK_WALL_TO_CENTER, SPEEDMAX_EXPLORE, SPEEDMAX_EXPLORE, SEARCH_ACCELERATION);
  while (not forward.is_finished()) {
    delay(2);
  }
  forward.set_position(HALF_CELL);
  Serial.println(F("Off we go..."));
  motion.wait_until_position(FULL_CELL - 10);
  // at the start of this loop we are always at the sensing point
  while (location != target) {
    if (button_pressed()) {
      break;
    }
    Serial.println();
    log_status('-');
    set_steering_mode(STEER_NORMAL);
    location = neighbour(location, heading);
    update_sensors();
    update_map();
    flood_maze(maze_goal());
    unsigned char newHeading = direction_to_smallest(location, heading);
    unsigned char hdgChange = (newHeading - heading) & 0x3;
    Serial.print(hdgChange);
    Serial.write(' ');
    Serial.write('|');
    Serial.write(' ');
    log_status('.');
    if (location == target) {
      end_run();
    } else if (!leftWall) {
      turn_smooth(SS90EL);
      heading = (heading + 3) & 0x03;
      log_status('x');
    } else if (!frontWall) {
      forward.adjust_position(-FULL_CELL);
      log_status('F');
      motion.wait_until_position(FULL_CELL - 10.0);
      log_status('x');
    } else if (!rightWall) {
      turn_smooth(SS90ER);
      heading = (heading + 1) & 0x03;
      log_status('x');
    } else {
      turn_around();
      heading = (heading + 2) & 0x03;
      log_status('x');
    }
  }
  Serial.println();
  Serial.println(F("Arrived!  "));
  for (int i = 0; i < 4; i++) {
    disable_sensors();
    delay(250);
    enable_sensors();
    delay(250);
  }
  disable_sensors();

  motion.reset_drive_system();
}

char hdg_letters[] = "FRAL";

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
int Mouse::search_to(unsigned char target) {

  flood_maze(target);
  delay(1000);
  enable_sensors();
  motion.reset_drive_system();
  enable_motor_controllers();
  if (not handStart) {
    // back up to the wall behind
    forward.start(-60, 120, 0, 1000);
    while (not forward.is_finished()) {
      delay(2);
    }
  }
  forward.start(BACK_WALL_TO_CENTER, SPEEDMAX_EXPLORE, SPEEDMAX_EXPLORE, SEARCH_ACCELERATION);
  while (not forward.is_finished()) {
    delay(2);
  }
  forward.set_position(HALF_CELL);
  Serial.println(F("Off we go..."));
  motion.wait_until_position(FULL_CELL - 10);
  // TODO. the robot needs to start each iteration at the sensing point
  while (location != target) {
    if (button_pressed()) {
      break;
    }
    Serial.println();
    log_status('-');
    set_steering_mode(STEER_NORMAL);
    location = neighbour(location, heading);
    update_sensors();
    update_map();
    flood_maze(target);
    unsigned char newHeading = direction_to_smallest(location, heading);
    unsigned char hdgChange = (newHeading - heading) & 0x3;
    Serial.print(hdg_letters[hdgChange]);
    Serial.write(' ');
    if (location == target) {
      end_run();
      heading = (heading + 2) & 0x03;
    } else {

      switch (hdgChange) {
        case 0: // ahead
          forward.adjust_position(-FULL_CELL);
          log_status('F');
          log_status('x');
          motion.wait_until_position(FULL_CELL - 10);
          break;
        case 1: // right
          turn_smooth(SS90ER);
          log_status('x');
          heading = (heading + 1) & 0x03;
          break;
        case 2: // behind
          turn_around();
          log_status('x');
          heading = (heading + 2) & 0x03;
          break;
        case 3: // left
          turn_smooth(SS90EL);
          log_status('x');
          heading = (heading + 3) & 0x03;
          break;
      }
    }
  }
  disable_sensors();
  Serial.println();
  Serial.println(F("Arrived!  "));
  for (int i = 0; i < 4; i++) {
    digitalWrite(LED_LEFT, 1);
    delay(250);
    digitalWrite(LED_LEFT, 0);
    delay(250);
  }

  motion.reset_drive_system();
  return 0;
}

/**
 * change the mouse heading but do not physically turn
 */

void Mouse::set_heading(unsigned char new_heading) {
  heading = new_heading;
}

void Mouse::turn_to_face(unsigned char newHeading) {
  unsigned char hdgChange = (newHeading - heading) & 0x3;
  switch (hdgChange) {
    case 0: // already got it
      break;
    case 1: // right
      turn_IP90R();
      break;
    case 2: // behind
      turnIP180();
      break;
    case 3: // left
      turn_IP90L();
      break;
  }
  heading = newHeading;
}

void Mouse::update_map() {
  switch (heading) {
    case NORTH:
      if (frontWall) {
        set_wall_present(location, NORTH);
      }
      if (rightWall) {
        set_wall_present(location, EAST);
      }
      if (leftWall) {
        set_wall_present(location, WEST);
      }
      break;
    case EAST:
      if (frontWall) {
        set_wall_present(location, EAST);
      }
      if (rightWall) {
        set_wall_present(location, SOUTH);
      }
      if (leftWall) {
        set_wall_present(location, NORTH);
      }
      break;
    case SOUTH:
      if (frontWall) {
        set_wall_present(location, SOUTH);
      }
      if (rightWall) {
        set_wall_present(location, WEST);
      }
      if (leftWall) {
        set_wall_present(location, EAST);
      }
      break;
    case WEST:
      if (frontWall) {
        set_wall_present(location, WEST);
      }
      if (rightWall) {
        set_wall_present(location, NORTH);
      }
      if (leftWall) {
        set_wall_present(location, SOUTH);
      }
      break;
    default:
      // This is an error. We should handle it.
      break;
  }
  walls[location] |= VISITED;
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
 * cells, regardless of visited state,  does not pass through any
 * unvisited cells.
 *
 * The walls can be saved to EEPROM after each pass. It left to the
 * reader as an exercise to do something useful with that.
 */
int Mouse::search_maze() {
  wait_for_user_start();
  Serial.println("Search TO");
  handStart = true;
  location = START;
  heading = NORTH;
  search_to(maze_goal());
  handStart = false;
  search_to(START);
  stop_motors();
  return 0;
}
