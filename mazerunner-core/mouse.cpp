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
#include "encoders.h"
#include "maze.h"
#include "motion.h"
#include "motors.h"
#include "profile.h"
#include "reports.h"
#include "sensors.h"
#include "ui.h"

Mouse dorothy;

char path[128];
char commands[128];
char p_mouse_state __attribute__((section(".noinit")));

static char dirLetters[] = "NESW";

void print_walls() {
  if (g_left_wall_present) {
    Serial.print('L');
  } else {
    Serial.print('-');
  }
  if (g_front_wall_present) {
    Serial.print('F');
  } else {
    Serial.print('-');
  }
  if (g_right_wall_present) {
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
  disable_steering();
  forward.start(remaining, forward.speed(), 0, forward.acceleration());
  while (not forward.is_finished()) {
    if (g_front_wall_sensor > (FRONT_REFERENCE - 150)) {
      break;
    }
    delay(2);
  }
  if (g_front_wall_present) {
    while (g_front_wall_sensor < FRONT_REFERENCE) {
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
  spin_turn(direction * 180, SPEEDMAX_SPIN_TURN, SPIN_TURN_ACCELERATION);
}

void turn_IP90R() {
  spin_turn(-90, SPEEDMAX_SPIN_TURN, SPIN_TURN_ACCELERATION);
}

void turn_IP90L() {
  spin_turn(90, SPEEDMAX_SPIN_TURN, SPIN_TURN_ACCELERATION);
}

void turnSS90L() {
  turn(90, 200, 2000);
}

void turnSS90R() {
  turn(-90, 200, 2000);
}

void move_forward(float distance, float top_speed, float end_speed) {
  forward.start(distance, top_speed, end_speed, SEARCH_ACCELERATION);
}

//***************************************************************************//

void Mouse::end_run() {
  bool has_wall = frontWall;
  disable_steering();
  log_status('T');
  float remaining = (FULL_CELL + HALF_CELL) - forward.position();
  forward.start(remaining, forward.speed(), 30, forward.acceleration());
  if (has_wall) {
    while (get_front_sensor() < 850) {
      delay(2);
    }
  } else {
    while (not forward.is_finished()) {
      delay(2);
    }
  }
  Serial.print(' ');
  Serial.print(get_front_sensor());
  Serial.print('@');
  Serial.print(forward.position());
  Serial.print(' ');
  // Be sure robot has come to a halt.
  forward.stop();
  spin_turn(-180, SPEEDMAX_SPIN_TURN, SPIN_TURN_ACCELERATION);
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
void Mouse::turn_SS90ER() {

  float run_in = 5.0;   // mm
  float run_out = 10.0; // mm
  float angle = -90.0;  // deg
  float omega = 280;    // deg/s
  float alpha = 4000;   // deg/s/s
  bool triggered = false;
  disable_steering();
  float distance = FULL_CELL + 10.0 + run_in - forward.position();
  forward.start(distance, forward.speed(), DEFAULT_TURN_SPEED, SEARCH_ACCELERATION);
  while (not forward.is_finished()) {
    delay(2);
    if (g_front_wall_sensor > 54) {
      forward.set_state(CS_FINISHED);
      triggered = true;
    }
  }
  if (triggered) {
    log_status('R');
  } else {
    log_status('r');
  }
  rotation.start(angle, omega, 0, alpha);
  while (not rotation.is_finished()) {
    delay(2);
  }
  forward.start(run_out, forward.speed(), DEFAULT_SEARCH_SPEED, SEARCH_ACCELERATION);
  while (not forward.is_finished()) {
    delay(2);
  }
  forward.set_position(FULL_CELL - 10.0);
}

void Mouse::turn_SS90EL() {
  float run_in = 5.0;   // mm
  float run_out = 10.0; // mm
  float angle = 90.0;   // deg
  float omega = 280;    // deg/s
  float alpha = 4000;   // deg/s/s
  bool triggered = false;
  disable_steering();
  float distance = FULL_CELL + 10.0 + run_in - forward.position();
  forward.start(distance, forward.speed(), DEFAULT_TURN_SPEED, SEARCH_ACCELERATION);
  while (not forward.is_finished()) {
    delay(2);
    if (g_front_wall_sensor > 54) {
      forward.set_state(CS_FINISHED);
      triggered = true;
    }
  }
  if (triggered) {
    log_status('L');
  } else {
    log_status('l');
  }
  rotation.start(angle, omega, 0, alpha);
  while (not rotation.is_finished()) {
    delay(2);
  }
  forward.start(run_out, forward.speed(), DEFAULT_SEARCH_SPEED, SEARCH_ACCELERATION);
  while (not forward.is_finished()) {
    delay(2);
  }
  forward.set_position(FULL_CELL - 10.0);
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
  disable_steering();
  log_status('A');
  float remaining = (FULL_CELL + HALF_CELL) - forward.position();
  forward.start(remaining, forward.speed(), 30, forward.acceleration());
  if (has_wall) {
    while (get_front_sensor() < FRONT_REFERENCE) {
      delay(2);
    }
  } else {
    while (not forward.is_finished()) {
      delay(2);
    }
  }
  // Be sure robot has come to a halt.
  forward.stop();
  spin_turn(-180, SPEEDMAX_SPIN_TURN, SPIN_TURN_ACCELERATION);
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
  disable_steering();
  location = 0;
  heading = NORTH;
  p_mouse_state = SEARCHING;
}

void Mouse::update_sensors() {
  rightWall = (g_right_wall_present);
  leftWall = (g_left_wall_present);
  frontWall = (g_front_wall_present);
}

void Mouse::log_status(char action) {
  Serial.print(' ');
  Serial.print(action);
  Serial.print('(');
  print_hex_2(location);
  Serial.print(dirLetters[heading]);
  Serial.print(')');
  Serial.print('[');
  print_justified(get_front_sensor(), 3);
  Serial.print(']');
  Serial.print('@');
  print_justified((int)forward.position(), 4);
  Serial.print(' ');
  print_walls();
  Serial.print(' ');
  Serial.print('|');
  Serial.print(' ');
}

void Mouse::follow_to(unsigned char target) {
  handStart = true;
  location = 0;
  heading = NORTH;
  initialise_maze(emptyMaze);
  flood_maze(maze_goal());
  // wait_for_front_sensor();
  delay(1000);
  enable_sensors();
  reset_drive_system();
  enable_motor_controllers();
  forward.start(BACK_WALL_TO_CENTER, SPEEDMAX_EXPLORE, SPEEDMAX_EXPLORE, SEARCH_ACCELERATION);
  while (not forward.is_finished()) {
    delay(2);
  }
  forward.set_position(HALF_CELL);
  Serial.println(F("Off we go..."));
  wait_until_position(FULL_CELL - 10);
  // at the start of this loop we are always at the sensing point
  while (location != target) {
    if (button_pressed()) {
      break;
    }
    Serial.println();
    log_status('-');
    enable_steering();
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
      turn_SS90EL();
      heading = (heading + 3) & 0x03;
      log_status('x');
    } else if (!frontWall) {
      forward.adjust_position(-FULL_CELL);
      log_status('F');
      wait_until_position(FULL_CELL - 10.0);
      log_status('x');
    } else if (!rightWall) {
      turn_SS90ER();
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

  report_status();
  reset_drive_system();
}

void Mouse::report_status() {
  print_hex_2(location);
  Serial.print(':');
  Serial.print(dirLetters[heading]);
  if (leftWall) {
    Serial.print('L');
  } else {
    Serial.print('-');
  }
  if (frontWall) {
    Serial.print('F');
  } else {
    Serial.print('-');
  }
  if (rightWall) {
    Serial.print('R');
  } else {
    Serial.print('-');
  }
  Serial.println();
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
int Mouse::search_to(unsigned char target) {

  flood_maze(target);
  // wait_for_front_sensor();
  delay(1000);
  enable_sensors();
  reset_drive_system();
  enable_motor_controllers();
  if (not handStart) {
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
  wait_until_position(FULL_CELL - 10);
  // TODO. the robot needs to start each iteration at the sensing point
  while (location != target) {
    if (button_pressed()) {
      break;
    }
    Serial.println();
    log_status('-');
    enable_steering();
    location = neighbour(location, heading);
    update_sensors();
    update_map();
    flood_maze(target);
    unsigned char newHeading = direction_to_smallest(location, heading);
    unsigned char hdgChange = (newHeading - heading) & 0x3;
    Serial.print(hdgChange);
    Serial.write(' ');
    Serial.write('|');
    Serial.write(' ');
    log_status('.');
    if (location == target) {
      end_run();
      heading = (heading + 2) & 0x03;
    } else {

      switch (hdgChange) {
        case 0: // ahead
          forward.adjust_position(-FULL_CELL);
          log_status('F');
          wait_until_position(FULL_CELL - 10);
          log_status('x');
          break;
        case 1: // right
          turn_SS90ER();
          heading = (heading + 1) & 0x03;
          log_status('x');
          break;
        case 2: // behind
          turn_around();
          heading = (heading + 2) & 0x03;
          log_status('x');
          break;
        case 3: // left
          turn_SS90EL();
          heading = (heading + 3) & 0x03;
          log_status('x');
          break;
      }
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

  report_status();
  reset_drive_system();
  return 0;
}

/**
 * change the mouse heading but do not physically turn
 */

void Mouse::set_heading(unsigned char new_heading) {
  heading = new_heading;
}

/***
 * inelegant but simple solution to the problem
 */
void Mouse::turn_to_face(unsigned char newHeading) {
  // debug << dirLetters[mouse.heading] << '>' << dirLetters[newHeading] << endl;
  switch (heading) {
    case NORTH:
      if (newHeading == EAST) {
        turn_IP90R();
      } else if (newHeading == SOUTH) {
        turnIP180();
      } else if (newHeading == WEST) {
        turn_IP90L();
      }
      break;
    case EAST:
      if (newHeading == SOUTH) {
        turn_IP90R();
      } else if (newHeading == WEST) {
        turnIP180();
      } else if (newHeading == NORTH) {
        turn_IP90L();
      }
      break;
    case SOUTH:
      if (newHeading == WEST) {
        turn_IP90R();
      } else if (newHeading == NORTH) {
        turnIP180();
      } else if (newHeading == EAST) {
        turn_IP90L();
      }
      break;
    case WEST:
      if (newHeading == NORTH) {
        turn_IP90R();
      } else if (newHeading == EAST) {
        turnIP180();
      } else if (newHeading == SOUTH) {
        turn_IP90L();
      }
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
  wait_for_front_sensor();
  //                                             motorsEnable();
  location = 0;
  heading = NORTH;
  int result = search_to(maze_goal());
  if (result != 0) {
    panic(1);
  }
  //  EEPROM.put(0, walls);
  // digitalWrite(RED_LED, 1);
  delay(200);
  result = search_to(0);
  stop_motors();
  if (result != 0) {
    panic(1);
  }
  //    EEPROM.put(0, walls);
  delay(200);
  return 0;
}
