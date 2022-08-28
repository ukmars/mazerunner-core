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

//--------------------------------------------------------------------------
// assume the maze is flooded and that a simple path string has been generated
// then run the mouse along the path.
// run-length encoding of straights is done on the fly.
// turns are in-place so the mouse stops after each straight.
//--------------------------------------------------------------------------
void Mouse::run_in_place_turns(int topSpeed) { // TODO
  expand_path(path);
  // debug << path << endl;
  // debug << commands << endl;
  // "HRH": in place right
  // "HLH": in place left
  // "HH":  half a cell forward
  // "HS":  end after half a cell
  int index = 0;
  while (commands[index] != 'S') {
    if (button_pressed()) {
      break;
    }
    if (commands[index] == 'B') {
      index++;
    } else if (commands[index] == 'H' && commands[index + 1] == 'R' && commands[index + 2] == 'H') {

      move_forward(HALF_CELL, topSpeed, 0);
      turn_IP90R();
      move_forward(HALF_CELL, topSpeed, topSpeed);
      index += 3;
    } else if (commands[index] == 'H' && commands[index + 1] == 'L' && commands[index + 2] == 'H') {
      move_forward(HALF_CELL, topSpeed, 0);
      turn_IP90L();
      move_forward(HALF_CELL, topSpeed, topSpeed);
      index += 3;
    } else if (commands[index] == 'H' && commands[index + 1] == 'H') {
      move_forward(HALF_CELL, topSpeed, topSpeed);
      index++;
    } else if (commands[index] == 'H' && commands[index + 1] == 'S') {
      move_forward(HALF_CELL, topSpeed, 0);
      index++;
    } else {
      // debug << F("Instruction error!\n");
      break;
    }
  }
  // assume we succeed
  location = maze_goal();
  report_status();
}

//--------------------------------------------------------------------------
// Assume the maze is flooded and that a path string already exists.
// Convert that to half-cell straights for easier processing
// next, convert all HRH and HLH occurrences to the corresponding smooth turns
// then run the mouse along the path.
// run-length encoding of straights is done on the fly.
// turns are smooth and care is taken to deal with the path end.
//--------------------------------------------------------------------------
void Mouse::run_smooth_turns(int topSpeed) {
  expand_path(path);
  // "HRH": smooth right
  // "HLH": smooth left
  // "HH":  half a cell forward
  // "HS":  end after half a cell
  int index = 0;
  while (commands[index] != 'S') {
    if (button_pressed()) {
      break;
    }
    if (commands[index] == 'B') {
      index++;
    } else if (commands[index] == 'H' && commands[index + 1] == 'R' && commands[index + 2] == 'H') {
      move_forward(20, topSpeed, SPEEDMAX_SMOOTH_TURN);
      // debug << 'R';
      turnSS90R();
      move_forward(20, topSpeed, topSpeed);
      index += 3;
    } else if (commands[index] == 'H' && commands[index + 1] == 'L' && commands[index + 2] == 'H') {
      move_forward(20, topSpeed, SPEEDMAX_SMOOTH_TURN);
      // debug << 'L';
      turnSS90L();
      move_forward(20, topSpeed, topSpeed);
      index += 3;
    } else if (commands[index] == 'H' && commands[index + 1] == 'H') {
      // debug << 'H';
      move_forward(HALF_CELL, topSpeed, topSpeed);
      ;
      index++;
    } else if (commands[index] == 'H' && commands[index + 1] == 'S') {
      // debug << 'H';
      move_forward(HALF_CELL, topSpeed, 0);
      index++;
    } else {
      // debug << F("Instruction error!\n");
      break;
    }
  }
  // debug << 'S' << endl;
  // assume we succeed
  location = maze_goal();
  report_status();
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

/***
 * Search the maze until there is a solution then make a path and run it
 * First with in-place turns, then with smooth turns;
 *
 * The mouse can be placed into any of the possible states before
 * calling this function so that individual actions can be tested.
 *
 * If you do not want to search exhaustively then do a single search
 * out and back again. Then block off all the walls in any cells that
 * are unvisited. Now any path generated will succeed even if it is
 * not optimal.
 */
int Mouse::run_maze() {
  // motorsEnable();
  if (p_mouse_state == SEARCHING) {
    wait_for_front_sensor();
    handStart = true;
    enable_steering();
    location = 0;
    heading = NORTH;
    search_to(maze_goal());
    search_to(START);
    turn_to_face(NORTH);
    delay(200);
    p_mouse_state = INPLACE_RUN;
  }
  if (p_mouse_state == INPLACE_RUN) {
    flood_maze(maze_goal());
    make_path(location);
    wait_for_front_sensor();
    Serial.println(F("Running in place"));
    run_in_place_turns(SPEEDMAX_STRAIGHT);
    Serial.println(F("Returning"));
    search_to(START);
    Serial.println(F("Done"));
    p_mouse_state = SMOOTH_RUN;
  }
  if (p_mouse_state == SMOOTH_RUN) {
    // now try with smooth turns;
    flood_maze(maze_goal());
    make_path(location);
    turn_to_face(direction_to_smallest(location, heading));
    delay(200);
    wait_for_front_sensor();
    Serial.println(F("Running smooth"));
    run_smooth_turns(SPEEDMAX_STRAIGHT);
    Serial.println(F("Returning"));
    search_to(START);
    Serial.println(F("Finished"));
    p_mouse_state = FINISHED;
  }
  stop_motors();
  return 0;
}

/***
 * Assumes the maze is already flooded to a single target cell and so
 * every cell will have a cost that decreases as the target is approached.
 *
 * Starting at the given cell, the algorithm repeatedly looks for the
 * smallest available neighbour and records the action taken to reach it.
 *
 * The process starts by assuming the mouse is heading NORTH in the start
 * cell since that is what would be the case at the start of a speed run.
 *
 * At each cell, the preference is to move forwards if possible
 *
 * If the pathfinder is called from any other cell, the mouse must first
 * turn to face to the smallest neighbour of that cell using the same
 * method as in this function.
 *
 * The resulting path is a simple string, null terminated, that can be
 * printed to the Serial.to make it easy to compare paths using different
 * flooding or path generating methods.
 *
 * The characters in the path string are:
 * 	'B' : always the first character, it marks the path start.
 * 	'F' : move forwards a full cell
 * 	'H' : used in speedruns to indicate movement of half a cell forwards
 * 	'R' : turn right in this cell
 * 	'L' : turn left in this cell
 * 	'A' : turn around (should never happen in a speedrun path)
 * 	'S' : the last character in the path, telling the mouse to stop
 *
 * For example, the Japan2007 maze, flooded with a simple Manhattan
 * flood, should produce the path string:
 *
 * BFFFRLLRRLLRRLLRFFRRFLLFFLRFRRLLRRLLRFFFFFFFFFRFFFFFRLRLLRRLLRRFFRFFFLFFFS
 *
 * The path string is processed by the mouse directly to make it move
 * along the path. At its simplest, this is just a case of executing
 * a single movement for each character in the string, using in-place turns.
 *
 * I would strongly recommend this style of path string. Not only can the
 * strings be used to compare routes very easily, they can be printed and
 * visually compared or followed by hand.
 *
 * Path strings are easily translated into more complex paths using
 * smooth turns an they are relatively easy to turn into a set of
 * commands that will represent a diagonal path.
 *
 * Further, short path strings  can be hand-generated to test the movement
 * of the mouse or to test the setup of different turn types.
 *
 * The pathGenerator is not terribly efficient and can take up to 20ms to
 * generate a path depending on the maze, start cell and target.
 *
 */

bool Mouse::make_path(unsigned char startCell = START) {
  bool solved = true;
  ;
  unsigned char cell = startCell;
  int nextCost = cost[cell] - 1; // assumes manhattan flood
  unsigned char commandIndex = 0;
  path[commandIndex++] = 'B';
  unsigned char direction = direction_to_smallest(cell, NORTH);
  while (nextCost >= 0) {
    unsigned char cmd = 'S';
    switch (direction) {
      case NORTH:
        if (is_exit(cell, NORTH) && neighbour_cost(cell, NORTH) == nextCost) {
          cmd = 'F';
          break;
        }
        if (is_exit(cell, EAST) && neighbour_cost(cell, EAST) == nextCost) {
          cmd = 'R';
          direction = DtoR[direction];
          break;
        }
        if (is_exit(cell, WEST) && neighbour_cost(cell, WEST) == nextCost) {
          cmd = 'L';
          direction = DtoL[direction];
          break;
        }
        if (is_exit(cell, SOUTH) && neighbour_cost(cell, SOUTH) == nextCost) {
          cmd = 'A';
          direction = DtoB[direction];
        }
        break;
      case EAST:
        if (is_exit(cell, EAST) && neighbour_cost(cell, EAST) == nextCost) {
          cmd = 'F';
          break;
        }
        if (is_exit(cell, SOUTH) && neighbour_cost(cell, SOUTH) == nextCost) {
          cmd = 'R';
          direction = DtoR[direction];
          break;
        }
        if (is_exit(cell, NORTH) && neighbour_cost(cell, NORTH) == nextCost) {
          cmd = 'L';
          direction = DtoL[direction];
          break;
        }
        if (is_exit(cell, WEST) && neighbour_cost(cell, WEST) == nextCost) {
          cmd = 'A';
          direction = DtoB[direction];
          break;
        }
        break;
      case SOUTH:
        if (is_exit(cell, SOUTH) && neighbour_cost(cell, SOUTH) == nextCost) {
          cmd = 'F';
          break;
        }
        if (is_exit(cell, WEST) && neighbour_cost(cell, WEST) == nextCost) {
          direction = DtoR[direction];
          cmd = 'R';
          break;
        }
        if (is_exit(cell, EAST) && neighbour_cost(cell, EAST) == nextCost) {
          cmd = 'L';
          direction = DtoL[direction];
          break;
        }
        if (is_exit(cell, NORTH) && neighbour_cost(cell, NORTH) == nextCost) {
          cmd = 'A';
          direction = DtoB[direction];
          break;
        }
        break;
      case WEST:
        if (is_exit(cell, WEST) && neighbour_cost(cell, WEST) == nextCost) {
          cmd = 'F';
          break;
        }
        if (is_exit(cell, NORTH) && neighbour_cost(cell, NORTH) == nextCost) {
          cmd = 'R';
          direction = DtoR[direction];
          break;
        }
        if (is_exit(cell, SOUTH) && neighbour_cost(cell, SOUTH) == nextCost) {
          cmd = 'L';
          direction = DtoL[direction];
          break;
        }
        if (is_exit(cell, EAST) && neighbour_cost(cell, EAST) == nextCost) {
          cmd = 'A';
          direction = DtoB[direction];
          break;
        }
        break;
      default:
        // this is an error. We should handle it.
        break;
    }
    cell = neighbour(cell, direction);
    if ((walls[cell] & VISITED) != VISITED) {
      solved = false;
    }
    nextCost--;
    path[commandIndex] = cmd;
    commandIndex++;
  }
  path[commandIndex] = 'S';
  commandIndex++;
  path[commandIndex] = '\0';
  return solved;
}

/***
 * Assumes that the maze is flooded and a path string has been generated.
 *
 * Convert  the simple path string to a set of commands using half-cell
 * moves instead of full-cell moves so that a sequence like
 * 		BFRFLS
 * becomes
 * 		BHHRHHHHLHHS
 *
 * The turns all have an implied full cell forward movement after them
 * The only real advantage of this is that it is easier to convert to smooth
 * turns by looking for patterns like HRH and HLH
 *

 */
void Mouse::expand_path(char *pathString) {
  int pathIndex = 0;
  int commandIndex = 0;
  commands[commandIndex++] = 'B';
  while (char c = pathString[pathIndex]) {
    switch (c) {
      case 'F':
        commands[commandIndex++] = 'H';
        commands[commandIndex++] = 'H';
        pathIndex++;
        break;
      case 'R':
        commands[commandIndex++] = 'R';
        commands[commandIndex++] = 'H';
        commands[commandIndex++] = 'H';
        pathIndex++;
        break;
      case 'L':
        commands[commandIndex++] = 'L';
        commands[commandIndex++] = 'H';
        commands[commandIndex++] = 'H';
        pathIndex++;
        break;
      case 'S':
        commands[commandIndex++] = 'S';
        pathIndex++;
        break;
      case 'B':
      case ' ': // ignore these so we can write easy-to-read path strings
        pathIndex++;
        break;
      default:
        // TODO: this is an error. We should handle it.
        break;
    }
  }
  commands[commandIndex] = '\0';
}

void Mouse::print_path() {
  for (int i = 0; i < 128 && path[i]; i++) {
    Serial.print(path[i]);
  }
  Serial.println();
}
