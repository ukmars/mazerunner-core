/*
 * File: mouse.h
 * Project: mazerunner
 * File Created: Friday, 23rd April 2021 9:09:16 am
 * Author: Peter Harrison
 * -----
 * Last Modified: Monday, 26th April 2021 10:42:44 pm
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

#ifndef MOUSE_H
#define MOUSE_H

#define SEARCH_ACCELERATION 3000
#define SPIN_TURN_ACCELERATION 3600
#define SPEEDMAX_EXPLORE 400
#define SPEEDMAX_STRAIGHT 800
#define SPEEDMAX_SMOOTH_TURN 500
#define SPEEDMAX_SPIN_TURN 360

enum {
  FRESH_START,
  SEARCHING,
  INPLACE_RUN,
  SMOOTH_RUN,
  FINISHED
};

/// TODO: should the whole mouse object be persistent?
class Mouse {
  public:
  Mouse();
  void init();
  void report_status();
  void update_sensors();
  void log_status(char action);
  void set_heading(unsigned char new_heading);
  void turn_to_face(unsigned char new_heading);
  void turn_SS90EL();
  void turn_SS90ER();
  void turn_around();
  void end_run();
  int search_to(unsigned char target);
  void follow_to(unsigned char target);
  void run_in_place_turns(int top_speed);
  void run_smooth_turns(int top_speed);
  void update_map();
  int search_maze();
  int run_maze();
  bool make_path(unsigned char startCell);
  void expand_path(char *pathString);
  void print_path();

  unsigned char heading;
  unsigned char location;
  bool leftWall;
  bool frontWall;
  bool rightWall;
  bool handStart;
};

extern char p_mouse_state;

extern char path[];
extern char commands[];

extern Mouse dorothy;

#endif // MOUSE_H