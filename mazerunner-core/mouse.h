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
#include "config.h"

#define SEARCH_ACCELERATION 3000
#define SPEEDMAX_EXPLORE 400
#define SPEEDMAX_STRAIGHT 800
#define SPEEDMAX_SMOOTH_TURN 500

#define OMEGA_MAX_SPIN_TURN 360
#define ALPHA_SPIN_TURN 3600

enum {
  FRESH_START,
  SEARCHING,
  INPLACE_RUN,
  SMOOTH_RUN,
  FINISHED
};

enum {
  SS90EL = 0,
  SS90ER = 1,
  SS90L = 3,
  SS90R = 3,
};

struct TurnParameters {
  int speed;
  int run_in;  // (mm)
  int run_out; // mm
  int angle;   // deg
  int omega;   // deg/s
  int alpha;   // deg/s/s
  int trigger; // sensor value
};

const TurnParameters turn_params[4] = {
    {DEFAULT_TURN_SPEED, 25, 10, -90, 280, 4000, TURN_THRESHOLD_SS90E}, // 0 => SS90EL
    {DEFAULT_TURN_SPEED, 20, 10, 90, 280, 4000, TURN_THRESHOLD_SS90E},  // 0 => SS90ER
    {DEFAULT_TURN_SPEED, 20, 10, -90, 280, 4000, TURN_THRESHOLD_SS90E}, // 0 => SS90L
    {DEFAULT_TURN_SPEED, 20, 10, 90, 280, 4000, TURN_THRESHOLD_SS90E},  // 0 => SS90R
};
/// TODO: should the whole mouse object be persistent?
class Mouse {
  public:
  Mouse();
  void init();
  void update_sensors();
  void log_status(char action);
  void set_heading(unsigned char new_heading);
  void turn_to_face(unsigned char new_heading);
  void turn_smooth(int turn_id);
  void turn_around();
  void end_run();
  int search_to(unsigned char target);
  void follow_to(unsigned char target);
  void update_map();
  int search_maze();

  unsigned char heading;
  unsigned char location;
  bool leftWall;
  bool frontWall;
  bool rightWall;
  bool handStart;
};

extern char p_mouse_state;

extern Mouse mouse;

#endif // MOUSE_H