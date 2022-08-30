/*
 * File: logger.h
 * Project: mazerunner
 * File Created: Tuesday, 23rd March 2021 10:18:19 pm
 * Author: Peter Harrison
 * -----
 * Last Modified: Wednesday, 14th April 2021 4:36:51 pm
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

#ifndef LOGGER_H
#define LOGGER_H

#include <Arduino.h>

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
 *   fwdVolts    - voltage sent to the motors for forward control
 *   rotVolts    - voltage sent to the motors for rotation control
 *
 * @brief Sends motion profile data about positions and angles
 */
void report_profile();
void report_profile_header();

void report_sensor_track_header();
void report_sensor_track();
void report_sensor_track_raw();

void report_front_sensor_track_header();
void report_front_sensor_track();

void report_wall_sensors();

// used for setting up the sensor calibration
void report_sensor_calibration();

/**
 * The controllers work by comparing the profiled positin and angle
 * with values obtained from the encoders.
 *
 * All four values are reported here.
 *
 * @brief Send the profiled and actual position and angle
 */

void print_hex_2(unsigned char value);
void print_justified(int value, int width);
void print_maze_plain();
void print_maze_with_costs();
void print_maze_with_directions();
void print_maze_wall_data();

#endif