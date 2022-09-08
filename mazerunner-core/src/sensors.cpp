/*
 * File: sensors.cpp
 * Project: mazerunner
 * File Created: Monday, 29th March 2021 11:05:58 pm
 * Author: Peter Harrison
 * -----
 * Last Modified: Friday, 9th April 2021 11:45:39 am
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

#include "sensors.h"
#include "..//config.h"
#include "digitalWriteFast.h"
#include <Arduino.h>
#include <util/atomic.h>
#include <wiring_private.h>

/**** Global variables  ****/

volatile float g_battery_voltage;
volatile float g_battery_scale;


/*** wall sensor variables ***/
volatile int g_lfs;
volatile int g_lss;
volatile int g_rss;
volatile int g_rfs;

volatile int g_front_sum;

volatile int g_lfs_raw;
volatile int g_lss_raw;
volatile int g_rss_raw;
volatile int g_rfs_raw;

/*** true if a wall is present ***/
volatile bool g_lss_has_wall;
volatile bool g_front_has_wall;
volatile bool g_rss_has_wall;

/*** steering variables ***/
uint8_t g_steering_mode = STEER_NORMAL;
bool g_steering_enabled;
volatile float g_cross_track_error;
volatile float g_steering_adjustment;
