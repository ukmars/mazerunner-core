/*
 * File: config.h
 * Project: mazerunner
 * File Created: Monday, 29th March 2021 11:04:59 pm
 * Author: Peter Harrison
 * -----
 * Last Modified: Thursday, 6th May 2021 9:17:32 am
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

#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

/***
 * The config.h file defines the actual robot variant which will be the target for the build.
 *
 * A 'proper' build setup would probably define this on the command line or in a make file.
 *
 * The other information in here holds system-wide configuration values that are not dependent
 * upon the target robot. That is things like the serial port baud rate and loop interval.
 *
 *
 */

/***
 * The defaults.h file holds the default settings values for the robot.
 * Here you can have different settings for individual robot variants. For
 * example, a variants with different gear ratios or wheel sizes
 *
 * The defaults shown here are compiled into the constant default Settings structure.
 * Normally, at start up there will be a copy of the last used settings structure
 * stored in EEPROM which is copied into the settings structure in RAM. If no valid
 * settings structure is found in EEPROM, the flash settings data is copied into
 * EEPROM and then loaded into RAM.
 * *
 * This cumbersome process makes it possible to have settings changed while the robot
 * is running and then save those settings to EEPROM so that they survive a reset.
 *
 * Remember always that just connecting the USB to the computer will cause a reset so
 * take care to use the auxiiary serial port (and possibly bluetooth) to make these
 * kinds of modifications.
 *
 * see the "settings.h" file for more details.
 */

/*************************************************************************/
#define ROBOT_ARIADNE 1
#define ROBOT_CLARA 3
#define ROBOT_DOROTHY 4
#define ROBOT_EMILY 5

#define ROBOT_NAME ROBOT_EMILY

/*************************************************************************/
#define MODE_TEST 0
#define MODE_RUN 1

#define ROBOT_MODE MODE_RUN

/*************************************************************************/
#define EVENT_HOME 1
#define EVENT_UK 2
#define EVENT_PORTUGAL 3

#define EVENT UK
/*************************************************************************/
#define BOARD_UKMARSBOT_V1 1

#define BOARD BOARD_UKMARSBOT_V1
/*************************************************************************/

#if BOARD == BOARD_UKMARSBOT_V1
#include "board/ukmarsbot-v1.h"
#else
#error "UNKOWN BOARD TYPE"
#endif

#if ROBOT_NAME == ROBOT_ARIADNE
#include "config/ariadne.h"
#elif ROBOT_NAME == ROBOT_CLARA
#include "config/dorothy.h"
#elif ROBOT_NAME == ROBOT_EMILY
#include "config/emily.h"
#else
#error "NO ROBOT DEFINED"
#endif

//***************************************************************************//
const uint32_t BAUDRATE = 115200;

// sometimes the controller needs the interval, sometimes the frequency
// define one and pre-calculate the other. The compiler does the work and no flash or
// RAM storage is used. Constants are used for better type checking and traceability.
const float LOOP_FREQUENCY = 500.0;
const float LOOP_INTERVAL = (1.0 / LOOP_FREQUENCY);

// force rewrite of EEPROM settings. Set this when developing
#define ALWAYS_USE_DEFAULT_SETTINGS 0

//***************************************************************************//

// This is the size fo each cell in the maze. Normally 180mm for a classic maze
const float FULL_CELL = 180.0f;
const float HALF_CELL = FULL_CELL / 2.0;

//***************************************************************************//

//***************************************************************************//
// change the revision if the settings structure changes to force rewrte of EEPROM
const int SETTINGS_REVISION = 10429;
const int DEFAULT_DECIMAL_PLACES = 5;

//***************************************************************************//

#endif
