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

#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>
#define CODE "MAZERUNNER-CORE"
/***
 * The config.h file determines the actual robot variant that
 * will be the target for the build.
 *
 * This files lets you pick a specific robot that has its unique
 * configuration stored in that file. In this way, you can write
 * generic code that will work with a variety of different actual
 * robots. There are a number of example robot files in the project.
 * You should pick one that is closest to your setup, copy it and
 * then amend the details in that copy. Finally, add or modify
 * the selection settings below to use your new robot configuration.
 *
 */

/*************************************************************************/
#define STRING2(x) #x
#define STRING(x) STRING2(x)

const float RADIANS_PER_DEGREE = 2 * PI / 360.0;
const float DEGREES_PER_RADIAN = 360.0 / 2 * PI;

/***
 * Structure definitions used in the software. Declared here for lack of a
 * better place because the structure is needed by the robot configs.
 *
 * Robot specific instances and values are in the robot config file.
 *
 * TODO: redefine these in terms of offsets from cell centre
 * TODO: replace trigger value with sensor identifier
 */
struct TurnParameters {
  int speed;         // mm/s    - constant forward speed during turn
  int entry_offset;  // mm      - distance from turn pivot to turn start
  int exit_offset;   // mm      - distance from turn pivot to turn end
  float angle;       // deg     - total turn angle
  float omega;       // deg/s   - maximum angular velocity
  float alpha;       // deg/s/s - angular acceleration
  int trigger;       //         - front sensor value at start of turn
};

/*************************************************************************/
/***
 * You may use a slightly different hardware platform than UKMARSBOT
 * Here you can include a suitable hardware configuration to define
 * things like IO pins, ADC channels and so on
 */

#define HARDWARE_UNKNOWN 0
#define HARDWARE_UKMARSBOT_1_3A 1

#define HARDWARE HARDWARE_UKMARSBOT_1_3A

#if HARDWARE == HARDWARE_UKMARSBOT_1_3A
#include "config-ukmarsbot.h"
#else
#error "NO HARDWARE DEFINED"
#endif

/*************************************************************************/
/***
 * It is possible that you might want to run the robot in a number of
 * different mazes with different calibration values. The config file
 * can have different sensor defaults for each of these environments
 * so here you can define which set will be used.
 */

#define EVENT_HOME 1
#define EVENT_UK 2
#define EVENT_PORTUGAL 3
#define EVENT_APEC 4

// choose the one you will be using BEFORE selecting the robot below
#define EVENT EVENT_UK
#if EVENT == EVENT_HOME
#define GOAL Location(2, 2)
#else
#define GOAL Location(7, 7)
#endif
// This is the size, in mm,  for each cell in the maze.
const float FULL_CELL = 180.0f;
const float HALF_CELL = FULL_CELL / 2.0;

/*************************************************************************/
/***
 * Even with the same basic hardware, you may build robots with different
 * characteristics such as the motor gear ratio or the wheel size or sensor
 * arrangement.
 *
 * These characteristics are kept in their own config files. Add you robot
 * to the list and create a corresponding config file with its custom values.
 *
 * If you have only one robot then you can reduce this section to a single
 * include line.
 */
#define ROBOT_NOT_DEFINED 0
#define ROBOT_CORE_OSMIUM 1
#define ROBOT_ORION 2

#define ROBOT ROBOT_CORE_OSMIUM

#if ROBOT == ROBOT_CORE_OSMIUM
#include "config-robot-osmium.h"
#elif ROBOT == ROBOT_ORION
#include "config-robot-orion.h"
#else
#error "NO ROBOT DEFINED"
#endif

/*************************************************************************/
/***
 * This piece of magic lets you define a variable, such as the maze, that can
 * survive a processor reset. The downside is that you MUST take care to
 * properly initialise it when necessary. If you just turn on the robot for
 * example, the maze will have random data in it.
 *
 * CLEAR THE MAZE BEFORE EVERY CONTEST
 *
 * The mazerunner code clears the maze if the user button is held down during
 * a reset.
 */
#define PERSISTENT __attribute__((section(".noinit")))

#endif
