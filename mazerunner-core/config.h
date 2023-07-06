/******************************************************************************
 * Project: mazerunner-core                                                   *
 * File:    config.h                                                          *
 * File Created: Sunday, 11th September 2022 3:34:57 pm                       *
 * Author: Peter Harrison                                                     *
 * -----                                                                      *
 * Last Modified: Wednesday, 2nd November 2022 12:41:18 pm                    *
 * -----                                                                      *
 * Copyright 2022 - 2022 Peter Harrison, Micromouseonline                     *
 * -----                                                                      *
 * Licence:                                                                   *
 *     Use of this source code is governed by an MIT-style                    *
 *     license that can be found in the LICENSE file or at                    *
 *     https://opensource.org/licenses/MIT.                                   *
 ******************************************************************************/

#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

/***
 * The config.h file determines the actual robot variant that
 * will be the target for the build.
 *
 * This files lets you pick a specific robot that has its unique
 * configuration stored in that file. In this way, you can write
 * generic code that will work with a variety of different actual
 * robots. there are a number of example robot files in the project.
 * You should pick one that is closest to your setup, copy it and
 * then amend the details in that copy. Finally, add or modify
 * the selection settings below to use your new robot configuration.
 *
 */

// work out the board architecture
#if defined(ARDUINO_ARCH_MEGAAVR)
const char board_name[] PROGMEM = "Nano Every - ATMEGA4809";
#elif defined(ARDUINO_ARCH_NRF52840)
const char board_name[] PROGMEM = "Nano 33 - NRF52840";
#elif defined(ARDUINO_ARCH_AVR)
const char board_name[] PROGMEM = "Nano - ATMega328P";
#else
const char board_name[] PROGMEM = "Unknown Board!";
#endif

/***
 * Redirection for printf()
 *
 * If you want to be able to use the standard C printf() function
 * you can uncomment the following definition.
 *
 * If your reason for doing so is to get better options for
 * justifying numeric output, have a look in the utils.h file
 * where there are other options available
 *
 * Be aware that doing using printf (or sprintf) may add quite a bit
 * to the size of your code. Also, floating point will still not be
 * available, at least for the AVR processors due to the Arduino
 * compiler settings.
 *
 * When commented out, the printf code is still compiled so your
 * program will increase in size but the output of printf will just
 * not appear anywhere.
 *
 */

//#define USE_PRINTF

/*************************************************************************/
/***
 * Structure definitions used in the software. Declared here for lack of a
 * better place.
 * Robot specific instances and values are in the robot config file.
 */
struct TurnParameters {
  int speed;   // mm/s
  int run_in;  // mm
  int run_out; // mm
  int angle;   // deg
  int omega;   // deg/s
  int alpha;   // deg/s/s
  int trigger; // sensor value
};

/***
 * You may use a slightly different hardware platform than UKMARSBOT
 * Here you can include a suitable hardware configuration to define
 * things like IO pins, ADC channels and so on
 */

#define HARDWARE_UNKNOWN 0
#define HARDWARE_UKMARSBOT_1_3A 1

#define HARDWARE HARDWARE_UKMARSBOT_1_3A

#if HARDWARE == HARDWARE_UKMARSBOT_1_3A
#include "config-hardware-ukmarsbot.h"
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

// choose the one you will be using
// this MUST be defined before selecting the robot below
#define EVENT EVENT_UK

/*************************************************************************/
/***
 * Even with the same basic hardware,you may build robots with different characteristics
 * such as the motor gear ratio or the whieel size of sensor arrangement.
 * These characteristics are kept in their own config files. Add you robot to the list and create
 * a corresponding config file with its custom values.
 *
 * If you have only one robot then you can reduce this section to a single
 * include line.
 */
#define ROBOT_NOT_DEFINED 0
#define ROBOT_FRANK 1
#define ROBOT_ORION 2

#define ROBOT_NAME ROBOT_ORION

#if ROBOT_NAME == ROBOT_FRANK
#include "config-robot-frank.h"
#elif ROBOT_NAME == ROBOT_ORION
#include "config-robot-orion.h"
#else
#error "NO ROBOT DEFINED"
#endif

/*************************************************************************/
/*************************************************************************/

//***************************************************************************//
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

// This is the size for each cell in the maze. Normally 180mm for a classic maze
const float FULL_CELL = 180.0f;
const float HALF_CELL = FULL_CELL / 2.0;

// the position in the cell where the sensors are sampled.
const float SENSING_POSITION = 170.0;

#endif
