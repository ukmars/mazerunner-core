/******************************************************************************
 * Project: <<project>>                                                       *
 * File Created: Sunday, 26th February 2023 5:05:47 pm                        *
 * Author: Peter Harrison                                                     *
 * -----                                                                      *
 * Last Modified: Tuesday, 7th March 2023 2:18:43 pm                          *
 * -----                                                                      *
 * Copyright 2022 - 2023 Peter Harrison, Micromouseonline                     *
 * -----                                                                      *
 * Licence:                                                                   *
 * MIT License                                                                *
 *                                                                            *
 * Copyright (c) 2023 Peter Harrison                                          *
 *                                                                            *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of *
 * this software and associated documentation files (the "Software"), to deal in *
 * the Software without restriction, including without limitation the rights to *
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies *
 * of the Software, and to permit persons to whom the Software is furnished to do *
 * so, subject to the following conditions:                                   *
 *                                                                            *
 * The above copyright notice and this permission notice shall be included in all *
 * copies or substantial portions of the Software.                            *
 *                                                                            *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR *
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,   *
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE *
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER     *
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, *
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE *
 * SOFTWARE.                                                                  *
 ******************************************************************************/
#pragma once

#include <Arduino.h>

// change the revision if the settings structure changes to force rewrite of EEPROM
const uint8_t SETTINGS_REVISION = 110;

//***** SENSOR CALIBRATION **************************************************//
#if EVENT == EVENT_HOME
// wall sensor thresholds and constants
// RAW values for the front sensor when the robot is backed up to a wall
const int FRONT_LEFT_CALIBRATION = 110;
const int FRONT_RIGHT_CALIBRATION = 174;
// RAW values for the side sensors when the robot is centred in a cell
// and there is no wall ahead
const int LEFT_CALIBRATION = 162;
const int RIGHT_CALIBRATION = 258;
// SS90E turn thresholds. This is the front sum reading to trigger a turn
// it changes a bit if there is an adjacent wall. The threshold is set for
// when the robot is 20mm past the threshold.
// This is when there is 90mm between robot front and wall ahead
const int TURN_THRESHOLD_SS90E = 115;
const int EXTRA_WALL_ADJUST = 5;

#elif EVENT == EVENT_UK

// RAW values for the front sensor when the robot is backed up to a wall
const int FRONT_LEFT_CALIBRATION = 83;
const int FRONT_RIGHT_CALIBRATION = 39;
// RAW side sensor values when robot is centred in a cell and wall ahead
const int LEFT_CALIBRATION = 80;
const int RIGHT_CALIBRATION = 72;

// SS90E turn thresholds. This is the front sum reading to trigger a turn
// it changes a bit if there is an adjacent wall. The threshold is set for
// when the robot is 20mm past the threshold.
const int TURN_THRESHOLD_SS90E = 100;
const int EXTRA_WALL_ADJUST = 6;

#elif EVENT == EVENT_PORTUGAL
// wall sensor thresholds and constants
// RAW values for the front sensor when the robot is backed up to a wall
const int FRONT_LEFT_CALIBRATION = 106;
const int FRONT_RIGHT_CALIBRATION = 134;
// RAW values for the side sensors when the robot is centred in a cell
// and there is no wall ahead
const int LEFT_CALIBRATION = 159;
const int RIGHT_CALIBRATION = 255;
// SS90E turn thresholds. This is the front sum reading to trigger a turn
// it changes a bit if there is an adjacent wall. The threshold is set for
// when the robot is 20mm past the threshold.
const int TURN_THRESHOLD_SS90E = 115;
const int EXTRA_WALL_ADJUST = 2;

#elif EVENT == EVENT_APEC
// wall sensor thresholds and constants
// RAW values for the front sensor when the robot is backed up to a wall
const int FRONT_LEFT_CALIBRATION = 88;
const int FRONT_RIGHT_CALIBRATION = 156;
// RAW values for the side sensors when the robot is centred in a cell
// and there is no wall ahead
const int LEFT_CALIBRATION = 125;
const int RIGHT_CALIBRATION = 200;
// SS90E turn thresholds. This is the front sum reading to trigger a turn
// it changes a bit if there is an adjacent wall. The threshold is set for
// when the robot is 20mm past the threshold.
const int TURN_THRESHOLD_SS90E = 115;
const int EXTRA_WALL_ADJUST = 6;

#endif

//***************************************************************************//

//***************************************************************************//
// We need to know about the drive mechanics.

const float WHEEL_DIAMETER = 32.240;
const float ENCODER_PULSES = 36.0;
const float GEAR_RATIO = 10.7917; // 19.54; // 11.40;
const float LEFT_CPM = 3687;
const float RIGHT_CPM = 3760;
const float CPM = (1000.0 * 2.0 * ENCODER_PULSES * GEAR_RATIO) / (PI * WHEEL_DIAMETER); // 4810.0;
// Mouse radius is the distance between the contact patches of the drive wheels.
// A good starting approximation is half the distance between the wheel centres.
// After testing, you may find the working value to be larger or smaller by some
// small amount.
const float MOUSE_RADIUS = 38.070;                          // 39.50; // Adjust on test
const float CPR = (2.0 * PI * MOUSE_RADIUS * CPM) / 1000.0; // 1145

// The robot is likely to have wheels of different diameters and that must be
// compensated for if the robot is to reliably drive in a straight line
// This number is a fraction of the applied motor voltage to be added to the left
// and subtracted from the right motor.
const float ROTATION_BIAS = -0.005; // Negative makes robot curve to left

const float MM_PER_COUNT = PI * WHEEL_DIAMETER / (ENCODER_PULSES * GEAR_RATIO);
const float COUNTS_PER_METER = 1000.0 / MM_PER_COUNT;
// we can calculate wheel diameter as:
// D = (1000 * ENCODER_PULSES * GEAR_RATIO)/(PI * COUNTS_PER_METER)
// push the robot 500mm on the ground and record the encoder sum

const float MM_PER_COUNT_LEFT = (1 - ROTATION_BIAS) * MM_PER_COUNT;
const float MM_PER_COUNT_RIGHT = (1 + ROTATION_BIAS) * MM_PER_COUNT;
const float DEG_PER_MM_DIFFERENCE = (180.0 / (2 * MOUSE_RADIUS * PI));

//*** MOTION CONTROL CONSTANTS **********************************************//

// Dynamic performance constants
const float FWD_KM = 475.0; // mm/s/Volt
const float FWD_TM = 0.190; // forward time constant
const float ROT_KM = 775.0; // deg/s/Volt
const float ROT_TM = 0.210; // rotation time constant

// Motor Feedforward
/***
 * Speed Feedforward is used to add a drive voltage proportional to the motor speed
 * The units are Volts per mm/s and the value will be different for each
 * robot where the motor + gearbox + wheel diamter + robot weight are different
 * You can experimentally determine a suitable value by turning off the controller
 * and then commanding a set voltage to the motors. The same voltage is applied to
 * each motor. Have the robot report its speed regularly or have it measure
 * its steady state speed after a period of acceleration.
 * Do this for several applied voltages from 0.5Volts to 3 Volts in steps of 0.5V
 * Plot a chart of steady state speed against voltage. The slope of that graph is
 * the speed feedforward, SPEED_FF.
 * Note that the line will not pass through the origin because there will be
 * some minimum voltage needed just to ovecome friction and get the wheels to turn at all.
 * That minimum voltage is the BIAS_FF. It is not dependent upon speed but is expressed
 * here as a fraction for comparison.
 */
const float SPEED_FF = (1.0 / FWD_KM);
const float ACC_FF = (FWD_TM / FWD_KM);
const float BIAS_FF = 0.340;
const float TOP_SPEED = (6.0 - BIAS_FF) / SPEED_FF;

// forward motion controller constants
const float FWD_ZETA = 0.707;
const float FWD_TD = FWD_TM;

const float FWD_KP = 16 * FWD_TM / (FWD_KM * FWD_ZETA * FWD_ZETA * FWD_TD * FWD_TD);
const float FWD_KD = LOOP_FREQUENCY * (8 * FWD_TM - FWD_TD) / (FWD_KM * FWD_TD);

// forward motion controller constants
const float ROT_ZETA = 0.707;
const float ROT_TD = ROT_TM;

const float ROT_KP = 16 * ROT_TM / (ROT_KM * ROT_ZETA * ROT_ZETA * ROT_TD * ROT_TD);
const float ROT_KD = LOOP_FREQUENCY * (8 * ROT_TM - ROT_TD) / (ROT_KM * ROT_TD);

// controller constants for the steering controller
const float STEERING_KP = 0.6; // 1.20; // 0.80;
const float STEERING_KD = 0.00;
const float STEERING_ADJUST_LIMIT = 10.0; // deg/s

// encoder polarity is set to account for reversal of the encoder phases
const int ENCODER_LEFT_POLARITY = (-1);
const int ENCODER_RIGHT_POLARITY = (1);

// similarly, the motors may be wired with different polarity and that
// is defined here so that setting a positive voltage always moves the robot
// forwards
const int MOTOR_LEFT_POLARITY = (1);
const int MOTOR_RIGHT_POLARITY = (-1);

//***************************************************************************//

//***** PERFORMANCE CONSTANTS************************************************//
// search and run speeds in mm/s and mm
const int SEARCH_TURN_SPEED = 300;
const int SMOOTH_TURN_SPEED = 500;
const int FAST_TURN_SPEED = 600;

const float SEARCH_ACCELERATION = 2000;
const float SEARCH_SPEED = 400;

const float SPIN_TURN_ACCELERATION = 3600;
const float SPIN_TURN_OMEGA = 360;

const float FAST_RUN_ACCELERATION = 3000;
const int FAST_RUN_SPEED_MAX = 2500;

const int OMEGA_SPIN_TURN = 360;
const int ALPHA_SPIN_TURN = 3600;
//***************************************************************************//

// This is the normalised value seen by the front sensor when the mouse is
// in its calibration position
const int LEFT_NOMINAL = 100;
const int RIGHT_NOMINAL = 100;
const int FRONT_NOMINAL = 100;

// Sensor brightness adjustment factor. The compiler calculates these so it saves processor time
const float FRONT_LEFT_SCALE = (float)FRONT_NOMINAL / FRONT_LEFT_CALIBRATION;
const float FRONT_RIGHT_SCALE = (float)FRONT_NOMINAL / FRONT_RIGHT_CALIBRATION;
const float LEFT_SCALE = (float)LEFT_NOMINAL / LEFT_CALIBRATION;
const float RIGHT_SCALE = (float)RIGHT_NOMINAL / RIGHT_CALIBRATION;

// the values above which, a wall is seen
const int LEFT_THRESHOLD = 40;       // minimum value to register a wall
const int RIGHT_THRESHOLD = 40;      // minimum value to register a wall
const int FRONT_THRESHOLD = 45;      // minimum value to register a wall
const int FRONT_SUM_REFERENCE = 840; // reading when mouse centered with wall ahead
//***************************************************************************//

const int left_edge_pos = 19;
const int right_edge_pos = 20;
