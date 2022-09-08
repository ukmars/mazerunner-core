/*
 * File: motors.h
 * Project: vw-control
 * File Created: Monday, 29th March 2021 11:04:59 pm
 * Author: Peter Harrison
 * -----
 * Last Modified: Sunday, 4th April 2021 11:56:54 pm
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

#ifndef MOTORS_H
#define MOTORS_H

// #include <Arduino.h>

extern float g_left_motor_volts;
extern float g_right_motor_volts;

//***************************************************************************//

void enable_motor_controllers();
void disable_motor_controllers();
void reset_motor_controllers();

/***
 * The motors module provides low  control of the drive motors
 * in a two-wheel differential drive robot.
 */

void update_motor_controllers(float steering_adjustment);

enum { PWM_488_HZ,
       PWM_3906_HZ,
       PWM_31250_HZ };

/***
 *  - set the motor driver pins as outputs
 *  - configure direction to be forwards
 *  - set pwm frequency to default 32kHz
 *  - set pwm drive to zero
 * @brief configure pins and pwm for motor drive
 */
void setup_motors();

/***
 * @brief write zero volts to both motors
 */
void stop_motors();

/***
 * @brief set the motor pwn drive to one of three possible values
 */
void set_motor_pwm_frequency(int frequency = PWM_31250_HZ);

/***
 * -255 <= pwm <= 255
 * @brief set motor direction and PWM
 */
void set_left_motor_pwm(int pwm);
void set_right_motor_pwm(int pwm);

/***
 * The input voltage be any value and will be scaled to compensate for changes
 * in battery voltage such that maximum drive to the motors is limited to the
 * value of MAX_MOTOR_VOLTS in the defaults (normally +/- 6.0 Volts)
 * @brief adjust the motor PWM to deliver the given volate to the motor
 */
void set_left_motor_volts(float volts);
void set_right_motor_volts(float volts);

#endif
