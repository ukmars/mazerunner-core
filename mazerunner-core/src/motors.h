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
#include "../config.h"
#include "digitalWriteFast.h"
#include "encoders.h"
#include "profile.h"
#include "sensors.h"

enum { PWM_488_HZ,
       PWM_3906_HZ,
       PWM_31250_HZ };

extern float g_left_motor_volts;
extern float g_right_motor_volts;


class Motors {
  public:
  bool s_controllers_output_enabled;
  float s_old_fwd_error;
  float s_old_rot_error;
  float s_fwd_error;
  float s_rot_error;

  void enable_motor_controllers() {
    s_controllers_output_enabled = true;
  }

  void disable_motor_controllers() {
    s_controllers_output_enabled = false;
  }

  void reset_motor_controllers() {
    s_fwd_error = 0;
    s_rot_error = 0;
    s_old_fwd_error = 0;
    s_old_rot_error = 0;
  }

  void setup_motors() {
    pinMode(MOTOR_LEFT_DIR, OUTPUT);
    pinMode(MOTOR_RIGHT_DIR, OUTPUT);
    pinMode(MOTOR_LEFT_PWM, OUTPUT);
    pinMode(MOTOR_RIGHT_PWM, OUTPUT);
    digitalWriteFast(MOTOR_LEFT_PWM, 0);
    digitalWriteFast(MOTOR_LEFT_DIR, 0);
    digitalWriteFast(MOTOR_RIGHT_PWM, 0);
    digitalWriteFast(MOTOR_RIGHT_DIR, 0);
    set_motor_pwm_frequency();
    stop_motors();
  }

  float position_controller() {
    s_fwd_error += forward.increment() - robot_fwd_increment();
    float diff = s_fwd_error - s_old_fwd_error;
    s_old_fwd_error = s_fwd_error;
    float output = FWD_KP * s_fwd_error + FWD_KD * diff;
    return output;
  }

  float angle_controller(float steering_adjustment) {
    s_rot_error += rotation.increment() - robot_rot_increment();
    s_rot_error += steering_adjustment;
    float diff = s_rot_error - s_old_rot_error;
    s_old_rot_error = s_rot_error;
    float output = ROT_KP * s_rot_error + ROT_KD * diff;
    return output;
  }

  void update_motor_controllers(float steering_adjustment) {
    float pos_output = position_controller();
    float rot_output = angle_controller(steering_adjustment);
    float left_output = 0;
    float right_output = 0;
    left_output = pos_output - rot_output;
    right_output = pos_output + rot_output;
    if (s_controllers_output_enabled) {
      set_right_motor_volts(right_output);
      set_left_motor_volts(left_output);
    }
  }
  /**
   * Direct register access could be used here for enhanced performance
   */
  void set_left_motor_pwm(int pwm) {
    pwm = MOTOR_LEFT_POLARITY * constrain(pwm, -255, 255);
    if (pwm < 0) {
      digitalWriteFast(MOTOR_LEFT_DIR, 1);
      analogWrite(MOTOR_LEFT_PWM, -pwm);
    } else {
      digitalWriteFast(MOTOR_LEFT_DIR, 0);
      analogWrite(MOTOR_LEFT_PWM, pwm);
    }
  }

  void set_right_motor_pwm(int pwm) {
    pwm = MOTOR_RIGHT_POLARITY * constrain(pwm, -255, 255);
    if (pwm < 0) {
      digitalWriteFast(MOTOR_RIGHT_DIR, 1);
      analogWrite(MOTOR_RIGHT_PWM, -pwm);
    } else {
      digitalWriteFast(MOTOR_RIGHT_DIR, 0);
      analogWrite(MOTOR_RIGHT_PWM, pwm);
    }
  }

  void set_left_motor_volts(float volts) {
    volts = constrain(volts, -MAX_MOTOR_VOLTS, MAX_MOTOR_VOLTS);
    g_left_motor_volts = volts;
    int motorPWM = (int)(volts * g_battery_scale);
    set_left_motor_pwm(motorPWM);
  }

  void set_right_motor_volts(float volts) {
    volts = constrain(volts, -MAX_MOTOR_VOLTS, MAX_MOTOR_VOLTS);
    g_right_motor_volts = volts;
    int motorPWM = (int)(volts * g_battery_scale);
    set_right_motor_pwm(motorPWM);
  }

void set_motor_pwm_frequency(int frequency = PWM_31250_HZ){
    switch (frequency) {
      case PWM_31250_HZ:
        // Divide by 1. frequency = 31.25 kHz;
        bitClear(TCCR1B, CS11);
        bitSet(TCCR1B, CS10);
        break;
      case PWM_3906_HZ:
        // Divide by 8. frequency = 3.91 kHz;
        bitSet(TCCR1B, CS11);
        bitClear(TCCR1B, CS10);
        break;
      case PWM_488_HZ:
      default:
        // Divide by 64. frequency = 488Hz;
        bitSet(TCCR1B, CS11);
        bitSet(TCCR1B, CS10);
        break;
    }
  }

  void stop_motors() {
    set_left_motor_volts(0);
    set_right_motor_volts(0);
  }
};

extern Motors motors;

#endif
