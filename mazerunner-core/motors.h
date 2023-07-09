/******************************************************************************
 * Project: mazerunner-core                                                   *
 * File:    motors.h                                                          *
 * File Created: Saturday, 10th September 2022 4:55:17 pm                     *
 * Author: Peter Harrison                                                     *
 * -----                                                                      *
 * Last Modified: Saturday, 29th October 2022 2:35:00 pm                      *
 * -----                                                                      *
 * Copyright 2022 - 2022 Peter Harrison, Micromouseonline                     *
 * -----                                                                      *
 * Licence:                                                                   *
 *     Use of this source code is governed by an MIT-style                    *
 *     license that can be found in the LICENSE file or at                    *
 *     https://opensource.org/licenses/MIT.                                   *
 ******************************************************************************/

#ifndef MOTORS_H
#define MOTORS_H

#include "battery.h"
#include "config.h"
#include "digitalWriteFast.h"
#include "encoders.h"
#include "profile.h"
#include <Arduino.h>
// #include "sensors.h"

enum { PWM_488_HZ,
       PWM_977_HZ,
       PWM_3906_HZ,
       PWM_31250_HZ };

class Motors {
public:
  /***
   *
   */
  void enable_controllers() {
    m_controller_output_enabled = true;
  }

  void disable_controllers() {
    m_controller_output_enabled = false;
  }

  void reset_controllers() {
    m_fwd_error = 0;
    m_rot_error = 0;
    m_previous_fwd_error = 0;
    m_previous_rot_error = 0;
  }

  void stop() {
    set_left_motor_volts(0);
    set_right_motor_volts(0);
  }

  void setup() {
    pinMode(MOTOR_LEFT_DIR, OUTPUT);
    pinMode(MOTOR_RIGHT_DIR, OUTPUT);
    pinMode(MOTOR_LEFT_PWM, OUTPUT);
    pinMode(MOTOR_RIGHT_PWM, OUTPUT);
    digitalWriteFast(MOTOR_LEFT_PWM, 0);
    digitalWriteFast(MOTOR_LEFT_DIR, 0);
    digitalWriteFast(MOTOR_RIGHT_PWM, 0);
    digitalWriteFast(MOTOR_RIGHT_DIR, 0);
    set_pwm_frequency();
    stop();
  }

  float position_controller() {
    m_fwd_error += forward.increment() - encoders.robot_fwd_change();
    float diff = m_fwd_error - m_previous_fwd_error;
    m_previous_fwd_error = m_fwd_error;
    float output = FWD_KP * m_fwd_error + FWD_KD * diff;
    return output;
  }

  float angle_controller(float steering_adjustment) {
    m_rot_error += rotation.increment() - encoders.robot_rot_change();
    m_rot_error += steering_adjustment;
    float diff = m_rot_error - m_previous_rot_error;
    m_previous_rot_error = m_rot_error;
    float output = ROT_KP * m_rot_error + ROT_KD * diff;
    return output;
  }

  /** calculate the voltage to be applied to the motor for a given speed
   *  the drive train is not symmetric and there is significant stiction.
   *  If used with PID, a simpler, single value will be sufficient.
   *
   *  Multiply by (1/x) is generally more efficient than just divide by x
   */

  float leftFeedForward(float speed) {
    static float oldSpeed = 0;
    float leftFF = speed * SPEED_FF + BIAS_FF;
    float acc = (speed - oldSpeed) * LOOP_FREQUENCY;
    oldSpeed = speed;
    float accFF = ACC_FF * acc;
    leftFF += accFF;
    return leftFF;
  }

  float rightFeedForward(float speed) {
    static float oldSpeed = 0;
    float rightFF = speed * SPEED_FF + BIAS_FF;
    float acc = (speed - oldSpeed) * LOOP_FREQUENCY;
    oldSpeed = speed;
    float accFF = ACC_FF * acc;
    rightFF += accFF;
    return rightFF;
  }

  void update_controllers(float steering_adjustment) {
    float pos_output = position_controller();
    float rot_output = angle_controller(steering_adjustment);
    float left_output = 0;
    float right_output = 0;
    left_output = pos_output - rot_output;
    right_output = pos_output + rot_output;

    float tangent_speed = rotation.speed() * MOUSE_RADIUS * (1 / 57.29);
    float left_speed = forward.speed() - tangent_speed;
    float right_speed = forward.speed() + tangent_speed;
    float left_ff = leftFeedForward(left_speed);
    float right_ff = rightFeedForward(right_speed);
    if (m_feedforward_enabled) {
      left_output += left_ff;
      right_output += right_ff;
    }
    if (m_controller_output_enabled) {
      set_right_motor_volts(right_output);
      set_left_motor_volts(left_output);
    }
  }

  int get_fwd_millivolts() {
    return 1000 * (get_right_motor_volts() + get_left_motor_volts());
  }

  int get_rot_millivolts() {
    return 1000 * (get_right_motor_volts() - get_left_motor_volts());
  }

  float get_left_motor_volts() {
    float volts = 0;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      volts = m_left_motor_volts;
    }
    return volts;
  }

  float get_right_motor_volts() {
    float volts = 0;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      volts = m_right_motor_volts;
    }
    return volts;
  }

  void set_left_motor_volts(float volts) {
    volts = constrain(volts, -MAX_MOTOR_VOLTS, MAX_MOTOR_VOLTS);
    m_left_motor_volts = volts;
    int motorPWM = (int)(volts * battery.compensation());
    set_left_motor_pwm(motorPWM);
  }

  void set_right_motor_volts(float volts) {
    volts = constrain(volts, -MAX_MOTOR_VOLTS, MAX_MOTOR_VOLTS);
    m_right_motor_volts = volts;
    int motorPWM = (int)(volts * battery.compensation());
    set_right_motor_pwm(motorPWM);
  }

  /***
   * PWM values are constrained to +/- 255 since the default for
   * analogueWrite is 8 bits. The sign is only used to determine
   * the direction.
   *
   * NOTE: it might be wise to check the resolution of the
   * analogueWrite function in other targtes
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

  void set_pwm_frequency(int frequency = PWM_31250_HZ) {
#if defined(ARDUINO_ARCH_AVR)
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
#elif defined(ARDUINO_ARCH_MEGAAVR)
    // TCA0 is used for analogWrite on pins 9 and 10
    // The clock for TCA0 is also used as the clock for TCBx
    // so changing that screws up millis() and anythin else timed of TCBx
    // so just ignore a frequency change request and leave it at 977Hz
#endif
  }

private:
  bool m_controller_output_enabled;
  bool m_feedforward_enabled = true;
  float m_previous_fwd_error;
  float m_previous_rot_error;
  float m_fwd_error;
  float m_rot_error;
  // these are maintained only for logging
  float m_left_motor_volts;
  float m_right_motor_volts;
};

extern Motors motors;

#endif
