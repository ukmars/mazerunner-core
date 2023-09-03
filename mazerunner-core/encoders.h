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

#pragma once

#include <Arduino.h>
#include <stdint.h>
#include "config.h"

/*******************************************************************************
 *
 * The encoders provide data for localisation of the robot. That is, the encoder
 * data is used to determine the position and orientation of the robot in its
 * environment. If it had one, the robot might also combine encoder data with
 * information form an IMU (gyroscope and accelerometer) to get a more accurate
 * and stable measure of pose (position + orientation).
 *
 * Encoders are subject to various errors and are not reliable when used alone.
 * In particular, simple robots like UKMARSBOt are likely to have low resolution
 * encoders and considerable backlash in the geartrain.
 *
 * ****************************************************************************/

// TODO: consider a single Encoder class with objects for each wheel.
//       Then a Localisation class would get two of these (and possibly
//       an IMU) to do the actual localisation.

class Encoders;

extern Encoders encoders;  // defined in main file to keep them all together

// Forward declaration of the callbacks ...
// ... which cannot be defined until the Encoder class is complete.
// tacky!
void callback_left_encoder_isr();
void callback_right_encoder_isr();

class Encoders {
 public:
  void begin() {
    pinMode(ENCODER_LEFT_CLK, INPUT);
    pinMode(ENCODER_LEFT_B, INPUT);
    pinMode(ENCODER_RIGHT_CLK, INPUT);
    pinMode(ENCODER_RIGHT_B, INPUT);
    attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_CLK), callback_left_encoder_isr, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_CLK), callback_right_encoder_isr, CHANGE);
    reset();
  }

  void reset() {
    ATOMIC {
      m_left_counter = 0;
      m_right_counter = 0;
      m_robot_distance = 0;
      m_robot_angle = 0;
    }
  }

  /**
   * For the ATmega328 and ATmega4809:
   *   Measurements indicate that even at 1500mm/s the total load due to
   *   the encoder interrupts is less than 3% of the available bandwidth.
   *   The ISR will respond to the XOR-ed pulse train from the encoder
   *   Runs in constant time of around 3us per interrupt.
   *   Would be faster with direct port access
   *
   * A more generic solution where the pin names are not constants would be slower
   * unless we can make their definition known at compile time.
   *
   * Note that the decoding method used here assumes that one of the channels
   * is a frequency-doubled clock signal. It is not decoding a 'normal'
   * quadrature input.
   */
  void left_input_change() {
    static bool oldA = false;
    static bool oldB = false;
    // bool newB = digitalReadFast(ENCODER_LEFT_B);
    bool newB = fast_read_pin(ENCODER_LEFT_B);
    bool newA = fast_read_pin(ENCODER_LEFT_CLK) ^ newB;
    int delta = ENCODER_LEFT_POLARITY * ((oldA ^ newB) - (newA ^ oldB));
    m_left_counter += delta;
    oldA = newA;
    oldB = newB;
  }

  void right_input_change() {
    static bool oldA = false;
    static bool oldB = false;
    bool newB = fast_read_pin(ENCODER_RIGHT_B);
    bool newA = fast_read_pin(ENCODER_RIGHT_CLK) ^ newB;
    int delta = ENCODER_RIGHT_POLARITY * ((oldA ^ newB) - (newA ^ oldB));
    m_right_counter += delta;
    oldA = newA;
    oldB = newB;
  }

  /**
   * @brief update the robot speeds and positions from the encoders
   *
   * The update method is called during each control cycle from the
   * systick event. It will use the change in encoder value since the
   * last call to update values for the current speed, angular velocity,
   * distance travelled and robot angle.
   *
   * The recorded speeds are going to show quite a lot of noise when
   * the encoder resolution is poor. Positions effectively integrate out
   * a fair bit of the noise.
   *
   * The speeds are not recorded directly, only the changes in encoder
   * readings. This slightly reduces the computational load in the
   * systick event and the low-level controllers are only using the
   * changes in robot position and angle, not the speed directly.
   * If you need to see a value for the current speed or angular
   * velocity in real units, use the robot_speed() and robot_omeag()
   * methods.
   *
   * Because update() is called from an interrupt service routine it
   * will be changing the values in ways that the higher level code may
   * not be able to know about. Always use the methods provided that
   * guard against unpredictable changes.
   *
   * If using an IMU, prefer that for measurement of angular velocity
   * and angle.
   */
  void update() {
    int left_delta = 0;
    int right_delta = 0;
    // Make sure values don't change while being read. Be quick.
    ATOMIC {
      left_delta = m_left_counter;
      right_delta = m_right_counter;
      m_left_counter = 0;
      m_right_counter = 0;
    }
    float left_change = left_delta * MM_PER_COUNT_LEFT;
    float right_change = right_delta * MM_PER_COUNT_RIGHT;
    m_fwd_change = 0.5 * (right_change + left_change);
    m_robot_distance += m_fwd_change;
    m_rot_change = (right_change - left_change) * DEG_PER_MM_DIFFERENCE;
    m_robot_angle += m_rot_change;
  }

  /**
   * These convenience methods provide safe access to the recorded values
   * from the encoders.
   *
   * The ATOMIC_BLOCK guards ensure that the values canot change  while
   * they are being retreived and are needed because the update() method
   * is called from an interrupt service routine. The guard block will
   * temporarily disable any other interrupts for the duration of the
   * block
   *
   * On 32 bit processors, they would not be needed and their use in this
   * code is sometimes not needed. However, the guards do not significantly
   * affect performance. They insert only three machine instructions per
   * guard block and the compiler will almost certainly inline the method
   * calls so there will not even be a function call overhead.
   */
  float robot_distance() {
    float distance;
    ATOMIC {
      distance = m_robot_distance;
    }
    return distance;
  }

  float robot_speed() {
    float speed;
    ATOMIC {
      speed = LOOP_FREQUENCY * m_fwd_change;
    }
    return speed;
  }

  float robot_omega() {
    float omega;
    ATOMIC {
      omega = LOOP_FREQUENCY * m_rot_change;
    }
    return omega;
  }

  float robot_fwd_change() {
    float distance;
    ATOMIC {
      ;
      distance = m_fwd_change;
    }
    return distance;
  }

  float robot_rot_change() {
    float distance;
    ATOMIC {
      distance = m_rot_change;
    }
    return distance;
  }

  float robot_angle() {
    float angle;
    ATOMIC {
      angle = m_robot_angle;
    }
    return angle;
  }

  // None of the variables in this class should be directly available to the rest
  // of the code without a guard to ensure atomic access
 private:
  volatile float m_robot_distance;
  volatile float m_robot_angle;
  // the change in distance or angle in the last tick.
  float m_fwd_change;
  float m_rot_change;
  // internal use only to track encoder input edges
  int m_left_counter;
  int m_right_counter;
};

// A bit of indirection for convenience because the encoder instance is
// unknown until the linker has done its thing
// This is ugly
inline void callback_left_encoder_isr() {
  encoders.left_input_change();
}

inline void callback_right_encoder_isr() {
  encoders.right_input_change();
}