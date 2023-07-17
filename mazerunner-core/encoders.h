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

#include "config.h"
#include "digitalWriteFast.h"
#include <Arduino.h>
#include <stdint.h>
#include <util/atomic.h>

/***
 *
 * The encoders provide data for loaclisation of the robot. That is, the encoder
 * data is used to determine the position and orientation of the robot in its
 * environment. If it had one, the robot might also combine encoder data with
 * information form an IMU (gyroscope and accelerometer) to get a more accurate
 * and stable measure of pose (position + orientation).
 *
 * Encoders are subject to various errors and are not reliable when used alone.
 * In particular, simple robots like UKMARSBOt are likely to have low resolution
 * encoders and considerable backlash in the geartrain.
 *
 */

// TODO: consider a single Encoder class and an Odometry class
// that has two Encoder instances

class Encoders;
// TODO: where should we really define  these global singletons?
extern Encoders encoders; // defined in main file to keep it all together

// Forward declaration of the callbacks ...
// ... which cannot be defined until the Encoder class is complete.
// tacky!
void callback_left();
void callback_right();

class Encoders {
public:
  void setup() {
    pinMode(ENCODER_LEFT_CLK, INPUT);
    pinMode(ENCODER_LEFT_B, INPUT);
    pinMode(ENCODER_RIGHT_CLK, INPUT);
    pinMode(ENCODER_RIGHT_B, INPUT);
    attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_CLK), callback_left, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_CLK), callback_right, CHANGE);
    reset();
  }

  void reset() {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
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
   */
  void left_input_change() {
    static bool oldA = false;
    static bool oldB = false;
    bool newB = digitalReadFast(ENCODER_LEFT_B);
    bool newA = digitalReadFast(ENCODER_LEFT_CLK) ^ newB;
    int delta = ENCODER_LEFT_POLARITY * ((oldA ^ newB) - (newA ^ oldB));
    m_left_counter += delta;
    oldA = newA;
    oldB = newB;
  }

  void right_input_change() {
    static bool oldA = false;
    static bool oldB = false;
    bool newB = digitalReadFast(ENCODER_RIGHT_B);
    bool newA = digitalReadFast(ENCODER_RIGHT_CLK) ^ newB;
    int delta = ENCODER_RIGHT_POLARITY * ((oldA ^ newB) - (newA ^ oldB));
    m_right_counter += delta;
    oldA = newA;
    oldB = newB;
  }

  void update() {
    int left_delta = 0;
    int right_delta = 0;
    // Make sure values don't change while being read. Be quick.
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      left_delta = m_left_counter;
      right_delta = m_right_counter;
      m_left_counter = 0;
      m_right_counter = 0;
    }
    float left_change = left_delta * MM_PER_COUNT_LEFT;
    float right_change = right_delta * MM_PER_COUNT_RIGHT;
    m_fwd_change = 0.5 * (right_change + left_change);
    m_rot_change = (right_change - left_change) * DEG_PER_MM_DIFFERENCE;
    m_robot_distance += m_fwd_change;
    m_robot_angle += m_rot_change;
  }

  float robot_distance() {
    float distance;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { distance = m_robot_distance; }
    return distance;
  }

  float robot_speed() {
    float speed;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { speed = LOOP_FREQUENCY * m_fwd_change; }
    return speed;
  }

  float robot_omega() {
    float omega;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { omega = LOOP_FREQUENCY * m_rot_change; }
    return omega;
  }

  float robot_fwd_change() {
    float distance;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { distance = m_fwd_change; }
    return distance;
  }

  float robot_rot_change() {
    float distance;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { distance = m_rot_change; }
    return distance;
  }

  float robot_angle() {
    float angle;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { angle = m_robot_angle; }
    return angle;
  }

  // None of the variables in this file should be directly available to the rest
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
inline void callback_left() {
  encoders.left_input_change();
}

inline void callback_right() {
  encoders.right_input_change();
}