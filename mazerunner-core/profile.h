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

#ifndef PROFILE_H
#define PROFILE_H

#include <Arduino.h>
#include "config.h"
//***************************************************************************//
class Profile;

extern Profile forward;
extern Profile rotation;

/***
 * The Profile class manages speed as a function of time or distance. A profile
 * is trapezoidal in shape and consists of up to three phases.
 *
 *  - Acceleration : changing from the initial speed to the maximum speed
 *  - Coasting     : moving at the maximum speed
 *  - Braking      : changing to the final speed
 *
 * The names are only really relevant to a subset of all possible profiles since
 * the coasting section may not be possible if the acceleration is low or the
 * maximum speed is otherwise unreachable. Also, note that the initial speed may
 * be higher than the maximum speed so the first phase actually needs braking.
 *
 * Once started, the profile must be regularly updated from the system control
 * loop and this must be done before the motor control task since it uses the
 * current profile speed in its calculation.
 *
 * The robot has two instances of a Profile - one for forward motion and one
 * for rotation. These are completely independent but the combination of the
 * two allows the robot full freedom of motion within the constraints imposed
 * by the drive arrangement. That is, any combination of forward and rotary
 * motion is possible at all times.
 *
 * Although the units in the comments are shown as mm, the class is unit
 * agnostic and you can interpret the units as mm, cm, deg, bananas or anything
 * else. Just be consistent when using speed and acceleration.
 */
class Profile {
 public:
  enum State : uint8_t {
    PS_IDLE = 0,
    PS_ACCELERATING = 1,
    PS_BRAKING = 2,
    PS_FINISHED = 3,
  };

  void reset() {
    ATOMIC {
      m_position = 0;
      m_speed = 0;
      m_target_speed = 0;
      m_state = PS_IDLE;
    }
  }

  bool is_finished() {
    return m_state == PS_FINISHED;
  }

  /// @brief  Begin a profile. Once started, it will automatically run to completion
  ///         Subsequent calls before completion supercede all the parameters.
  ///         Called may monitor progress using is_finished() method
  /// @param distance     (mm)     always positive
  /// @param top_speed    (mm/s)   negative values move the robot in reverse
  /// @param final_speed  (mm/s)
  /// @param acceleration (mm/s/s)

  void start(float distance, float top_speed, float final_speed, float acceleration) {
    m_sign = (distance < 0) ? -1 : +1;
    if (distance < 0) {
      distance = -distance;
    }
    if (distance < 1.0) {
      m_state = PS_FINISHED;
      return;
    }
    if (final_speed > top_speed) {
      final_speed = top_speed;
    }

    m_position = 0;
    m_final_position = distance;
    m_target_speed = m_sign * fabsf(top_speed);
    m_final_speed = m_sign * fabsf(final_speed);
    m_acceleration = fabsf(acceleration);
    if (m_acceleration >= 1) {
      m_one_over_acc = 1.0f / m_acceleration;
    } else {
      m_one_over_acc = 1.0;
    }
    m_state = PS_ACCELERATING;
  }

  // Start a profile and wait for it to finish. This is a blocking call.
  void move(float distance, float top_speed, float final_speed, float acceleration) {
    start(distance, top_speed, final_speed, acceleration);
    wait_until_finished();
  }

  /// @brief causes the profile to immediately terminate with the speed to zero
  ///        note that even when the state is PS_FINISHED, the profiler will
  ///        continue to try and reach the target speed. (zero in this case)
  void stop() {
    ATOMIC {
      m_target_speed = 0;
    }
    finish();
  }

  /// @brief  Force a profile to finish with the target speed set to the final speed
  void finish() {
    ATOMIC {
      m_speed = m_target_speed;
      m_state = PS_FINISHED;
    }
  }

  void wait_until_finished() {
    while (m_state != PS_FINISHED) {
      delay(2);
    }
  }

  void set_state(State state) {
    m_state = state;
  }

  /// @brief  Calculate the distance needed to get to the final speed from the
  ///         current speed using the current acceleration.
  /// @return distance (mm)
  float get_braking_distance() {
    return fabsf(m_speed * m_speed - m_final_speed * m_final_speed) * 0.5 * m_one_over_acc;
  }

  /// @brief  gets the distance travelled (mm) since the last call to start(). If there
  ///         was a prior call to set_position() distance is incremented from there.
  /// @return distance travelled (mm)
  float position() {
    float pos;
    ATOMIC {
      pos = m_position;
    }
    return pos;
  }

  /// @brief Get the current speed
  /// @return
  float speed() {
    float speed;
    ATOMIC {
      speed = m_speed;
    }
    return speed;
  }

  float acceleration() {
    float acc;
    ATOMIC {
      acc = m_acceleration;
    }
    return acc;
  }

  void set_speed(float speed) {
    ATOMIC {
      m_speed = speed;
    }
  }
  void set_target_speed(float speed) {
    ATOMIC {
      m_target_speed = speed;
    }
  }

  // normally only used to alter position for forward error correction
  void adjust_position(float adjustment) {
    ATOMIC {
      m_position += adjustment;
    }
  }

  void set_position(float position) {
    ATOMIC {
      m_position = position;
    }
  }

  // update is called from within systick and should be safe from interrupts
  void update() {
    if (m_state == PS_IDLE) {
      return;
    }
    float delta_v = m_acceleration * LOOP_INTERVAL;
    float remaining = fabsf(m_final_position) - fabsf(m_position);
    if (m_state == PS_ACCELERATING) {
      if (remaining < get_braking_distance()) {
        m_state = PS_BRAKING;
        if (m_final_speed == 0) {
          m_target_speed = m_sign * 5.0f;  // magic number to make sure we reach zero
        } else {
          m_target_speed = m_final_speed;
        };
      }
    }
    // try to reach the target speed
    if (m_speed < m_target_speed) {
      m_speed += delta_v;
      if (m_speed > m_target_speed) {
        m_speed = m_target_speed;
      }
    }
    if (m_speed > m_target_speed) {
      m_speed -= delta_v;
      if (m_speed < m_target_speed) {
        m_speed = m_target_speed;
      }
    }
    // increment the position
    m_position += m_speed * LOOP_INTERVAL;
    if (m_state != PS_FINISHED && remaining < 0.125) {
      m_state = PS_FINISHED;
      m_target_speed = m_final_speed;
    }
  }

 private:
  volatile uint8_t m_state = PS_IDLE;
  volatile float m_speed = 0;
  volatile float m_position = 0;
  int8_t m_sign = 1;
  float m_acceleration = 0;
  float m_one_over_acc = 1;
  float m_target_speed = 0;
  float m_final_speed = 0;
  float m_final_position = 0;
};

#endif