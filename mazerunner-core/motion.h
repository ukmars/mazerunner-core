/******************************************************************************
 * Project: mazerunner-core                                                   *
 * File:    motion.h                                                          *
 * File Created: Saturday, 10th September 2022 11:18:32 am                    *
 * Author: Peter Harrison                                                     *
 * -----                                                                      *
 * Last Modified: Thursday, 3rd November 2022 8:50:31 am                      *
 * -----                                                                      *
 * Copyright 2022 - 2022 Peter Harrison, Micromouseonline                     *
 * -----                                                                      *
 * Licence:                                                                   *
 *     Use of this source code is governed by an MIT-style                    *
 *     license that can be found in the LICENSE file or at                    *
 *     https://opensource.org/licenses/MIT.                                   *
 ******************************************************************************/

#ifndef MOTION_H
#define MOTION_H

#include "motors.h"
#include "profile.h"
#include "sensors.h"
#include "serial.h"
#include <Arduino.h>
class Motion {
public:
  /**
   * Before the robot begins a sequence of moves, this method can be used to
   * make sure everything starts off in a known state.
   *
   * @brief Reset profiles, counters and controllers. Motors off. Steering off.
   */
  void reset_drive_system() {
    motors.stop();
    motors.disable_controllers();
    sensors.set_steering_mode(STEERING_OFF);
    encoders.reset();
    forward.reset();
    rotation.reset();
    motors.reset_controllers();
    motors.enable_controllers();
  }

  //***************************************************************************//

  /**
   * Performs a turn. Regardless of whether the robot is moving or not
   *
   * The function is given three parameters
   *
   *  - angle  : positive is a left turn (deg)
   *  - omega  : angular velocity of middle phase (deg/s)
   *  - alpha  : angular acceleration of in/out phases (deg/s/s)
   *
   * If the robot is moving forward, it will execute a smooth, integrated
   * turn. The turn will only be repeatable if it is always performed at the
   * same forward speed.
   *
   * If the robot is stationary, it will execute an in-place spin turn.
   *
   * The parameter alpha will indirectly determine the turn radius. During
   * the accelerating phase, the angular velocity, will increase until it
   * reaches the value omega.
   * The minimum radius during the constant phase is
   *   radius = (speed/omega) * (180/PI)
   * The effective radius will be larger because it takes some time
   * for the rotation to accelerate and decelerate. The parameter alpha
   * controls that.
   *
   * Note that a real mouse may behave slightly different for left and
   * right turns and so the parameters for, say, a 90 degree left turn
   * may be slightly different to those for a 90 degree right turn.
   *
   * @brief execute an arbitrary in-place or smooth turn
   */
  void turn(float angle, float omega, float alpha) {
    // get ready to turn
    rotation.reset();
    rotation.start(angle, omega, 0, alpha);
    rotation.wait_until_finished();
  }

  /**
   *
   * @brief turn in place. Force forward speed to zero
   */
  void spin_turn(float angle, float omega, float alpha) {
    forward.set_target_speed(0);
    while (forward.speed() != 0) {
      delay(2);
    }
    turn(angle, omega, alpha);
  };

  //***************************************************************************//
  /**
   * These are examples of ways to use the motion control functions
   */

  /**
   * The robot is assumed to be moving. This call will stop at a specific
   * distance. Clearly, there must be enough distance remaining for it to
   * brake to a halt.
   *
   * The current values for speed and acceleration are used.
   *
   * Calling this with the robot stationary is undefined. Don't do that.
   *
   * @brief bring the robot to a halt at a specific distance
   */
  void stop_at(float position) {
    float remaining = position - forward.position();
    console.print(F("STOP AT"));
    forward.start(remaining, forward.speed(), 0, forward.acceleration());
    forward.wait_until_finished();
  }

  /**
   * The robot is assumed to be moving. This call will stop  after a
   * specific distance has been travelled
   *
   * Clearly, there must be enough distance remaining for it to
   * brake to a halt.
   *
   * The current values for speed and acceleration are used.
   *
   * Calling this with the robot stationary is undefined. Don't do that.
   *
   * @brief bring the robot to a halt after a specific distance
   */
  void stop_after(float distance) {
    console.print(F("STOP AFTER"));
    forward.start(distance, forward.speed(), 0, forward.acceleration());
    forward.wait_until_finished();
  }

  /**
   * The robot is assumed to be moving. This utility function call will just
   * do a busy-wait until the forward profile gets to the supplied position.
   *
   * @brief wait until the given position is reached
   */
  void wait_until_position(float position) {
    while (forward.position() < position) {
      delay(2);
    }
  }

  /**
   * The robot is assumed to be moving. This utility function call will just
   * do a busy-wait until the forward profile has moved by the given distance.
   *
   * @brief wait until the given distance has been travelled
   */
  void wait_until_distance(float distance) {
    float target = forward.position() + distance;
    wait_until_position(target);
  }
};

extern Motion motion;

#endif
