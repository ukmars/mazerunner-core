/*
 * File: motion.h
 * Project: mazerunner
 * File Created: Wednesday, 24th March 2021 6:29:08 pm
 * Author: Peter Harrison
 * -----
 * Last Modified: Friday, 30th April 2021 11:06:46 am
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

#ifndef MOTION_H
#define MOTION_H

#include "motors.h"
#include "profile.h"
#include "sensors.h"
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
    motors.stop_motors();
    motors.disable_motor_controllers();
    sensors.set_steering_mode(STEERING_OFF);
    encoders.reset();
    motors.reset_motor_controllers();
    forward.reset();
    rotation.reset();
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
    while (not rotation.is_finished()) {
      delay(2);
    }
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
    forward.start(remaining, forward.speed(), 0, forward.acceleration());
    Serial.print("STOP AT");
    while (not forward.is_finished()) {
      delay(2);
    }
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
    forward.start(distance, forward.speed(), 0, forward.acceleration());
    Serial.print("STOP AFTER");
    while (not forward.is_finished()) {
      delay(2);
    }
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
