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

#ifndef MOTION_H
#define MOTION_H

#include <Arduino.h>
#include "motors.h"
#include "profile.h"

/***
 *
 * The motion class handles all the higher level Locomotion tasks. It is responsible
 * for converting instructions from a path planner into actual movement of the
 * robot.
 *
 * Motion control needs to know about the dynamics and kinematics of the robot
 * and may need environmental data as well. In particular, the battery voltage and
 * characteristics of the motors will be important.
 *
 * The output from the motion controller should be suitable control signals for
 * the actuators that make the robot move. In UKMARSBOT, that will simply be
 * voltages for each of the two drive motors. Other robots may have more motors,
 * an independent steering system or other kinds of effectors such as grippers or
 * weapons. Remember that the motors may be stepper motors which will need a
 * slightly modified approach.
 *
 * Here, the motion control makes use of simple trapezoidal profiles to calculate
 * continuously varying speeds for linear and rotary motion components.
 *
 * How these speeds are converted into individual control signals for the drive
 * system is the job of the Motors class. Thus is should be relatively easy to
 * change that class to suit different drive systems.
 *
 * TODO: Motion should have be given the profilers
 * TODO: should it also be given the motors?
 */
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
    encoders.reset();
    forward.reset();
    rotation.reset();
    motors.reset_controllers();
    motors.enable_controllers();
  }

  void stop() {
    motors.stop();
  }

  void disable_drive() {
    motors.disable_controllers();
  }

  float position() {
    return forward.position();
  }

  float velocity() {
    return forward.speed();
  }

  float acceleration() {
    return forward.acceleration();
  }

  void set_target_velocity(float velocity) {
    forward.set_target_speed(velocity);
  }

  float angle() {
    return rotation.position();
  }

  float omega() {
    return rotation.speed();
  }

  float alpha() {
    return rotation.acceleration();
  }

  void start_move(float distance, float top_speed, float final_speed, float acceleration) {
    forward.start(distance, top_speed, final_speed, acceleration);
  }

  bool move_finished() {
    return forward.is_finished();
  }

  void move(float distance, float top_speed, float final_speed, float acceleration) {
    forward.move(distance, top_speed, final_speed, acceleration);
  }

  void start_turn(float distance, float top_speed, float final_speed, float acceleration) {
    rotation.start(distance, top_speed, final_speed, acceleration);
  }

  bool turn_finished() {
    return rotation.is_finished();
  }

  void turn(float distance, float top_speed, float final_speed, float acceleration) {
    rotation.move(distance, top_speed, final_speed, acceleration);
  }

  void update() {
    forward.update();
    rotation.update();
  }

  void set_position(float pos) {
    forward.set_position(pos);
  }

  void adjust_forward_position(float delta) {
    forward.adjust_position(delta);
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
    rotation.move(angle, omega, 0, alpha);
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
    forward.move(remaining, forward.speed(), 0, forward.acceleration());
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
    forward.move(distance, forward.speed(), 0, forward.acceleration());
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
