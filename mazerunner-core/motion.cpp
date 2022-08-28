/*
 * File: motion.cpp
 * Project: mazerunner
 * File Created: Wednesday, 24th March 2021 6:29:39 pm
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

#include "motion.h"
#include "motors.h"
#include "profile.h"
#include "reports.h"
#include "sensors.h"
#include <Arduino.h>

//***************************************************************************//
/*
 * This file contains example functions illustrating how the profiles
 * might be used to make the robot move.
 *
 * Other examples are to be found in tests.cpp
 */
//***************************************************************************//

/**
 * Before the robot begins a sequence of moves, this method can be used to
 * make sure everything starts off in a known state.
 *
 * @brief Reset profiles, counters and controllers. Motors off. Steering off.
 */
void reset_drive_system() {
  stop_motors();
  disable_motor_controllers();
  disable_steering();
  reset_encoders();
  reset_motor_controllers();
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
    report_profile();
  }
}

/**
 *
 * @brief turn in place. Force forward speed to zero
 */
void spin_turn(float degrees, float speed, float acceleration) {
  forward.set_target_speed(0);
  while (forward.speed() != 0) {
    delay(2);
  }
  turn(degrees, speed, acceleration);
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
  while (not forward.is_finished()) {
    report_profile();
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
  while (not forward.is_finished()) {
    report_profile();
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

//***************************************************************************//

/**
 * The following functions are examples that might be used during a search
 * or speed run of the mouse in a maze.
 *
 * Each will start with the robot at a cell boundary and end at the next
 * cell boundary.
 */

//***************************************************************************//
/**
 * On entry, the robot is expected to be moving forwards at a suitable speed
 * for the turn. It is expected that this function is called at some fixed
 * predefined point before the turn. Since the turn is used during a maze
 * search, that point will be when the robot crosses the threshold between
 * cells. When the function exits, the robot should once more be on the
 * threshold between cells and still moving forwards.
 *
 * The radius if a turn is a function of the forward speed and the angular
 * velocity of the robot. radius = velocity/omega where omega is in rad/s.
 *
 * The turn can be tuned for a particular speed but will need to be retuned
 * for other speeds. There is a way to keep the turn shape invariant with
 * speed. See the Minos 2015 schedule:
 *
 * http://www.micromouseonline.com/2015/06/29/minos-2015-presentations/
 *
 *
 * You can normally expect the turn to be symmetrical.
 *
 * Five parameters affect the turn:
 *
 * run_in is the distance from the cell boundary to the beginning of the
 * actual turn.
 *
 * run_out is the distance from the end of the rotation to the threshold
 * with the next cell.
 *
 * @brief performs an integrated, smooth left turn at low speed.
 */
void turn_SS90L_example() {
  float run_in = 20.0;       // mm
  float run_out = 20.0;      // mm
  float turn_speed = 300;    // mm/s
  float acceleration = 2000; // mm/s/s
  float angle = 90.0;        // deg
  float omega = 280;         // deg/s
  float alpha = 2000;        // deg/s/s
  forward.start(run_in, turn_speed, turn_speed, acceleration);
  while (not forward.is_finished()) {
    report_profile();
  }
  rotation.start(angle, omega, 0, alpha);
  while (not rotation.is_finished()) {
    report_profile();
  }
  forward.start(run_out, turn_speed, turn_speed, acceleration);
  while (not forward.is_finished()) {
    report_profile();
  }
}
/**
 * See the rurn_SS90EL() function for full details.
 *
 * The 90 degree right turn has its own function in case the
 * parameters need to be different. That can happen because the
 * robot is not symmetrical.
 *
 * @brief performs an integrated, smooth right turn at low speed.
 */
void turn_SS90R_example() {
  float run_in = 20.0;       // mm
  float run_out = 20.0;      // mm
  float turn_speed = 300;    // mm/s
  float acceleration = 2000; // mm/s/s
  float angle = -90.0;       // deg
  float omega = 280;         // deg/s
  float alpha = 2000;        // deg/s/s
  forward.start(run_in, turn_speed, turn_speed, acceleration);
  while (not forward.is_finished()) {
    report_profile();
  }
  rotation.start(angle, omega, 0, alpha);
  while (not rotation.is_finished()) {
    report_profile();
  }
  forward.start(run_out, turn_speed, turn_speed, acceleration);
  while (not forward.is_finished()) {
    report_profile();
  }
}
