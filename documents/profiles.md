# Profiles

Movement of UKMARSBOT is managed by the velocity profiles. Code for the Profile class is found in profile.h.

A Profile is a computer generates set of velocities that varies over time. Both forward and rotary motion have profiles and both are treated exactly the same by the code. In fact, the forward and rotation profiles are both instances of the same C++ class. The only difference is that one has units of mm, mm/s and mm/s/s and the other has units of deg, deg/s and deg/s/s. Nothing in the Profile code cares about what the units represent - they are just numbers - and so the same code can be used for both.

## Profile features

Profiles have three phases.

 - **Accelerating** where the speed is changing from the start speed to the running speed.
 - **Constant velocity** where the speed is not changing.
 - **Braking** where the speed is changing to the final speed.

 The easiest type of profile to understand is one where the speed is zero, increases to a maximum value, remains steady until braking is needed and then reduces the speed steadily until at rest again.

```
                      ---------------------
    ^                /                     \
  V |               /                       \
    |              /                         \
      -------------                           ----------
      time ->

```

To describe such a profile, a number of parameters are needed. These are:

 - **distance** is the complete distance (or angle) over which the movement occurs
 - **maximum speed** is the speed of the central, constance velocity phase
 - **end speed** is the final speed that the profile must reach when distance is complete
 - **acceleration** is the permitted rate of change of speed

A profile is always started with these parameters and it runs until it is finished. In this context, 'finished' just means that the given distance has been reached.. User code can wait and do nothing until the profile finishes or it can perform other tasks while it waits.

Speeds and accelerations are always positive but the distance can be negative so that the robot can move forwards or backwards and can turn left (positive) or right (negative).

Profiles are very flexible. For example, if the distance is very small, the profile may not be able to reach the maximum speed but will still perform whatever acceleration and braking it can. The starting speed and ending speed do not have to be zero. If the profiler is already running at some speed, the accelerating phase will simply try to match the given maximum speed, even if it is smaller than the current speed. The end speed need not be zero in which case the profiler will continue to run at the specified speed even after it has finished.

All of the robot's movements are created by starting and manipulating profiles.

Because the two profiles are independent, either can be started, stopped or modified at any time. For example, to make the robot perform a smooth, continuous turn, you could start a forward profile so that it finishes at a constant speed and then begin a rotation profile that turns by just 90 degrees. The result will be a forward movement of the robot followed by a smooth right-angle turn nd then the robot will continue in a straight line. A final forward profile can be started to bring it to a halt after some distance. The radius of the turn will be determined by the combination of the robot's forward speed and maximum angular velocity during the turn.

## Profile updates

Once started by user code, both the forward and rotation profiles are updates automatically by the systick service which normally runs 500 times per second. Thus, once started, a profile will continue to generate speeds and so update the controllers. A profile can be disabled by setting it into an IDLE state. The update still runs but the output does not drive the motors.
