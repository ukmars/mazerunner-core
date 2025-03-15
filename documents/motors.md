# Motors

UKMARSBOT uses a pair of metal gearmotors compatible with the Pololu mini metal gearmotor. These have a 6 Volt N20-sized motor attached to a small gearbox. Gear ratios vary from 5:1 upwards. A common choice is the 20:1 gearbox. Note that the actual gear ratio is never exactly that stated for the motor because of the choice of internal gears. On my example the actual gear ratio is 19.54:1. Any compatible motor with gearbox can be fitted.

For feedback of speed and position, the motors must have an extended rear shaft with an encoder fitted that generates electrical pulses as the motor shaft turns. The encoders are dealt with in detail in their own section.

## PWM

The default PWM frequency for the Arduino nano is only 490Hz. For these motors, that is not a good choice and the code configures the PWM generator to run at 31.25kHz. The higher frequency give a smoother response and is out of the range of human hearing so there will be no whining from the motor drive.

DC electric motors will run at a constant speed for a given input voltage and load. Since the PWM duty cycle is just a percentage of the available drive voltage, arrangements are made in the motor control code to compensate for changes in the battery Voltage. Your code should set a drive Voltage for the motors rather than a PWM duty cycle and the driver code will make the necessary calculations.

## Wheel control

It might seem natural for robot control software to work by setting the speeds of the individual motors. Set the motors to the same speed and the robot will drive straight. Set one motor faster than the other and the robot will turn to one side. In the UKMARSBOT software, things are done a little differently.

Speed control managed in two components - _forward_ motion and _rotation_. Each is controlled separately and the two components are combined to generate commands for each of the wheels. Without going into the maths to much, the individual speeds are calculated from

    left = forward - rotation
    right = forward + rotation

If rotation is zero, both wheels get the same speed and the robot moves in a straight line controlled by the forward component. If the forward component is zero, the robot will turn on the spot with an angular velocity controlled by the rotation component. Notice that a positive rotation will turn the robot left.

Any combination for the forward and rotation components can be used to move the robot in linear motion, smooth curved turns or in-place spin turns. You don't have to do any work trying to calculate individual wheel speeds - that is already done for you.

## PD Controllers

Each of the motion types has its own Proportional-Differential (PD) controller that ensures that the wheels are doing what they need to for that motion type. You will have heard of PID controllers and may wonder what happened to the I term. For the type of control employed in UKMARSBOT, the I term is not needed and a simplified controller can be used. This is because the controllers used are not controlling the speed directly. Instead, they control the _position_ and PD control is more than adequate for position control in this configuration. At every systick cycle, the control software works out what the new position should be for the current forward and rotation speeds and compares that with the position of the robot as reported by the encoders. the controllers then generate commands to try and reduce that error to zero. For continuous motion, the set points change at every tick and the controllers move the robot to catch up.

This kind of control scheme is reliable and robust in this application but only useful for the small distance changes between control loop cycles. It is not a good scheme for trying to tell the robot it needs to be at a new location 180mm away.

The new positions are generated in small increments every cycle by the motion profilers. These are described in their own section.

Each of the motion PD controllers has a pair of constants that determine its behaviour. These are the following constant in `config.h`

```
// forward motion controller constants
const float FWD_KP = 2.0;
const float FWD_KD = 1.1;

// rotation motion controller constants
const float ROT_KP = 2.1;
const float ROT_KD = 1.2;
```

The values shown are probably acceptable for a standard UKMARSBOT using 6 Volt motors with 12 pulse encoders and 20:1 gearboxes. If your robot has a different drivetrain, you may want to tune these values somewhat. The system is not overly sensitive to the controller gains. A separate section will look at how to tune the controllers to get a better response.
