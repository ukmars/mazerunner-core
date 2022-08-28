#New UKMARSBOT Controllers

Note that this an alpha release and is a work in progress and subject to change. Suggestions and bug notices welcome.

This package has a re-write of the profile and control code for UKMARSBOT.

The previous code used an underlying velocity control model and it was not proving stable. This code has a position control model for the motors.

The motor speeds are no longer set directly. Instead, a real-time profiler uses the set speed to generate a series of new positions for the wheels and the controller tries to get the wheels to the desired position.

This is not position control in the sense that you issue a command for the robot to move by, say, 180mm in one step. That would not work because the controller output would rapibly saturate and try to spin up the wheels as fast as possible to get to the new position.

Instead, the motion is in a series of very small movements at each tick of the control loop. for example, when running at 1000mm/s, the steps would be 2mm per tick. The result is smooth, continuous motion.

Both forward and rotary motion are profiled and controlled in the same way. Each has its own profiler and controller.

## Calibration, Tuning and Testing

### Sensor setup

Before doing anything else, get the sensors running and fairly well calibrated. You will need to be tehered or have a connected BT dongle set at 115200baud. Sensors readings are normalised and you need to set the raw, non-normalised reading to a reference value in some known robot configuration. For the left and right sensor, place the robot centrally between two walls with the sensor spot adjusted to be about 20mm in front of the robot and at the same height as the emitter. For the front sensor, the robot is placed with its back edge against a wall in a cell with a wall ahead. Again, the sensor spot should be at about the same height as the emitter.

  - Set the function switches to 1 and reset the controller
  - the emitters should light up
  - values will stream to the monitor in three pairs normalised and raw in parentheses.
  - the first pair is normalised left (raw left)
  - the second pair is the front sensor (raw front)
  - the third pair is for the right sensor (raw right)

Write down the raw values obtained with the robot properly oriented and enter them into config.h as the FRONT_CALIBRATION, LEFT_CALIBRATION and RIGHT_CALIBRATION values. I have visible emitters and, for me, the values are:


    //***** SENSOR CALIBRATION **************************************************//
    // wall sensor thresholds and constants
    // the observed values for the front sensor when the robot is backed up to a wall
    const int FRONT_CALIBRATIOM = 77;
// the observed values for the side sensors when the robot is centred in a cell
// and there is no wall ahead
const int LEFT_CALIBRATIOM = 97;
const int RIGHT_CALIBRATIOM = 105;

Re-program the controller and run this test again. You should see that the normalised values for all sensors are about 100 now when the robot is in its calibration position.

With the sensor calibration done, running tests should be easier.

If you get wildly different reference values you may be using different sensor types or you may need to check the component values.

### Wall Detection Thresholds

Once the sesnors have been calibrated and the readings are normalised, it is possible to decide upon the wall detection thresholds.

#### Left and Right Walls

For the left and right sensors, a value between 40% and 60% of the normal readings is often used. The most obvious value is 50% which would correspond with half the emitter spot sitting over a post or wall edge. A slightly smaller threshold will detect the wall a little later, a slightly larger threshold will detect the wall a little sooner. You can adjust this to suit your robot and your needs. The exact position at which the robot sees a post edge can be used to help it measure its position more exactly so it is a good idea to try and orient the sensors so that both sides detect an edge at close to the same point.

#### Front Walls

Detecting a front wall reliably can be surprisingly hard. Normally, the robot will still be a couple of centimeters short of a cell boundary when it does its wall checks. At that point, the side sensors reliably point into the side walls in the cell ahead. The front sensor though is not getting much signal back and that signal will be subject to relatively large amounts of noise and uncertainty. For the kinds of sensor senitivity used in the test robot, the front sensor only gives a normalised reading fo 20-30 when the wheels are 20mm short of the cell boundary and there is a wall ahead. At that point, with a detection threshold of only 20, it is possible to miss a wall. With no wall, the sensors on the test robot give readings that are nearly all less than 7. From these results, it would seem reasonable to choose a threshold of 15 for the wall detection. There is still the possibility of false positives and false negatives so if reliability is a problem, it would be a good idea to sample several times. For best reliablility, sample as the robot crosses the cell boundary. Just those few mm of travel can mean double the returned signal.

### Running tests

There are a number of test routines provided. Select a test by using the dip switches on the robot.

Set the switches and press the button when ready. The sensors will light up to indicate that the mouse is 'armed'. Set the mouse down and, when you are rady to run the test, place your hand/finger/kipper/whatever just in front of the front sensor. When you take it away again, the sensors will go dark and, after a short delay, the test will run.

In the ```config.h``` file there are a number of configuration defaults that govern the behaviour of the  control system ad the robot.

### Calibration - TEST 5

For accurate movements, you will need to calibrate the drivetrain. You will need to obtain values for the wheel diameter, gearbox ratio, number of pulses per revolution from the encoders and wheel spacing.

Measure these as best you can and use your measurements as a starting point. Test will let you push the mouse on the round. Move the robot some known dustance and compare that with the value reported on the monitor.

This manual test lets you at least get close before fine tuning the readings later.

An estimate for the ```MOUSE_RADIUS``` constant is obtained by simply measuring the distance between the centres of the wheels. The mouse radius is half that. Later, when doing the rotation test, the value can be adjusted to get turns of as close as possible to 360 degrees.

### Tuning

Once the basic calibration is done, there are four controller constants to adjust - two each for forward and rotary motion. The existing values should be useable if you have a similar drivetrain to me. That is Pimoroni motors with 20:1 gearboxes. If the response seems a bit 'noisy' then reduce the xxx_KD constant a little. If that does not work, reduce the xxx_KP. A separate document will be needed for motor turning. Test options let you sit the robot up off the ground and test the motor controllers.

#### Test 6 and Test 7

These tests exercise the controllers and put the robot through a sequence of movements - forward for test 6, rotation for test 7. They run for 2 seconds and pump telemetry out to the terminal. You can graph the data in Excel and adjust the controller constants to get the best response. Note that the encoders are intrinsicaly noisy and so, if you try to calculate the speed, you will likely see a very spiky response. Within reason, we don't much care about the speed. If the controllers are dog their job, it will work itself out.


### Other Tests

There are two other tests available that let you see how the profilers may be used. See the code in tests.cpp for details.


## Terminology

Forward motion is governed by __position__, measured in mm. The profiler provides a set point as a position and the encoders provide an estimate of the robot position. The position is not a location in space. It is really a distance since the counters and profiler were reset. (perhaps __distance__ would be a better term?). Measurement of position is done in real units - millimeters for distances and degrees for angles.

## Controllers

The low-level control is by way of very simple PD controllers. In this appliation, there is no need for integral terms. Because of the simplicity of the technique there is no need for a full-blown PID controller class and the control code is just a couple of lines.

At each tick, a new set position is generated by the profiler(s) and this is compared with the robot position as obtained from the encoders. The PD controller generates output voltages intended to correct this error. These vltages are mixed to gether and applied to each wheel motor separately.

### D Term Filtering
For the rotation controller (only, for now) there is an aditional filtering action on the D term to try and get smoother motion. This is probably not needed. The filtering constant can be set to 1.0 to eliminate the filtering. Smaller values provide greater levels of filtering. If the value is too small, the response will be oscilatory because then the proportional gain is too high. the given value of 0.5 is adequate for testing.

## Profilers

To generate the sequence of positions that make the root move, a __profiler__ is used. Since rotatry and forward motion are essentially the same thing, a single Profile class is used for both. There are two instancesof Profile in the code and they are called ```forward``` and ```rotation```.


## Using the Profiles

The project attached to this file has a number of test examples that should illustrate ways to use the Profile code. In general there are a couple of ways to use a Profile.

### Position based profiles.
