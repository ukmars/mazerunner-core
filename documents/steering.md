# Steering

However well tuned and reliable your robot might be, it will need some way to use the maze walls and posts to ensure that it is travelling in a nice straight line, properly centered between the walls. Without this, it will be difficult to make sure that turns are accurate an repeatable and, of course, a crash will spoil your day.

In case you have a line following setup as well as a maze based robot, you should know that the principles described here work just as well for line following.

UKMARSBOT has two sensors that look at the walls to either side. Either or both those walls may not be present but there will always be at least a post every 180mm in a classic maze. These sensors do not measure the distance to the walls directly. Instead, they measure the amount of light reflected from the walls and use that as an indicator of distance. Why should you care about the distinction? Well, the sensor response if very non-linear. That is, halving the distance does not just double the reflected light, it increases it by much more. That can be a benefit in some ways but, if you wanted to do a full analysis and design of a steering control system, it is a bit of a problem.

Luckily, the steering problem can e easily solved, in large part, by a very simple approach.

## Normalisation and calibration

Sensor responses are always normalised by calibrating for a given maze. That means that the steering control can always expect to use sensor readings in a known range even if the particular robot has more or less sensitive sensors. The UKMARSBOT code assumes that the sensors have been correctly calibrated and normalised so that the side sensor give a normalised reading of 100 when the robot is correctly positioned between two walls.

## Error calculation

There are two kinds of error that indicate the robot is off course in the maze. An _offset_ error is where the robot is too far to one side or another. A _heading_ error is where the robot is not pointing parallel to the walls. Both these errors give similar responses from the sensors and it is difficult to distinguish one form the other. Fortunately, they are both corrected in the same way - by turning away from the wall that is too close.

If there is a wall on either side of the robot, all that is needed then is to calculate the difference between the two sensor readings and that will give us our error. In the code, this is referred to as the cross-track-error (CTE). This term is also used for line following and it is just easier to have one simple term to describe these things.

The calculation is done such that a negative reading indicates that the robot is too close to a wall on the left. When using the normalised sensor readings, the error can be just

    cte = right_sensor - left_sensor;

If one of the walls is missing then it you jut calculate the error seen by the sensor that has a wall but it must be doubled because only one wall is contributing instead of two.

In the code, the error for each wall is first calculated and then the overall error is calculated based on which walls are present.

## Edges

As the robot sensors sweep across a wall edge, the response will drop off briefly and the sensors will give the impression that the robot is drifting away from a wall. there is a risk that the controller might then try to 'follow the edge' and deviate from its proper path a little. This is a real problem sometimes and careful tuning of the steering controller may be needed to make sure it is not too disruptive.

## Steering control

Once the robot has a measure of the error, it must have a way to correct its heading to try and get that error to zero. It may be tempting to come up with an y number of elaborate schemes but, for most purposes, the simplest is the best.

Earlier it was noted that the sensor error can look exactly like an angular error in the robot. That suggests that you can take the sensor error and just add it to the encoder rotation error. The robot controller will see that as a rotation error and attempt to correct it as a part of its normal control mechanism. For proper correction, you will need to decide how much of the sensor error is used for this feedback. Too much and the robot will be twitchy and try to follow every small change in sensor reading. Too little and the robot will be very slow to correct. This is just Proportional control - the P in PID. It may be improved by also adding a Derivative term but you may find it is not needed and that proportional control is enough. Both KP and KD constants are provided in config.h so that you can tune your robot for your needs.

When tuning the steering constants, try to aim for a robot that will correct modest errors within 1 to 2 cells of travel. Don't make it too aggressive or you can end up with large corrections still under way as you approach a turn and that rarely ends well.
