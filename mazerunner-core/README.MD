# Mazerunner-Core

The original ukmarsbot-mazerunner software is a broad, multi-purpose set of routines that lets you set up your robot and run it in a maze-based contest. As such, it was full of useful but possibly misleading code fragments as it tried to do everything in one place.

This **mazerunner-core** repository does just the contest part of the job along with a small number of test routines. To characterise and configure many of the options like controller constants and so on, you will need other software.


## Functions

The available options are relatively limited but will be enough to let you calibrate the wall sensors and perform basic turn configuration as well as run the robot as a maze solver or wall follower.

### Maze Solving
The maze solver code is very basic and will just search back and forth between the start and the goal. On each pass, the path taken will improve so long as a better path has been discovered. The code is not optimal and makes no attempt to do things like run faster through known sections or run subsequent passes more quickly. That is left as an exercise for the user.

### Wall Following
Wall following is also very simple and will follow a left wall until the robot is physically stopped. Because the same techniques for motion and turning are used here and the maze solver, you can use the wall following function to help you tune the turns. Simply make a rectangular 'racetrack' maze and have the robot run around it either clockwise or anticlockwise so that is does repeated turns. This makes it easy to see the effect of changing the turn parameters.

## Bluetooth

The Serial port on the side of the robot can accept a HC-05 or HC-06 Bluetooth Serial adaptor. From board revision V1.1 onwards you should be able to plug the adaptor directly into the port but you will need to check the actual pin order on your HC05/6. V1.0 motherboards will need an adaptor and some modification to the port.

The software includes a Serial command line interface that can be used with the normal USB connector and the BT adaptor simultaneously. A BT adaptoor is by far the better option as it is always ready and will not reset the processor when used. Also, the USB cable is a serious impediment when trying to do things like calibrate sensors.

## Operation

When the code is loaded onto the robot, operation is primarily governed by the function selection switches and the user button.

 - select a function on the switches
 - reset the processor
 - activate the function by pressing the button

Some builtin functions have a secondary start that is triggered by briefly occluding the front sensors. For example, in maze running mode, the user button starts the function. The bult-in LED blinks to indicate that it is ready and you can place the robot in the maze. The robot will not actually start a run until triggered by occluding the front sensors. This is my preference for a start that does not disturb the robot. You may choose whatever technique suits you.

***NOTE*** the maze map is stored in EEPROM. To clear the stored maze, hold down the user button and reset the processor. Hold down the button until the user LED fashes.