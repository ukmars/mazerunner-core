# Release 0.2.0

This ran successfully on a stock UKMARSBOT in competition on October 7th 2023. It may still contain bugs and you will need to setup and calibrate your own robot before you can expect it to run well.

It is intended to be a complete, fairly bare-bones micromouse software suite. It should contain all the basic functionality needed to allow a UKMARSBOT or close derivative to explore a classic micromouse maze and search to the center and back to the start.

There are a couple of calibation routines by way of an example of how you might do the basic sensor calibration. There is also a command line interpreter for those who have a radio serial link such as a bluetooth module.



---

# Mazerunner Core

This repository holds a version of the mazerunner code for a version of UKMARSBOT that can run in a classic micromouse maze.

The code assumes you have a standard UKMARSBOT assembled, using an Arduino nano with a basic or advanced wall sensor connected.

All the project code is in the directory `mazerunner-core`

### Arduino IDE users

For the impatient, if you are wanting to run this code in the Arduino IDE then you can download or clone the complete repository and simply open the `mazerunner-core.ino` file. The rduino IDE will be able to build and flash the prject. It will also open all the other files in the `mazerunner-core` directory. That may be surprising to you but it is just trying to be helpful.


### Platform IO users

The project is maintained and developed using PlatformIO with Visual Studio Code. If you have not used this as a development platform, I highly recommend it as a huge improvement over the very dated Arduino IDE. You can find instructions for installing VSCode and PlatformIO at https://ukmars.org/resources/platformio-vscode-windows/

## Initial Configuration

Before flashing your robot, look in the config.h file. In here you can place entries that let you maintain your own robot-specific config file containing a number of default settings describing things like the motor gear ratio, wheel diameter and wheel spacing.

Each physical robot has its own config file. In the main config.h, add an entry for your robot name and create a corresponding config file in the same directory. You can start by copying one of the existing robot config files. Check that these are at least close to the values required for your robot.

Because it is not possible to know how you wired your motors and encoders, you will also find defaults for the motor and encoder polarity. You will need to first check that moving the wheels forwards by hand increases the corresponding encoder count. If not, change the encoder polarity settings as appropriate. If the robot then moves in the wrong direction or just turns instead of moving forward, adjust the relevant motor polarity settings.

The config-ukmarsbot file contains things like pin definitions. These will be correct for the UKMARSBOT using a four-emitter sensor board. If you have a custom derivative, this is where you get to assign pins ot functions.

## Getting started

To characterise your robot, you will need a separate test suite. Use code from the main mazerunner repository or the ukmarsbot-test repository or, of course, write your own. At the very least you will need to configure the encoder resolution and do some sensor calibration.

**The first time you start the robot, make sure that switches are set to zero**. That is, all the switches should be 'down' towards the rear of the robot. Now connect the serial monitor at 115200 baud. When the robot boots, you should see a prompt written to the monitor.

Enter a qustion mark followed by return to see a summary of the available commands.

Most of the functions are activated by first pressing the user button and then bringing your hand close to the front sensor and away again. This is a convenient way to start an action without disturbing the robot. Before that will work, you must calibrate the sensor response. The instructions for that, and the other tests are in the README.MD file found in the code folder.

## Bluetooth

You will gain a lot of flexibility by connecting a Bluetooth adaptor such as the HC-05 or HC-06 to the serial port on the front left of the robot. On Versions 1.1 and later, you can simply plug that straight in. The V1.0 boards will need some minor modification.

Check the default baud rate for your BT adaptor as these can vary.


## Extending the code

At the time of writing, this code contains an absolute minimal set of code needed to search a classic micromouse maze and then run it using the best route found. There is also a wall-following mode since it is a trivial change to the maze-solver.

## Updates

Be aware that, if you download a newer copy of the code, and simply unpack it into the same folder, you will overwrite your code and your changes will be lost. Don't be like Julian, don't do that.

## Contributing

If you have any thoughts about the code, suggestions for changes or improvements, please use the github issues mechanism so that other users can benefit from your observations. Those other users may also be able to offer assistance if the author(s) are not available.
