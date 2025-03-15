# Command Line Interface

If you have a USB lead connected to your UKMARSBOT, or a Serial Bluetooth device connected, you can interact with it via a terminal program.

At present, a small number of commands are implemented that will let you print out the maze details, view and set some of the configuration settings. These are mostly single letter commands and the robot will respond immediately. For example, with a terminal connected, just press the 's' key, followed by return, and a single line of sensor configuration data will be sent.

Note that the commands are case insensitive and anything you type is converted to upper case by the robot.

## Simple commands

So far the following single-character commands are interpreted. Where a command is followed by 'n', that is an indication that a numeric argument must be supplied.


| cmd       | Function                                        |
|:----------|-------------------------------------------------|
| W         | 'Walls' - display the current maze map          |
| R         | 'Route' - display the current best route        |
| S         | 'Sensors' - one line of sensor data             |
| T n       | 'Test' - Run Test number n                      |
| U n       | 'User' - Run User function n                    |
| $         | Settings commands - see below                   |

---

## Settings commands

Many of the constants needed fortuning and calibrating the robot are stored ins a settings structure. These values are populated either from the `config.h` file or read from EEPROM. At present, the values are all read from the `config.h` defaults. Future releases will use the EEPROM based values. Meanwhile, it is possible to view and edit these settings without re-programming the robot completely. Values you enter will not be saved after a reset unless you do so explicitly so take notes while experimenting and edit the code later if necessary.

The following settings commands are implemented:

| cmd  | Function                                                  |
|:----:|-----------------------------------------------------------|
|  $   | Display all setting by index number                       |
|  $n  | Display a single setting for index number n               |
|  $$  | display all settings as C declarations with name and type |
|  $@  | fetch all settings from EEPROM, overwrite current values  |
|  $!  | store all current settings values to EEPROM               |
|  $#  | reset all settings to defaults from config.h              |

The reason there are so ways to see the settings values - with and without a full declaration - is to make it easier to use a simple short-hand manual method for changing settings and for later use with a host-based manager. It is likely that the short-form version will have the setting name appended as a comment at some time.

### View and change a setting

To view a single setting use the form $n where n is the index number of the settings as shown in the list that you get from the $ settings list command. For example, the command $2 will display the current value for the forward controller KP setting

To change a setting use the form $n=vvvvv, where n is the setting number and vvvvv is the new value. For example, to change the forward controller KP setting to 2.13, you can enter `$2=2.13`. the new value will be echoed to the screen for confirmation.

There is no undo so double check before writing to EEPROM. You can always get back to the compiled in defaults with `$#`.
