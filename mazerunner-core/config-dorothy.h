#ifndef DOROTHY_H
#define DOROTHY_H

const uint32_t BAUDRATE = 115200;
const int SENSOR_COUNT = 4;
const float MAX_MOTOR_VOLTS = 6.0;

const int EEPROM_ADDR_SETTINGS = 0x0000;
#define SETTINGS_SIGNATURE 0xF1F0C00F
// encoder polarity is either 1 or -1 and is used to account for reversal of the encoder phases
#define ENCODER_LEFT_POLARITY (-1)
#define ENCODER_RIGHT_POLARITY (1)

// similarly, the motors may be wired with different polarity and that is defined here so that
// setting a positive voltage always moves the robot forwards
#define MOTOR_LEFT_POLARITY (1)
#define MOTOR_RIGHT_POLARITY (-1)

#define WHEEL_DIAMETER 32.0
#define ENCODER_PULSES 12.0
#define GEAR_RATIO 19.54
#define MOUSE_RADIUS 37.72

// forward motion uses the sum of the two encoders for odometry hence the 2.0 constant
#define DEFAULTS_MM_PER_COUNT (PI * WHEEL_DIAMETER / (2.0 * ENCODER_PULSES * GEAR_RATIO))

// rotation uses the difference between the encoders
#define DEFAULTS_DEG_PER_COUNT ((360.0 * DEFAULTS_MM_PER_COUNT) / (2.0 * PI * MOUSE_RADIUS))

#define DEFAULTS_FWD_KP 1.0
#define DEFAULTS_FWD_KD 5.0

#define DEFAULTS_ROT_KP 0.35
#define DEFAULTS_ROT_KD 4.0

// controller constants for the line follower configuration
#define DEFAULTS_WALL_KP 0.4
#define DEFAULTS_WALL_KD 0.0

// time delay for sensors to respond to emitters
#define DEFAULTS_EMITTER_ON_TIME 50

// these are aliases of convenience
const int LED_LEFT = USER_IO_6;
const int LED_RIGHT = USER_IO_12;

const int EMITTER_A = USER_IO_11;
const int EMITTER_B = USER_IO_12;

#endif