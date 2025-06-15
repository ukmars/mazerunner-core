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

#pragma once
#include <Arduino.h>

// hardware configuration for for UKMARSBOT with an Arduino nano processor

//**** IO CONFIGURATION ****************************************************//
const uint8_t ENCODER_LEFT_CLK = 2;
const uint8_t ENCODER_RIGHT_CLK = 3;
const uint8_t ENCODER_LEFT_B = 4;
const uint8_t ENCODER_RIGHT_B = 5;
const uint8_t USER_IO = 6;
const uint8_t MOTOR_LEFT_DIR = 7;
const uint8_t MOTOR_RIGHT_DIR = 8;
const uint8_t MOTOR_LEFT_PWM = 9;
const uint8_t MOTOR_RIGHT_PWM = 10;
const uint8_t EMITTER_A = 11;
const uint8_t EMITTER_B = 12;

// the sensor ADC channels in case we have no special use for a given channel
const uint8_t SENSOR_0 = A0;
const uint8_t SENSOR_1 = A1;
const uint8_t SENSOR_2 = A2;
const uint8_t SENSOR_3 = A3;
const uint8_t SENSOR_4 = A4;
const uint8_t SENSOR_5 = A5;
const uint8_t SWITCHES_PIN = A6;
const uint8_t BATTERY_PIN = A7;

/******************************************************************************
 * The switch input is driven by a resistor chain forming a potential divider.
 * These are the measured thresholds if using the specified resistor values.
 *
 * The adc_thresholds may need adjusting for non-standard resistors.
 * Use the adc_reading() method to find the ADC values for each switch
 * combination and enter them in this table
 */
const int adc_thesholds[] PROGMEM = {660, 647, 630, 614, 590, 570, 545, 522, 461, 429, 385, 343, 271, 212, 128, 44, 0};

/******************************************************************************
 * FAST IO for ATMEGA328 ONLY
 *
 * There are places in the code (ADC and ENCODERS) where it is important that
 * you are able to access IO pins as quickly as possible. Some processor are fast
 * enough that this is not a problem. The ATMega328 using the Arduino framework
 * is not one of those cases so the macros below translate simple pin IO
 * functions into single machine code instructions.
 *
 * Extracted from digitalWriteFast:
 *      Optimized digital functions for AVR microcontrollers
 *      by Watterott electronic (www.watterott.com)
 *      based on https://code.google.com/p/digitalwritefast
 *
 * If you are using a different processor, you will either need to reimplement
 * these functions or use a suitable built-in function if it is fast enough
 */
#if defined(__AVR_ATmega328__) || defined(__AVR_ATmega328P__)
#define __digitalPinToPortReg(P) (((P) <= 7) ? &PORTD : (((P) >= 8 && (P) <= 13) ? &PORTB : &PORTC))
#define __digitalPinToDDRReg(P) (((P) <= 7) ? &DDRD : (((P) >= 8 && (P) <= 13) ? &DDRB : &DDRC))
#define __digitalPinToPINReg(P) (((P) <= 7) ? &PIND : (((P) >= 8 && (P) <= 13) ? &PINB : &PINC))
#define __digitalPinToBit(P) (((P) <= 7) ? (P) : (((P) >= 8 && (P) <= 13) ? (P)-8 : (P)-14))

// general macros/defines
#if !defined(BIT_READ)
#define BIT_READ(value, bit) ((value) & (1UL << (bit)))
#endif
#if !defined(BIT_SET)
#define BIT_SET(value, bit) ((value) |= (1UL << (bit)))
#endif
#if !defined(BIT_CLEAR)
#define BIT_CLEAR(value, bit) ((value) &= ~(1UL << (bit)))
#endif
#if !defined(BIT_WRITE)
#define BIT_WRITE(value, bit, bitvalue) (bitvalue ? BIT_SET(value, bit) : BIT_CLEAR(value, bit))
#endif

#define fast_write_pin(P, V) BIT_WRITE(*__digitalPinToPortReg(P), __digitalPinToBit(P), (V));
#define fast_read_pin(P) ((int)(((BIT_READ(*__digitalPinToPINReg(P), __digitalPinToBit(P))) ? HIGH : LOW)))

#else
#define fast_write_pin(P, V) digitalWrite(P, V)
#define fast_read_pin(P) digitalRead(P)
#endif
/******************************************************************************
 * ATOMIC OPERATIONS for ATMEGA328 ONLY
 * Since the ATMega328 is an 8 bit processor it is possible that you will end
 * up trying to read a multi-byte quantity that is modified in an interrupt while
 * you are doing the read or write. The result is a corrupt value. 32 bit processors
 * are unlikely to suffer from this since quantities are read in a single operation.
 *
 * The AVR compiler provides a method for you to disable interrupts for the
 * duration of a block of code and then restore the state at the end of the block.
 *
 * It is not enough to simply turn off interrupts and then turn them back on because
 * you need to remember the state of the interrupt enable flag at the start of the
 * block.
 *
 * These macros do this for you and should be either modified for different processors
 * or bypassed if needed.
 *
 * Use like this:
 * ATOMIC {
 * // code to protect
 * }
 *
 */
#if defined(__AVR__)
#include <util/atomic.h>
#define ATOMIC ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
#else
#define ATOMIC
#endif
//***************************************************************************//

/***
 * Finally, a little-known provision of the compiler lets you
 * configure the standard printf() function to print directly
 * to a serial device. There is a cost overhead if you are not
 * using printf() or sprintf() elsewhere but it is a great convenience
 * if you want formatted printing to the serial port.
 *
 * To use this facility add a call to redirectPrintf() early in the
 * setup() function of your code.
 *
 * TODO: this does not work with the NRF52 compiler.
 */

#if !defined(ARDUINO_ARCH_NRF52840)
// Function that printf and related will use to print
int serial_putchar(char c, FILE *f) {
  if (c == '\n') {
    // TODO do we need to add carriage returns? I think not.
    Serial.write('\r');
  }
  return Serial.write(c) == 1 ? 0 : 1;
}

FILE serial_stdout;
void redirectPrintf() {
  // Redirect stdout so that we can use printf() with the console
  fdev_setup_stream(&serial_stdout, serial_putchar, NULL, _FDEV_SETUP_WRITE);
  stdout = &serial_stdout;
}
#else
void redirectPrintf(){};
#endif
