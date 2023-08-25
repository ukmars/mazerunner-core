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

//***************************************************************************//
// Some physical constants that are likely to be board -specific
// with robot against back wall, how much travel is there to the cell center?
const int BACK_WALL_TO_CENTER = 48;

//***************************************************************************//
// Control loop timing. Pre-calculate to save time in interrupts
const float LOOP_FREQUENCY = 500.0;
const float LOOP_INTERVAL = (1.0 / LOOP_FREQUENCY);

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
//***************************************************************************//
