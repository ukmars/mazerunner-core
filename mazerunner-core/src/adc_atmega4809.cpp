/******************************************************************************
 * Project: mazerunner-core                                                   * 
 * File:    adc_atmega328.cpp                                                 * 
 * File Created: Wednesday, 26th October 2022 12:11:36 am                     * 
 * Author: Peter Harrison                                                     * 
 * -----                                                                      * 
 * Last Modified: Thursday, 27th October 2022 6:27:51 pm                      * 
 * -----                                                                      * 
 * Copyright 2022 - 2022 Peter Harrison, Micromouseonline                     * 
 * -----                                                                      * 
 * Licence:                                                                   * 
 *     Use of this source code is governed by an MIT-style                    * 
 *     license that can be found in the LICENSE file or at                    * 
 *     https://opensource.org/licenses/MIT.                                   * 
 ******************************************************************************/

#include "adc_atmega4809.h"

/** @brief Sample all the sensor channels with and without the emitter on
 *
 * At the end of the 500Hz systick interrupt, the ADC interrupt is enabled
 * and a conversion started. After each ADC conversion the interrupt gets
 * generated and this ISR is called. The eight channels are read in turn with
 * the sensor emitter(s) off.
 *
 * At the end of that sequence, the emitter(s) get turned on and a dummy ADC
 * conversion is started to provide a delay while the sensors respond.
 * After that, all channels are read again to get the lit values.
 *
 * The lit section handles the sensor channels in two groups so that several
 * channels can be illuminated by one emitters while the others use a different
 * emitter. This is a little clunky but essential to avoid crosstalk between the
 * forward- and side-looking sensor types
 *
 * After all the channels have been read twice, the ADC interrupt is disabbled
 * and the sensors are idle until triggered again.
 *
 * The ADC service runs all the time even with the sensors 'disabled'. In this
 * software, 'enabled' only means that the emitters are turned on in the second
 * phase. Without that, you might expect the sensor readings to be zero.
 *
 * Timing tests indicate that the sensor ISR consumes no more that 5% of the
 * available system bandwidth.
 *
 * There are actually 16 available channels on the ATMEGA328p and channel 8 is
 * the internal temperature sensor. Channel 15 is Gnd. If appropriate, a read of channel
 * 15 can be used to zero the ADC sample and hold capacitor.
 *
 * NOTE: All the channels are read even though only 5 are used for the maze
 * robot. This gives worst-case timing so there are no surprises if more
 * sensors are added.
 *
 * If different types of sensor are used or the I2C is needed, there
 * will need to be changes here.
 */

#ifndef ARDUINO_ARCH_AVR
#error This is only meant to run on an an AVR processor
#endif

#ifdef ARDUINO_ARCH_MEGAAVR

ISR(ADC_vect) {
  adc_isr(adc);
}

#endif
