/******************************************************************************
 * Project: mazerunner-core                                                   *
 * File:    adc.cpp                                                           *
 * File Created: Thursday, 27th October 2022 12:12:56 pm                      *
 * Author: Peter Harrison                                                     *
 * -----                                                                      *
 * Last Modified: Monday, 31st October 2022 12:51:02 am                       *
 * -----                                                                      *
 * Copyright 2022 - 2022 Peter Harrison, Micromouseonline                     *
 * -----                                                                      *
 * Licence:                                                                   *
 *     Use of this source code is governed by an MIT-style                    *
 *     license that can be found in the LICENSE file or at                    *
 *     https://opensource.org/licenses/MIT.                                   *
 ******************************************************************************/
#include "adc.h"
#include "config.h"
#include "digitalWriteFast.h"

/***
 * This function should be independent of the hardware but is called from the
 * hardware end-of-conversion ISR.
 *
 * In the adc interface class, it is declared as a friend so that it can access
 * the methods and properties of the actual class instance
 *
 *
 * @brief Sample all the sensor channels with and without the emitter on
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

AnalogueConverter adc;

ISR(ADC_vect) {
  adc_isr(adc);
}

// int battery_adc_reading;
// int switches_adc_reading;
// static int sensor_phase = 0;
// static int channel = 0;

void adc_isr(AnalogueConverter &adc) {
  switch (adc.m_phase) {
    case 0:
      // always start conversions as soon as  possible so they get a
      // full 50us to convert
      adc.start_conversion(7);
      break;
    case 1:
      adc.m_adc[7] = adc.get_adc_result();
      adc.start_conversion(6);
      break;
    case 2:
      adc.m_adc[6] = adc.get_adc_result();
      adc.start_conversion(0);
      break;
    case 3:
      adc.m_adc_dark[0] = adc.get_adc_result();
      adc.start_conversion(1);
      break;
    case 4:
      adc.m_adc_dark[1] = adc.get_adc_result();
      adc.start_conversion(2);
      break;
    case 5:
      adc.m_adc_dark[2] = adc.get_adc_result();
      adc.start_conversion(3);
      break;
    case 6:
      adc.m_adc_dark[3] = adc.get_adc_result();
      adc.start_conversion(4);
      break;
    case 7:
      adc.m_adc_dark[4] = adc.get_adc_result();
      adc.start_conversion(5);
      break;
    case 8:
      adc.m_adc_dark[5] = adc.get_adc_result();
      if (adc.m_emitters_enabled) {
        // got all the dark ones so light them up
        digitalWriteFast(adc.emitter_diagonal(), 1);
        digitalWriteFast(adc.emitter_front(), 1);
      }
      adc.start_conversion(7); // dummy read of the battery to provide delay
      // wait at least one cycle for the detectors to respond
      break;
    case 9:
      adc.get_adc_result(); // dummy read to clear the flag
      adc.start_conversion(0);
      break;
    case 10:
      adc.m_adc[0] = adc.get_adc_result() - adc.m_adc_dark[0];
      adc.start_conversion(1);
      break;
    case 11:
      adc.m_adc[1] = adc.get_adc_result() - adc.m_adc_dark[1];
      adc.start_conversion(2);
      break;
    case 12:
      adc.m_adc[2] = adc.get_adc_result() - adc.m_adc_dark[2];
      adc.start_conversion(3);
      break;
    case 13:
      adc.m_adc[3] = adc.get_adc_result() - adc.m_adc_dark[3];
      // uncoditionally turn off emitters for safety
      digitalWriteFast(adc.emitter_diagonal(), 0);
      digitalWriteFast(adc.emitter_front(), 0);
      bitClear(ADCSRA, ADIE); // turn off the interrupt
      break;
    default:
      break;
  }
  adc.m_phase++;
}