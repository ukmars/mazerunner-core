#include "adc.h"

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

void adc_isr(AnalogueConverter adc) {
  switch (adc.m_sensor_phase) {
    case 0:
      adc.start_conversion(0);
      break;
    case 1:
      adc.m_adc_reading[0] = adc.get_adc_result();
      adc.start_conversion(1);
      break;
    case 2:
      adc.m_adc_reading[1] = adc.get_adc_result();
      adc.start_conversion(2);
      break;
    case 3:
      adc.m_adc_reading[2] = adc.get_adc_result();
      adc.start_conversion(3);
      break;
    case 4:
      adc.m_adc_reading[3] = adc.get_adc_result();
      adc.start_conversion(4);
      break;
    case 5:
      adc.m_adc_reading[4] = adc.get_adc_result();
      adc.start_conversion(5);
      break;
    case 6:
      adc.m_adc_reading[5] = adc.get_adc_result();
      adc.start_conversion(6);
      break;
    case 7:
      adc.m_adc_reading[6] = adc.get_adc_result();
      adc.start_conversion(7);
      break;
    case 8:
      adc.m_adc_reading[7] = adc.get_adc_result();
      // Now all the 'dark' readings have been taken and it is time to
      // get the 'lit' results. These are in two groups. Start with Group A.
      // These are the side sensors for the advanced wall sensor. The basic wall
      // sensor board has all channels in group A
      adc.emitter_on(adc.m_emitter_a);
      adc.start_conversion(15); // dummy adc conversion to create a delay
      // wait at least one cycle for the detectors to respond
      break;
    case 9:
      adc.start_conversion(0);
      break;
    case 10:
      adc.m_adc_reading[0] = adc.get_adc_result() - adc.m_adc_reading[0];
      adc.start_conversion(3);
      break;
    case 11:
      adc.m_adc_reading[3] = adc.get_adc_result() - adc.m_adc_reading[3];
      // Now group B. These are the front sensors for the advanced board
      // Since the Basic board only one group, the same emittter will be lit
      adc.emitter_off(adc.m_emitter_a);
      adc.emitter_on(adc.m_emitter_b);
      adc.start_conversion(15); // dummy adc conversion to create a delay
      // wait at least one cycle for the detectors to respond
    case 12:
      adc.start_conversion(1);
      break;
    case 13:
      adc.m_adc_reading[1] = adc.get_adc_result() - adc.m_adc_reading[1];
      adc.start_conversion(2);
      break;
    case 14:
      adc.m_adc_reading[2] = adc.get_adc_result() - adc.m_adc_reading[2];
      adc.start_conversion(4);
      break;
    case 15:
      adc.m_adc_reading[4] = adc.get_adc_result() - adc.m_adc_reading[4];
      adc.start_conversion(A5);
      break;
    case 16:
      adc.m_adc_reading[5] = adc.get_adc_result() - adc.m_adc_reading[5];
      adc.emitter_off(adc.m_emitter_b);
      _NOP();
      adc.end_sensor_cycle();
      break;
    default:
      break;
  }
  adc.m_sensor_phase++;
}

ISR(ADC_vect) {
  adc_isr(adc);
}
