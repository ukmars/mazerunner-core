#include "adc.h"


ISR(ADC_vect) {
  adc.update_channel();
}
