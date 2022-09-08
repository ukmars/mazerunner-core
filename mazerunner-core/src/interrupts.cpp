#include "../config.h"
#include "digitalWriteFast.h"
#include "encoders.h"
#include "systick.h"
#include <Arduino.h>

static int encoder_left_counter;
static int encoder_right_counter;
/**
 * Measurements indicate that even at 1500mm/s thetotal load due to
 * the encoder interrupts is less than 3% of the available bandwidth.
 */

// INT0 will respond to the XOR-ed pulse train from the leftencoder
// runs in constant time of around 3us per interrupt.
// would be faster with direct port access
ISR(INT0_vect) {
  encoders.update_left();
}

// INT1 will respond to the XOR-ed pulse train from the right encoder
// runs in constant time of around 3us per interrupt.
// would be faster with direct port access
ISR(INT1_vect) {
  encoders.update_right();
}

ISR(TIMER2_COMPA_vect, ISR_NOBLOCK) {
  systick.update();
}


ISR(ADC_vect){
    sensors.update();
}