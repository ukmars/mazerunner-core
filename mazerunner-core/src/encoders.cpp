#include "encoders.h"

/**
 * Measurements indicate that even at 1500mm/s the total load due to
 * the encoder interrupts is less than 3% of the available bandwidth.
 */

// INT0 will respond to the XOR-ed pulse train from the left encoder
// runs in constant time of around 3us per interrupt.
// would be faster with direct port access
ISR(INT0_vect) {
  encoders.left_input_change();
}

// INT1 will respond to the XOR-ed pulse train from the right encoder
// runs in constant time of around 3us per interrupt.
// would be faster with direct port access
ISR(INT1_vect) {
  encoders.right_input_change();
}
