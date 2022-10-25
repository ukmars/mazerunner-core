#include "systick.h"


ISR(TIMER2_COMPA_vect, ISR_NOBLOCK) {
  systick.update();
}
