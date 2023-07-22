#ifndef LOWSIDE_CS_TIMER_LIB_H
#define LOWSIDE_CS_TIMER_LIB_H

#include "LowsideCurrentSense.h"


class LowsideCurrentSenseTimer: public LowsideCurrentSense
{
  public:
    /**
      LowsideCurrentSenseTimer class constructor
      @param shunt_resistor shunt resistor value
      @param gain current-sense op-amp gain
      @param phA A phase adc pin
      @param phB B phase adc pin
      @param phC C phase adc pin (optional)
    */
    LowsideCurrentSenseTimer(float shunt_resistor, float gain, int pinA, int pinB, int pinC = _NC);

  private:

};

#endif
