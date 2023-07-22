#include "LowsideCurrentSenseTimer.h"

// LowsideCurrentSensor constructor
//  - shunt_resistor  - shunt resistor value
//  - gain  - current-sense op-amp gain
//  - phA   - A phase adc pin
//  - phB   - B phase adc pin
//  - phC   - C phase adc pin (optional)
LowsideCurrentSenseTimer::LowsideCurrentSenseTimer(float _shunt_resistor, float _gain, int _pinA, int _pinB, int _pinC)
	: LowsideCurrentSense(_shunt_resistor, _gain, _pinA, _pinB, _pinC)
{
}


