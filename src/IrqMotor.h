#ifndef IrqMotor_h
#define IrqCMotor_h

#include "Arduino.h"
#include "BLDCMotor.h"

class IrqMotor: public BLDCMotor
{
  public:
    IrqMotor(int pp,  float R = NOT_SET, float KV = NOT_SET, float L = NOT_SET);

	// overriding
    int initFOC() override;
    void loopFOC()	{};
    void move(float target = NOT_SET) override;

	void IrqHandler();

	unsigned long iAdcMicros = 0;
	unsigned long iAdcMicrosLoop = 0;
	int iUsLoopFOC = 300;

  protected:

	unsigned long iAdcLast = 0;
	unsigned long iAdcMicrosLoopLast = 0;
};

#endif