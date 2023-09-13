#include "IrqMotor.h"

IrqMotor* pIrqMotor = NULL;
boolean bIrqMotorThreadNotRunning = true; // thread safety

IrqMotor::IrqMotor(int pp, float _R, float _KV, float _inductance)
: BLDCMotor(pp, _R, _KV, _inductance)
{
  pIrqMotor = this;
}


int IrqMotor::initFOC() 
{
  int iRet = BLDCMotor::initFOC();

  adc_interrupt_flag_clear(ADC_INT_EOIC);
  adc_interrupt_enable(ADC_INT_EOIC);
  nvic_irq_enable(ADC_CMP_IRQn, 3, 3);

  return iRet;
}


void IrqMotor::move(float new_target)	
{
  if(_isset(new_target)) target = new_target;
}


void IrqMotor::IrqHandler()
{
  unsigned long iNow = _micros();
  iAdcMicros = iNow - iAdcLast;
  iAdcLast = iNow;

  if (enabled)
  {
    if (bIrqMotorThreadNotRunning && (iNow - iAdcMicrosLoopLast > iUsLoopFOC) ) // 250 us = 4 kHz
    {
      bIrqMotorThreadNotRunning = false;

      BLDCMotor::loopFOC();
      BLDCMotor::move(target);
      iAdcMicrosLoop = iNow - iAdcMicrosLoopLast;
      iAdcMicrosLoopLast = iNow;

      bIrqMotorThreadNotRunning = true;
    }
  }
}


extern "C" 
{  
  void ADC_CMP_IRQHandler(void)
  {
    if ( adc_interrupt_flag_get(ADC_INT_EOIC) != RESET)
    {
      adc_interrupt_flag_clear(ADC_INT_EOIC);
      if (pIrqMotor)  pIrqMotor->IrqHandler();
    }
  }
}
