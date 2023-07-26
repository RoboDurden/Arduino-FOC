#ifndef LOWSIDE_CS_TIMER_LIB_H
#define LOWSIDE_CS_TIMER_LIB_H

#include "LowsideCurrentSense.h"
#include <HardwareTimer.h>
#include "../BLDCMotor.h"
#include <vector>

#define TIMER_HZ_default 8000


class LowsideCurrentSenseTimer: public LowsideCurrentSense
{
public:
  LowsideCurrentSenseTimer(float shunt_resistor, float gain, int pinA, int pinB, int pinC = _NC);

  void (*callback)(void);
  void AddCallback(void (*cb)() = nullptr);


  static std::vector<BLDCMotor*> apMotor; 
	static long iMicrosTimerCb;
	static long iMicrosAdcReady;
	static unsigned long iCount;
	static unsigned long iTimerHz;

  static int iAdcCount;
  static uint16_t* ai_AdcBuffer;
  static uint16_t ai_AdcBuffer0;
  static uint16_t ai_AdcBuffer1;
  static uint16_t ai_AdcBuffer2;
  static uint16_t ai_AdcBuffer3;


  int init() override;
  int init(uint32_t _instanceTimer, unsigned long iH = TIMER_HZ_default);

  void linkMotor(BLDCMotor* _pMotor);
  void AddAdc(int pin, float fConvert, unsigned int iOffsetCount = 0);
  void AddCurrentSensor(LowsideCurrentSenseTimer& oCurrentSensor);
  
  float ReadAdcVoltage(const int pin);
  uint16_t ReadAanalog(const int pin);
  float ReadAdc(const int pin);
  uint16_t ReadBufferPos(const int iPos);

  // function reading an ADC value and returning the read voltage
  float _readADCVoltageLowSide(const int pin, const void* cs_params)
  {
    return ((LowsideCurrentSenseTimer*) cs_params)->ReadAdcVoltage(pin);
  }

private:

  LowsideCurrentSenseTimer* pMaster;
	HardwareTimer* pTimer;
  // DMA (ADC) structs
  dma_parameter_struct dma_init_struct_adc;

  typedef struct
  {
    int pin;
    float fConvert;
    unsigned int iOffsetCount;
  } add_adc_t;
  std::vector<add_adc_t> ao_AddAdc; 

  typedef struct
  {
    int pinA;
    int pinB;
    int pinC;
  } add_cs_t;
  std::vector<add_cs_t> ao_AddCS; 

  std::vector<int> aiPin; 
  uint8_t aiAdcMap[DIGITAL_PINS_NUM];

	static void timer_cb();
  void _AddPin(const int pin);
  void _RCU_init(void);
  void _DMA_init(void);
  void _ADC_init(void);
};

#endif
