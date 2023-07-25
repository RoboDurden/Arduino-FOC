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
    /**
      LowsideCurrentSenseTimer class constructor
      @param shunt_resistor shunt resistor value
      @param gain current-sense op-amp gain
      @param phA A phase adc pin
      @param phB B phase adc pin
      @param phC C phase adc pin (optional)
    */
    LowsideCurrentSenseTimer(float shunt_resistor, float gain, int pinA, int pinB, int pinC = _NC);

    void (*callback)(void);

    void AddCallback(void (*cb)() = nullptr);


    static std::vector<BLDCMotor*> apMotor; 


    //void linkMotor(BLDCMotor* _pMotor) {pMotor = _pMotor;}
    void linkMotor(BLDCMotor* _pMotor);


    int init() override;
    int init(uint32_t _instanceTimer, unsigned long iH = TIMER_HZ_default);

    void AddAdc(int pin, float fConvert, unsigned int iOffsetCount = 0);
    void  AddCurrentSensor(LowsideCurrentSenseTimer& oCurrentSensor);
    float ReadAdcVoltage(const int pin);
    // function reading an ADC value and returning the read voltage
    float _readADCVoltageLowSide(const int pin, const void* cs_params)
    {
      return ((LowsideCurrentSenseTimer*) cs_params)->ReadAdcVoltage(pin);
    }

    float ReadAdc(const int pin);

	static long iMicrosTimerCb;
	static long iMicrosAdcReady;
	static unsigned long iCount;
	static unsigned long iTimerHz;

  private:

	HardwareTimer* pTimer;


	static void timer_cb();


  dma_parameter_struct dma_init_struct_adc;

  uint8_t iAdc;

  LowsideCurrentSenseTimer* pMaster;

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


  static uint16_t* ai_AdcBuffer;

  uint8_t aiAdcMap[DIGITAL_PINS_NUM];
  int iAdcCount;
  
void _InitPin(const int pin);


  void _RCU_init(void);
  void _DMA_init(void);
  void _ADC_init(void);
};

#endif
