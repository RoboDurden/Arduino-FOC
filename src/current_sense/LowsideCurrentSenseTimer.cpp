#include "LowsideCurrentSenseTimer.h"


#define LED_GREEN PA15
#define LED_ORANGE PA12
#define LED_RED PB3

// ADC defines
#define VBATT_PIN	GPIO_PIN_4
#define VBATT_PORT GPIOA
#define VBATT_CHANNEL ADC_CHANNEL_4
#define CURRENT_DC_PIN	GPIO_PIN_6
#define CURRENT_DC_PORT GPIOA
#define CURRENT_DC_CHANNEL ADC_CHANNEL_6


long LowsideCurrentSenseTimer::iMicrosTimerCb;
long LowsideCurrentSenseTimer::iMicrosAdcReady;
unsigned long LowsideCurrentSenseTimer::iCount;
unsigned long LowsideCurrentSenseTimer::iTimerHz;

uint16_t* LowsideCurrentSenseTimer::ai_AdcBuffer;

//HardwareTimer oTX(TIMER2);


std::vector<BLDCMotor*> LowsideCurrentSenseTimer::apMotor;

// LowsideCurrentSensor constructor
//  - shunt_resistor  - shunt resistor value
//  - gain  - current-sense op-amp gain
//  - phA   - A phase adc pin
//  - phB   - B phase adc pin
//  - phC   - C phase adc pin (optional)
LowsideCurrentSenseTimer::LowsideCurrentSenseTimer(float _shunt_resistor, float _gain, int _pinA, int _pinB, int _pinC)
	: LowsideCurrentSense(_shunt_resistor, _gain, _pinA, _pinB, _pinC)
{
	pMaster = NULL;
	iAdcCount = 0;
	pTimer = NULL;
}


void LowsideCurrentSenseTimer::linkMotor(BLDCMotor* _pMotor)
{
	apMotor.push_back(_pMotor);
}

void LowsideCurrentSenseTimer::AddCallback(void (*cb)())
{
  // attach interrupt if functions provided
  if(cb != nullptr) callback = cb;
}


void  LowsideCurrentSenseTimer::AddAdc(int pin, float fConvert, unsigned int iOffsetCount)
{
	ao_AddAdc.push_back((add_adc_t){pin, fConvert, iOffsetCount});
}

void  LowsideCurrentSenseTimer::AddCurrentSensor(LowsideCurrentSenseTimer& oCS)
{
	oCS.pMaster = this;
	ao_AddCS.push_back( (add_cs_t){oCS.pinA, oCS.pinB, oCS.pinC} );
}

// Lowside sensor init function
int LowsideCurrentSenseTimer::init()
{
	return init(TIMER2);
}

float LowsideCurrentSenseTimer::ReadAdc(const int pin)
{
	for (int i=ao_AddAdc.size()-1; i>=0; i--)
		if (pin == ao_AddAdc[i].pin)
		{
			int iAdc = aiAdcMap[pin];
			uint16_t iAnalog = ai_AdcBuffer[aiAdcMap[pin]];
			float fConvert = ao_AddAdc[i].fConvert;
			float fAdc = iAnalog * fConvert;
			float f = ai_AdcBuffer[aiAdcMap[pin]] * ao_AddAdc[i].fConvert;
			return f;

			//return ai_AdcBuffer[aiAdcMap[pin]] * ao_AddAdc[i].fConvert;
		}

/*
	int iSize = sizeof(ao_AddAdc) / sizeof(add_adc_t);
	for (int i=0; i<iSize; i++)	
		if (pin == ao_AddAdc[i].pin)
			return ai_AdcBuffer[aiAdcMap[pin]] * ao_AddAdc[i].fConvert;
*/

	return 0;
}


#define _ADC_VOLTAGE 3.3f
#define _ADC_RESOLUTION 4096.0f

float LowsideCurrentSenseTimer::ReadAdcVoltage(const int pin)
{
	if (pMaster)
		return pMaster->ReadAdcVoltage(pin);

	if (_isset(pin))
		return ai_AdcBuffer[aiAdcMap[pin]] * _ADC_VOLTAGE / _ADC_RESOLUTION;
	
	return 0;
}

void LowsideCurrentSenseTimer::_InitPin(const int pin)
{
	if (_isset(pin))
	{
		uint32_t iVBATT_PIN_OLD	GPIO_PIN_4;
		uint32_t iVBATT_PORT = GPIOA;
		uint32_t iVBATT_CHANNEL ADC_CHANNEL_4;
		uint32_t iCURRENT_DC_PIN_OLD	GPIO_PIN_6;
		uint32_t iCURRENT_DC_PORT GPIOA;
		uint32_t iCURRENT_DC_CHANNEL ADC_CHANNEL_6;

		PinName pinname = DIGITAL_TO_PINNAME(pin);
		uint32_t gd_pin =  gpio_pin[GD_PIN_GET(pinname)];
		uint32_t gd_port = gpio_port[GD_PORT_GET(pinname)];
		//(int)gpio_input_bit_get(gd_port, gd_pin);
	
		gpio_mode_set(gd_port, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, gd_pin);

		adc_regular_channel_config(iAdcCount, gd_pin, ADC_SAMPLETIME_13POINT5);

		aiAdcMap[pin] = iAdcCount;
		iAdcCount++;
	}	
} 


void LowsideCurrentSenseTimer::timer_cb() 
//void timer_cb() 
{
    //digitalWrite(LED_RED, digitalRead(LED_RED) ^ 1);
	LowsideCurrentSenseTimer::iCount++;
    digitalWrite(LED_ORANGE, (LowsideCurrentSenseTimer::iCount % LowsideCurrentSenseTimer::iTimerHz) < (LowsideCurrentSenseTimer::iTimerHz/4) );

	// Start ADC conversion
	adc_software_trigger_enable(ADC_REGULAR_CHANNEL);
	LowsideCurrentSenseTimer::iMicrosTimerCb = getCurrentMicros();
}


int LowsideCurrentSenseTimer::init(uint32_t instanceTimer, unsigned long iHz)
{
	//LowsideCurrentSense::init();

	params = this;

	iAdcCount = 0;
	//_InitPin(pinA);
	//_InitPin(pinB);
	//_InitPin(pinC);
	for (int i=0; i<ao_AddCS.size(); i++)	
	{
		_InitPin(ao_AddCS[i].pinA);
		_InitPin(ao_AddCS[i].pinB);
		_InitPin(ao_AddCS[i].pinC);
	}
	for (int i=0; i<ao_AddAdc.size(); i++)	
		_InitPin(ao_AddAdc[i].pin);

	ai_AdcBuffer = new uint16_t[iAdcCount];
	ai_AdcBuffer[0] = ai_AdcBuffer[1] = 42;
	
	_RCU_init();
	_DMA_init();
	_ADC_init();	// the Gen2 style. Adc finishes 7-8 microseconds after TIMER2-callback triggered new conversion :-)

	iTimerHz = iHz;

	if(pTimer)
	{
		pTimer->stop();
		delete pTimer;
	}	

	pTimer = new HardwareTimer(instanceTimer);
    pTimer->setPeriodTime(iTimerHz, FORMAT_HZ);
    pTimer->attachInterrupt(&timer_cb);
    pTimer->start();

/*	
    oTX.setPeriodTime(iTimerHz, FORMAT_HZ);
    oTX.attachInterrupt(&timer_cb);
    oTX.start();
*/
	//delay(1);

    // calibrate zero offsets
    //calibrateOffsets();
    // set the initialized flag
	initialized = true;
    return 1;
}



void LowsideCurrentSenseTimer::_RCU_init(void)
{
	// enable GPIOA clock
	rcu_periph_clock_enable(RCU_GPIOA);
	// enable ADC clock
	rcu_periph_clock_enable(RCU_ADC);
	// enable DMA clock
	rcu_periph_clock_enable(RCU_DMA);
	// config ADC clock
	rcu_adc_clock_config(RCU_ADCCK_APB2_DIV6);
}

void LowsideCurrentSenseTimer::_DMA_init(void)
{
// Configure ADC clock (APB2 clock is DIV1 -> 72MHz, ADC clock is DIV6 -> 12MHz)
	rcu_adc_clock_config(RCU_ADCCK_APB2_DIV6);
	
	// Interrupt channel 0 enable
	nvic_irq_enable(DMA_Channel0_IRQn, 1, 0);
	
	// Initialize DMA channel 0 for ADC
	dma_deinit(DMA_CH0);
	dma_init_struct_adc.direction = DMA_PERIPHERAL_TO_MEMORY;
	dma_init_struct_adc.memory_addr = (uint32_t) &ai_AdcBuffer;
	dma_init_struct_adc.memory_inc = DMA_MEMORY_INCREASE_ENABLE;
	dma_init_struct_adc.memory_width = DMA_MEMORY_WIDTH_16BIT;
	dma_init_struct_adc.number = iAdcCount;
	dma_init_struct_adc.periph_addr = (uint32_t)&ADC_RDATA;
	dma_init_struct_adc.periph_inc = DMA_PERIPH_INCREASE_DISABLE;
	dma_init_struct_adc.periph_width = DMA_PERIPHERAL_WIDTH_16BIT;
	dma_init_struct_adc.priority = DMA_PRIORITY_ULTRA_HIGH;
	dma_init(DMA_CH0, &dma_init_struct_adc);
	
	// Configure DMA mode
	dma_circulation_enable(DMA_CH0);
	dma_memory_to_memory_disable(DMA_CH0);
	
	// Enable DMA transfer complete interrupt
	dma_interrupt_enable(DMA_CH0, DMA_CHXCTL_FTFIE);
	
	// At least clear number of remaining data to be transferred by the DMA 
	dma_transfer_number_config(DMA_CH0, 2);
	
	// Enable DMA channel 0
	dma_channel_enable(DMA_CH0);
}	

// Gen2 code that needs a TIMER to trigger the adc conversion which then triggers an event handler
void LowsideCurrentSenseTimer::_ADC_init(void)
{
	adc_channel_length_config(ADC_REGULAR_CHANNEL, iAdcCount);

/*
		gpio_mode_set(VBATT_PORT, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, VBATT_PIN);
		gpio_mode_set(CURRENT_DC_PORT, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, CURRENT_DC_PIN);

		adc_channel_length_config(ADC_REGULAR_CHANNEL, 2);
		adc_regular_channel_config(0, VBATT_CHANNEL, ADC_SAMPLETIME_13POINT5);
		adc_regular_channel_config(1, CURRENT_DC_CHANNEL, ADC_SAMPLETIME_13POINT5);
*/

	adc_data_alignment_config(ADC_DATAALIGN_RIGHT);
	
	// Set trigger of ADC
	adc_external_trigger_config(ADC_REGULAR_CHANNEL, ENABLE);
	adc_external_trigger_source_config(ADC_REGULAR_CHANNEL, ADC_EXTTRIG_REGULAR_NONE);	// ADC_EXTTRIG_REGULAR_T2_TRGO or ADC_EXTTRIG_REGULAR_NONE
	
	// Disable the temperature sensor, Vrefint and vbat channel
	adc_tempsensor_vrefint_disable();
	adc_vbat_disable();

	// ADC analog watchdog disable
	adc_watchdog_disable();
	
	// Enable ADC (must be before calibration)
	adc_enable();
	
	// Calibrate ADC values
	adc_calibration_enable();
	
	// Enable DMA request
	adc_dma_mode_enable();

	adc_interrupt_enable(ADC_INT_EOC);
	//nvic_irq_enable(ADC_CMP_IRQn, 0U);    
	adc_software_trigger_enable(ADC_REGULAR_CHANNEL);
	
	// Set ADC to scan mode
	adc_special_function_config(ADC_SCAN_MODE, ENABLE);		//  without extern"C" this makes the MCU hang: processor receives an unexpected interrupt
}

extern "C"
{
	// This function handles DMA_Channel0_IRQHandler interrupt
	// Is called, when the ADC scan sequence is finished
	void DMA_Channel0_IRQHandler(void)
	{
		LowsideCurrentSenseTimer::iMicrosAdcReady = getCurrentMicros() - LowsideCurrentSenseTimer::iMicrosTimerCb;

		for (int i=LowsideCurrentSenseTimer::apMotor.size()-1; i>=0; i--)	
			if (LowsideCurrentSenseTimer::apMotor[i]->enabled)
				LowsideCurrentSenseTimer::apMotor[i]->loopFOC();

		digitalWrite(LED_GREEN, (LowsideCurrentSenseTimer::iCount % LowsideCurrentSenseTimer::iTimerHz) < (LowsideCurrentSenseTimer::iTimerHz/2) );

		if (dma_interrupt_flag_get(DMA_CH0, DMA_INT_FLAG_FTF))
			dma_interrupt_flag_clear(DMA_CH0, DMA_INT_FLAG_FTF);        
	}


}
