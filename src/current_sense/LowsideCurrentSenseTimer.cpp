#include "LowsideCurrentSenseTimer.h"
#include "./communication/SimpleFOCDebug.h"

// to see the interrupthandler getting called:
#define LED_GREEN PA15
#define LED_ORANGE PA12
#define LED_RED PB3

// definition of static element variables:

uint16_t* LowsideCurrentSenseTimer::ai_AdcBuffer;
 uint16_t LowsideCurrentSenseTimer::ai_AdcBufferLog[8];

uint16_t LowsideCurrentSenseTimer::ai_AdcBuffer0;	// for StmStudio..
uint16_t LowsideCurrentSenseTimer::ai_AdcBuffer1;
uint16_t LowsideCurrentSenseTimer::ai_AdcBuffer2;
uint16_t LowsideCurrentSenseTimer::ai_AdcBuffer3;
int LowsideCurrentSenseTimer::iAdcCount;

long LowsideCurrentSenseTimer::iMicrosTimerCb;
long LowsideCurrentSenseTimer::iMicrosAdcReady;
unsigned long LowsideCurrentSenseTimer::iCount;
unsigned long LowsideCurrentSenseTimer::iTimerHz;
uint32_t LowsideCurrentSenseTimer::iCountTimer0;

std::vector<BLDCMotor*> LowsideCurrentSenseTimer::apMotor;


extern "C"		// c-style callback interrupt handler :-/
{
	// This function handles DMA_Channel0_IRQHandler interrupt
	// Is called, when the ADC scan sequence is finished
	void DMA_Channel0_IRQHandler(void)
	{
		LowsideCurrentSenseTimer::iMicrosAdcReady = getCurrentMicros() - LowsideCurrentSenseTimer::iMicrosTimerCb;

		for (int i=LowsideCurrentSenseTimer::apMotor.size()-1; i>=0; i--)	
			if (LowsideCurrentSenseTimer::apMotor[i]->enabled)
				LowsideCurrentSenseTimer::apMotor[i]->loopFOC();

//qq	copy to uint16 for StmStudio to import
		for (int i=LowsideCurrentSenseTimer::iAdcCount-1; i>=0; i--)
			LowsideCurrentSenseTimer::ai_AdcBufferLog[i] = LowsideCurrentSenseTimer::ai_AdcBuffer[i];

		LowsideCurrentSenseTimer::ai_AdcBuffer0 = LowsideCurrentSenseTimer::ai_AdcBuffer[0];
		LowsideCurrentSenseTimer::ai_AdcBuffer1 = LowsideCurrentSenseTimer::ai_AdcBuffer[1];
		LowsideCurrentSenseTimer::ai_AdcBuffer2 = LowsideCurrentSenseTimer::ai_AdcBuffer[2];
		LowsideCurrentSenseTimer::ai_AdcBuffer3 = LowsideCurrentSenseTimer::ai_AdcBuffer[3];
		//motor.loopFOC();
		digitalWrite(LED_GREEN, (LowsideCurrentSenseTimer::iCount % LowsideCurrentSenseTimer::iTimerHz) < (LowsideCurrentSenseTimer::iTimerHz/2) );

		if (dma_interrupt_flag_get(DMA_CH0, DMA_INT_FLAG_FTF))
			dma_interrupt_flag_clear(DMA_CH0, DMA_INT_FLAG_FTF);        
	}

}



LowsideCurrentSenseTimer::LowsideCurrentSenseTimer(float _shunt_resistor, float _gain, int _pinA, int _pinB, int _pinC)
	: LowsideCurrentSense(_shunt_resistor, _gain, _pinA, _pinB, _pinC)
{
	pMaster = NULL;
	pTimer = NULL;
}


void LowsideCurrentSenseTimer::linkMotor(BLDCMotor* _pMotor)
{
	apMotor.push_back(_pMotor);
}

// not working
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

float LowsideCurrentSenseTimer::ReadAdc(const int pin)
{
	for (int i=ao_AddAdc.size()-1; i>=0; i--)
		if (pin == ao_AddAdc[i].pin)
		{
			int iBufferPos = aiAdcMap[pin];
			float fConvert = ao_AddAdc[i].fConvert;
			uint16_t iAnalog = ai_AdcBuffer[iBufferPos];
			float f = iAnalog * fConvert;
			float f2 = ai_AdcBuffer[aiAdcMap[pin]] * ao_AddAdc[i].fConvert;
			return ai_AdcBuffer[aiAdcMap[pin]] * ao_AddAdc[i].fConvert;

		}
	return 0;
}

uint16_t LowsideCurrentSenseTimer::ReadAanalog(const int pin)
{
	for (int i=ao_AddAdc.size()-1; i>=0; i--)
		if (pin == ao_AddAdc[i].pin)
			return ai_AdcBuffer[aiAdcMap[pin]];
	return 0;
}

uint16_t LowsideCurrentSenseTimer::ReadBufferPos(const int iPos)
{
	return ai_AdcBuffer[iPos];
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

void LowsideCurrentSenseTimer::_AddPin(const int pin)
{
	if (_isset(pin))
	{
		aiAdcMap[pin] = aiPin.size();
		aiPin.push_back(pin);
	}	
} 




// Lowside sensor init function
int LowsideCurrentSenseTimer::init()
{
	return init(TIMER2);
}

int LowsideCurrentSenseTimer::init(uint32_t instanceTimer, unsigned long iHz)
{
	//LowsideCurrentSense::init();

	aiPin.empty();

	_AddPin(pinA);
	_AddPin(pinB);
	_AddPin(pinC);
	for (int i=0; i<ao_AddCS.size(); i++)	
	{
		_AddPin(ao_AddCS[i].pinA);
		_AddPin(ao_AddCS[i].pinB);
		_AddPin(ao_AddCS[i].pinC);
	}
	for (int i=0; i<ao_AddAdc.size(); i++)	
		_AddPin(ao_AddAdc[i].pin);

//qq
	LowsideCurrentSenseTimer::iAdcCount = aiPin.size();

	if (ai_AdcBuffer)	delete ai_AdcBuffer;
	ai_AdcBuffer = new uint16_t[aiPin.size()];
	//ai_AdcBuffer = new uint16_t[3];

	ai_AdcBuffer[0] = ai_AdcBuffer[1] = ai_AdcBuffer[2] = 1500;

	_RCU_init();
	_DMA_init();
	_ADC_init();	// the Gen2 style. Adc finishes 7-8 microseconds after TIMER2-callback triggered new conversion :-)
	//ADC_init2();	// some other code found in the net. Maybe continious adc. But time between two handler callbacks varies from 5-130 us :-/

	iTimerHz = iHz;

	if(pTimer)
	{
		pTimer->stop();
		delete pTimer;
	}	


	// Brushless Control DC (BLDC) defines
	// Channel G
	#define BLDC_GH_PIN PA10
	#define BLDC_GL_PIN PB15
	// Channel B
	#define BLDC_BH_PIN PA9
	#define BLDC_BL_PIN PB14
	// Channel Y
	#define BLDC_YH_PIN PA8
	#define BLDC_YL_PIN PB13

/*
	pwmDevice_t oPwmDevice = getTimerDeviceFromPinname(DIGITAL_TO_PINNAME(BLDC_GH_PIN));	// -> 2 = TIMER_CH_2
	SIMPLEFOC_DEBUG("Timer_CH ", oPwmDevice.channel);
	oPwmDevice = getTimerDeviceFromPinname(DIGITAL_TO_PINNAME(BLDC_GL_PIN));	// -> 1 = TIMER_CH_1
	SIMPLEFOC_DEBUG("Timer_CH ", oPwmDevice.channel);
	oPwmDevice = getTimerDeviceFromPinname(DIGITAL_TO_PINNAME(BLDC_BH_PIN));	// -> 1 = TIMER_CH_1
	SIMPLEFOC_DEBUG("Timer_CH ", oPwmDevice.channel);
	oPwmDevice = getTimerDeviceFromPinname(DIGITAL_TO_PINNAME(BLDC_BL_PIN));	// -> 0 = TIMER_CH_0
	SIMPLEFOC_DEBUG("Timer_CH ", oPwmDevice.channel);
	oPwmDevice = getTimerDeviceFromPinname(DIGITAL_TO_PINNAME(BLDC_YH_PIN));	// -> 0 = TIMER_CH_0
	SIMPLEFOC_DEBUG("Timer_CH ", oPwmDevice.channel);
	oPwmDevice = getTimerDeviceFromPinname(DIGITAL_TO_PINNAME(BLDC_YL_PIN));	// -> 0 = TIMER_CH_0
	SIMPLEFOC_DEBUG("Timer_CH ", oPwmDevice.channel);
	*/

/*
	pTimer = new HardwareTimer(instanceTimer);
    pTimer->setPeriodTime(iTimerHz, FORMAT_HZ);
    pTimer->attachInterrupt(&timer_cb);
    pTimer->start();

	pTimer = new HardwareTimer(instanceTimer);
	pTimer->attachInterrupt(&timer_cb);

*/

	//delay(1);

    // calibrate zero offsets
    //calibrateOffsets();
    // set the initialized flag
	initialized = true;
    return 1;
}

extern "C"
{
	#define TIMER_BLDC TIMER0
	void TIMER0_BRK_UP_TRG_COM_IRQHandler(void)
	{
		LowsideCurrentSenseTimer::iCountTimer0 = timer_counter_read(TIMER0);

		if (SET == timer_interrupt_flag_get(TIMER_BLDC,TIMER_INT_FLAG_UP))
		{
			//digitalWrite(LED_RED, digitalRead(LED_RED) ^ 1);
			LowsideCurrentSenseTimer::iCount++;
			digitalWrite(LED_ORANGE, (LowsideCurrentSenseTimer::iCount % LowsideCurrentSenseTimer::iTimerHz) < (LowsideCurrentSenseTimer::iTimerHz/4) );

			// Start ADC conversion
			adc_software_trigger_enable(ADC_REGULAR_CHANNEL);
			LowsideCurrentSenseTimer::iMicrosTimerCb = getCurrentMicros();


			timer_interrupt_flag_clear(TIMER_BLDC,TIMER_INT_FLAG_UP);
		}
	}
	/*
*/
	void TIMER0_BRK_UP_TRG_COM_IRQnHandler(void)
	{
		if (SET == timer_interrupt_flag_get(TIMER_BLDC,TIMER_INT_FLAG_UP))
		{
			//digitalWrite(LED_RED, digitalRead(LED_RED) ^ 1);
			LowsideCurrentSenseTimer::iCount++;
			digitalWrite(LED_ORANGE, (LowsideCurrentSenseTimer::iCount % LowsideCurrentSenseTimer::iTimerHz) < (LowsideCurrentSenseTimer::iTimerHz/4) );

			// Start ADC conversion
			adc_software_trigger_enable(ADC_REGULAR_CHANNEL);
			LowsideCurrentSenseTimer::iMicrosTimerCb = getCurrentMicros();


			timer_interrupt_flag_clear(TIMER_BLDC,TIMER_INT_FLAG_UP);
		}
	}

}


void LowsideCurrentSenseTimer::timer_cb() 
{
	LowsideCurrentSenseTimer::iCountTimer0 = timer_counter_read(TIMER0);

    //digitalWrite(LED_RED, digitalRead(LED_RED) ^ 1);
	LowsideCurrentSenseTimer::iCount++;

	digitalWrite(LED_ORANGE, (LowsideCurrentSenseTimer::iCount % LowsideCurrentSenseTimer::iTimerHz) < (LowsideCurrentSenseTimer::iTimerHz/4) );

	// Start ADC conversion
	adc_software_trigger_enable(ADC_REGULAR_CHANNEL);
	LowsideCurrentSenseTimer::iMicrosTimerCb = getCurrentMicros();
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
	dma_init_struct_adc.memory_addr = (uint32_t) ai_AdcBuffer;
	dma_init_struct_adc.memory_inc = DMA_MEMORY_INCREASE_ENABLE;
	dma_init_struct_adc.memory_width = DMA_MEMORY_WIDTH_16BIT;
	dma_init_struct_adc.number = aiPin.size();
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
	dma_transfer_number_config(DMA_CH0, aiPin.size());
	
	// Enable DMA channel 0
	dma_channel_enable(DMA_CH0);
}	



// Gen2 code that needs a TIMER to trigger the adc conversion which then triggers an event handler
void LowsideCurrentSenseTimer::_ADC_init(void)
{
	adc_channel_length_config(ADC_REGULAR_CHANNEL, aiPin.size());
	//adc_channel_length_config(ADC_REGULAR_CHANNEL, 1);

	int iSize = aiPin.size();
	for (int i=0; i<iSize; i++)
	{
		PinName pinname = DIGITAL_TO_PINNAME(aiPin[i]);
		uint32_t gd_pin =  gpio_pin[GD_PIN_GET(pinname)];
		uint32_t gd_port = gpio_port[GD_PORT_GET(pinname)];
		uint8_t gd_channel = get_adc_channel(pinname);
	
		gpio_mode_set(gd_port, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, gd_pin);
		adc_regular_channel_config(i, gd_channel , ADC_SAMPLETIME_13POINT5);
	}

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
