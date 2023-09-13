#ifndef GD32_DRIVER_MCU_DEF
#define GD32_DRIVER_MCU_DEF
#include "../../hardware_api.h"

#if defined(_GD32_DEF_)   // Declared in ArduinoCore-GD32/cores/arduino/gd32/gd32_def.h

// default pwm parameters
#define _PWM_RESOLUTION 12 // 12bit
#define _PWM_FREQUENCY 16000 // 16khz
#define _PWM_FREQUENCY_MAX 50000 // 50khz

// 6pwm parameters
#define _HARDWARE_6PWM 1
#define _SOFTWARE_6PWM 0
#define _ERROR_6PWM -1

#define RCU_TIMER_BLDC RCU_TIMER0
#define TIMER_BLDC TIMER0
#define DEAD_TIME 60

// Channel G
#define TIMER_BLDC_CHANNEL_G TIMER_CH_2
// Channel B
#define TIMER_BLDC_CHANNEL_B TIMER_CH_1
// Channel Y
#define TIMER_BLDC_CHANNEL_Y TIMER_CH_0
// Channel for Sampling Point
#define TIMER_BLDC_CHANNEL_S TIMER_CH_3
#define SAMPLING_POINT 0.98 // The closer to 1 the closer to the middle of LOW side ON time. Don't use 1, it will overlap with the update event and not trigger

// Break input
#define TIMER_BLDC_EMERGENCY_SHUTDOWN_PIN PB12

typedef struct GD32DriverParams {
  uint32_t timers[6] = {NULL};
  uint32_t channels[6];
  long pwm_frequency;
  long unsigned int range;
  float dead_zone;
  uint8_t interface_type;
} GD32DriverParams;


#endif
#endif