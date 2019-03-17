
#include "timer.h"
#include "dma.h"
#include "target.h"

//typedef struct timerHardware_s {
//    TIM_TypeDef *tim;
//    ioTag_t tag;
//    uint8_t channel;
//    timerUsageFlag_e usageFlags;
//    uint8_t output;
//    uint8_t alternateFunction;
//#if defined(USE_DSHOT)
//    DMA_Stream_TypeDef *dmaStream;		//	for STM32F4
//    uint32_t dmaChannel;					// 	for STM32F1
//    uint8_t dmaIrqHandler;
//#endif
//} timerHardware_t;

const timerHardware_t timerHardware[USABLE_TIMER_CHANNEL_COUNT] = {
	{	/* TIM2 PWM 2 for Motor 1 Encoder Phase A */
		.tim = TIM2,
		.tag = IO_TAG(PA0),
		.channel = TIM_Channel_1,
		.usageFlags = TIM_USE_ENCODER,
		.output = TIMER_OUTPUT_NONE,
		.alternateFunction = GPIO_AF_TIM2,
#ifdef USE_DSHOT
		.dmaStream = NULL,
//		.dmaChannel = 0,
		.dmaIrqHandler = 0
#endif
	},
	{	/* TIM2 PWM 3 for Motor 1 Encoder Phase B */
		.tim = TIM2,
		.tag = IO_TAG(PA1),
		.channel = TIM_Channel_2,
		.usageFlags = TIM_USE_ENCODER,
		.output = TIMER_OUTPUT_NONE,
		.alternateFunction = GPIO_AF_TIM2,
	},
	{	/* TIM2 PWM 2 for Motor 2 Encoder Phase A */
		.tim = TIM4,
		.tag = IO_TAG(PB6),
		.channel = TIM_Channel_1,
		.usageFlags = TIM_USE_ENCODER,
		.output = TIMER_OUTPUT_NONE,
		.alternateFunction = GPIO_AF_TIM4,
	},
	{	/* TIM2 PWM 2 for Motor 2 Encoder Phase B */
		.tim = TIM4,
		.tag = IO_TAG(PB7),
		.channel = TIM_Channel_2,
		.usageFlags = TIM_USE_ENCODER,
		.output = TIMER_OUTPUT_NONE,
		.alternateFunction = GPIO_AF_TIM4,
	},
	{	/* TIM1 PWM generator for DC Brushed MOTOR 1 */
		.tim = TIM1,
		.tag = IO_TAG(PA8),
		.channel = TIM_Channel_1,
		.usageFlags = TIM_USE_MOTOR,
		.output = TIMER_OUTPUT_STANDARD,
		.alternateFunction = GPIO_AF_TIM1,
	},
	{	/* TIM1 PWM generator for DC Brushed MOTOR 2 */
		.tim = TIM1,
		.tag = IO_TAG(PA10),				// TIM1 PA9 (TIM_Channel_2) is not working for some reason
		.channel = TIM_Channel_3,
		.usageFlags = TIM_USE_MOTOR,
		.output = TIMER_OUTPUT_STANDARD,
		.alternateFunction = GPIO_AF_TIM1,
	},
	{	/* TIM3 Input Capture Mode for ultrasound 1 echo pin */
		.tim = TIM3,
		.tag = IO_TAG(PB0),
		.channel = TIM_Channel_3,
		.usageFlags = TIM_USE_PWM,
		.output = TIMER_OUTPUT_NONE,
		.alternateFunction = GPIO_AF_TIM3,
	},
	{	/* TIM3 Input Capture Mode for ultrasound 2 echo pin */
		.tim = TIM3,
		.tag = IO_TAG(PB1),
		.channel = TIM_Channel_4,
		.usageFlags = TIM_USE_PWM,
		.output = TIMER_OUTPUT_NONE,
		.alternateFunction = GPIO_AF_TIM3,
	},
};
