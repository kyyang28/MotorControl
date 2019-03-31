#ifndef __ADC_IMPL_H
#define __ADC_IMPL_H

#include "adc.h"
#include "RCCTypes.h"
#include "platform.h"

#if defined(STM32F4)
#define ADC_TAG_MAP_COUNT				16
#endif

typedef enum {
	ADCINVALID = -1,
	ADCDEV_1 = 0,
#if defined(STM32F4)
	ADCDEV_2,
	ADCDEV_3
#endif
}AdcDevice;

typedef struct adcTagMap_s {
	ioTag_t tag;
	uint8_t channel;
}adcTagMap_t;

typedef struct adcDevice_s {
	ADC_TypeDef *ADCx;
	RccPeriphTag_t rccAdc;
#if defined(STM32F4)
	DMA_Stream_TypeDef *DMAy_Streamx;
	uint32_t channel;
#endif
	
}adcDevice_t;

extern adcOperationConfig_t adcOperationConfig[ADC_CHANNEL_COUNT];
extern const adcDevice_t adcHardware[];
extern const adcTagMap_t adcTagMap[ADC_TAG_MAP_COUNT];
extern volatile uint16_t adcValues[ADC_CHANNEL_COUNT];

uint8_t adcChannelByTag(ioTag_t ioTag);

#endif	// __ADC_IMPL_H
