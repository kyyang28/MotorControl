
#include <stdint.h>
#include "target.h"
#include "adc.h"
#include "adc_impl.h"

#ifdef USE_ADC

adcOperationConfig_t adcOperationConfig[ADC_CHANNEL_COUNT];
volatile uint16_t adcValues[ADC_CHANNEL_COUNT];

uint8_t adcChannelByTag(ioTag_t ioTag)
{
	for (uint8_t i = 0; i < ARRAYLEN(adcTagMap); i++) {
		if (ioTag == adcTagMap[i].tag) {
			return adcTagMap[i].channel;
		}
	}
	
	return 0;
}

uint16_t adcGetChannelSample(uint8_t channel)
{
	return adcValues[adcOperationConfig[channel].dmaIndex];
}

#else

uint16_t adcGetChannelSample(uint8_t channel)
{
	UNUSED(channel);
	return 0;
}

#endif	// USE_ADC
