#ifndef __ADC_H
#define __ADC_H

#include "io.h"

typedef enum {
	ADC_MOTOR_CURRENT1 = 0,
	ADC_MOTOR_CURRENT2 = 1,
	ADC_CHANNEL_COUNT
}AdcChannel;

typedef struct adcChannelConfig_s {
	bool enabled;
	ioTag_t ioTag;
}adcChannelConfig_t;

typedef struct adcConfig_s {
	adcChannelConfig_t motorCurrentMeter1;
	adcChannelConfig_t motorCurrentMeter2;
	uint16_t resolutionScale;
}adcConfig_t;

typedef struct adcOperationConfig_s {
	ioTag_t tag;
	uint8_t adcChannel;				// ADC channel number
	uint8_t dmaIndex;
	bool enabled;
	uint8_t sampleTime;
}adcOperationConfig_t;

void adcInit(adcConfig_t *adcConfig);
uint16_t adcGetChannelSample(uint8_t channel);

#endif	// __ADC_H
