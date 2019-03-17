#ifndef __RX_PWM_H
#define __RX_PWM_H

#include "io.h"

#define PPM_RCVR_TIMEOUT            	(0u)
#define PWM_INPUT_PORT_COUNT			(8u)
#define PWM_ENCODER_INPUT_PORT_COUNT	(4u)
#define PWM_ULTRASOUND_ECHO_PORT_COUNT	(2u)		// change this to change the number of ultrasound sensors that utilised

typedef enum {
    INPUT_FILTERING_DISABLED = 0,
    INPUT_FILTERING_ENABLED
} inputFilteringMode_e;

typedef struct pwmConfig_s {
	ioTag_t ioTags[PWM_INPUT_PORT_COUNT];
	inputFilteringMode_e inputFilteringMode;
}pwmConfig_t;

/* Encoder PWM Configuration */
typedef struct pwmEncoderConfig_s {
	ioTag_t ioTags[PWM_ENCODER_INPUT_PORT_COUNT];
	inputFilteringMode_e inputFilteringMode;
}pwmEncoderConfig_t;

/* Ultrasound PWM Configuration */
typedef struct ultrasoundTimerConfig_s {
	ioTag_t trigger1IOTag;
	ioTag_t trigger2IOTag;
	ioTag_t ioTags[PWM_ULTRASOUND_ECHO_PORT_COUNT];
	inputFilteringMode_e inputFilteringMode;
}ultrasoundTimerConfig_t;

void pwmRxInit(const pwmConfig_t *pwmConfig);
void pwmEncoderInit(const pwmEncoderConfig_t *pwmEncoderConfig);
void ultrasoundTimerInit(ultrasoundTimerConfig_t *ultrasoundTimerConfig);

/* Read duration of high level pulse */
//uint16_t pwmRead(uint8_t channel);
uint32_t pwmRead(uint8_t channel);		// TODO: change back to return type uint16_t later

bool isPWMDataBeingReceived(void);

#endif	// __RX_PWM_H
