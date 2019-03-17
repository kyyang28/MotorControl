#ifndef __BUTTON_H
#define __BUTTON_H

#include "exti.h"

#define USER_BUTTON_PIN             PA0			// taken by TIM2 CH1 Encoder Phase A

typedef struct buttonDev_s {
	extiCallbackRec_t exti;
	const extiConfig_t *btnIntExtiConfig;
	bool btnPressed;
}buttonDev_t;

typedef struct button_s {
	ioTag_t btnPin;					// PA15	for external button
	buttonDev_t dev;
}button_t;

extern button_t button;

/* On-board user button */
bool buttonInit(void);
void userBtnPollInit(void);
void userBtnPollOps(void);

/* External button for SBWMR mode switches (Such as activating balancing and obstacle avoidance modes) */
void modeSwitchBtnPollInit(struct button_s *buttonModeSwitchConfig);

#endif	// __BUTTON_H
