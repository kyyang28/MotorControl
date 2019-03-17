
#include <stdio.h>
#include <string.h>
#include "pwm_output.h"			// including timer.h, ledTimer.h, motors.h
#include "timer.h"

static IO_t motorDriverAIN1;
static IO_t motorDriverAIN2;
static IO_t motorDriverBIN1;
static IO_t motorDriverBIN2;
static pwmOutputPort_t dcBrushedMotors[MAX_SUPPORTED_DC_BRUSHED_MOTORS_FOR_SBWMR];

#define TIMER_ARR				7200

bool pwmMotorsEnabled = false;

//		TIM1->CCMR2|=6<<12;        //CH4 PWM1模式
//		TIM1->CCMR1|=6<<4;         //CH1 PWM1模式
//		TIM1->CCMR2|=1<<11;        //CH4预装载使能
//		TIM1->CCMR1|=1<<3;         //CH1预装载使能
//		TIM1->CCER|=1<<12;         //CH4输出使能
//		TIM1->CCER|=1<<0;          //CH1输出使能		

static void pwmOCConfig(TIM_TypeDef *tim, uint8_t channel, uint16_t value, uint8_t output)
{
	TIM_OCInitTypeDef TIM_OCInitStructure;
	
	TIM_OCStructInit(&TIM_OCInitStructure);
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	
	if (output & TIMER_OUTPUT_N_CHANNEL) {
		TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
		TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;
		TIM_OCInitStructure.TIM_OCNPolarity = (output & TIMER_OUTPUT_INVERTED) ? TIM_OCNPolarity_High : TIM_OCNPolarity_Low;
	}else {
//		printf("output: %u, %s, %d\r\n", output, __FUNCTION__, __LINE__);			// output = 1
		TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
		TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
		TIM_OCInitStructure.TIM_OCPolarity = (output & TIMER_OUTPUT_INVERTED) ? TIM_OCPolarity_Low : TIM_OCPolarity_High;	// TIM_OCInitStructure.TIM_OCPolarity = 0x0
//		printf("TIM_OCInitStructure.TIM_OCPolarity: 0x%x, %s, %d\r\n", TIM_OCInitStructure.TIM_OCPolarity, __FUNCTION__, __LINE__);
	}
	
	TIM_OCInitStructure.TIM_Pulse = value;
	
	timerOCInit(tim, channel, &TIM_OCInitStructure);
	timerOCPreloadConfig(tim, channel, TIM_OCPreload_Enable);
}

void pwmWriteDcBrushedMotor(uint8_t index, uint16_t value)
{
//	printf("%u, %u\r\n", index, value);
//	printf("ccr%d addr: 0x%x\r\n", index, (uint32_t)dcBrushedMotors[index].ccr);
	*dcBrushedMotors[index].ccr = value;
}

//static IO_t motorDriverAIN1;
//static IO_t motorDriverAIN2;
//static IO_t motorDriverBIN1;
//static IO_t motorDriverBIN2;

void dcBrushedMotorInit(const dcBrushedMotorConfig_t *dcBrushedMotorConfig)
{
	motorDriverAIN1 = IOGetByTag(dcBrushedMotorConfig->AIN1);
	motorDriverAIN2 = IOGetByTag(dcBrushedMotorConfig->AIN2);
	motorDriverBIN1 = IOGetByTag(dcBrushedMotorConfig->BIN1);
	motorDriverBIN2 = IOGetByTag(dcBrushedMotorConfig->BIN2);

	IOInit(motorDriverAIN1, OWNER_MOTOR, 0);
	IOInit(motorDriverAIN2, OWNER_MOTOR, 1);
	IOInit(motorDriverBIN1, OWNER_MOTOR, 2);
	IOInit(motorDriverBIN2, OWNER_MOTOR, 3);

	IOConfigGPIO(motorDriverAIN1, IOCFG_OUT_PP);
	IOConfigGPIO(motorDriverAIN2, IOCFG_OUT_PP);
	IOConfigGPIO(motorDriverBIN1, IOCFG_OUT_PP);
	IOConfigGPIO(motorDriverBIN2, IOCFG_OUT_PP);

	/* Motor 1 and 2 PWM Init */
	for (int motorIndex = 0; motorIndex < MAX_SUPPORTED_DC_BRUSHED_MOTORS_FOR_SBWMR; motorIndex++) {
		IO_t PWM = IOGetByTag(dcBrushedMotorConfig->ioTagsPWM[motorIndex]);
		const timerHardware_t *timerHardware = timerGetByTag(dcBrushedMotorConfig->ioTagsPWM[motorIndex], TIM_USE_MOTOR);
		
		IOInit(PWM, OWNER_MOTOR, 4 + motorIndex);
		IOConfigGPIOAF(PWM, IOCFG_AF_PP, timerHardware->alternateFunction);

		/* Time base structure configuration */
		configTimeBase4Encoder(timerHardware->tim, TIMER_ARR, 0);	// TIMER_ARR = 7200 - 1 = 7199, PSC = 0
		
		/* Output compare PWM generator configuration */
		pwmOCConfig(timerHardware->tim, timerHardware->channel, 0, timerHardware->output);
		
		/* For advanced TIMER, TIM1 and TIM8 to output PWM signals */
		TIM_CtrlPWMOutputs(timerHardware->tim, ENABLE);
		
		TIM_Cmd(timerHardware->tim, ENABLE);				// Enable TIMER		

		dcBrushedMotors[motorIndex].ccr = timerChCCR(timerHardware);	// Config the channel address for duty cycle
		dcBrushedMotors[motorIndex].period = TIMER_ARR;					// Config the PWM period
		dcBrushedMotors[motorIndex].tim = timerHardware->tim;			// Config the Timer number
		
		*dcBrushedMotors[motorIndex].ccr = 0;							// No duty cycle
	}
}
