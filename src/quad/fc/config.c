
#include <stdio.h>				// debugging purposes
#include <string.h>
#include "configMaster.h"		// including led.h, ledTimer.h, motors.h, sound_beeper.h, target.h, serial.h, boardAlignment.h
#include "config_profile.h"
#include "config_eeprom.h"
//#include "led.h"
//#include "ledTimer.h"			// for testing led timer ONLY
//#include "motors.h"				// including mixer.h
//#include "sound_beeper.h"
//#include "target.h"
//#include "serial.h"
#include "sensor.h"
#include "pwm_output.h"			// including timer.h
#include "feature.h"
#include "accgyro.h"
#include "acceleration.h"
#include "imu.h"
//#include "boardAlignment.h"
#include "filter.h"
#include "rc_controls.h"
#include "rx.h"
#include "common.h"
#include "maths.h"
#include "mixer.h"
#include "fc_rc.h"
#include "sdcard.h"
#include "blackbox_io.h"
#include "button.h"
#include "ultrasound_hcsr04.h"

#define BRUSHED_MOTORS_PWM_RATE 			16000
#define BRUSHLESS_MOTORS_PWM_RATE 			480

/* master config structure with data independent from profiles */
master_t masterConfig;
profile_t *currentProfile;
controlRateConfig_t *currentControlRateProfile;
static uint8_t currentControlRateProfileIndex = 0;

void setProfile(uint8_t profileIndex)
{
	currentProfile = &masterConfig.profile[profileIndex];
	currentControlRateProfileIndex = currentProfile->activeRateProfile;
	currentControlRateProfile = &currentProfile->controlRateProfile[currentControlRateProfileIndex];
}

//static void resetPidProfile(pidProfile_t *pidProfile)
//{
//	pidProfile->P8[ROLL] = 44;		// ROLL = 0
//	pidProfile->I8[ROLL] = 40;
//	pidProfile->D8[ROLL] = 30;
//	pidProfile->P8[PITCH] = 58;		// PITCH = 1
//	pidProfile->I8[PITCH] = 50;
//	pidProfile->D8[PITCH] = 35;
//	pidProfile->P8[YAW] = 70;
//	pidProfile->I8[YAW] = 45;
//	pidProfile->D8[YAW] = 20;
//    pidProfile->P8[PIDALT] = 50;
//    pidProfile->I8[PIDALT] = 0;
//    pidProfile->D8[PIDALT] = 0;
//    pidProfile->P8[PIDPOS] = 15;   // POSHOLD_P * 100;
//    pidProfile->I8[PIDPOS] = 0;    // POSHOLD_I * 100;
//    pidProfile->D8[PIDPOS] = 0;
//    pidProfile->P8[PIDPOSR] = 34;  // POSHOLD_RATE_P * 10;
//    pidProfile->I8[PIDPOSR] = 14;  // POSHOLD_RATE_I * 100;
//    pidProfile->D8[PIDPOSR] = 53;  // POSHOLD_RATE_D * 1000;
//    pidProfile->P8[PIDNAVR] = 25;  // NAV_P * 10;
//    pidProfile->I8[PIDNAVR] = 33;  // NAV_I * 100;
//    pidProfile->D8[PIDNAVR] = 83;  // NAV_D * 1000;
//    pidProfile->P8[PIDLEVEL] = 50;
//    pidProfile->I8[PIDLEVEL] = 50;
//    pidProfile->D8[PIDLEVEL] = 100;
//    pidProfile->P8[PIDMAG] = 40;
//    pidProfile->P8[PIDVEL] = 55;
//    pidProfile->I8[PIDVEL] = 55;
//    pidProfile->D8[PIDVEL] = 75;
//	
//	pidProfile->pidSumLimit = PIDSUM_LIMIT;
//	pidProfile->pidSumLimitYaw = PIDSUM_LIMIT_YAW;
//	pidProfile->yaw_lpf_hz = 0;
//	pidProfile->itermWindupPointPercent = 50;
//	pidProfile->dterm_filter_type = FILTER_BIQUAD;			// FILTER_BIQUAD = 1
//	pidProfile->dterm_lpf_hz = 100;							// filtering on by default
//	pidProfile->dterm_notch_hz = 260;
//	pidProfile->dterm_notch_cutoff = 160;
//	pidProfile->vbatPidCompensation = 0;
//	pidProfile->pidAtMinThrottle = PID_STABILISATION_ON;	// PID_STABILISATION_ON = 1, PID_STABILISATION_OFF = 0
//	pidProfile->levelAngleLimit = 55;
//	pidProfile->levelSensitivity = 55;
//	pidProfile->setpointRelaxRatio = 100;
//	pidProfile->dtermSetpointWeight = 60;
//	pidProfile->yawRateAccelLimit = 10.0f;
//	pidProfile->rateAccelLimit = 0.0f;
//	pidProfile->itermThrottleThreshold = 350;
//	pidProfile->itermAcceleratorGain = 3.0f;
//}

//void resetProfile(profile_t *profile)
//{
//	resetPidProfile(&profile->pidProfile);
//	
//	/* MAX_RATEPROFILES = 3 */
//	for (int rateIndex = 0; rateIndex < MAX_RATEPROFILES; rateIndex++) {
//		resetControlRateConfig(&profile->controlRateProfile[rateIndex]);
//	}
//	
//	profile->activeRateProfile = 0;			// set activeRateProfile to be the first one
//}

void ResetSerialPinConfig(serialPinConfig_t *pSerialPinConfig)
{
	for (int port = 0; port < SERIAL_PORT_MAX_INDEX; port++) {
		pSerialPinConfig->ioTagRx[port] = IO_TAG(NONE);
		pSerialPinConfig->ioTagTx[port] = IO_TAG(NONE);
	}
	
	for (int index = 0; index < SERIAL_PORT_COUNT; index++) {
		switch (serialPortIdentifiers[index]) {
			case SERIAL_PORT_USART1:
#ifdef USE_UART1
				pSerialPinConfig->ioTagRx[SERIAL_PORT_IDENTIFIER_TO_RESOURCE_INDEX(SERIAL_PORT_USART1)] = IO_TAG(UART1_RX_PIN);
				pSerialPinConfig->ioTagTx[SERIAL_PORT_IDENTIFIER_TO_RESOURCE_INDEX(SERIAL_PORT_USART1)] = IO_TAG(UART1_TX_PIN);
#endif
				break;

			case SERIAL_PORT_USART2:
#ifdef USE_UART2
//				pSerialPinConfig->ioTagRx[SERIAL_PORT_IDENTIFIER_TO_RESOURCE_INDEX(SERIAL_PORT_USART2)] = IO_TAG(UART2_RX_PIN);
//				pSerialPinConfig->ioTagTx[SERIAL_PORT_IDENTIFIER_TO_RESOURCE_INDEX(SERIAL_PORT_USART2)] = IO_TAG(UART2_TX_PIN);
#endif
				break;

			case SERIAL_PORT_USART3:
#ifdef USE_UART3
				pSerialPinConfig->ioTagRx[SERIAL_PORT_IDENTIFIER_TO_RESOURCE_INDEX(SERIAL_PORT_USART3)] = IO_TAG(UART3_RX_PIN);
				pSerialPinConfig->ioTagTx[SERIAL_PORT_IDENTIFIER_TO_RESOURCE_INDEX(SERIAL_PORT_USART3)] = IO_TAG(UART3_TX_PIN);
#endif
				break;

			case SERIAL_PORT_USART4:
				break;

			case SERIAL_PORT_USART5:
				break;

			case SERIAL_PORT_USART6:
#ifdef USE_UART6
				pSerialPinConfig->ioTagRx[SERIAL_PORT_IDENTIFIER_TO_RESOURCE_INDEX(SERIAL_PORT_USART6)] = IO_TAG(UART6_RX_PIN);
				pSerialPinConfig->ioTagTx[SERIAL_PORT_IDENTIFIER_TO_RESOURCE_INDEX(SERIAL_PORT_USART6)] = IO_TAG(UART6_TX_PIN);
#endif
				break;

			case SERIAL_PORT_USART7:
				break;

			case SERIAL_PORT_USART8:
				break;

			case SERIAL_PORT_SOFTSERIAL1:
				break;

			case SERIAL_PORT_SOFTSERIAL2:
				break;
			
			case SERIAL_PORT_USB_VCP:
				break;
			
			case SERIAL_PORT_NONE:
				break;
		}
	}
}

void ResetSerialConfig(serialConfig_t *serialConfig)
{
	memset(serialConfig, 0, sizeof(serialConfig_t));
	serialConfig->serial_update_rate_hz = 100;
	serialConfig->reboot_character		= 'R';
	
	for (int index = 0; index < SERIAL_PORT_COUNT; index++) {
		serialConfig->portConfigs[index].identifier			= serialPortIdentifiers[index];
		serialConfig->portConfigs[index].msp_baudrateIndex	= BAUD_115200;
		serialConfig->portConfigs[index].gps_baudrateIndex	= BAUD_115200;
//		serialConfig->portConfigs[index].gps_baudrateIndex	= BAUD_57600;
		serialConfig->portConfigs[index].blackbox_baudrateIndex	= BAUD_115200;		// blackbox port for debugging purposes
		serialConfig->portConfigs[index].telemetry_baudrateIndex = BAUD_AUTO;
	}
	
	serialConfig->portConfigs[0].functionMask = FUNCTION_MSP;
}

void ResetLedStatusConfig(LedStatusConfig_t *ledStatusConfig)
{
	for (int i = 0; i < LED_NUMBER; i++) {
		ledStatusConfig->ledTags[i] = IO_TAG_NONE;
	}
	
#ifdef LED3
	ledStatusConfig->ledTags[0] = IO_TAG(LED3);	// LED3 = PD13 ==> DEFIO_TAG__PD13 ==> 4D
#endif

#ifdef LED4
	ledStatusConfig->ledTags[1] = IO_TAG(LED4);	// LED4 = PD12 ==> DEFIO_TAG__PD12 ==> 4C
#endif

#ifdef LED5
	ledStatusConfig->ledTags[2] = IO_TAG(LED5);	// LED5 = PD14 ==> DEFIO_TAG__PD14 ==> 4E
#endif

#ifdef LED6
	ledStatusConfig->ledTags[3] = IO_TAG(LED6);	// LED6 = PD15 ==> DEFIO_TAG__PD15 ==> 4F
#endif
	
	ledStatusConfig->polarity = 0;
}

static void ResetDCBrushedMotorConfig(dcBrushedMotorConfig_t *dcBrushedMotorConfig)
{
	dcBrushedMotorConfig->AIN1 = IO_TAG(DC_BRUSHED_MOTOR1_AIN1);		// PB13
	dcBrushedMotorConfig->AIN2 = IO_TAG(DC_BRUSHED_MOTOR1_AIN2);		// PB12
	dcBrushedMotorConfig->BIN1 = IO_TAG(DC_BRUSHED_MOTOR2_BIN1);		// PB14
	dcBrushedMotorConfig->BIN2 = IO_TAG(DC_BRUSHED_MOTOR2_BIN2);		// PB15

	/* MAX_SUPPORTED_DC_BRUSHED_MOTORS_FOR_SBWMR = 2 */
	int motorIndex = 0;
	for (int i = 0; i < USABLE_TIMER_CHANNEL_COUNT && motorIndex < MAX_SUPPORTED_DC_BRUSHED_MOTORS_FOR_SBWMR; i++) {
		if (timerHardware[i].usageFlags & TIM_USE_MOTOR) {
			dcBrushedMotorConfig->ioTagsPWM[motorIndex] = timerHardware[i].tag;
			motorIndex++;
		}
	}
}

static void ResetPwmEncoderConfig(pwmEncoderConfig_t *pwmEncoderConfig)
{
	for (int inputIndex = 0; inputIndex < PWM_ENCODER_INPUT_PORT_COUNT; inputIndex++) {
		if (timerHardware[inputIndex].usageFlags & TIM_USE_ENCODER) {
			pwmEncoderConfig->ioTags[inputIndex] = timerHardware[inputIndex].tag;
		}
	}
}

#if 0
static void ResetUltrasoundTimerConfig(ultrasoundTimerConfig_t *ultrasoundTimerConfig)
{
	/* Initialisation for standard trigger output pins */
	ultrasoundTimerConfig->trigger1IOTag = IO_TAG(ULTRASOUND_1_TRIGGER);
	ultrasoundTimerConfig->trigger2IOTag = IO_TAG(ULTRASOUND_2_TRIGGER);

	/* Initialisation for timer pwm echo input pins
	 * PWM_ULTRASOUND_ECHO_PORT_COUNT = 2
	 */
	int ultrasoundIndex = 0;
	for (int i = 0; i < USABLE_TIMER_CHANNEL_COUNT && ultrasoundIndex < PWM_ULTRASOUND_ECHO_PORT_COUNT; i++) {
		if (timerHardware[i].usageFlags & TIM_USE_PWM) {
			ultrasoundTimerConfig->ioTags[ultrasoundIndex] = timerHardware[i].tag;		// program is stucked at this line for some reasons when using TIM1
			ultrasoundIndex++;
		}
	}
}
#endif

static void ResetUltrasoundConfig(ultrasoundConfig_t *ultrasoundConfig)
{
/* ULTRASOUND_1_TRIGGER = PC8, ULTRASOUND_1_ECHO = PC9 */
#if defined(ULTRASOUND_1_TRIGGER) && defined(ULTRASOUND_1_ECHO)
	/* Initialisation of ultrasound 1 */
	ultrasoundConfig->triggerTag[0] = IO_TAG(ULTRASOUND_1_TRIGGER);
	ultrasoundConfig->echoTag[0] = IO_TAG(ULTRASOUND_1_ECHO);
#else
#error Ultrasound1 is not defined
#endif

/* ULTRASOUND_2_TRIGGER = PC10, ULTRASOUND_2_ECHO = PC11 */
#if defined(ULTRASOUND_2_TRIGGER) && defined(ULTRASOUND_2_ECHO)	
	/* Initialisation of ultrasound 2 */
	ultrasoundConfig->triggerTag[1] = IO_TAG(ULTRASOUND_2_TRIGGER);
	ultrasoundConfig->echoTag[1] = IO_TAG(ULTRASOUND_2_ECHO);
#else
#error Ultrasound2 is not defined
#endif

/* ULTRASOUND_3_TRIGGER = PC12, ULTRASOUND_3_ECHO = PD0 */
#if defined(ULTRASOUND_3_TRIGGER) && defined(ULTRASOUND_3_ECHO)
	/* Initialisation of ultrasound 3 */
	ultrasoundConfig->triggerTag[2] = IO_TAG(ULTRASOUND_3_TRIGGER);
	ultrasoundConfig->echoTag[2] = IO_TAG(ULTRASOUND_3_ECHO);
#else
#error Ultrasound3 is not defined
#endif

/* ULTRASOUND_4_TRIGGER = PD1, ULTRASOUND_4_ECHO = PD2 */
#if defined(ULTRASOUND_4_TRIGGER) && defined(ULTRASOUND_4_ECHO)
	/* Initialisation of ultrasound 4 */
	ultrasoundConfig->triggerTag[3] = IO_TAG(ULTRASOUND_4_TRIGGER);
	ultrasoundConfig->echoTag[3] = IO_TAG(ULTRASOUND_4_ECHO);
#else
#error Ultrasound4 is not defined
#endif

/* ULTRASOUND_5_TRIGGER = PD3, ULTRASOUND_5_ECHO = PD3 */
#if defined(ULTRASOUND_5_TRIGGER) && defined(ULTRASOUND_5_ECHO)
	/* Initialisation of ultrasound 5 */
	ultrasoundConfig->triggerTag[4] = IO_TAG(ULTRASOUND_5_TRIGGER);
	ultrasoundConfig->echoTag[4] = IO_TAG(ULTRASOUND_5_ECHO);
#else
#error Ultrasound5 is not defined
#endif

/* ULTRASOUND_6_TRIGGER = PD6, ULTRASOUND_6_ECHO = PD7 */
#if defined(ULTRASOUND_6_TRIGGER) && defined(ULTRASOUND_6_ECHO)
	/* Initialisation of ultrasound 6 */
	ultrasoundConfig->triggerTag[5] = IO_TAG(ULTRASOUND_6_TRIGGER);
	ultrasoundConfig->echoTag[5] = IO_TAG(ULTRASOUND_6_ECHO);
#else
#error Ultrasound6 is not defined
#endif
}

static void ResetButtonModeSwitchConfig(button_t *buttonConfig)
{
	buttonConfig->btnPin = IO_TAG(BUTTON_MODE_SWITCH);
}

static void ResetOLEDConfig(oledConfig_t *oledConfig)
{
	oledConfig->RST = IO_TAG(OLED_RST);		// OLED_RST = PC13
	oledConfig->DC = IO_TAG(OLED_DC);		// OLED_DC = PB4
	oledConfig->SCL = IO_TAG(OLED_SCL);		// OLED_SCL = PC15
	oledConfig->SDA = IO_TAG(OLED_SDA);		// OLED_SDA = PC14	
}

static void ResetAccelerometerTrims(flightDynamicsTrims_t *accelerometerTrims)
{
	accelerometerTrims->values.roll = 0;
	accelerometerTrims->values.pitch = 0;
	accelerometerTrims->values.yaw = 0;
}

/*
 * serialPortIdentifiers[0] = USB_VCP
 * serialPortIdentifiers[1] = USART1
 * serialPortIdentifiers[2] = USART2
 * serialPortIdentifiers[3] = USART3
 * serialPortIdentifiers[4] = USART4
 * serialPortIdentifiers[5] = USART5
 * serialPortIdentifiers[6] = USART6
 * serialPortIdentifiers[7] = USART7
 * serialPortIdentifiers[8] = USART8
 * serialPortIdentifiers[9] = SOFTSERIAL1
 * serialPortIdentifiers[10] = SOFTSERIAL2
 */
void targetConfiguration(master_t *config)
{
	int index;
	
	/* USART1 for MSP */
	index = findSerialPortIndexByIdentifier(SERIAL_PORT_USART1);			// index = 0 (SERIAL_PORT_USART1)
	config->serialConfig.portConfigs[index].functionMask = FUNCTION_MSP;

	/* USART2 */
//	index = findSerialPortIndexByIdentifier(SERIAL_PORT_USART2);
//	config->serialConfig.portConfigs[index].functionMask = FUNCTION_GPS;
	
	/* USART3 */
	index = findSerialPortIndexByIdentifier(SERIAL_PORT_USART3);		// index = 1 (serialPortIdentifiers[1] contains SERIAL_PORT_USART3 which is 2)
	config->serialConfig.portConfigs[index].functionMask = FUNCTION_BLACKBOX;			// USART3 used for printf debugger JUST FOR TESTING PURPOSE NOW

	/* USART6 */
	index = findSerialPortIndexByIdentifier(SERIAL_PORT_USART6);		// index = 2 (serialPortIdentifiers[2] contains SERIAL_PORT_USART6 which is 5)
	config->serialConfig.portConfigs[index].functionMask = FUNCTION_RX_SERIAL;
//	config->serialConfig.portConfigs[index].functionMask = FUNCTION_TELEMETRY_FRSKY;	// USART6 used for FRSKY TELEMETRY
	
	/* Mode activations (HARD-CODED for now)
	 * 
	 * IMPORTANT: fc_msp functions will be implemented later to replace the hard-coded mode activation codes
	 */
	/* Setup the range of ARMING switch, which using AUX1 (L04 and !L04 on Frsky Taranis), auxChannelIndex = 0
	 *
	 * ARMING is switched ON between the range of 900 to 1150.
	 */
	config->modeActivationProfile.modeActivationConditions[0].modeId			= BOXARM;			// BOXARM = 0
	config->modeActivationProfile.modeActivationConditions[0].auxChannelIndex	= AUX1 - NON_AUX_CHANNEL_COUNT;	// AUX1 - NON_AUX_CHANNEL_COUNT = 4 - 4 = 0
	config->modeActivationProfile.modeActivationConditions[0].range.startStep	= CHANNEL_VALUE_TO_STEP(900);
	config->modeActivationProfile.modeActivationConditions[0].range.endStep		= CHANNEL_VALUE_TO_STEP(1150);	// Between 900 and 1150 is the ARMING range

	/* Setup the range of BEEPER switch, which using AUX2 (SB on Frsky Taranis), auxChannelIndex = 1
	 *
	 * BEEPER is switched ON between the range of 900 to 1150.
	 */
	config->modeActivationProfile.modeActivationConditions[1].modeId			= BOXBEEPERON;		// BOXBEEPERON = 13
	config->modeActivationProfile.modeActivationConditions[1].auxChannelIndex	= AUX2 - NON_AUX_CHANNEL_COUNT;	// AUX2 - NON_AUX_CHANNEL_COUNT = 5 - 4 = 1
	config->modeActivationProfile.modeActivationConditions[1].range.startStep	= CHANNEL_VALUE_TO_STEP(900);
	config->modeActivationProfile.modeActivationConditions[1].range.endStep		= CHANNEL_VALUE_TO_STEP(1150);	// Between 900 and 1150 is the BEEPER range
	
	/* Setup the range of FlightMode switch, which using AUX3 (SD on Frsky Taranis), auxChannelIndex = 2
	 *
	 * Channel value between 900 and 1150 is setup to ANGLE mode.
	 * Channel value between 1350 and 1650 is setup to HORIZON mode.
	 * Channel value between 1800 and 2100 is setup to ACRO(RATE) mode.
	 */
	config->modeActivationProfile.modeActivationConditions[2].modeId			= BOXANGLE;			// BOXANGLE = 1
	config->modeActivationProfile.modeActivationConditions[2].auxChannelIndex	= AUX3 - NON_AUX_CHANNEL_COUNT;	// AUX3 - NON_AUX_CHANNEL_COUNT = 6 - 4 = 2
	config->modeActivationProfile.modeActivationConditions[2].range.startStep	= CHANNEL_VALUE_TO_STEP(900);
	config->modeActivationProfile.modeActivationConditions[2].range.endStep		= CHANNEL_VALUE_TO_STEP(1150);	// Between 900 and 1150 is the ANGLEMode range
	
	config->modeActivationProfile.modeActivationConditions[3].modeId			= BOXHORIZON;		// BOXHORIZON = 2
	config->modeActivationProfile.modeActivationConditions[3].auxChannelIndex	= AUX3 - NON_AUX_CHANNEL_COUNT;	// AUX3 - NON_AUX_CHANNEL_COUNT = 6 - 4 = 2
	config->modeActivationProfile.modeActivationConditions[3].range.startStep	= CHANNEL_VALUE_TO_STEP(1350);
	config->modeActivationProfile.modeActivationConditions[3].range.endStep		= CHANNEL_VALUE_TO_STEP(1650);	// Between 1350 and 1650 is the HORIZONMode range
	
	/* Setup the range of AirMode switch, which using AUX4 (SG on Frsky Taranis), auxChannelIndex = 3
	 *
	 * AIRMode is switched ON between the range of 900 to 1150.
	 */
	config->modeActivationProfile.modeActivationConditions[4].modeId			= BOXAIRMODE;		// BOXAIRMODE = 28
	config->modeActivationProfile.modeActivationConditions[4].auxChannelIndex	= AUX4 - NON_AUX_CHANNEL_COUNT;	// AUX4 - NON_AUX_CHANNEL_COUNT = 7 - 4 = 3
	config->modeActivationProfile.modeActivationConditions[4].range.startStep	= CHANNEL_VALUE_TO_STEP(900);
	config->modeActivationProfile.modeActivationConditions[4].range.endStep		= CHANNEL_VALUE_TO_STEP(1150);
}

#ifdef BEEPER
void ResetBeeperConfig(beeperConfig_t *beeperConfig)
{
#ifdef BEEPER_INVERTED
	beeperConfig->isOpenDrain = false;
	beeperConfig->isInverted = true;
#else
	beeperConfig->isOpenDrain = false;			// use IO push-pull, with both transistors connected to the VCC and GND
//	beeperConfig->isOpenDrain = true;			// use IO open-drain, no transistor connected to VCC, only with transistor connected to the GND
	beeperConfig->isInverted = false;
#endif
	beeperConfig->ioTag = IO_TAG(BEEPER);
}
#endif

//void resetRcControlsConfig(rcControlsConfig_t *rcControlsConfig)
//{
//	rcControlsConfig->deadband = 0;
//	rcControlsConfig->yaw_deadband = 0;
//	rcControlsConfig->alt_hold_deadband = 40;
//	rcControlsConfig->alt_hold_fast_change = 1;
//}

#ifdef USE_SDCARD
void resetsdcardConfig(sdcardConfig_t *sdcardConfig)
{
#if defined(SDCARD_DMA_CHANNEL_TX)
	sdcardConfig->useDma = true;
#else
	sdcardConfig->useDma = false;
//	printf("%s, %s, %d\r\n", __FILE__, __FUNCTION__, __LINE__);
#endif
}
#endif

void createDefaultConfig(master_t *config)
{	
	/* Clear all configuration */
	memset(config, 0, sizeof(master_t));
	
	/* Feature configuration */
	uint32_t *featuresPtr = &config->enabledFeatures;
	intFeatureClearAll(featuresPtr);
	intFeatureSet(DEFAULT_RX_FEATURE, featuresPtr);

#ifdef DEFAULT_FEATURES
	intFeatureSet(DEFAULT_FEATURES, featuresPtr);		// Initialise default features including airmode right now.
#endif
	
	/* Setup the debug mode for debugging purposes */
	config->debug_mode = DEBUG_MODE;		// DEBUG_MODE default is DEBUG_NONE
	config->task_statistics = true;
	
	/* Global settings */
	config->current_profile_index = 0;		// default profile number
	config->imuConfig.dcm_kp = 2500;		// 1.0 * 10000
	config->imuConfig.dcm_ki = 0;
	config->gyroConfig.gyro_lpf = GYRO_LPF_256HZ;			// 256Hz default
#ifdef STM32F10X
	config->gyroConfig.gyro_sync_denom = 8;
	config->pidConfig.pid_process_denom = 1;
#elif defined(USE_GYRO_SPI_MPU6000) || defined(USE_GYRO_SPI_MPU6500) || defined(USE_GYRO_SPI_MPU9250)
	config->gyroConfig.gyro_sync_denom = 1;
    config->pidConfig.pid_process_denom = 2;				// for F450 normal quad
//    config->pidConfig.pid_process_denom = 4;				// for F210 racing quad
#else
	config->gyroConfig.gyro_sync_denom = 4;
	config->pidConfig.pid_process_denom = 2;
#endif
	config->gyroConfig.gyro_soft_lpf_type = FILTER_PT1;
	config->gyroConfig.gyro_soft_lpf_hz = 90;
	config->gyroConfig.gyro_soft_notch_hz_1 = 400;
	config->gyroConfig.gyro_soft_notch_cutoff_1 = 300;
	config->gyroConfig.gyro_soft_notch_hz_2 = 200;
	config->gyroConfig.gyro_soft_notch_cutoff_2 = 100;
	
//	config->gyroConfig.gyro_align = CW90_DEG;
//	config->gyroConfig.gyro_align = CW270_DEG;
	config->gyroConfig.gyro_align = ALIGN_DEFAULT;
//	config->accelerometerConfig.acc_align = CW270_DEG;
	config->accelerometerConfig.acc_align = ALIGN_DEFAULT;
	config->accelerometerConfig.acc_hardware = ACC_MPU9250;		// could use ACC_DEFAULT as well

	ResetAccelerometerTrims(&config->accelerometerConfig.accZero);
    ResetRollAndPitchTrims(&config->accelerometerConfig.accelerometerTrims);
	config->accelerometerConfig.acc_lpf_hz = 10.0f;

	/* Board alignment */
	config->boardAlignment.rollDegrees = 0;
	config->boardAlignment.pitchDegrees = 0;
	config->boardAlignment.yawDegrees = 0;
	
	/* This threshold means how much average gyro reading could differ before re-calibration is triggered */
	config->gyroConfig.gyroMovementCalibrationThreshold = 48;		// moron_threshold of CLI, range from 0 to 200
	config->accelerometerConfig.acc_hardware = ACC_DEFAULT;
//	config->rcControlsConfig.yaw_control_direction = 1;
	
	ResetSerialPinConfig(&config->serialPinConfig);
	ResetSerialConfig(&config->serialConfig);
	ResetLedStatusConfig(&config->ledStatusConfig);
//	ResetLedTimerConfig(&config->ledTimerConfig);
//	ResetMixerConfig(&config->mixerConfig);
//	ResetMotorConfig(&config->motorConfig);
	
	ResetDCBrushedMotorConfig(&config->dcBrushedMotorConfig);
	
	/* custom mixer, clear by defaults */
//	for (int i = 0; i < MAX_SUPPORTED_MOTORS; i++) {
//		config->customMotorMixer[i].throttle = 0.0f;
//	}
    
//#ifdef USE_PWM
//	ResetPwmConfig(&config->pwmConfig);
//	config->pwmConfig.inputFilteringMode = INPUT_FILTERING_DISABLED;
//#endif

	/* Initialise pwm encoder */
	ResetPwmEncoderConfig(&config->pwmEncoderConfig);
	config->pwmEncoderConfig.inputFilteringMode = INPUT_FILTERING_DISABLED;
	
	/* Initialise ultrasound */	
	ResetUltrasoundConfig(&config->ultrasoundConfig);
//	config->ultrasoundConfig.inputFilteringMode = INPUT_FILTERING_DISABLED;	
	
//	config->rxConfig.serialrx_provider = SERIALRX_SBUS;
//	config->rxConfig.halfDuplex = 0;
//	config->rxConfig.rx_spi_protocol = 0;			// TODO: 0 for now
//	config->rxConfig.sbus_inversion = 1;
//	config->rxConfig.spektrum_sat_bind = 0;
//	config->rxConfig.spektrum_sat_bind_autoreset = 1;
//	config->rxConfig.midrc = 1500;
//	config->rxConfig.mincheck = 1100;
//	config->rxConfig.maxcheck = 1900;
//	config->rxConfig.rx_min_usec = 885;				// any of first 4 channels below this value will trigger rx loss detection
//	config->rxConfig.rx_max_usec = 2115;			// any of first 4 channels above this value will trigger rx loss detection

	/* Failsafe initialisation */
//    for (int i = 0; i < MAX_SUPPORTED_RC_CHANNEL_COUNT; i++) {
//        rxFailsafeChannelConfiguration_t *channelFailsafeConfiguration = &config->rxConfig.failsafe_channel_configurations[i];
//		/* i < NON_AUX_CHANNEL_COUNT means i = 0, 1, 2, 3 which are ROLL, PITCH, THROTTLE, YAW
//		 * Related to getRxfailValue() function
//		 */
//        channelFailsafeConfiguration->mode = (i < NON_AUX_CHANNEL_COUNT) ? RX_FAILSAFE_MODE_AUTO : RX_FAILSAFE_MODE_HOLD;
//        channelFailsafeConfiguration->step = (i == THROTTLE) ? CHANNEL_VALUE_TO_RXFAIL_STEP(config->rxConfig.rx_min_usec) : CHANNEL_VALUE_TO_RXFAIL_STEP(config->rxConfig.midrc);
//	}

//	config->rxConfig.rssi_channel = 0;
//	config->rxConfig.rssi_scale = RSSI_SCALE_DEFAULT;
//	config->rxConfig.rssi_invert = 0;

//	config->rxConfig.rcInterpolation = RC_SMOOTHING_AUTO;
//	config->rxConfig.rcInterpolationChannels = 0;
//	config->rxConfig.rcInterpolationInterval = 19;
//	config->rxConfig.fpvCamAngleDegrees = 0;
//	config->rxConfig.max_aux_channel = DEFAULT_AUX_CHANNEL_COUNT;			// DEFAULT_AUX_CHANNEL_COUNT = 18 - 4 = 14
//	
//	config->rxConfig.airModeActivateThreshold = 1350;
	
//	resetAllRxChannelRangeConfigurations(config->rxConfig.channelRanges);		// set min and max values
	
//	config->armingConfig.gyro_cal_on_first_arm = 0;				// TODO: cleanup retarded arm support (allow disarm/arm on throttle down + roll left/right)
//	config->armingConfig.disarm_kill_switch = 1;				// allow disarm via AUX switch regardless of throttle value
//	config->armingConfig.auto_disarm_delay = 5;					// 5 seconds (allow automatically disarming multicopters after auto_disarm_delay seconds of zero throttle. Disabled when 0.)

	config->imuConfig.smallAngle = 25;							// setup the arming angle to 25 deg, so if the quadcopter is tilted above 25 deg, it won't arm.
	
//#ifdef RX_CHANNELS_TAER
//	parseRcChannels("TAER1234", &config->rxConfig);
//#else
//	printf("%s, %s, %d\r\n", __FILE__, __FUNCTION__, __LINE__);
//	parseRcChannels("AETR12", &config->rxConfig);		// 6 channels mapping, A: Roll, E: Pitch, T: Throttle, R: Yaw
//	parseRcChannels("AETR1234", &config->rxConfig);		// 8 channels mapping, A: Roll, E: Pitch, T: Throttle, R: Yaw
//#endif

//	config->throttleCorrectionConfig.throttleCorrectionAngle = 800;			// 80.0 deg with althold or 45.0 for FPV
//	config->throttleCorrectionConfig.throttleCorrectionValue = 0;			// 10 with althold or 40 for FPV

//#ifdef USE_SDCARD
//	intFeatureSet(FEATURE_SDCARD, featuresPtr);
//	resetsdcardConfig(&config->sdcardConfig);
//#endif

//#ifdef BLACKBOX
//#if defined(ENABLE_BLACKBOX_LOGGING_ON_SDCARD_BY_DEFAULT)
//	intFeatureSet(FEATURE_BLACKBOX, featuresPtr);
//	config->blackboxConfig.device = BLACKBOX_DEVICE_SDCARD;		// BLACKBOX using SDCARD media
//#endif
//	config->blackboxConfig.rate_num = 1;
//	config->blackboxConfig.rate_denom = 1;
//	config->blackboxConfig.on_motor_test = 0;	// default off
//#endif

#ifdef BEEPER
	ResetBeeperConfig(&config->beeperConfig);
#endif

	/* Initialise button mode switch */
	ResetButtonModeSwitchConfig(&config->buttonModeSwitchConfig);

	/* Initialise OLED */
	ResetOLEDConfig(&config->oledConfig);

//	resetProfile(&config->profile[0]);
//	
//	resetRcControlsConfig(&config->rcControlsConfig);

	/* merely to force a reset if the person inadvertently flashes the wrong target */
	strncpy(config->boardIdentifier, TARGET_BOARD_IDENTIFIER, sizeof(TARGET_BOARD_IDENTIFIER));

	targetConfiguration(config);
	
	/* copy first profile into remaining profile */
	for (int i = 1; i < MAX_PROFILE_COUNT; i++) {			// MAX_PROFILE_COUNT = 3
		memcpy(&config->profile[i], &config->profile[0], sizeof(profile_t));
	}	
}

static void resetConfig(void)
{
	createDefaultConfig(&masterConfig);
	
	setProfile(0);
}

void resetEEPROM(void)
{
	resetConfig();
}

void checkEEPROMContainsValidData(void)
{
	/* do not check the validity as the configuration does not even write into the EEPROM flash */
//	if (isEEPROMContentValid()) {
//		return;
//	}
	
	resetEEPROM();
}

void activateConfig(void)
{
//	generateThrottleCurve();
	
//	useRcControlsConfig(ModeActivationProfile()->modeActivationConditions, &masterConfig.motorConfig, &currentProfile->pidProfile);
	
	setAccelerationTrims(&AccelerometerConfig()->accZero);
	
	setAccelerationFilter(AccelerometerConfig()->acc_lpf_hz);		// acc_lpf_hz = 10 Hz
	
	/* Assign masterConfig.motorConfig, masterConfig.mixerConfig and masterConfig.rxConfig to internal motorConfig, mixerConfig and rxConfig in mixer.c */
//	mixerUseConfigs(&masterConfig.motorConfig, &masterConfig.mixerConfig, &masterConfig.rxConfig);
	
	/* IMU Configuration */
//	printf("throttleCorrectionAngle: %u\r\n", ThrottleCorrectionConfig()->throttleCorrectionAngle);
//	printf("throttleCorrectionValue: %u\r\n", ThrottleCorrectionConfig()->throttleCorrectionValue);
	imuConfigure(&masterConfig.imuConfig, &currentProfile->pidProfile, ThrottleCorrectionConfig()->throttleCorrectionAngle);
}

void validateAndFixGyroConfig(void)
{
	/* Prevent invalid gyro notch cutoff */
	if (GyroConfig()->gyro_soft_notch_cutoff_1 >= GyroConfig()->gyro_soft_notch_hz_1) {
		GyroConfig()->gyro_soft_notch_hz_1 = 0;
	}
	
	if (GyroConfig()->gyro_soft_notch_cutoff_2 >= GyroConfig()->gyro_soft_notch_hz_2) {
		GyroConfig()->gyro_soft_notch_hz_2 = 0;
	}
	
	float samplingTime = 0.000125f;			// 1 / 8000 (gyro sampling frequency 8K) = 0.000125 
	
//	printf("gyro_lpf: %u\r\n", GyroConfig()->gyro_lpf);
	
	if (GyroConfig()->gyro_lpf != GYRO_LPF_256HZ && GyroConfig()->gyro_lpf != GYRO_LPF_NONE) {
		/* When gyro set to 1Khz, always set pid looptime 1:1 to sampling time */
		PidConfig()->pid_process_denom = 1;
		GyroConfig()->gyro_sync_denom = 1;
		GyroConfig()->gyro_use_32khz = false;
		samplingTime = 0.001f;
	}
	
//	if (GyroConfig()->gyro_use_32khz) {
//		samplingTime = 0.00003125;
//	} else {
//		
//	}
	
	/* Check for looptime restrictions based on motor protocol.
	 * Motor times have safety margin
  	 */
//	printf("samplingTime: %f\r\n", samplingTime);
//	printf("gyro_sync_denom: %u\r\n", GyroConfig()->gyro_sync_denom);
//	printf("pid_process_denom: %u\r\n", PidConfig()->pid_process_denom);
	const float pidLooptime = samplingTime * GyroConfig()->gyro_sync_denom * PidConfig()->pid_process_denom;
//	printf("pidLooptime: %f\r\n", pidLooptime);
	float motorUpdateRestriction;
	
	switch (MotorConfig()->motorPwmProtocol) {
		case PWM_TYPE_STANDARD:
			motorUpdateRestriction = 1.0f / BRUSHLESS_MOTORS_PWM_RATE;
			break;
		
		default:
			motorUpdateRestriction = 0.00003125f;
	}
	
//	printf("pidLooptime: %f\r\n", pidLooptime);
//	printf("motorUpdateRestriction: %f\r\n", motorUpdateRestriction);
	
	if (pidLooptime < motorUpdateRestriction) {
		/* MAX_PID_PROCESS_DENOM = 16 */
		const uint8_t maxPidProcessDenom = constrain(motorUpdateRestriction / (samplingTime * GyroConfig()->gyro_sync_denom), 1, MAX_PID_PROCESS_DENOM);
//		printf("maxPidProcessDenom: %u\r\n", maxPidProcessDenom);
		PidConfig()->pid_process_denom = MIN(PidConfig()->pid_process_denom, maxPidProcessDenom);
	}
	
//	printf("pid_process_denom: %u\r\n", PidConfig()->pid_process_denom);
	
	/* Prevent overriding the max rate of motors 
	 * ONLY when MotorConfig()->useUnsyncedPwm = true && motorPwmProtocol <= PWM_TYPE_BRUSHED && motorPwmProtocol != PWM_TYPE_STANDARD 
	 * 
	 * TODO: Implement this when using UnsyncedPwm and motorProtocol = PWM_TYPE_ONESHOT125, PWM_TYPE_ONESHOT42 and PWM_TYPE_MULTISHOT
	 */
}

void validateAndFixConfig(void)
{
	if (!(featureConfigured(FEATURE_RX_PARALLEL_PWM) || featureConfigured(FEATURE_RX_SERIAL))) {
		featureSet(DEFAULT_RX_FEATURE);
	}
	
	if (featureConfigured(FEATURE_RX_SERIAL)) {
		featureClear(FEATURE_RX_PARALLEL_PWM);
	}
	
	if (featureConfigured(FEATURE_RX_PARALLEL_PWM)) {
		featureClear(FEATURE_RX_SERIAL);
	}
	
	/* Prevent invalid dterm notch cutoff frequency
	 *
	 * currentProfile->pidProfile.dterm_notch_cutoff = 160
	 * currentProfile->pidProfile.dterm_notch_hz = 260
	 */
	if (currentProfile->pidProfile.dterm_notch_cutoff >= currentProfile->pidProfile.dterm_notch_hz) {
		currentProfile->pidProfile.dterm_notch_hz = 0;
	}
	
	/* Validate Gyro configuration */
	validateAndFixGyroConfig();
}

void beeperOffSet(uint32_t mask)
{
	masterConfig.beeper_off_flags |= mask;
}

void beeperOffSetAll(uint8_t beeperCount)
{
	masterConfig.beeper_off_flags = (1 << beeperCount) - 1;
}

void beeperOffClear(uint32_t mask)
{
	masterConfig.beeper_off_flags &= ~(mask);
}

void beeperOffClearAll(void)
{
	masterConfig.beeper_off_flags = 0;
}

uint32_t getBeeperOffMask(void)
{
	return masterConfig.beeper_off_flags;
}

void setBeeperOffMask(uint32_t mask)
{
	masterConfig.beeper_off_flags = mask;
}

uint32_t getPreferredBeeperOffMask(void)
{
	return masterConfig.preferred_beeper_off_flags;
}

void setPreferredBeeperOffMask(uint32_t mask)
{
	masterConfig.preferred_beeper_off_flags = mask;
}
