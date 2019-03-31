
#include "system_stm32f4xx.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "misc.h"
#include "nvic.h"
#include "system.h"
#include "led.h"
#include "sound_beeper.h"
#include "config.h"
#include "configMaster.h"	// including gyro.h, acceleration.h, boardAlignment.h
#include "config_eeprom.h"
#include "exti.h"
#include "button.h"
#include "serial.h"
//#include "msp_serial.h"
//#include "printf.h"
#include "gps.h"
#include "rxSerial1Test.h"
#include "rxSerial3Test.h"
#include "bluetoothSerial6.h"
#include <stdio.h>
#include "bus_i2c.h"
#include "bitband_i2c_soft.h"					// self-implemented i2c protocol
#include "bus_spi.h"
#include "initialisation.h"
#include "accgyro_spi_mpu9250.h"
//#include "gyro.h"
//#include "acceleration.h"
//#include "boardAlignment.h"
#include "maths.h"

//#include "mpu6050.h"
//#include "mpu6050_soft_i2c.h"					// for MPU6050 testing purposes
//#include "mpu9250_soft_i2c.h"					// for MPU9250 testing purposes
//#include "inv_mpu.h"							// for MPU6050/MPU9250 DMP testing purposes
//#include "inv_mpu_dmp_motion_driver.h"			// for MPU6050/MPU9250 DMP testing purposes

#include "pwm_output.h"			// including timer.h ledTimer.h
#include "rx_pwm.h"
//#include "rx.h"
#include "feature.h"

#include "time.h"				// allow to use timeUs_t which is uint32_t
//#include "fc_core.h"
#include "fc_tasks.h"           // fcTasksInit()
#include "scheduler.h"          // cfTask_t

//#include "mixer.h"

//#include "debug.h"

//#include "sdcard.h"
//#include "asyncfatfs.h"

//#include "blackbox.h"
//#include "blackbox_io.h"

//#include "pid.h"

#include "runtime_config.h"
#include "imu.h"
#include "oled.h"
//#include "ultrasound.h"
#include "adc.h"


typedef enum {
	SYSTEM_STATE_INITIALISING			= 0,
	SYSTEM_STATE_CONFIG_LOADED			= (1 << 0),
	SYSTEM_STATE_SENSORS_READY			= (1 << 1),
	SYSTEM_STATE_MOTORS_READY			= (1 << 2),
	SYSTEM_STATE_TRANSPONDER_ENABLED	= (1 << 3),
	SYSTEM_STATE_ALL_READY				= (1 << 7)
}systemState_e;

uint8_t systemState = SYSTEM_STATE_INITIALISING;

void systemInit(void);

#if 1
struct __FILE
{
    int dummy;
};

FILE __stdout;

int fputc(int ch, FILE *f)
{
    /* Send byte to USART */
//	gpsWrite(ch);
//	rxSerial1TestWrite(ch);
//	rxSerial3TestWrite(ch);
	bluetoothSerial6Write(ch);
    
    /* If everything is OK, you have to return character written */
    return ch;
    /* If character is not correct, you can return EOF (-1) to stop writing */
    //return -1;
}
#else
#ifdef __GNUC__
	#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
	#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

PUTCHAR_PROTOTYPE
{
//	rxSerial3TestWrite(ch);
	bluetoothSerial6Write(ch);
	return ch;
}
#endif

void main_process(void)
{
    scheduler();
}

int main(void)
{
//	printfSupportInit();
	
	systemInit();
	
	/* Initialise IO (needed for all IO operations) */
	IOGlobalInit();
	
	/* Initialise EEPROM */
	initEEPROM();
	
	/* Check if EEPROM contains valid data */
	checkEEPROMContainsValidData();

	/* Initialise serial port usage list */
	serialInit(SerialConfig());

	/* Initialise debugging serial port */
//	rxSerial1TestInit();
	rxSerial3TestInit();
	
	/* Initialise bluetooth serial */
	bluetoothSerial6Init();

	/* Write masterConfig info into FLASH EEPROM
	 *
	 * TODO: (ISSUE) After calling writeEEPROM() function, the program will not be running when STM32F4 board is powered on
	 */
//	writeEEPROM();	// TODO: writeEEPROM() should be included inside the checkEEPROMContainsValidData() function
					//       separated here just for using printf (serialInit and rxSerial3TestInit initialised before writeEEPROM() and readEEPROM())
	
	/* Read masterConfig info from FLASH EEPROM */
	readEEPROM();
	
	systemState |= SYSTEM_STATE_CONFIG_LOADED;
	
//	debugMode = masterConfig.debug_mode;
	
	/* IMPORTANT: 
	 * 		DO NOT FORGET TO CALL latchActiveFeatures() function to perform the following action
	 *			activeFeaturesLatch = masterConfig.enabledFeatures
	 * Latch active features to be used for feature() in the remainder of init()
	 */
	latchActiveFeatures();
//	printf("masterConfig.enabledFeatures: 0x%x, %s, %d\r\n", masterConfig.enabledFeatures, __FUNCTION__, __LINE__);		// 0x2000 (1 << 13) FEATURE_RX_PARALLEL_PWM
	
	/* Initialise leds */
	LedInit(LedStatusConfig());
	
	/* Initialise external interrupt */
	EXTIInit();
	
//	buttonInit();			// On-board user button interrupt initialisation

	/* External button for SBWMR mode switches (Such as activating balancing and obstacle avoidance modes) */
	modeSwitchBtnPollInit(ButtonModeSwitchConfig());
	
	/* allow configuration to settle */
	delay(100);

	/* Timer must be initialised before any channel is allocated */
	timerInit();					// reinitialise the LED IO configuration to timer AF_PP if USE_LEDTIMER has been set.
									// INFO: To use NORMAL LEDs, turn off the USE_LEDTIMER micro in target.h

	/* DC brushed motor init, timer ARR = 8400  */
	dcBrushedMotorInit(DCBrushedMotorConfig());

	/* Initialise Timer Encoder Interface Mode for Incremental Encoders attached on DC Brushed Motors */
	pwmEncoderInit(PwmEncoderConfig());
	
#ifdef USE_SPI			// USE_SPI is defined in target.h
	#ifdef USE_SPI_DEVICE_1
//		printf("%s, %d\r\n", __FUNCTION__, __LINE__);
		spiInit(SPIDEV_1);		// SPIDEV_1 = 0
	#endif
	#ifdef USE_SPI_DEVICE_2
//		printf("%s, %d\r\n", __FUNCTION__, __LINE__);
		spiInit(SPIDEV_2);
	#endif
#endif

#ifdef BEEPER
	beeperInit(BeeperConfig());
#endif

#if defined(BEEPER)
	/* Board power on beeper */
	for (int i = 0; i < 10; i++) {
		delay(25);
		BEEP_ON;
		delay(25);
		BEEP_OFF;
	}
#endif

#if 0
//#if defined(USE_IMU)			// USE_IMU is defined in target.h
	if (!sensorsAutodetect(GyroConfig(), AccelerometerConfig(), UltrasoundConfig())) {
		//failureMode();
//		printf("Failed to initialise IMU!, %s, %d\r\n", __FUNCTION__, __LINE__);
		while (1) {
			/* BLUE LED */
			LED5_ON;
			delay(100);
			LED5_OFF;
			delay(100);
		}
	}
//#endif
#endif
	
#ifdef USE_ADC
	adcInit(AdcConfig());
#endif
	
	systemState |= SYSTEM_STATE_SENSORS_READY;
	
	/* OLED init */
	oledInit(OLEDConfig());
	
	/* IMU init for data fusing Euler angles (Roll, Pitch and Yaw) */
	imuInit();
	
#if defined(USE_IMU)
	/* set gyro calibration cycles */
	gyroSetCalibrationCycles();
	
	/* set accelerometer calibration cycles */
	accSetCalibrationCycles(CALIBRATING_ACC_CYCLES);
#endif

	/* Initialise Motor Current Meter */
//	motorCurrentMeterInit(MotorCurrentMeterConfig());
	
	/* Latch active features again as some of them are modified by init() */
	latchActiveFeatures();
	
    /* Initialise all the RTOS tasks */
    fcTasksInit();
	
	systemState |= SYSTEM_STATE_ALL_READY;
    
	/* Main loop */
	while (1) {

        main_process();

	}
//	return 0;
}

void EnableGPIOClk(void)
{
	/* AHB1 clocks enable */
	RCC_AHB1PeriphClockCmd(
		RCC_AHB1Periph_GPIOA 	|
		RCC_AHB1Periph_GPIOB 	|
		RCC_AHB1Periph_GPIOC 	|
		RCC_AHB1Periph_GPIOD 	|
		RCC_AHB1Periph_GPIOE 	|
		RCC_AHB1Periph_GPIOH 	|
		RCC_AHB1Periph_CRC	 	|
		RCC_AHB1Periph_FLITF 	|
		RCC_AHB1Periph_SRAM1	|
		RCC_AHB1Periph_SRAM2	|
		RCC_AHB1Periph_BKPSRAM	|
		RCC_AHB1Periph_DMA1		|
		RCC_AHB1Periph_DMA2		|
		0, ENABLE
	);
	
	/* AHB2 clocks enable */
	RCC_AHB2PeriphClockCmd(0, ENABLE);
	
	/* APB1 clocks enable */
	RCC_APB1PeriphClockCmd(
		RCC_APB1Periph_PWR		|
//		RCC_APB1Periph_I2C3		|	
//		RCC_APB1Periph_I2C2		|	
//		RCC_APB1Periph_I2C1		|	
		RCC_APB1Periph_USART2	|	
		RCC_APB1Periph_SPI3		|	
		RCC_APB1Periph_SPI2		|	
		RCC_APB1Periph_WWDG		|	
		RCC_APB1Periph_TIM5		|	
		RCC_APB1Periph_TIM4		|	
		RCC_APB1Periph_TIM3		|	
		RCC_APB1Periph_TIM2		|	
		0, ENABLE
	);
	
	/* APB2 clocks enable */
	RCC_APB2PeriphClockCmd(
		RCC_APB2Periph_ADC1		|
		RCC_APB2Periph_SPI5		|
		RCC_APB2Periph_TIM11	|
		RCC_APB2Periph_TIM10	|
		RCC_APB2Periph_TIM9		|
		RCC_APB2Periph_TIM1		|
		RCC_APB2Periph_SYSCFG	|
		RCC_APB2Periph_SPI4		|
		RCC_APB2Periph_SPI1		|
		RCC_APB2Periph_SDIO		|		
		RCC_APB2Periph_USART6	|		
		RCC_APB2Periph_USART1	|
		0, ENABLE
	);
	
	/* Initialise GPIOA */
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_PuPd		= GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Pin			= GPIO_Pin_All;
//	GPIO_InitStructure.GPIO_Pin			&= ~(GPIO_Pin_11 | GPIO_Pin_12);		// leave USB D+/D- alone
	GPIO_InitStructure.GPIO_Pin			&= ~(GPIO_Pin_13 | GPIO_Pin_14);		// leave JTAG pins alone
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* Initialise GPIOB */
	GPIO_InitStructure.GPIO_Pin			= GPIO_Pin_All;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	/* Initialise GPIOC */
	GPIO_InitStructure.GPIO_Pin			= GPIO_Pin_All;
	GPIO_Init(GPIOC, &GPIO_InitStructure);			// Board ID, AUDIO MCLK, SCLK, SDIN, OTG_FS_PowerSwitchOn
	GPIO_Init(GPIOD, &GPIO_InitStructure);			// 4 USER LEDS, AUDIO, OTG_FS
	GPIO_Init(GPIOE, &GPIO_InitStructure);
	GPIO_Init(GPIOH, &GPIO_InitStructure);
}

void systemInit(void)
{
	/* Configure the system clock */
	SetSysClock();

	/* Configure NVIC preempt/priority groups */
	NVIC_PriorityGroupConfig(NVIC_PRIORITY_GROUPING);	// SCB->AIRCR, [10:8] = 101, 2 bits of preemption priority, 2 bits of subpriority (response priority)

	/* Clear the reset flags */
	RCC_ClearFlag();		// RCC->CSR |= RCC_CSR_RMVF;
	
	/* Enable AHB, APB1, APB2 Peripheral clocks and configure GPIOx, where x = A, B, C, D, E, H */
	EnableGPIOClk();
	
	/* Initialise SysTick counter */
	SysTick_Init();
	
	/* Configure SysTick in milliseconds time base */
	SysTick_Config(SystemCoreClock / 1000);
}
