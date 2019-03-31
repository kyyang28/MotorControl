
#include <stdio.h>
#include <math.h>
#include "fc_tasks.h"
#include "scheduler.h"
#include "fc_core.h"
#include "common.h"
#include "rx.h"
#include "led.h"
#include "imu.h"		// including time.h
#include "gyro.h"
#include "acceleration.h"
#include "configMaster.h"
#include "runtime_config.h"
#include "led.h"
#include "pwm_output.h"
#include "maths.h"
#include "button.h"
#include "system.h"
#include "rx_pwm.h"
#include "oled.h"
#include "ultrasound.h"
#include "vnh5019CurrentSensing.h"

//#define TASKS_LEDS_TESTING

/* taskMotorEncoder is updated every 10 ms, inactivity is triggered every 10ms * 1000 = 10000 ms = 10 secs */
#define INACTIVITY_CONDITION			1000

#define TASK_PERIOD_HZ(hz)              (1000000 / (hz))            // units in microseconds (us)
#define TASK_PERIOD_MS(ms)              ((ms) * 1000)
#define TASK_PERIOD_US(us)              (us)

#define CURRENT_INTERVAL				(6 * 3500)				// in microseconds

int LeftEncoder, RightEncoder;
int stabilisePwmVal, velocityPwmVal, yawPwmVal;
int leftMotorPwm, rightMotorPwm;
int yawMagnitude = 95;
//int yawMagnitude = 45;
uint32_t stationaryFlag = 0;
float velocityUpdatedMovement = 0.0f;
int balanceSetpoint = 0;
//int balanceSetpoint = 0;
float velocitySetpoint = 0.0f;
//float yawSetpoint = 0.0f;

bool isMotorActivated = true;

extern uint8_t driveForward, driveReverse, turnLeft, turnRight;

bool stopFlag = true;
bool isCollisionAvoidanceModeActivated = false;			// collision avoidance is switched off by default

static int32_t ultrasound1DistanceData = ULTRASOUND_OUT_OF_RANGE;
static int32_t ultrasound2DistanceData = ULTRASOUND_OUT_OF_RANGE;
static int32_t ultrasound3DistanceData = ULTRASOUND_OUT_OF_RANGE;
static int32_t ultrasound4DistanceData = ULTRASOUND_OUT_OF_RANGE;
static int32_t ultrasound5DistanceData = ULTRASOUND_OUT_OF_RANGE;
static int32_t ultrasound6DistanceData = ULTRASOUND_OUT_OF_RANGE;

//static void taskUpdateRxMain(timeUs_t currentTimeUs);
static void taskUpdateAccelerometer(timeUs_t currentTimeUs);
static void taskMotorEncoder(timeUs_t currentTimeUs);
static void taskUpdateGyro(timeUs_t currentTimeUs);
//static void taskOLEDDisplay(timeUs_t currentTimeUs);
//static void taskUltrasound1ReadData(timeUs_t currentTimeUs);
//static void taskUltrasound2ReadData(timeUs_t currentTimeUs);
//static void taskUltrasound3ReadData(timeUs_t currentTimeUs);
//static void taskUltrasound4ReadData(timeUs_t currentTimeUs);
//static void taskUltrasound5ReadData(timeUs_t currentTimeUs);
//static void taskUltrasound6ReadData(timeUs_t currentTimeUs);
static void taskLeftMotorCurrentMeter(timeUs_t currentTimeUs);
static void taskRightMotorCurrentMeter(timeUs_t currentTimeUs);
//static void taskBluetoothReceive(timeUs_t currentTimeUs);

/* Tasks initialisation */
cfTask_t cfTasks[TASK_COUNT] = {
    [TASK_SYSTEM] = {
        .taskName = "SYSTEM",
        .taskFunc = taskSystem,
        .desiredPeriod = TASK_PERIOD_HZ(10),            	// 1000000 / 10 = 100000 us = 100 ms
        .staticPriority = TASK_PRIORITY_MEDIUM_HIGH,		// TASK_PRIORITY_MEDIUM_HIGH = 4
    },
    
	/* desiredPeriod = 4000 us = 4 ms = 250 Hz for F450 quad */
    [TASK_GYRO] = {
        .taskName = "GYRO",
        .taskFunc = taskUpdateGyro,
        .desiredPeriod = TASK_GYROPID_DESIRED_PERIOD,       // desiredPeriod = TASK_GYROPID_DESIRED_PERIOD = 125 us using STM32F4
        .staticPriority = TASK_PRIORITY_HIGH,           	// TASK_PRIORITY_HIGH = 5
    },
	
	[TASK_ACCEL] = {
		.taskName = "ACCEL",
		.taskFunc = taskUpdateAccelerometer,
		.desiredPeriod = TASK_PERIOD_HZ(1000),				// 1000000 / 1000 = 1000 us, every 1ms
		.staticPriority = TASK_PRIORITY_HIGH,				// TASK_PRIORITY_HIGH = 5
	},
	
	[TASK_ATTITUDE] = {
		.taskName = "ATTITUDE",
		.taskFunc = taskIMUUpdateAttitude,
		.desiredPeriod = TASK_PERIOD_HZ(100),				// 1000000 / 100 = 10000 us = 10 ms
		.staticPriority = TASK_PRIORITY_MEDIUM,				// TASK_PRIORITY_MEDIUM = 3
	},

    [TASK_MOTORENCODER] = {
        .taskName = "MOTOR_ENCODER",
        .taskFunc = taskMotorEncoder,
        .desiredPeriod = TASK_PERIOD_HZ(100),            	// 1000000 / 100 = 10000 us = 10 ms
        .staticPriority = TASK_PRIORITY_REALTIME,			// TASK_PRIORITY_REALTIME = 6
    },
	
//	[TASK_OLEDDISPLAY] = {
//		.taskName = "OLED_DISPLAY",
//		.taskFunc = taskOLEDDisplay,
//		.desiredPeriod = TASK_PERIOD_HZ(20),				// 1000000 / 20 = 50000 us = 50 ms
//		.staticPriority = TASK_PRIORITY_MEDIUM,				// TASK_PRIORITY_MEDIUM = 3
//	},

#if 0
#if defined(ULTRASOUND)	
	[TASK_ULTRASOUND1_UPDATE] = {
		.taskName = "ULTRASOUND1_UPDATE",
		.taskFunc = ultrasound1Update,
		.desiredPeriod = TASK_PERIOD_MS(70),				// 70 ms * 1000 = 70000 us, 70 ms required so that ultrasound pulses do not interference with each other
		.staticPriority = TASK_PRIORITY_MEDIUM,				// TASK_PRIORITY_MEDIUM = 1
	},
	
	[TASK_ULTRASOUND1_READDATA] = {
		.taskName = "ULTRASOUND1_READDATA",
		.taskFunc = taskUltrasound1ReadData,
		.desiredPeriod = TASK_PERIOD_HZ(40),				// 1000000 / 40 = 25000 us = 25 ms = 40 Hz from HCSR-04 datasheet
		.staticPriority = TASK_PRIORITY_MEDIUM,				// TASK_PRIORITY_MEDIUM = 1
	},
	[TASK_ULTRASOUND2_UPDATE] = {
		.taskName = "ULTRASOUND2_UPDATE",
		.taskFunc = ultrasound2Update,
		.desiredPeriod = TASK_PERIOD_MS(70),				// 70 ms * 1000 = 70000 us, 70 ms required so that ultrasound pulses do not interference with each other
		.staticPriority = TASK_PRIORITY_MEDIUM,				// TASK_PRIORITY_MEDIUM = 1
	},
	
	[TASK_ULTRASOUND2_READDATA] = {
		.taskName = "ULTRASOUND2_READDATA",
		.taskFunc = taskUltrasound2ReadData,
		.desiredPeriod = TASK_PERIOD_HZ(40),				// 1000000 / 40 = 25000 us = 25 ms = 40 Hz from HCSR-04 datasheet
		.staticPriority = TASK_PRIORITY_MEDIUM,				// TASK_PRIORITY_MEDIUM = 1
	},
	
	[TASK_ULTRASOUND3_UPDATE] = {
		.taskName = "ULTRASOUND3_UPDATE",
		.taskFunc = ultrasound3Update,
		.desiredPeriod = TASK_PERIOD_MS(70),				// 70 ms * 1000 = 70000 us, 70 ms required so that ultrasound pulses do not interference with each other
		.staticPriority = TASK_PRIORITY_MEDIUM,				// TASK_PRIORITY_MEDIUM = 1
	},
	[TASK_ULTRASOUND3_READDATA] = {
		.taskName = "ULTRASOUND3_READDATA",
		.taskFunc = taskUltrasound3ReadData,
		.desiredPeriod = TASK_PERIOD_HZ(40),				// 1000000 / 40 = 25000 us = 25 ms = 40 Hz from HCSR-04 datasheet
		.staticPriority = TASK_PRIORITY_MEDIUM,				// TASK_PRIORITY_MEDIUM = 1
	},

	[TASK_ULTRASOUND4_UPDATE] = {
		.taskName = "ULTRASOUND4_UPDATE",
		.taskFunc = ultrasound4Update,
		.desiredPeriod = TASK_PERIOD_MS(70),				// 70 ms * 1000 = 70000 us, 70 ms required so that ultrasound pulses do not interference with each other
		.staticPriority = TASK_PRIORITY_MEDIUM,				// TASK_PRIORITY_MEDIUM = 1
	},
	[TASK_ULTRASOUND4_READDATA] = {
		.taskName = "ULTRASOUND4_READDATA",
		.taskFunc = taskUltrasound4ReadData,
		.desiredPeriod = TASK_PERIOD_HZ(40),				// 1000000 / 40 = 25000 us = 25 ms = 40 Hz from HCSR-04 datasheet
		.staticPriority = TASK_PRIORITY_MEDIUM,				// TASK_PRIORITY_MEDIUM = 1
	},
	
	[TASK_ULTRASOUND5_UPDATE] = {
		.taskName = "ULTRASOUND5_UPDATE",
		.taskFunc = ultrasound5Update,
		.desiredPeriod = TASK_PERIOD_MS(70),				// 70 ms * 1000 = 70000 us, 70 ms required so that ultrasound pulses do not interference with each other
		.staticPriority = TASK_PRIORITY_MEDIUM,				// TASK_PRIORITY_MEDIUM = 1
	},
	[TASK_ULTRASOUND5_READDATA] = {
		.taskName = "ULTRASOUND5_READDATA",
		.taskFunc = taskUltrasound5ReadData,
		.desiredPeriod = TASK_PERIOD_HZ(40),				// 1000000 / 40 = 25000 us = 25 ms = 40 Hz from HCSR-04 datasheet
		.staticPriority = TASK_PRIORITY_MEDIUM,				// TASK_PRIORITY_MEDIUM = 1
	},

	[TASK_ULTRASOUND6_UPDATE] = {
		.taskName = "ULTRASOUND6_UPDATE",
		.taskFunc = ultrasound6Update,
		.desiredPeriod = TASK_PERIOD_MS(70),				// 70 ms * 1000 = 70000 us, 70 ms required so that ultrasound pulses do not interference with each other
		.staticPriority = TASK_PRIORITY_MEDIUM,				// TASK_PRIORITY_MEDIUM = 1
	},
	[TASK_ULTRASOUND6_READDATA] = {
		.taskName = "ULTRASOUND6_READDATA",
		.taskFunc = taskUltrasound6ReadData,
		.desiredPeriod = TASK_PERIOD_HZ(40),				// 1000000 / 40 = 25000 us = 25 ms = 40 Hz from HCSR-04 datasheet
		.staticPriority = TASK_PRIORITY_MEDIUM,				// TASK_PRIORITY_MEDIUM = 1
	},
#endif
#endif
	
#if defined(USE_ADC)
	[TASK_LEFT_MOTOR_CURRENT_METER] = {
		.taskName = "LEFT_MOTOR_CURRENT_METER",
		.taskFunc = taskLeftMotorCurrentMeter,
		.desiredPeriod = TASK_PERIOD_HZ(50),				// 50 Hz = 1000000 / 50 = 20000 us = 20 ms
		.staticPriority = TASK_PRIORITY_MEDIUM,
	},
	
	[TASK_RIGHT_MOTOR_CURRENT_METER] = {
		.taskName = "RIGHT_MOTOR_CURRENT_METER",
		.taskFunc = taskRightMotorCurrentMeter,
		.desiredPeriod = TASK_PERIOD_HZ(50),				// 50 Hz = 1000000 / 50 = 20000 us = 20 ms
		.staticPriority = TASK_PRIORITY_MEDIUM,
	},
#endif
};

static void taskUpdateGyro(timeUs_t currentTimeUs)
{
	/* gyroUpdate */
	gyroUpdate(currentTimeUs);
}

static void taskUpdateAccelerometer(timeUs_t currentTimeUs)
{
//	UNUSED(currentTimeUs);
	
	accUpdate(currentTimeUs, &AccelerometerConfig()->accelerometerTrims);
}

/* Encoder speed values */
int Read_Encoder(uint8_t TIMX)
{
    int Encoder_TIM;
	switch(TIMX)
	{
		case 2:
			Encoder_TIM = (short)TIM2->CNT;
			TIM2->CNT = 0;						// Clear CNT to zero to retrieve the speed of the motor
			break;
		
		case 3:
			Encoder_TIM = (short)TIM3->CNT;
			TIM3->CNT = 0;
			break;
		
		case 4:
			Encoder_TIM = (short)TIM4->CNT;
			TIM4->CNT = 0;
			break;
		
		default:
			Encoder_TIM = 0;
	}
	
//	printf("Encoder_TIM: %d\r\n", Encoder_TIM);
	
	return Encoder_TIM;
}

void limitMotorPwm(int *motor1, int *motor2)
{
	/* The maximum PWM value is 7200 setup by dcBrushedMotorInit(), 6900 for the upper bound */
	int Amplitude = 6900;
	
    if (*motor1 < -Amplitude)
		*motor1 = -Amplitude;	
	
	if (*motor1 > Amplitude)
		*motor1 = Amplitude;

    if (*motor2 < -Amplitude)
		*motor2 = -Amplitude;	
	
	if (*motor2 > Amplitude)
		*motor2 = Amplitude;

//	printf("motor[LimitPwm]: %d\r\n", motor);
}

/**************************************************************************
Description: incremental PI controller
Parameters: Encoder value, target speed
Returns: Motor updated PWM value

Incremental discrete PID equation
pwm += Kp[e(k) - e(k-1)] + Ki * e(k) + Kd * [e(k) - 2e(k-1) + e(k-2)]
e(k) represents the current error
e(k-1) represents the previous error
pwm is the updated incremental PID value.

For speed control, the PI-only controller is utilised
pwm += Kp[e(k) - e(k-1)] + Ki * e(k)
**************************************************************************/
int Incremental_PIController(int Encoder, int Target)
{
	float Kp = 80, Ki = 1.2;
//	float updatedPWM;
	static int error, updatedPWM, prev_error;
//	static int error, prev_error;
//	printf("Encoder: %d\r\n", Encoder);
	error = Encoder - Target;                					// Calculate the error
//	printf("Bias: %d\r\n", Bias);
	updatedPWM += Kp * (error - prev_error) + Ki * error;   	// Incremental PI Controller
//	printf("Pwm: %d\r\n", Pwm);
	prev_error = error;	                   						// Store previous bias
	return updatedPWM;                         					// Return PID PWM value
}

//#define DC_BRUSHED_MOTOR1_AIN1	PB13
//#define DC_BRUSHED_MOTOR1_AIN2	PB12
//#define DC_BRUSHED_MOTOR2_BIN1	PB14
//#define DC_BRUSHED_MOTOR2_BIN2	PB15

void updateMotorPwm(int *motorPwm1, int *motorPwm2)
{
//	volatile uint32_t *motor2PwmAddr = (volatile uint32_t *)((volatile char *)&TIM1->CCR1 + 0x4);
	IO_t l_AIN1 = IOGetByTag(DCBrushedMotorConfig()->AIN1);		// PB13
	IO_t l_AIN2 = IOGetByTag(DCBrushedMotorConfig()->AIN2);		// PB12
	IO_t l_BIN1 = IOGetByTag(DCBrushedMotorConfig()->BIN1);		// PB14
	IO_t l_BIN2 = IOGetByTag(DCBrushedMotorConfig()->BIN2);		// PB15

//	printf("motor pwm: %d\r\n", motor);
	if (*motorPwm1 > 0) {
//		printf("*motorPwm1: %d, %d\r\n", *motorPwm1, __LINE__);
//		IOWrite(DCBrushedMotorConfig()->AIN1, true);			// clear AIN1 to LOW
//		IOWrite(DCBrushedMotorConfig()->AIN2, false);			// set AIN2 to HIGH
//		GPIOB->BSRR |= 1<<29;				// set AIN1 to LOW
//		GPIOB->BSRR |= 1<<12;				// set AIN2 to HIGH
#if 1
		IOHi(l_AIN2);
		IOLo(l_AIN1);
#else
		IOHi(l_AIN1);
		IOLo(l_AIN2);
#endif
	} else {
//		printf("*motorPwm1: %d, %d\r\n", *motorPwm1, __LINE__);
//		GPIOB->BSRR |= 1<<13;				// set AIN1 to HIGH
//		GPIOB->BSRR |= 1<<28;				// set AIN2 to LOW
//		IOWrite(DCBrushedMotorConfig()->AIN1, false);			// set AIN1 to HIGH
//		IOWrite(DCBrushedMotorConfig()->AIN2, true);			// clear AIN2 to LOW
#if 1
		IOHi(l_AIN1);
		IOLo(l_AIN2);
#else
		IOHi(l_AIN2);
		IOLo(l_AIN1);
#endif
	}
	
	pwmWriteDcBrushedMotor(0, ABS(*motorPwm1));	// 0 represents motor 1, write motor pwm value to motor 1 (PWMA)
	
	if (*motorPwm2 > 0) {
//		IOLo(DCBrushedMotorConfig()->BIN2);
//		IOHi(DCBrushedMotorConfig()->BIN1);
//		printf("*motorPwm2: %d, %d\r\n", *motorPwm2, __LINE__);
#if 0
		IOHi(l_BIN2);
		IOLo(l_BIN1);
#else
		IOHi(l_BIN1);
		IOLo(l_BIN2);
#endif		
	} else {
//		printf("*motorPwm2: %d, %d\r\n", *motorPwm2, __LINE__);
//		IOLo(DCBrushedMotorConfig()->BIN1);
//		IOHi(DCBrushedMotorConfig()->BIN2);
#if 0
		IOHi(l_BIN1);
		IOLo(l_BIN2);
#else
		IOHi(l_BIN2);
		IOLo(l_BIN1);
#endif		
	}
		
	pwmWriteDcBrushedMotor(1, ABS(*motorPwm2));	// 1 represents motor 2, write motor pwm value to motor 12 (PWMB)
}

int LeftMotorTargetVelocity = 10;
int RightMotorTargetVelocity = 10;
//int PWMMotor1 = -1000;                 // Motor PWM value
//int PWMMotor2 = -1000;                 // Motor PWM value
//int leftMotorPwm = -200;
//int rightMotorPwm = -200;

static void taskMotorEncoder(timeUs_t currentTimeUs)
{	
//	printf("currentTimeUs: %u\r\n", currentTimeUs);
	
	LeftEncoder = Read_Encoder(2);		// 2: TIM2, left encoder
	RightEncoder = Read_Encoder(4);		// 4: TIM4, right encoder
	
//	printf("%d, %d\r\n", LeftEncoder, RightEncoder);
	
	LED6_ON;
	
	/* gyroUpdate */
//	gyroUpdate(currentTimeUs);			// gyro updated in taskUpdateGyro function

#if 0
    if (gyro.dev.mpuDetectionResult.sensor == MPU_9250_SPI && gyro.dev.calibrationFlag) {
//        printf("%u,%.4f,%.4f,%.4f\r\n", currentTimeUs, gyro.gyroADCf[X], gyro.gyroADCf[Y], gyro.gyroADCf[Z]);
//        printf("%.4f\t%.4f\t%.4f\t%d\r\n", gyro.gyroADCf[X], gyro.gyroADCf[Y], gyro.gyroADCf[Z], attitude.raw[Y]);
//        printf("%.4f\t%.4f\t%d\r\n", gyro.gyroADCf[Y], gyro.gyroADCf[Z], attitude.raw[Y]);
//        printf("%d\r\n", attitude.raw[Y]);
//        printf("%d\r\n", acc.accSmooth[Z]);
    }
#endif
		
////	if ((gyro.dev.mpuDetectionResult.sensor == MPU_9250_SPI) && gyro.dev.calibrationFlag) {
		
//        printf("%.4f\t%.4f\t%d\r\n", gyro.gyroADCf[Y], gyro.gyroADCf[Z], attitude.raw[Y]);
				
		leftMotorPwm = Incremental_PIController(LeftEncoder, LeftMotorTargetVelocity);
		rightMotorPwm = Incremental_PIController(RightEncoder, RightMotorTargetVelocity);

//		leftMotorPwm = -1680;			// 1680 / 8400 = 0.2 (20% duty cycle)
//		rightMotorPwm = 1680;
//		leftMotorPwm = -200;
//		rightMotorPwm = 200;

		/* Motor PWM boundary limitation */
		limitMotorPwm(&leftMotorPwm, &rightMotorPwm);
		
		updateMotorPwm(&leftMotorPwm, &rightMotorPwm);
//	}

	printf("%u,%d,%d,%d,%d,%d,%d\r\n", currentTimeUs, LeftEncoder, ABS(leftMotorPwm), (int32_t)round(filteredLeftMotorCurrentMeterValue), 
					RightEncoder, ABS(rightMotorPwm), (int32_t)round(filteredLeftMotorCurrentMeterValue));
		
//	printf("(LeftEncoder, LeftMotor_PWM) = (%d, %d)\r\n", LeftEncoder, leftMotorPwm);
//	printf("(RightEncoder, RightMotor_PWM) = (%d, %d)\r\n", RightEncoder, rightMotorPwm);
}

static void taskLeftMotorCurrentMeter(timeUs_t currentTimeUs)
{
	if (feature(FEATURE_CURRENT_METER)) {
		static uint32_t motorCurrentLastRecorded = 0;
		const int32_t motorCurrentSinceLastRecorded = cmp32(currentTimeUs, motorCurrentLastRecorded);
//		printf("motorCurrentSinceLastRecorded: %d\r\n", motorCurrentSinceLastRecorded);
		
		/* taskMotorCurrentMeter period is 20 ms, CURRENT_INTERVAL = 21 ms, motor current sensing process is updated every 40 ms (20 ms * 2) */
		if (motorCurrentSinceLastRecorded >= CURRENT_INTERVAL) {
			motorCurrentLastRecorded = currentTimeUs;
//			printf("curr: %d, interval: %u\r\n", motorCurrentSinceLastRecorded, CURRENT_INTERVAL);
			updateVNH5019LeftMotorCurrentSensor(motorCurrentSinceLastRecorded);
		}
	}
}

static void taskRightMotorCurrentMeter(timeUs_t currentTimeUs)
{
	if (feature(FEATURE_CURRENT_METER)) {
		static uint32_t motorCurrentLastRecorded = 0;
		const int32_t motorCurrentSinceLastRecorded = cmp32(currentTimeUs, motorCurrentLastRecorded);
//		printf("motorCurrentSinceLastRecorded: %d\r\n", motorCurrentSinceLastRecorded);
		
		/* taskMotorCurrentMeter period is 20 ms, CURRENT_INTERVAL = 21 ms, motor current sensing process is updated every 40 ms (20 ms * 2) */
		if (motorCurrentSinceLastRecorded >= CURRENT_INTERVAL) {
			motorCurrentLastRecorded = currentTimeUs;
//			printf("curr: %d, interval: %u\r\n", motorCurrentSinceLastRecorded, CURRENT_INTERVAL);
			updateVNH5019RightMotorCurrentSensor(motorCurrentSinceLastRecorded);
		}
	}
}

void fcTasksInit(void)
{
    /* Clear RTOS queue and enable SYSTEM TASK */
    schedulerInit();

	/* Enable GYRO Task */
//    setTaskEnabled(TASK_GYRO, true);
	
	/* Enable ACCELEROMETER TASK */
//	if (sensors(SENSOR_ACC)) {
//		setTaskEnabled(TASK_ACCEL, true);
//		rescheduleTask(TASK_ACCEL, acc.accSamplingInterval);
//	}
	
	/* Enable MOTOR ENCODER TESTING Task */
	setTaskEnabled(TASK_MOTORENCODER, true);
		
	/* Enable ATTITUDE TASK (Data fusion of Euler angles (Roll, Pitch, and Yaw angles)) */
//	setTaskEnabled(TASK_ATTITUDE, sensors(SENSOR_ACC));

	setTaskEnabled(TASK_LEFT_MOTOR_CURRENT_METER, true);
	setTaskEnabled(TASK_RIGHT_MOTOR_CURRENT_METER, true);
}
