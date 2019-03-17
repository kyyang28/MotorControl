
#include <stdio.h>
#include "exti.h"
#include "nvic.h"
#include "system.h"
#include "target.h"
#include "ultrasound_hcsr04.h"

#ifdef ULTRASOUND

/*
 * 1. Using IO trigger for at least 10 us high level signal
 * 2. Tiggering duration is suggested to be greater than 60 ms in order to prevent the interference between trigger signal and the echo signal
 * 3. Ultrasound frequency of reading process is 40 Hz, which is 25 ms.
 */

static IO_t ultrasound1_triggerIOPin;
static IO_t ultrasound1_echoIOPin;
static IO_t ultrasound2_triggerIOPin;
static IO_t ultrasound2_echoIOPin;
static IO_t ultrasound3_triggerIOPin;
static IO_t ultrasound3_echoIOPin;
static IO_t ultrasound4_triggerIOPin;
static IO_t ultrasound4_echoIOPin;
static IO_t ultrasound5_triggerIOPin;
static IO_t ultrasound5_echoIOPin;
static IO_t ultrasound6_triggerIOPin;
static IO_t ultrasound6_echoIOPin;

static extiCallbackRec_t hcsr04_ultrasound1_extiCallbackRec;
static extiCallbackRec_t hcsr04_ultrasound2_extiCallbackRec;
static extiCallbackRec_t hcsr04_ultrasound3_extiCallbackRec;
static extiCallbackRec_t hcsr04_ultrasound4_extiCallbackRec;
static extiCallbackRec_t hcsr04_ultrasound5_extiCallbackRec;
static extiCallbackRec_t hcsr04_ultrasound6_extiCallbackRec;

static uint32_t ultrasound1_lastMeasurementAt;
static uint32_t ultrasound2_lastMeasurementAt;
static uint32_t ultrasound3_lastMeasurementAt;
static uint32_t ultrasound4_lastMeasurementAt;
static uint32_t ultrasound5_lastMeasurementAt;
static uint32_t ultrasound6_lastMeasurementAt;

volatile int32_t ultrasound1_measurement = -1;
volatile int32_t ultrasound2_measurement = -1;
volatile int32_t ultrasound3_measurement = -1;
volatile int32_t ultrasound4_measurement = -1;
volatile int32_t ultrasound5_measurement = -1;
volatile int32_t ultrasound6_measurement = -1;

void hcsr04_extiHandler_ultrasound1(extiCallbackRec_t *cb)
{
	static uint32_t start_timing;
	uint32_t stop_timing;
	UNUSED(cb);
	
	if (IORead(ultrasound1_echoIOPin) != 0) {
		/* High level signal */
		start_timing = micros();
//		printf("start_timing: %u\r\n", start_timing);
	} else {
		/* Low level signal */
		stop_timing = micros();
//		printf("stop_timing: %u\r\n", stop_timing);
		
		if (stop_timing > start_timing) {
			ultrasound1_measurement = stop_timing - start_timing;
//			printf("measurement: %d\r\n", ultrasound1_measurement);
		}
	}
}

void hcsr04_extiHandler_ultrasound2(extiCallbackRec_t *cb)
{
	static uint32_t start_timing;
	uint32_t stop_timing;
	UNUSED(cb);
	
	if (IORead(ultrasound2_echoIOPin) != 0) {
		/* High level signal */
		start_timing = micros();
//		printf("start_timing: %u\r\n", start_timing);
	} else {
		/* Low level signal */
		stop_timing = micros();
//		printf("stop_timing: %u\r\n", stop_timing);
		
		if (stop_timing > start_timing) {
			ultrasound2_measurement = stop_timing - start_timing;
//			printf("measurement: %d\r\n", ultrasound2_measurement);
		}
	}
}

void hcsr04_extiHandler_ultrasound3(extiCallbackRec_t *cb)
{
	static uint32_t start_timing;
	uint32_t stop_timing;
	UNUSED(cb);
	
	if (IORead(ultrasound3_echoIOPin) != 0) {
		/* High level signal */
		start_timing = micros();
//		printf("start_timing: %u\r\n", start_timing);
	} else {
		/* Low level signal */
		stop_timing = micros();
//		printf("stop_timing: %u\r\n", stop_timing);
		
		if (stop_timing > start_timing) {
			ultrasound3_measurement = stop_timing - start_timing;
//			printf("measurement: %d\r\n", ultrasound3_measurement);
		}
	}
}

void hcsr04_extiHandler_ultrasound4(extiCallbackRec_t *cb)
{
	static uint32_t start_timing;
	uint32_t stop_timing;
	UNUSED(cb);
	
	if (IORead(ultrasound4_echoIOPin) != 0) {
		/* High level signal */
		start_timing = micros();
//		printf("start_timing: %u\r\n", start_timing);
	} else {
		/* Low level signal */
		stop_timing = micros();
//		printf("stop_timing: %u\r\n", stop_timing);
		
		if (stop_timing > start_timing) {
			ultrasound4_measurement = stop_timing - start_timing;
//			printf("measurement: %d\r\n", ultrasound4_measurement);
		}
	}
}

void hcsr04_extiHandler_ultrasound5(extiCallbackRec_t *cb)
{
	static uint32_t start_timing;
	uint32_t stop_timing;
	UNUSED(cb);
	
	if (IORead(ultrasound5_echoIOPin) != 0) {
		/* High level signal */
		start_timing = micros();
//		printf("start_timing: %u\r\n", start_timing);
	} else {
		/* Low level signal */
		stop_timing = micros();
//		printf("stop_timing: %u\r\n", stop_timing);
		
		if (stop_timing > start_timing) {
			ultrasound5_measurement = stop_timing - start_timing;
//			printf("measurement: %d\r\n", ultrasound5_measurement);
		}
	}
}

void hcsr04_extiHandler_ultrasound6(extiCallbackRec_t *cb)
{
	static uint32_t start_timing;
	uint32_t stop_timing;
	UNUSED(cb);
	
	if (IORead(ultrasound6_echoIOPin) != 0) {
		/* High level signal */
		start_timing = micros();
//		printf("start_timing: %u\r\n", start_timing);
	} else {
		/* Low level signal */
		stop_timing = micros();
//		printf("stop_timing: %u\r\n", stop_timing);
		
		if (stop_timing > start_timing) {
			ultrasound6_measurement = stop_timing - start_timing;
//			printf("measurement: %d\r\n", ultrasound5_measurement);
		}
	}
}

void hcsr04_init(const ultrasoundConfig_t *ultrasoundConfig, ultrasoundRange_t *ultrasoundRange)
{
//	printf("%s, %d\r\n", __FUNCTION__, __LINE__);
	ultrasoundRange->maxRangeCm = HCSR04_MAX_RANGE_CM;										// HCSR04_MAX_RANGE_CM = 400 cm = 4 m
	ultrasoundRange->detectionConeDeciDegrees = HCSR04_DETECTION_CONE_DECIDEGREES;			// HCSR04_DETECTION_CONE_DECIDEGREES = 300
	ultrasoundRange->detectionConeExtendedDeciDegrees = HCSR04_DETECTION_CONE_EXTENDED_DECIDEGREES;	// HCSR04_DETECTION_CONE_EXTENDED_DECIDEGREES = 450
	
#if defined(STM32F4)
	/* Enable SYSCFG clock, otherwise the EXTI irq handlers are not called */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
#endif
	
	/* +------------------------------------------------------------------------------------------------------------------+ */
	/* +---------------------------------------- Ultrasound 1 configuration ----------------------------------------------+ */
	/* +------------------------------------------------------------------------------------------------------------------+ */
	/* Configure trigger pin of ultrasound sensor 1 */
	ultrasound1_triggerIOPin = IOGetByTag(ultrasoundConfig->triggerTag[0]);
	IOInit(ultrasound1_triggerIOPin, OWNER_ULTRASOUND_TRIGGER, 0);
	IOConfigGPIO(ultrasound1_triggerIOPin, IOCFG_OUT_PP);
	
	/* Configure echo pin of ultrasound sensor 1 */
	ultrasound1_echoIOPin = IOGetByTag(ultrasoundConfig->echoTag[0]);
	IOInit(ultrasound1_echoIOPin, OWNER_ULTRASOUND_ECHO, 0);
	IOConfigGPIO(ultrasound1_echoIOPin, IOCFG_IN_FLOATING);
	
#ifdef USE_EXTI
//	printf("%s, %d\r\n", __FUNCTION__, __LINE__);
	EXTIHandlerInit(&hcsr04_ultrasound1_extiCallbackRec, hcsr04_extiHandler_ultrasound1);	// hcsr04_extiCallbackRec->fn = hcsr04_extiHandler
	EXTIConfig(ultrasound1_echoIOPin, &hcsr04_ultrasound1_extiCallbackRec, NVIC_PRIO_ULTRASOUND1_EXTI, EXTI_Trigger_Rising_Falling);
	EXTIEnable(ultrasound1_echoIOPin, true);
#endif
	
	/* +------------------------------------------------------------------------------------------------------------------+ */
	/* +---------------------------------------- Ultrasound 2 configuration ----------------------------------------------+ */
	/* +------------------------------------------------------------------------------------------------------------------+ */
	/* Configure trigger pin of ultrasound sensor 2 */
	ultrasound2_triggerIOPin = IOGetByTag(ultrasoundConfig->triggerTag[1]);
	IOInit(ultrasound2_triggerIOPin, OWNER_ULTRASOUND_TRIGGER, 1);
	IOConfigGPIO(ultrasound2_triggerIOPin, IOCFG_OUT_PP);
	
	/* Configure echo pin of ultrasound sensor 2 */
	ultrasound2_echoIOPin = IOGetByTag(ultrasoundConfig->echoTag[1]);
	IOInit(ultrasound2_echoIOPin, OWNER_ULTRASOUND_ECHO, 1);
	IOConfigGPIO(ultrasound2_echoIOPin, IOCFG_IN_FLOATING);
	
#ifdef USE_EXTI
//	printf("%s, %d\r\n", __FUNCTION__, __LINE__);
	EXTIHandlerInit(&hcsr04_ultrasound2_extiCallbackRec, hcsr04_extiHandler_ultrasound2);	// hcsr04_extiCallbackRec->fn = hcsr04_extiHandler
	EXTIConfig(ultrasound2_echoIOPin, &hcsr04_ultrasound2_extiCallbackRec, NVIC_PRIO_ULTRASOUND1_EXTI, EXTI_Trigger_Rising_Falling);
//	EXTIConfig(ultrasound2_echoIOPin, &hcsr04_ultrasound2_extiCallbackRec, NVIC_PRIO_ULTRASOUND2_EXTI, EXTI_Trigger_Rising_Falling);
	EXTIEnable(ultrasound2_echoIOPin, true);
#endif

	/* +------------------------------------------------------------------------------------------------------------------+ */
	/* +---------------------------------------- Ultrasound 3 configuration ----------------------------------------------+ */
	/* +------------------------------------------------------------------------------------------------------------------+ */
	/* Configure trigger pin of ultrasound sensor 3 */
	ultrasound3_triggerIOPin = IOGetByTag(ultrasoundConfig->triggerTag[2]);
	IOInit(ultrasound3_triggerIOPin, OWNER_ULTRASOUND_TRIGGER, 2);
	IOConfigGPIO(ultrasound3_triggerIOPin, IOCFG_OUT_PP);
	
	/* Configure echo pin of ultrasound sensor 3 */
	ultrasound3_echoIOPin = IOGetByTag(ultrasoundConfig->echoTag[2]);
	IOInit(ultrasound3_echoIOPin, OWNER_ULTRASOUND_ECHO, 2);
	IOConfigGPIO(ultrasound3_echoIOPin, IOCFG_IN_FLOATING);
	
#ifdef USE_EXTI
//	printf("%s, %d\r\n", __FUNCTION__, __LINE__);
	EXTIHandlerInit(&hcsr04_ultrasound3_extiCallbackRec, hcsr04_extiHandler_ultrasound3);	// hcsr04_extiCallbackRec->fn = hcsr04_extiHandler
	EXTIConfig(ultrasound3_echoIOPin, &hcsr04_ultrasound3_extiCallbackRec, NVIC_PRIO_ULTRASOUND1_EXTI, EXTI_Trigger_Rising_Falling);
//	EXTIConfig(ultrasound3_echoIOPin, &hcsr04_ultrasound3_extiCallbackRec, NVIC_PRIO_ULTRASOUND2_EXTI, EXTI_Trigger_Rising_Falling);
	EXTIEnable(ultrasound3_echoIOPin, true);
#endif

	/* +------------------------------------------------------------------------------------------------------------------+ */
	/* +---------------------------------------- Ultrasound 4 configuration ----------------------------------------------+ */
	/* +------------------------------------------------------------------------------------------------------------------+ */
	/* Configure trigger pin of ultrasound sensor 4 */
	ultrasound4_triggerIOPin = IOGetByTag(ultrasoundConfig->triggerTag[3]);
	IOInit(ultrasound4_triggerIOPin, OWNER_ULTRASOUND_TRIGGER, 3);
	IOConfigGPIO(ultrasound4_triggerIOPin, IOCFG_OUT_PP);
	
	/* Configure echo pin of ultrasound sensor 4 */
	ultrasound4_echoIOPin = IOGetByTag(ultrasoundConfig->echoTag[3]);
	IOInit(ultrasound4_echoIOPin, OWNER_ULTRASOUND_ECHO, 3);
	IOConfigGPIO(ultrasound4_echoIOPin, IOCFG_IN_FLOATING);
	
#ifdef USE_EXTI
//	printf("%s, %d\r\n", __FUNCTION__, __LINE__);
	EXTIHandlerInit(&hcsr04_ultrasound4_extiCallbackRec, hcsr04_extiHandler_ultrasound4);	// hcsr04_extiCallbackRec->fn = hcsr04_extiHandler
	EXTIConfig(ultrasound4_echoIOPin, &hcsr04_ultrasound4_extiCallbackRec, NVIC_PRIO_ULTRASOUND1_EXTI, EXTI_Trigger_Rising_Falling);
//	EXTIConfig(ultrasound4_echoIOPin, &hcsr04_ultrasound4_extiCallbackRec, NVIC_PRIO_ULTRASOUND2_EXTI, EXTI_Trigger_Rising_Falling);
	EXTIEnable(ultrasound4_echoIOPin, true);
#endif

	/* +------------------------------------------------------------------------------------------------------------------+ */
	/* +---------------------------------------- Ultrasound 5 configuration ----------------------------------------------+ */
	/* +------------------------------------------------------------------------------------------------------------------+ */
	/* Configure trigger pin of ultrasound sensor 5 */
	ultrasound5_triggerIOPin = IOGetByTag(ultrasoundConfig->triggerTag[4]);
	IOInit(ultrasound5_triggerIOPin, OWNER_ULTRASOUND_TRIGGER, 4);
	IOConfigGPIO(ultrasound5_triggerIOPin, IOCFG_OUT_PP);
	
	/* Configure echo pin of ultrasound sensor 5 */
	ultrasound5_echoIOPin = IOGetByTag(ultrasoundConfig->echoTag[4]);
	IOInit(ultrasound5_echoIOPin, OWNER_ULTRASOUND_ECHO, 4);
	IOConfigGPIO(ultrasound5_echoIOPin, IOCFG_IN_FLOATING);
	
#ifdef USE_EXTI
//	printf("%s, %d\r\n", __FUNCTION__, __LINE__);
	EXTIHandlerInit(&hcsr04_ultrasound5_extiCallbackRec, hcsr04_extiHandler_ultrasound5);	// hcsr04_extiCallbackRec->fn = hcsr04_extiHandler
	EXTIConfig(ultrasound5_echoIOPin, &hcsr04_ultrasound5_extiCallbackRec, NVIC_PRIO_ULTRASOUND1_EXTI, EXTI_Trigger_Rising_Falling);
//	EXTIConfig(ultrasound5_echoIOPin, &hcsr04_ultrasound5_extiCallbackRec, NVIC_PRIO_ULTRASOUND2_EXTI, EXTI_Trigger_Rising_Falling);
	EXTIEnable(ultrasound5_echoIOPin, true);
#endif

	/* +------------------------------------------------------------------------------------------------------------------+ */
	/* +---------------------------------------- Ultrasound 6 configuration ----------------------------------------------+ */
	/* +------------------------------------------------------------------------------------------------------------------+ */
	/* Configure trigger pin of ultrasound sensor 6 */
	ultrasound6_triggerIOPin = IOGetByTag(ultrasoundConfig->triggerTag[5]);
	IOInit(ultrasound6_triggerIOPin, OWNER_ULTRASOUND_TRIGGER, 5);
	IOConfigGPIO(ultrasound6_triggerIOPin, IOCFG_OUT_PP);
	
	/* Configure echo pin of ultrasound sensor 6 */
	ultrasound6_echoIOPin = IOGetByTag(ultrasoundConfig->echoTag[5]);
	IOInit(ultrasound6_echoIOPin, OWNER_ULTRASOUND_ECHO, 5);
	IOConfigGPIO(ultrasound6_echoIOPin, IOCFG_IN_FLOATING);
	
#ifdef USE_EXTI
//	printf("%s, %d\r\n", __FUNCTION__, __LINE__);
	EXTIHandlerInit(&hcsr04_ultrasound6_extiCallbackRec, hcsr04_extiHandler_ultrasound6);	// hcsr04_extiCallbackRec->fn = hcsr04_extiHandler
	EXTIConfig(ultrasound6_echoIOPin, &hcsr04_ultrasound6_extiCallbackRec, NVIC_PRIO_ULTRASOUND1_EXTI, EXTI_Trigger_Rising_Falling);
//	EXTIConfig(ultrasound6_echoIOPin, &hcsr04_ultrasound6_extiCallbackRec, NVIC_PRIO_ULTRASOUND2_EXTI, EXTI_Trigger_Rising_Falling);
	EXTIEnable(ultrasound6_echoIOPin, true);
#endif

	/* force first measurement in hcsr04_get_distance() function */
	ultrasound1_lastMeasurementAt = millis() - 60;
	ultrasound2_lastMeasurementAt = millis() - 60;
	ultrasound3_lastMeasurementAt = millis() - 60;
	ultrasound4_lastMeasurementAt = millis() - 60;
	ultrasound5_lastMeasurementAt = millis() - 60;
	ultrasound6_lastMeasurementAt = millis() - 60;
	
//	printf("%s, %d\r\n", __FUNCTION__, __LINE__);
}

void hcsr04_ultrasound1_start_sequence(void)
{
	uint32_t now = millis();
		
	/* Repeated interval of trig signal should be greater than 60 ms to avoid interference between connective measurements */
	if (now < (ultrasound1_lastMeasurementAt + 60)) {
//		printf("%s, %d\r\n", __FUNCTION__, __LINE__);
		return;
	}
	
	ultrasound1_lastMeasurementAt = now;
	
//	printf("%s, %d\r\n", __FUNCTION__, __LINE__);
	
	/* Trigger ultrasound sensor 1 */
	IOHi(ultrasound1_triggerIOPin);
	delayMicroseconds(15);
	IOLo(ultrasound1_triggerIOPin);
}

void hcsr04_ultrasound2_start_sequence(void)
{
	uint32_t now = millis();
		
	/* Repeated interval of trig signal should be greater than 60 ms to avoid interference between connective measurements */
	if (now < (ultrasound2_lastMeasurementAt + 60)) {
//		printf("%s, %d\r\n", __FUNCTION__, __LINE__);
		return;
	}
	
	ultrasound2_lastMeasurementAt = now;
	
//	printf("%s, %d\r\n", __FUNCTION__, __LINE__);
	
	/* Trigger ultrasound sensor 2 */
	IOHi(ultrasound2_triggerIOPin);
	delayMicroseconds(15);
	IOLo(ultrasound2_triggerIOPin);
}

void hcsr04_ultrasound3_start_sequence(void)
{
	uint32_t now = millis();
		
	/* Repeated interval of trig signal should be greater than 60 ms to avoid interference between connective measurements */
	if (now < (ultrasound3_lastMeasurementAt + 60)) {
//		printf("%s, %d\r\n", __FUNCTION__, __LINE__);
		return;
	}
	
	ultrasound3_lastMeasurementAt = now;
	
//	printf("%s, %d\r\n", __FUNCTION__, __LINE__);
	
	/* Trigger ultrasound sensor 3 */
	IOHi(ultrasound3_triggerIOPin);
	delayMicroseconds(15);
	IOLo(ultrasound3_triggerIOPin);
}

void hcsr04_ultrasound4_start_sequence(void)
{
	uint32_t now = millis();
		
	/* Repeated interval of trig signal should be greater than 60 ms to avoid interference between connective measurements */
	if (now < (ultrasound4_lastMeasurementAt + 60)) {
//		printf("%s, %d\r\n", __FUNCTION__, __LINE__);
		return;
	}
	
	ultrasound4_lastMeasurementAt = now;
	
//	printf("%s, %d\r\n", __FUNCTION__, __LINE__);
	
	/* Trigger ultrasound sensor 4 */
	IOHi(ultrasound4_triggerIOPin);
	delayMicroseconds(15);
	IOLo(ultrasound4_triggerIOPin);
}

void hcsr04_ultrasound5_start_sequence(void)
{
	uint32_t now = millis();
		
	/* Repeated interval of trig signal should be greater than 60 ms to avoid interference between connective measurements */
	if (now < (ultrasound5_lastMeasurementAt + 60)) {
//		printf("%s, %d\r\n", __FUNCTION__, __LINE__);
		return;
	}
	
	ultrasound5_lastMeasurementAt = now;
	
//	printf("%s, %d\r\n", __FUNCTION__, __LINE__);
	
	/* Trigger ultrasound sensor 4 */
	IOHi(ultrasound5_triggerIOPin);
	delayMicroseconds(15);
	IOLo(ultrasound5_triggerIOPin);
}

void hcsr04_ultrasound6_start_sequence(void)
{
	uint32_t now = millis();
		
	/* Repeated interval of trig signal should be greater than 60 ms to avoid interference between connective measurements */
	if (now < (ultrasound6_lastMeasurementAt + 60)) {
//		printf("%s, %d\r\n", __FUNCTION__, __LINE__);
		return;
	}
	
	ultrasound6_lastMeasurementAt = now;
	
//	printf("%s, %d\r\n", __FUNCTION__, __LINE__);
	
	/* Trigger ultrasound sensor 4 */
	IOHi(ultrasound6_triggerIOPin);
	delayMicroseconds(15);
	IOLo(ultrasound6_triggerIOPin);
}

/* Get the distance which was measured by the last pulse (in centimetres) */
int32_t hcsr04_ultrasound1_get_distance(void)
{
	/* First calculation: distance = [measurement (in us) * 340 (m/s) * 100 (cm to m) / 1000000 (s to us)] / 2
	 *
	 * 340 m/s = 340 * 100 / 1000000 = 0.034 cm/microsecond, convert to microsecond/cm gives 1 / 0.034 cm/microseconds = 29.411764705882352941176470588235
	 * 
	 * The amount of microseconds stored in measurement divided by 29.411764705882352941176470588235, i.e. measurement / 29.411764705882352941176470588235
	 * gives the twice the distance, dividing by 2 yields the final desired distance.
	 *
	 * measurement / (1 / 0.034 cm/us) / 2 = measurement / 29.411764705882352941176470588235 / 2 = measurement / (29.411764705882352941176470588235 * 2)
	 * = measurement / 58.82352941176470588235294117647 = measurement / 59
	 */
	int32_t distance = ultrasound1_measurement / 59;
//	int32_t distance = (measurement * 340 * 100 / 1000000) / 2;
	
	return distance;
}

/* Get the distance which was measured by the last pulse (in centimetres) */
int32_t hcsr04_ultrasound2_get_distance(void)
{
	/* First calculation: distance = [measurement (in us) * 340 (m/s) * 100 (cm to m) / 1000000 (s to us)] / 2
	 *
	 * 340 m/s = 340 * 100 / 1000000 = 0.034 cm/microsecond, convert to microsecond/cm gives 1 / 0.034 cm/microseconds = 29.411764705882352941176470588235
	 * 
	 * The amount of microseconds stored in measurement divided by 29.411764705882352941176470588235, i.e. measurement / 29.411764705882352941176470588235
	 * gives the twice the distance, dividing by 2 yields the final desired distance.
	 *
	 * measurement / (1 / 0.034 cm/us) / 2 = measurement / 29.411764705882352941176470588235 / 2 = measurement / (29.411764705882352941176470588235 * 2)
	 * = measurement / 58.82352941176470588235294117647 = measurement / 59
	 */
	int32_t distance = ultrasound2_measurement / 59;
//	int32_t distance = (measurement * 340 * 100 / 1000000) / 2;
	
	return distance;
}

/* Get the distance which was measured by the last pulse (in centimetres) */
int32_t hcsr04_ultrasound3_get_distance(void)
{
	/* First calculation: distance = [measurement (in us) * 340 (m/s) * 100 (cm to m) / 1000000 (s to us)] / 2
	 *
	 * 340 m/s = 340 * 100 / 1000000 = 0.034 cm/microsecond, convert to microsecond/cm gives 1 / 0.034 cm/microseconds = 29.411764705882352941176470588235
	 * 
	 * The amount of microseconds stored in measurement divided by 29.411764705882352941176470588235, i.e. measurement / 29.411764705882352941176470588235
	 * gives the twice the distance, dividing by 2 yields the final desired distance.
	 *
	 * measurement / (1 / 0.034 cm/us) / 2 = measurement / 29.411764705882352941176470588235 / 2 = measurement / (29.411764705882352941176470588235 * 2)
	 * = measurement / 58.82352941176470588235294117647 = measurement / 59
	 */
	int32_t distance = ultrasound3_measurement / 59;
//	int32_t distance = (measurement * 340 * 100 / 1000000) / 2;
	
	return distance;
}

/* Get the distance which was measured by the last pulse (in centimetres) */
int32_t hcsr04_ultrasound4_get_distance(void)
{
	/* First calculation: distance = [measurement (in us) * 340 (m/s) * 100 (cm to m) / 1000000 (s to us)] / 2
	 *
	 * 340 m/s = 340 * 100 / 1000000 = 0.034 cm/microsecond, convert to microsecond/cm gives 1 / 0.034 cm/microseconds = 29.411764705882352941176470588235
	 * 
	 * The amount of microseconds stored in measurement divided by 29.411764705882352941176470588235, i.e. measurement / 29.411764705882352941176470588235
	 * gives the twice the distance, dividing by 2 yields the final desired distance.
	 *
	 * measurement / (1 / 0.034 cm/us) / 2 = measurement / 29.411764705882352941176470588235 / 2 = measurement / (29.411764705882352941176470588235 * 2)
	 * = measurement / 58.82352941176470588235294117647 = measurement / 59
	 */
	int32_t distance = ultrasound4_measurement / 59;
//	int32_t distance = (measurement * 340 * 100 / 1000000) / 2;
	
	return distance;
}

/* Get the distance which was measured by the last pulse (in centimetres) */
int32_t hcsr04_ultrasound5_get_distance(void)
{
	/* First calculation: distance = [measurement (in us) * 340 (m/s) * 100 (cm to m) / 1000000 (s to us)] / 2
	 *
	 * 340 m/s = 340 * 100 / 1000000 = 0.034 cm/microsecond, convert to microsecond/cm gives 1 / 0.034 cm/microseconds = 29.411764705882352941176470588235
	 * 
	 * The amount of microseconds stored in measurement divided by 29.411764705882352941176470588235, i.e. measurement / 29.411764705882352941176470588235
	 * gives the twice the distance, dividing by 2 yields the final desired distance.
	 *
	 * measurement / (1 / 0.034 cm/us) / 2 = measurement / 29.411764705882352941176470588235 / 2 = measurement / (29.411764705882352941176470588235 * 2)
	 * = measurement / 58.82352941176470588235294117647 = measurement / 59
	 */
	int32_t distance = ultrasound5_measurement / 59;
//	int32_t distance = (measurement * 340 * 100 / 1000000) / 2;
	
	return distance;
}

/* Get the distance which was measured by the last pulse (in centimetres) */
int32_t hcsr04_ultrasound6_get_distance(void)
{
	/* First calculation: distance = [measurement (in us) * 340 (m/s) * 100 (cm to m) / 1000000 (s to us)] / 2
	 *
	 * 340 m/s = 340 * 100 / 1000000 = 0.034 cm/microsecond, convert to microsecond/cm gives 1 / 0.034 cm/microseconds = 29.411764705882352941176470588235
	 * 
	 * The amount of microseconds stored in measurement divided by 29.411764705882352941176470588235, i.e. measurement / 29.411764705882352941176470588235
	 * gives the twice the distance, dividing by 2 yields the final desired distance.
	 *
	 * measurement / (1 / 0.034 cm/us) / 2 = measurement / 29.411764705882352941176470588235 / 2 = measurement / (29.411764705882352941176470588235 * 2)
	 * = measurement / 58.82352941176470588235294117647 = measurement / 59
	 */
	int32_t distance = ultrasound6_measurement / 59;
//	int32_t distance = (measurement * 340 * 100 / 1000000) / 2;
	
	return distance;
}

#endif
