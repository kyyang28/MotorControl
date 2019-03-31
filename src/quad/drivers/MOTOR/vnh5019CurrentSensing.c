
#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include "adc.h"
#include "filter.h"
#include "vnh5019CurrentSensing.h"
#include "configMaster.h"

#define MOTOR_CURRENT_LPF_CUTOFF_FREQ					0.4f
#define MOTOR_CURRENT_FILTER_SAMPLING_FREQ				50000			// 50000 us

#define ADC_VREF										3300			// in mV, 3300 mV = 3.3 V

/* VNH5019 current sensing module has roughly 144 mV / A */

/* currentMeterValue in milliamp (mA) */
//float unfilteredCurrentMeterValue1 = 0.0f;
//float unfilteredCurrentMeterValue2 = 0.0f;
float filteredLeftMotorCurrentMeterValue = 0.0f;
float filteredRightMotorCurrentMeterValue = 0.0f;

/* Return value in milliamps (mA) */
static float convertADCCountToMilliAmps(uint16_t adcValue)
{
	float milliVolts;

//	printf("resolutionScale: %u\r\n", AdcConfig()->resolutionScale);
	
	/* ADC_VREF = 3300 mV = 3.3 V
	 * 
	 * ADC resolution is 12 bits, 2^12 = 4096
	 * 
	 * (3300 mV / 4096(2^12) - offset) / 144 (mV/A) = 0.00559488932291667 (Amps)
	 *
	 * 0.00559488932291667 (Amps) * 1000 = 5.59488932291667 (Milliamps)
	 */
	milliVolts = ((uint32_t)adcValue * ADC_VREF) / AdcConfig()->resolutionScale;		// ADC resolution 12-bit, resolutionScale = 2^12 = 4096
	milliVolts -= MotorCurrentMeterConfig()->currentMeterOffset;
	
//	printf("offset: %u\r\n", MotorCurrentMeterConfig()->currentMeterOffset);			// 0
//	printf("scale: %u\r\n", MotorCurrentMeterConfig()->currentMeterScale);				// 144
	
	/* Multiply by 1000 to convert amps to milliamps */
	return (1000 * milliVolts / MotorCurrentMeterConfig()->currentMeterScale);		// 144 mV / A for VNH5019 Motor Driver Sensing Unit
}

static void updateVNH5019LeftMotorCurrent(void)
{
	static biquadFilter_t motorCurrentFilter;
	static bool isMotorCurrentFilterInitialised;
	
	if (!isMotorCurrentFilterInitialised) {
		biquadFilterInitLPF(&motorCurrentFilter, MOTOR_CURRENT_LPF_CUTOFF_FREQ, MOTOR_CURRENT_FILTER_SAMPLING_FREQ);
		isMotorCurrentFilterInitialised = true;
	}
	
	/* Get ADC value */
	uint16_t leftMotorCurrentSample = adcGetChannelSample(ADC_MOTOR_CURRENT1);
	
//	printf("leftMotorCurrentSample: %u\r\n", leftMotorCurrentSample);
	
	/* Convert ADC value to amps */
//	unfilteredCurrentMeterValue1 = convertADCCountToMilliAmps(leftMotorCurrentSample);
	filteredLeftMotorCurrentMeterValue = convertADCCountToMilliAmps(biquadFilterApply(&motorCurrentFilter, leftMotorCurrentSample));
	
//	printf("f1: %d\t\t", (int32_t)round(filteredLeftMotorCurrentMeterValue));
//	printf("uf1: %d\t\tf1: %d\t\tuf2: %d\t\tf2: %d\r\n", (int32_t)round(unfilteredCurrentMeterValue1), (int32_t)round(filteredCurrentMeterValue1), (int32_t)round(unfilteredCurrentMeterValue2), (int32_t)round(filteredCurrentMeterValue2));
}

static void updateVNH5019RightMotorCurrent(void)
{
	static biquadFilter_t motorCurrentFilter;
	static bool isMotorCurrentFilterInitialised;
	
	if (!isMotorCurrentFilterInitialised) {
		biquadFilterInitLPF(&motorCurrentFilter, MOTOR_CURRENT_LPF_CUTOFF_FREQ, MOTOR_CURRENT_FILTER_SAMPLING_FREQ);
		isMotorCurrentFilterInitialised = true;
	}
	
	/* Get ADC value */
	uint16_t rightMotorCurrentSample = adcGetChannelSample(ADC_MOTOR_CURRENT2);
	
//	printf("rightMotorCurrentSample: %u\r\n", rightMotorCurrentSample);
	
	/* Convert ADC value to amps */
//	unfilteredCurrentMeterValue2 = convertADCCountToMilliAmps(rightMotorCurrentSample);
	filteredRightMotorCurrentMeterValue = convertADCCountToMilliAmps(biquadFilterApply(&motorCurrentFilter, rightMotorCurrentSample));
	
//	printf("f2: %d\r\n", (int32_t)round(filteredRightMotorCurrentMeterValue));
}

static void updateVNH5019CurrentDrawn(int32_t lastUpdateAt)
{
	
}

void updateVNH5019LeftMotorCurrentSensor(int32_t lastUpdateAt)
{
	updateVNH5019LeftMotorCurrent();
	
	updateVNH5019CurrentDrawn(lastUpdateAt);
}

void updateVNH5019RightMotorCurrentSensor(int32_t lastUpdateAt)
{
	updateVNH5019RightMotorCurrent();
	
	updateVNH5019CurrentDrawn(lastUpdateAt);
}
