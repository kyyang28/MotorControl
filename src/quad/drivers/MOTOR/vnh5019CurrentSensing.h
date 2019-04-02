#ifndef __VNH5019CURRENTSENSING_H
#define __VNH5019CURRENTSENSING_H

#include <stdint.h>

typedef struct motorCurrentMeterConfig_s {
	int16_t currentMeterScale;
	int16_t currentMeterOffset;
}motorCurrentMeterConfig_t;

//extern float unfilteredCurrentMeterValue1;
//extern float unfilteredCurrentMeterValue2;
extern float filteredLeftMotorCurrentMeterValue;
extern float filteredRightMotorCurrentMeterValue;
//extern int32_t meanFilteredLeftMotorCurrentMeterValue;
//extern int32_t meanFilteredRightMotorCurrentMeterValue;

void updateVNH5019LeftMotorCurrentSensor(int32_t lastUpdateAt);
void updateVNH5019RightMotorCurrentSensor(int32_t lastUpdateAt);

#endif	// __VNH5019CURRENTSENSING_H
