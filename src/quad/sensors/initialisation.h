#ifndef __INITIALISATION_H
#define __INITIALISATION_H

#include <stdbool.h>
#include "gyro.h"
#include "acceleration.h"
#include "barometer.h"
#include "compass.h"
#include "ultrasound_hcsr04.h"

bool sensorsAutodetect(const gyroConfig_t *gyroConfig, const accelerometerConfig_t *accelerometerConfig, const ultrasoundConfig_t *ultrasoundConfig);

#endif	// __INITIALISATION_H
