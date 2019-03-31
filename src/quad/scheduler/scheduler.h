#ifndef __SCHEDULER_H
#define __SCHEDULER_H

#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include "time.h"           // included <stdint.h>
#include "target.h"

typedef enum {
    TASK_PRIORITY_IDLE = 0,
    TASK_PRIORITY_LOW = 1,
    TASK_PRIORITY_MEDIUM = 3,
    TASK_PRIORITY_MEDIUM_HIGH = 4,
    TASK_PRIORITY_HIGH = 5,
    TASK_PRIORITY_REALTIME = 6,
    TASK_PRIORITY_MAX = 255
}cfTaskPriority_e;

typedef enum {
    /* Actual tasks */
    TASK_SYSTEM = 0,				// 0
    TASK_GYRO,						// 1
    TASK_ACCEL,						// 2
    TASK_ATTITUDE,					// 3
    TASK_RX,						// 4
    TASK_SERIAL,					// 5
	TASK_MOTORENCODER,				// 6
	TASK_OLEDDISPLAY,				// 7
	TASK_ULTRASOUND1_UPDATE,		// 8
	TASK_ULTRASOUND1_READDATA,		// 9
	TASK_ULTRASOUND2_UPDATE,		// 10
	TASK_ULTRASOUND2_READDATA,		// 11
	TASK_ULTRASOUND3_UPDATE,		// 12
	TASK_ULTRASOUND3_READDATA,		// 13
	TASK_ULTRASOUND4_UPDATE,		// 14
	TASK_ULTRASOUND4_READDATA,		// 15
	TASK_ULTRASOUND5_UPDATE,		// 16
	TASK_ULTRASOUND5_READDATA,		// 17
	TASK_ULTRASOUND6_UPDATE,		// 18
	TASK_ULTRASOUND6_READDATA,		// 19
	TASK_LEFT_MOTOR_CURRENT_METER,	// 20
	TASK_RIGHT_MOTOR_CURRENT_METER,	// 21
//	TASK_BTRX,						// 22
//    TASK_LED3,
//    TASK_LED4,
//    TASK_LED5,
//    TASK_LED6,

#ifdef BEEPER
    TASK_BEEPER,
#endif
    
    /* Count of real number of tasks */
    TASK_COUNT,
    
    /* Service task IDs */
    TASK_NONE = TASK_COUNT,
    TASK_SELF
}cfTaskId_e;

typedef struct {
    /* Configuration */
    const char *taskName;
    const char *subTaskName;
    bool (*checkFunc)(timeUs_t currentTimeUs, timeDelta_t currentDeltaTimeUs);
    void (*taskFunc)(timeUs_t currentTImeUs);
    uint32_t desiredPeriod;                         // period of task execution
    const uint8_t staticPriority;                   // dynamic priority grows in steps of this size(staticPriority), it shouldn't be zero.
        
    /* Scheduling */
    uint16_t dynamicPriority;                       // measurement of how old task was last executed, used to avoid task starvation
    uint16_t taskAgeCycles;
    timeUs_t lastExecutedAt;                        // last time of invocation
    timeUs_t lastSignaledAt;                        // time of invocation event for event-driven tasks
    uint32_t taskLatestDeltaTime;
    
#ifndef SKIP_TASK_STATISTICS    
    /* Statistics */
    timeUs_t movingSumExecutionTime;                // moving sum over 32 samples
    timeUs_t maxExecutionTime;
    timeUs_t totalExecutionTime;                    // total time consumed by task since boot
#endif
}cfTask_t;

extern cfTask_t cfTasks[TASK_COUNT];

void schedulerInit(void);
void scheduler(void);

void rescheduleTask(cfTaskId_e taskId, uint32_t newPeriodInMicros);
void setTaskEnabled(cfTaskId_e taskId, bool enabled);

void taskSystem(timeUs_t currentTimeUs);

uint32_t getTaskDeltaTime(cfTaskId_e taskId);

#endif
