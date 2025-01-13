/*
 * readTask.h
 *
 *      Author: vinic
 */

#ifndef INC_READTASK_H_
#define INC_READTASK_H_

#define MAX_SUBSCRIBERS 4
#define QUEUE_LENGTH 10
#define SENSOR_TABLE_SIZE 2
#define MAX_SEMAPHORE 10

#include <stddef.h>
#include <stdint.h>

// Device header
#include "stm32f0xx.h"

// BSP functions
#include "bsp.h"

// FreeRTOS headers
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"
#include "queue.h"
#include "event_groups.h"

#include "printf-stdarg.h"

typedef struct {
    uint8_t sem_id;        // Semaphore ID to use for publication
    uint8_t sensor_id;     // Awaited sensor ID
    uint8_t sensor_state;  // Awaited sensor state
} subscribe_message_t;

// My functions
BaseType_t vTaskPubInit();
BaseType_t subscribe(uint8_t sem_id, uint8_t sensor_id, uint8_t sensor_state);

// Using subscribe_message_t, so this needs to go after declaration

extern xSemaphoreHandle sems[MAX_SEMAPHORE];

#endif /* INC_READTASK_H_ */
