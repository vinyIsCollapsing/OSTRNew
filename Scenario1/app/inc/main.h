/*
 * main.h
 *
 */

#ifndef INC_MAIN_H_
#define INC_MAIN_H_

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
#include "stream_buffer.h"

#include "printf-stdarg.h"
// Tasks headers
#include "readTask.h"
#include "writeTask.h"


#endif /* INC_MAIN_H_ */
