/*
 * main.h
 *
 *  Created on: 4 nov. 2022
 *      Author: laure
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

// Global functions
int my_printf	(const char *format, ...);
int my_sprintf	(char *out, const char *format, ...);

#endif /* INC_MAIN_H_ */
