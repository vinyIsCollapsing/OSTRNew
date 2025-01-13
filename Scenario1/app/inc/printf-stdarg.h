/*
 * printf-stdarg.h
 *
 *  Created on: Jan 13, 2025
 *      Author: vinic
 */

#ifndef INC_PRINTF_STDARG_H_
#define INC_PRINTF_STDARG_H_

#include <stdarg.h>

#include "stm32f0xx.h"

// FreeRTOS headers
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"
#include "queue.h"
#include "event_groups.h"
#include "stream_buffer.h"

BaseType_t myPrintfInit();

int my_printf(const char *format, ...);
int my_sprintf(char *out, const char *format, ...);


#endif /* INC_PRINTF_STDARG_H_ */
