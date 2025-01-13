/*
 * writeTask.h
 *
 *      Author: vinic
 */

#ifndef INC_WRITETASK_H_
#define INC_WRITETASK_H_

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

// #define SUBS_LEN 10
#define COMMAND_MESSAGE_LENGTH 20
#define WRITE_QUEUE_LENGTH 5

// Define the command_message_t type as an array of xx char
typedef uint8_t command_message_t[COMMAND_MESSAGE_LENGTH];

BaseType_t writeTaskInit();
BaseType_t sendMessage(command_message_t *message);

#endif /* INC_WRITETASK_H_ */
