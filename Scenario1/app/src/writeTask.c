/*
 * writeTask.c
 *
 *      Author: vinic
 */


#include "writeTask.h"

extern int my_printf(const char *format, ...);

static void vTaskWrite(void *pvParameters);

static xTaskHandle vTaskWrite_handle;
static xQueueHandle xWriteQueue;

void writeTaskInit(void *pvParameters){
    // Create the subscription queue
    xWriteQueue = xQueueCreate(WRITE_QUEUE_LENGTH, sizeof(command_message_t));

    xTaskCreate(vTaskWrite, "vTask_Write", 128, NULL, 1, &vTaskWrite_handle);

    my_printf("WRITE TASK DEFINED\r\n");
}

BaseType_t sendMessage(command_message_t *message){
	return xQueueSendToBack(xWriteQueue, &message, 0);
}

/*
 * Task_Write
 */

void vTaskWrite(void *pvParameters){
	command_message_t *msgQueue;
	while(1){
		xQueueReceive((xWriteQueue), &msgQueue, portMAX_DELAY);

		my_printf("%s\r\n", msgQueue);
	}
}
