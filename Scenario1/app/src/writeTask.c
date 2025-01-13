/*
 * writeTask.c
 *
 *      Author: vinic
 */


#include "writeTask.h"

static void vTaskWrite(void *pvParameters);

static xTaskHandle vTaskWrite_handle;
static xQueueHandle xWriteQueue;

extern xSemaphoreHandle xSem_DMA_TC;

BaseType_t writeTaskInit(void *pvParameters){
    // Create the subscription queue
    xWriteQueue = xQueueCreate(WRITE_QUEUE_LENGTH, sizeof(command_message_t));

    xTaskCreate(vTaskWrite, "vTask_Write", 128, NULL, 1, &vTaskWrite_handle);

    my_printf("WRITE TASK DEFINED\r\n");

    return pdPASS;
}

BaseType_t sendMessage(command_message_t *message){
	return xQueueSendToBack(xWriteQueue, &message, 0);
}

/*
 * Task_Write
 */

static void vTaskWrite(void *pvParameters){
	command_message_t *msgQueue;
	char *str;
	size_t i;

	while(1){
		xQueueReceive((xWriteQueue), &msgQueue, portMAX_DELAY);

		str = (char *) msgQueue;

		for(i = 0; str[i] != '\0'; i++) {
			tx_dma_buffer[i] = str[i];
		}

		DMA1_Channel4->CNDTR = i;
		DMA1_Channel4->CCR |= DMA_CCR_EN;

		xSemaphoreTake(xSem_DMA_TC, portMAX_DELAY);
		DMA1_Channel4->CCR &= ~DMA_CCR_EN;

		// my_printf("%s\r\n", msgQueue);
	}
}
