/*
 * readTask.c
 *
 *      Author: vinic
 */
#include "readTask.h"

static xTaskHandle vTaskPub_handle;
static xQueueHandle xSubscribeQueue;
xSemaphoreHandle sems[MAX_SEMAPHORE];

static void vTask_Pub(void *pvParameters);
static void updateSubs(subscribe_message_t *subs, subscribe_message_t *new_sub);
static void print_subscription_table(subscribe_message_t *subs);
static void uartSensor(uint8_t *sensors);
static void publish(subscribe_message_t *subs, uint8_t *sensors);

BaseType_t vTaskPubInit(){
	size_t i;

	xSubscribeQueue = xQueueCreate(QUEUE_LENGTH, sizeof(subscribe_message_t));

	for(i = 0; i < MAX_SEMAPHORE; i++) {
		sems[i] = xSemaphoreCreateBinary();
	}

    // xTaskCreate(vTask_Pub, "vTask_Pub", 128, NULL, 1, &vTaskPub_handle);

    my_printf("READ TASK DEFINED\r\n");

    return pdPASS;

}

static void vTask_Pub(void *pvParameters) {
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = 200 / portTICK_PERIOD_MS;

    xLastWakeTime = xTaskGetTickCount();

    subscribe_message_t subscription_table[MAX_SUBSCRIBERS] = {0};
    subscribe_message_t msg;
    uint8_t sensors[SENSOR_TABLE_SIZE];
    size_t i;

    // char rx_byte;

    // Reseting the message
    for(i = 0; i < MAX_SUBSCRIBERS; i++) {
    	subscription_table[i].sem_id = 0;
    	subscription_table[i].sensor_id = 0;
    	subscription_table[i].sensor_state = 0;
    }

	for(i = 0; i < SENSOR_TABLE_SIZE; i++) {
		sensors[i] = 0;
	}


    while (1) {
    	// BSP_LED_Toggle();

    	if(xQueueReceive(xSubscribeQueue, &msg, 0)){
    		updateSubs(subscription_table, &msg);
    		print_subscription_table(subscription_table);
    	} else {
    		my_printf(".");
    	}

    	uartSensor(sensors);

		publish(subscription_table, sensors);


        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

BaseType_t subscribe(uint8_t sem_id, uint8_t sensor_id, uint8_t sensor_state)
{
	subscribe_message_t data = {
		.sem_id = sem_id,
		.sensor_id = sensor_id,
		.sensor_state = sensor_state
	};

	return xQueueSend(xSubscribeQueue, &data, 0);
}


/*
 * Update the subscription table
 */
static void updateSubs(subscribe_message_t *subs, subscribe_message_t *new_sub) {
    size_t i;

    my_printf("Subscribing...");

    // Check for duplicates
    for (i = 0; i < MAX_SUBSCRIBERS; i++) {
        if (subs[i].sem_id == new_sub->sem_id &&
            subs[i].sensor_id == new_sub->sensor_id &&
            subs[i].sensor_state == new_sub->sensor_state) {
            my_printf("Subscription already exists\r\n");
            return;
        }
    }

    // Add the new subscription to the first available slot
    for (i = 0; i < MAX_SUBSCRIBERS; i++) {
        if (subs[i].sem_id == 0) {
            subs[i] = *new_sub;
            my_printf("Adding subscription in slot [%d]\r\n", i);
            return;
        }
    }

    // If the table is full
    my_printf("No available slots for new subscription\r\n");
}


/*
 * Display the subscription table
 */
static void print_subscription_table(subscribe_message_t *subs) {
    for (int i = 0; i < MAX_SUBSCRIBERS; i++) {
        my_printf("[%d] %d %d %d\r\n", i, subs[i].sem_id, subs[i].sensor_id, subs[i].sensor_state);
    }
}


static void uartSensor(uint8_t *sensors)
{
	uint8_t rx;
	size_t i;

	if( (USART2->ISR & USART_ISR_RXNE) != USART_ISR_RXNE ) return;

	rx = USART2->RDR;

	switch(rx) {
	case 'a':
		sensors[1] = 0;
		break;

	case 'b':
		sensors[1] = 1;
		break;

	case 'c':
		sensors[2] = 0;
		break;

	case 'd':
		sensors[2] = 1;
		break;
	}

	my_printf("sensors = [ ");
	for(i = 1; i <= SENSOR_TABLE_SIZE; i++) {
		my_printf("%d ", sensors[i]);
	}
	my_printf("]\r\n");
}

static void publish(subscribe_message_t *subs, uint8_t *sensors) {
	size_t i;
	uint8_t sensor, sem;

	for(i = 0; i < MAX_SUBSCRIBERS; i++) {
		if(subs[i].sem_id == 0) continue;

		sensor = subs[i].sensor_id;
		sem = subs[i].sem_id;

		if(sensors[sensor] == subs[i].sensor_state) continue;

		xSemaphoreGive(sems[sem]);

		my_printf("published\r\n");

		subs[i].sem_id = 0;
		subs[i].sensor_id = 0;
		subs[i].sensor_state = 0;
	}
}
