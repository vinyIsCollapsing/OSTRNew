/*
 *  main.c
 *
 */

#include "main.h"

// #define SUBS_LEN 10
#define MAX_SUBSCRIBERS 4
#define QUEUE_LENGTH 10
#define SENSOR_TABLE_SIZE 2
#define MAX_SEMAPHORE 10

// Static functions
static void SystemClock_Config(void);
void vApplicationMallocFailedHook(void);

// FreeRTOS tasks
void vTask1(void *pvParameters);
void vTask2(void *pvParameters);
void vTask_Pub(void *pvParameters);

// FreeRTOS task handles
xTaskHandle vTask1_handle;
xTaskHandle vTask2_handle;
xTaskHandle vTaskPub_handle;

// Kernel objects
xSemaphoreHandle xSem;

xQueueHandle xSubscribeQueue;

typedef struct {
    uint8_t sem_id;        // Semaphore ID to use for publication
    uint8_t sensor_id;     // Awaited sensor ID
    uint8_t sensor_state;  // Awaited sensor state
} subscribe_message_t;

uint8_t sensor_states[SENSOR_TABLE_SIZE] = {0};
typedef uint8_t command_message_t[60];				// Define the command_message_t type as an array of xx char


// Using subscribe_message_t, so this needs to go after declaration
static void updateSubs(subscribe_message_t *subs, subscribe_message_t *new_sub);
static void print_subscription_table(subscribe_message_t *subs);

BaseType_t subscribe(uint8_t sem_id, uint8_t sensor_id, uint8_t sensor_state);



// Main program
int main() {
    uint32_t free_heap_size;

    // Configure system clock
    SystemClock_Config();

    // Initialize LED pin
    BSP_LED_Init();

    // Initialize debug console
    BSP_Console_Init();

    // Start trace recording
    vTraceEnable(TRC_START);

    // Create the subscription queue
    xSubscribeQueue = xQueueCreate(QUEUE_LENGTH, sizeof(subscribe_message_t));

    // Create semaphore
    xSem = xSemaphoreCreateBinary();
    vTraceSetSemaphoreName(xSem, "xSem");
    //xSem2 = xSemaphoreCreateBinary();
    //vTraceSetSemaphoreName(xSem2, "xSem2");

    // Initialize push-button interrupt
    BSP_PB_Init();

    // Initialize NVIC
    BSP_NVIC_Init();

    // Report free heap size
    free_heap_size = xPortGetFreeHeapSize();
    my_printf("Free Heap Size is %d bytes\r\n", free_heap_size);

    // Create tasks
    my_printf("Creating Tasks...");
    xTaskCreate(vTask1, "Task_1", 128, NULL, 1, &vTask1_handle);
    xTaskCreate(vTask2, "Task_2", 128, NULL, 1, &vTask2_handle);
    xTaskCreate(vTask_Pub, "vTask_Pub", 128, NULL, 1, &vTaskPub_handle);
    my_printf("OK\r\n");

    // Report free heap size
    free_heap_size = xPortGetFreeHeapSize();
    my_printf("Free Heap Size is %d bytes\r\n", free_heap_size);

    // Start the scheduler
    my_printf("Now Starting Scheduler...\r\n");
    vTaskStartScheduler();

    while (1) {
        // The program should never reach here...
    }
}

/*
 * Task_1
 */
void vTask1(void *pvParameters) {
    while (1) {
    	subscribe(1,1,1);
    	vTaskDelay(5000 / portTICK_PERIOD_MS);
    }
}

/*
 * Task_2
 */
void vTask2(void *pvParameters) {
    while(1){
    	subscribe(2,2,0);
    	vTaskDelay(4000 / portTICK_PERIOD_MS);
    }
}

/*
 * Task_Pub
 */
void vTask_Pub(void *pvParameters) {
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = 200 / portTICK_PERIOD_MS;

    xLastWakeTime = xTaskGetTickCount();

    subscribe_message_t subscription_table[MAX_SUBSCRIBERS] = {0};
    subscribe_message_t msg;
    // char rx_byte;

    // Reseting the message
    for(size_t i = 0; i < MAX_SUBSCRIBERS; i++) {
    	subscription_table[i].sem_id = 0;
    	subscription_table[i].sensor_id = 0;
    	subscription_table[i].sensor_state = 0;
    }

    while (1) {
    	BSP_LED_Toggle();

    	if(xQueueReceive(xSubscribeQueue, &msg, portMAX_DELAY)){
    		updateSubs(subscription_table, &msg);
    		print_subscription_table(subscription_table);
    	} else {
    		my_printf(".");
    	}

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

/*
 * Hook function for malloc failure
 */
void vApplicationMallocFailedHook() {
    while (1);
}

/*
 * Configure the system clock for the Nucleo STM32F072RB board
 * HSE input Bypass Mode            -> 8MHz
 * SYSCLK, AHB, APB1                -> 48MHz
 * PA8 as MCO with /16 prescaler    -> 3MHz
 */
static void SystemClock_Config() {
    uint32_t HSE_Status;
    uint32_t PLL_Status;
    uint32_t SW_Status;
    uint32_t timeout = 0;

    timeout = 1000000;

    // Start HSE in Bypass Mode
    RCC->CR |= RCC_CR_HSEBYP;
    RCC->CR |= RCC_CR_HSEON;

    // Wait until HSE is ready
    do {
        HSE_Status = RCC->CR & RCC_CR_HSERDY_Msk;
        timeout--;
    } while ((HSE_Status == 0) && (timeout > 0));

    // Select HSE as PLL input source
    RCC->CFGR &= ~RCC_CFGR_PLLSRC_Msk;
    RCC->CFGR |= (0x02 << RCC_CFGR_PLLSRC_Pos);

    // Set PLL PREDIV to /1
    RCC->CFGR2 = 0x00000000;

    // Set PLL MUL to x6
    RCC->CFGR &= ~RCC_CFGR_PLLMUL_Msk;
    RCC->CFGR |= (0x04 << RCC_CFGR_PLLMUL_Pos);

    // Enable the main PLL
    RCC->CR |= RCC_CR_PLLON;

    // Wait until PLL is ready
    do {
        PLL_Status = RCC->CR & RCC_CR_PLLRDY_Msk;
        timeout--;
    } while ((PLL_Status == 0) && (timeout > 0));

    // Set AHB prescaler to /1
    RCC->CFGR &= ~RCC_CFGR_HPRE_Msk;
    RCC->CFGR |= RCC_CFGR_HPRE_DIV1;

    // Set APB1 prescaler to /1
    RCC->CFGR &= ~RCC_CFGR_PPRE_Msk;
    RCC->CFGR |= RCC_CFGR_PPRE_DIV1;

    // Enable FLASH Prefetch Buffer and set Flash Latency
    FLASH->ACR = FLASH_ACR_PRFTBE | FLASH_ACR_LATENCY;

    // Select the main PLL as system clock source
    RCC->CFGR &= ~RCC_CFGR_SW;
    RCC->CFGR |= RCC_CFGR_SW_PLL;

    // Wait until PLL becomes main switch input
    do {
        SW_Status = (RCC->CFGR & RCC_CFGR_SWS_Msk);
        timeout--;
    } while ((SW_Status != RCC_CFGR_SWS_PLL) && (timeout > 0));

    // Update SystemCoreClock global variable
    SystemCoreClockUpdate();
}
