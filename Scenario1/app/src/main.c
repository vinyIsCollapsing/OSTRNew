/*
 *  main.c
 *
 */

#include "main.h"

// Static functions
static void SystemClock_Config(void);
void vApplicationMallocFailedHook(void);

// FreeRTOS tasks
void vTask1(void *pvParameters);
void vTask2(void *pvParameters);

// FreeRTOS task handles
xTaskHandle vTask1_handle;
xTaskHandle vTask2_handle;

// Kernel objects
xSemaphoreHandle xSem;
xSemaphoreHandle ledMutex;

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

    myPrintfInit();

    // Create semaphore
    xSem = xSemaphoreCreateBinary();
    vTraceSetSemaphoreName(xSem, "xSem");
	ledMutex = xSemaphoreCreateMutex();

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

    my_printf("OK\r\n");

    vTaskPubInit();
    writeTaskInit();

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
	command_message_t msgTask1;

	while (1) {
    	my_sprintf((char *) msgTask1, "message_#1\r\n");
    	sendMessage(&msgTask1);
		vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

/*
 * Task_2
 */
void vTask2(void *pvParameters) {
	command_message_t msgTask2;

	while(1){
    	my_sprintf((char *) msgTask2, "message_#2\r\n");
    	sendMessage(&msgTask2);
    	vTaskDelay(100 / portTICK_PERIOD_MS);
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

/*
 * TP2: readTask
subscribe(1,1,0);
xSemaphoreTake(sems[1], portMAX_DELAY);
xSemaphoreTake(ledMutex, portMAX_DELAY);
BSP_LED_On();
vTaskDelay(100 / portTICK_PERIOD_MS);
BSP_LED_Off();
xSemaphoreGive(ledMutex);

subscribe(1, 1, 1);
xSemaphoreTake(sems[1], portMAX_DELAY);
xSemaphoreTake(ledMutex, portMAX_DELAY);
BSP_LED_On();
vTaskDelay(100 / portTICK_PERIOD_MS);
BSP_LED_Off();
vTaskDelay(200 / portTICK_PERIOD_MS);
BSP_LED_On();
vTaskDelay(100 / portTICK_PERIOD_MS);
BSP_LED_Off();
xSemaphoreGive(ledMutex);
*/

/*
 * TP2: readTask
subscribe(2, 2, 0);
xSemaphoreTake(sems[2], portMAX_DELAY);
xSemaphoreTake(ledMutex, portMAX_DELAY);
BSP_LED_On();
vTaskDelay(200 / portTICK_PERIOD_MS);
BSP_LED_Off();
xSemaphoreGive(ledMutex);

subscribe(2, 2, 1);
xSemaphoreTake(sems[2], portMAX_DELAY);
xSemaphoreTake(ledMutex, portMAX_DELAY);
BSP_LED_On();
vTaskDelay(200 / portTICK_PERIOD_MS);
BSP_LED_Off();
vTaskDelay(400 / portTICK_PERIOD_MS);
BSP_LED_On();
vTaskDelay(200 / portTICK_PERIOD_MS);
BSP_LED_Off();
xSemaphoreGive(ledMutex);
*/
