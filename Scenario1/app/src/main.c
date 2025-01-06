/*
 *  main.c
 *
 */

// TODO: terminar a parte 5 pulsos de led, captura de semaforo e sequencia de abcd

#include "main.h"

// #define SUBS_LEN 10
#define MAX_SUBSCRIBERS 4
#define SENSOR_TABLE_SIZE 2

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
xSemaphoreHandle xSem1;
xSemaphoreHandle xSem2;
xQueueHandle xSubscribeQueue;

typedef struct {
    uint8_t sem_id;        // Semaphore ID to use for publication
    uint8_t sensor_id;     // Awaited sensor ID
    uint8_t sensor_state;  // Awaited sensor state
} subscribe_message_t;

uint8_t sensor_states[SENSOR_TABLE_SIZE] = {0};

// Using subscribe_message_t, so this needs to go after declaration
static void updateSubs(subscribe_message_t *subs, subscribe_message_t *new_sub);
static void print_subscription_table(subscribe_message_t *subs);

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

    // Create semaphore
    xSem1 = xSemaphoreCreateBinary();
    xSem2 = xSemaphoreCreateBinary();
    vTraceSetSemaphoreName(xSem1, "xSem1");
    vTraceSetSemaphoreName(xSem2, "xSem2");

    // Initialize push-button interrupt
    BSP_PB_Init();

    // Initialize NVIC
    BSP_NVIC_Init();

    // Report free heap size
    free_heap_size = xPortGetFreeHeapSize();
    my_printf("Free Heap Size is %d bytes\r\n", free_heap_size);

    // Create the subscription queue
    xSubscribeQueue = xQueueCreate(10, sizeof(subscribe_message_t)); // Holding 10 messages

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
    typedef enum { SENSOR_INACTIVE, SENSOR_ACTIVE } sensor_state_t;
    sensor_state_t state = SENSOR_INACTIVE;

    subscribe_message_t msg;

    msg.sem_id = 1;
    msg.sensor_id = 1;
    msg.sensor_state = 1;

    while (1) {
        switch (state) {
            case SENSOR_INACTIVE:
                xQueueSendToBack(xSubscribeQueue, &msg, 0);
                my_printf("Task_1: Subscribed to Sensor 1, waiting for state=1.\r\n");

                if (xSemaphoreTake(xSem1, portMAX_DELAY) == pdTRUE) {
                    my_printf("Task_1: Semaphore taken, Sensor 1 active.\r\n");

                    // Realiza um flash curto
                    BSP_LED_On();
                    BSP_DELAY_ms(100);
                    BSP_LED_Off();
                    BSP_DELAY_ms(250);

                    // Atualiza o estado e a assinatura
                    state = SENSOR_ACTIVE;
                    msg.sensor_state = 0; // Próximo estado esperado
                }
                break;

            case SENSOR_ACTIVE:
                // Envia a assinatura para o estado inativo
                xQueueSendToBack(xSubscribeQueue, &msg, 0);
                my_printf("Task_1: Subscribed to Sensor 1, waiting for state=0.\r\n");

                // Aguarda o semáforo
                if (xSemaphoreTake(xSem1, portMAX_DELAY) == pdTRUE) {
                    my_printf("Task_1: Semaphore taken, Sensor 1 inactive.\r\n");

                    // Realiza dois flashes curtos
                    BSP_LED_On();
                    BSP_DELAY_ms(100);
                    BSP_LED_Off();
                    BSP_DELAY_ms(250);
                    BSP_LED_On();
                    BSP_DELAY_ms(100);
                    BSP_LED_Off();
                    BSP_DELAY_ms(250);

                    // Atualiza o estado e a assinatura
                    state = SENSOR_INACTIVE;
                    msg.sensor_state = 1; // Próximo estado esperado
                }
                break;

            default:
                state = SENSOR_INACTIVE;
                break;
        }
    }
}

/*
 * Task_2
 */
void vTask2(void *pvParameters) {
    typedef enum { SENSOR_INACTIVE, SENSOR_ACTIVE } sensor_state_t;
    sensor_state_t state = SENSOR_INACTIVE;

    subscribe_message_t msg;

    msg.sem_id = 2;
    msg.sensor_id = 2;
    msg.sensor_state = 1;

    while (1) {
        switch (state) {
            case SENSOR_INACTIVE:
                // Envia a assinatura para o estado ativo
                my_printf("Task_2: Subscribed to Sensor 2, waiting for state=1.\r\n");
                xQueueSendToBack(xSubscribeQueue, &msg, 0);

                // Aguarda o semáforo para o estado ativo
                if (xSemaphoreTake(xSem2, portMAX_DELAY) == pdTRUE) {
                    my_printf("Task_2: Semaphore taken, Sensor 2 active.\r\n");

                    // Realiza um flash longo
                    BSP_LED_On();
                    BSP_DELAY_ms(500);
                    BSP_LED_Off();
                    BSP_DELAY_ms(500);

                    // Atualiza o estado e a assinatura para o próximo ciclo
                    state = SENSOR_ACTIVE;
                    msg.sensor_state = 0; // Próximo estado esperado
                }
                break;

            case SENSOR_ACTIVE:
                // Envia a assinatura para o estado inativo
                my_printf("Task_2: Subscribed to Sensor 2, waiting for state=0.\r\n");
                xQueueSendToBack(xSubscribeQueue, &msg, 0);

                // Aguarda o semáforo para o estado inativo
                if (xSemaphoreTake(xSem2, portMAX_DELAY) == pdTRUE) {
                    my_printf("Task_2: Semaphore taken, Sensor 2 inactive.\r\n");

                    // Realiza dois flashes longos
                    BSP_LED_On();
                    BSP_DELAY_ms(500);
                    BSP_LED_Off();
                    BSP_DELAY_ms(500);
                    BSP_LED_On();
                    BSP_DELAY_ms(500);
                    BSP_LED_Off();
                    BSP_DELAY_ms(500);

                    // Atualiza o estado e a assinatura para o próximo ciclo
                    state = SENSOR_INACTIVE;
                    msg.sensor_state = 1; // Próximo estado esperado
                }
                break;

            default:
                state = SENSOR_INACTIVE;
                break;
        }
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
    char rx_byte;

    while (1) {
        // Process subscription queue
        if (xQueueReceive(xSubscribeQueue, &msg, 0)) {
            BSP_LED_Toggle(); // Toggle LED when a subscription is received
            my_printf("Subscribing : SemID=%d SensID=%d State=%d\r\n", msg.sem_id, msg.sensor_id, msg.sensor_state);
            updateSubs(subscription_table, &msg);
        }

        // Print the subscription table for debugging
        print_subscription_table(subscription_table);

        // Poll UART RX register for sensor state updates
        if ((USART2->ISR & USART_ISR_RXNE) == USART_ISR_RXNE) {
            rx_byte = USART2->RDR;
            my_printf("You've hit the '%c' key\r\n", rx_byte);

            // Update sensor states based on key input
            switch (rx_byte) {
                case 'a': sensor_states[0] = 0; BSP_LED_Toggle(); break; // Toggle LED on input
                case 'b': sensor_states[0] = 1; BSP_LED_Toggle(); break;
                case 'c': sensor_states[1] = 0; BSP_LED_Toggle(); break;
                case 'd': sensor_states[1] = 1; BSP_LED_Toggle(); break;
                default: my_printf("Unknown command: '%c'\r\n", rx_byte); break;
            }
        }

        // Display the current sensor states
        my_printf("Sensors state = [ %d %d ]\r\n", sensor_states[0], sensor_states[1]);

        // Check the subscription table and fulfill requests
        for (int i = 0; i < MAX_SUBSCRIBERS; i++) {
            if (subscription_table[i].sem_id != 0) {
                uint8_t sensor_id = subscription_table[i].sensor_id;
                uint8_t expected_state = subscription_table[i].sensor_state;

                if (sensor_states[sensor_id - 1] == expected_state) {
                    my_printf("Publishing: SemID=%d, SensID=%d, State=%d\r\n",
                              subscription_table[i].sem_id, sensor_id, expected_state);

                    if (subscription_table[i].sem_id == 1) {
                        xSemaphoreGive(xSem1);
                    } else if (subscription_table[i].sem_id == 2) {
                        xSemaphoreGive(xSem2);
                    }

                    BSP_LED_Toggle(); // Toggle LED when fulfilling a subscription

                    my_printf("Deleting subscription in slot [%d]\r\n", i);
                    subscription_table[i].sem_id = 0;
                    subscription_table[i].sensor_id = 0;
                    subscription_table[i].sensor_state = 0;
                }
            }
        }

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

/*
 * Update the subscription table
 */
static void updateSubs(subscribe_message_t *subs, subscribe_message_t *new_sub) {
    int i;

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
