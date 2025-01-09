/*
 * bsp.h
 *
 */

#ifndef INC_BSP_H_
#define INC_BSP_H_

#include "stm32f0xx.h"
#include "FreeRTOSConfig.h"


/*
 * LED driver functions
 */

void	BSP_LED_Init	(void);
void	BSP_LED_On		(void);
void	BSP_LED_Off		(void);
void	BSP_LED_Toggle	(void);

/*
 * Push-Button driver functions
 */

void       BSP_PB_Init		(void);
uint8_t    BSP_PB_GetState	(void);

/*
 * Debug Console init
 */

void	BSP_Console_Init	(void);

/*
 * NVIC functions
 */

void BSP_NVIC_Init(void);

/*
 * Software counting delays
 */

void BSP_DELAY_ms	(uint32_t delay);
void BSP_DELAY_us	(uint32_t delay);

#endif /* INC_BSP_H_ */
