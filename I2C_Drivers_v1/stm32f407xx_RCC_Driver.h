/*
 * stm32f407xx_RCC_Driver.h
 *
 *  Created on: 01 Oct 2020
 *      Author: Suvashan
 */

#ifndef INC_STM32F407XX_RCC_DRIVER_H_
#define INC_STM32F407XX_RCC_DRIVER_H_

#include "stm32f407xx.h"


uint32_t RCC_GetPClk(void);
uint32_t RCC_GetPClk2(void);
uint32_t  RCC_GetPLLClock(void);


#endif /* INC_STM32F407XX_RCC_DRIVER_H_ */
