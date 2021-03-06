
/*
 * stm32f407xx_GPIO_Drivers.h
 *
 *  Created on: Sep 1, 2020
 *      Author: Suvashan Pillay
 */

/**
 * @file stm32f407xx_GPIO_Drivers.h
 * @author Suvashan Pillay
 * @brief Header file for GPIO Drivers
 *
 */

/**
 * \mainpage File Description
 *
 * STM32F407xx/STM32F405xx is a popular microcontroller used in product development and manufacturing. The following
 * drivers provide an introductory insight to the microcontroller, in terms of GPIO initialisation and usage. No
 * Hardware Abstraction Layer (HAL) has been used. The STM32F407VGTX DISC-1 board had been used for application
 * implementation, along with the ST-LINK (ST-LINK GDB SERVER) debugger.
 *
 */



#ifndef STM32F407XX_GPIO_DRIVERS_H_
#define STM32F407XX_GPIO_DRIVERS_H_

////////////////////Uncomment the include statement below, before implementing an application////////

//#include "stm32f407xx.h"

/**
 *\section
 *
 * @brief Configuration parameters for GPIOx Port, x = A...I,J,K
 */

typedef struct
{
	uint8_t GPIO_PinNumber;			/**< GPIOx pin number required for port. x = 1..15*/
	uint8_t GPIO_PinMode;			/**< GPIOx pin Mode required */
	uint8_t GPIO_PinSpeed;			/**< GPIOx pin Speed required */
	uint8_t GPIO_PinPuPdControl;	/**< GPIOx pin pull-up or pull-down resistor required */
	uint8_t GPIO_PinOPType;			/**< GPIOx pin output type required */
	uint8_t GPIO_PinAltFunMode;		/**< GPIOx pin alternate function mode required */
}GPIO_PinConfig_t;


/**
 * @brief GPIO Handle responsible for storing GPIO pointer and configuration structure
 * for the respective GPIO.
 */
typedef struct
{
	GPIO_RegDef_t *pGPIOx;       		/**<pointer for GPIOx base address */
	GPIO_PinConfig_t GPIO_PinConfig;   /**<structure holding GPIOx pin configuration settings */

}GPIO_Handle_t;



#define GPIO_PIN_NO_0  		0
#define GPIO_PIN_NO_1  		1
#define GPIO_PIN_NO_2  		2
#define GPIO_PIN_NO_3 		3
#define GPIO_PIN_NO_4  		4
#define GPIO_PIN_NO_5  		5
#define GPIO_PIN_NO_6  		6
#define GPIO_PIN_NO_7  		7
#define GPIO_PIN_NO_8  		8
#define GPIO_PIN_NO_9  		9
#define GPIO_PIN_NO_10  	10
#define GPIO_PIN_NO_11 		11
#define GPIO_PIN_NO_12  	12
#define GPIO_PIN_NO_13 		13
#define GPIO_PIN_NO_14 		14
#define GPIO_PIN_NO_15 		15


#define GPIO_MODE_IN 		0
#define GPIO_MODE_OUT 		1
#define GPIO_MODE_ALTFN 	2
#define GPIO_MODE_ANALOG 	3
#define GPIO_MODE_IT_FT     4
#define GPIO_MODE_IT_RT     5
#define GPIO_MODE_IT_RFT    6


#define GPIO_OP_TYPE_PP   	0
#define GPIO_OP_TYPE_OD   	1


#define GPIO_SPEED_LOW		0
#define GPIO_SPEED_MEDIUM	1
#define GPIO_SPEED_FAST		2
#define GPIO_SPEED_HIGH		3


#define GPIO_NO_PUPD   		0
#define GPIO_PIN_PU			1
#define GPIO_PIN_PD			2

#define IRQ_NO_EXTI0 		6
#define IRQ_NO_EXTI1 		7
#define IRQ_NO_EXTI2 		8
#define IRQ_NO_EXTI3 		9
#define IRQ_NO_EXTI4 		10
#define IRQ_NO_EXTI9_5 		23
#define IRQ_NO_EXTI15_10 	40
#define IRQ_NO_SPI1			35
#define IRQ_NO_SPI2         36
#define IRQ_NO_SPI3         51
#define IRQ_NO_I2C1_EV     	31
#define IRQ_NO_I2C1_ER     	32
#define IRQ_NO_I2C2_EV		33
#define IRQ_NO_I2C2_ER		34
#define IRQ_NO_I2C3_EV		72
#define IRQ_NO_I2C3_ER		73
#define IRQ_NO_USART1	    37
#define IRQ_NO_USART2	    38
#define IRQ_NO_USART3	    39
#define IRQ_NO_UART4	    52
#define IRQ_NO_UART5	    53
#define IRQ_NO_USART6	    71

#define NVIC_IRQ_PRI0		0
#define NVIC_IRQ_PRI1		1
#define NVIC_IRQ_PRI2		2
#define NVIC_IRQ_PRI3		3
#define NVIC_IRQ_PRI4		4
#define NVIC_IRQ_PRI5		5
#define NVIC_IRQ_PRI6		6
#define NVIC_IRQ_PRI7		7
#define NVIC_IRQ_PRI8		8
#define NVIC_IRQ_PRI9		9
#define NVIC_IRQ_PRI10		10
#define NVIC_IRQ_PRI11		11
#define NVIC_IRQ_PRI12		12
#define NVIC_IRQ_PRI13		13
#define NVIC_IRQ_PRI14		14
#define NVIC_IRQ_PRI15		15

/**
 *  @brief Enable the peripheral clock on the desired port
 *
 * @param pGPIOx pointer to the desired port (GPIOx: x = A..I,J,K)
 * @param EnorDi ENABLE/DISABLE
 *
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi);

/**
 * @brief Initialises the configuration of the the port and pin and its properties
 * @param pGPIOHandle address of the pointer assigned to the handle
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);

/**
 * @brief Clears the respective port, allowing reuse
 * @param pGPIOx pointer to the desired port
 */
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

/**
 *  @brief Read the binary value from a GPIO pin
 *
 * @param pGPIOx pointer to the desired port
 * @param PinNumber desired pin number to be read
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

/** @brief Read the binary value from a GPIO port
 *
 * @param pGPIOx pointer to the desired port
 */
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);

/**
 * @brief Writes a binary value to a GPIO pin
 *
 * @param pGPIOx pointer to the desired port
 * @param PinNumber desired pin to be written
 * @param Value GPIO_PIN_SET/GPIO_PIN_RESET
 */
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value);

/**
 * @brief Writes a binary value to a GPIO PORT
 *
 * @param pGPIOx pointer to the desired port
 * @param Value GPIO_PIN_SET/GPIO_PIN_RESET
 */
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value);

/**
 * @brief Toggles an output pin
 *
 * @param pGPIOx pointer to the desired port
 * @param PinNumber desired pin to be toggled
 */
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

/**
 * @brief Enables a designated interrupt for the respective pin of a GPIOx port
 *
 * @param IRQNumber	External interrupt identifier (to be found in device header file)
 * @param EnorDi ENABLE/DISABLE
 */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);

/**
 * @brief Enables corresponding IRQ for the NVIC
 *
 * @param IRQNumber	External interrupt identifier (to be found in device header file)
 * @param IRQPriority Corresponding NVIC number for the External interrupt identfier
 */
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);

/**
 * @brief Enables a designated interrupt for the respective pin of a GPIOx port
 *
 * @param PinNumber	PinNumber's interrupt to be cleared
 *
 */
void GPIO_IRQHandling(uint8_t PinNumber);

#endif /* STM32F407XX_GPIO_DRIVERS_H_ */

