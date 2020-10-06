/*
 * stm32f407xx_USART_Drivers.h
 *
 *  Created on: 30 Sep 2020
 *      Author: Suvashan
 */

/**
 * @file stm32f407xx_USART_Drivers.h
 * @author Suvashan Pillay
 * @brief Header file for USART Drivers
 *
 */

/**
 * \mainpage File Description
 *
 * STM32F407xx/STM32F405xx are popular families of microcontrollers used in product development and manufacturing. The following
 * drivers provide an introductory insight to the microcontroller family, in terms of USART initialisation and usage. No
 * Hardware Abstraction Layer (HAL) has been used. The STM32F407VGTX DISC-1 board had been used for application
 * implementation, along with the ST-LINK (ST-LINK GDB SERVER) debugger. To use the following code, one has to also
 * utilize the stm32f407xx_GPIO_Drivers.h library for GPIO configuration of peripherals.
 *
 */

#include "stm32f407xx.h"

#ifndef INC_STM32F407XX_USART_DRIVERS_H_
#define INC_STM32F407XX_USART_DRIVERS_H_

/*
 *
 *@USART_Mode
 *Possible options for USART_Mode
 */

#define USART_MODE_ONLY_TX 		0			// Bit 3:2
#define USART_MODE_ONLY_RX 		1
#define USART_MODE_TXRX  		2

/*
 *@USART_Baud
 *Possible options for USART_Baud
 */
#define USART_STD_BAUD_1200					1200
#define USART_STD_BAUD_2400					2400
#define USART_STD_BAUD_9600					9600
#define USART_STD_BAUD_19200 				19200
#define USART_STD_BAUD_38400 				38400
#define USART_STD_BAUD_57600 				57600
#define USART_STD_BAUD_115200 				115200
#define USART_STD_BAUD_230400 				230400
#define USART_STD_BAUD_460800 				460800
#define USART_STD_BAUD_921600 				921600
#define USART_STD_BAUD_2M 					2000000
#define USART_STD_BAUD_3M 					3000000

/*
 *@USART_NoOfStopBits
 *Possible options for USART_NoOfStopBits
 */
#define USART_STOPBITS_1     	0
#define USART_STOPBITS_0_5   	1
#define USART_STOPBITS_2     	2
#define USART_STOPBITS_1_5   	3

/*
 *@USART_WordLength
 *Possible options for USART_WordLength
 */
#define USART_WORDLEN_8BITS  	0
#define USART_WORDLEN_9BITS  	1

/*
 *@USART_ParityControl
 *Possible options for USART_ParityControl
 */
#define USART_PARITY_EN_ODD   	2				//bit 10:9 to en/di and to select even/odd
#define USART_PARITY_EN_EVEN  	1
#define USART_PARITY_DISABLE  	0

/*
 *@USART_HWFlowControl
 *Possible options for USART_HWFlowControl
 */
#define USART_HW_FLOW_CTRL_NONE    		0				//bit 9:8
#define USART_HW_FLOW_CTRL_CTS    		1
#define USART_HW_FLOW_CTRL_RTS    		2
#define USART_HW_FLOW_CTRL_CTS_RTS		3

/////////////////////////////////////////////////////

#define USART_BUSY_IN_RX 1
#define USART_BUSY_IN_TX 2
#define USART_READY 0

#define 	USART_EVENT_TX_CMPLT   	0
#define		USART_EVENT_RX_CMPLT   	1
#define		USART_EVENT_IDLE      	2
#define		USART_EVENT_CTS       	3
#define		USART_EVENT_PE        	4
#define		USART_ERR_FE     		5
#define		USART_ERR_NE    	 	6
#define		USART_ERR_ORE    		7

//flags
#define USART_TXE_FLAG      (1 << USART_SR_TXE)
#define USART_RXNE_FLAG		(1 << USART_SR_RXNE)
#define USART_IDLE_FLAG		(1 << USART_SR_IDLE)
#define USART_TC_FLAG		(1 << USART_SR_TC)
#define USART_CTS_FLAG		(1 << USART_SR_CTS)

/**
 *\section
 *
 * @brief Configuration parameters for USARTx , x = 1,2,3,4,5
 */

typedef struct{

	uint8_t USART_Mode;					/**<Mode for chosen USARTx peripheral */
	uint32_t USART_Baud;				/**<desired baud rate for USARTx peripheral */
	uint8_t USART_StopBits;				/**<Number of stop bits to be used */
	uint8_t USART_WordLength;			/**<Desired word length of received or transmitted data */
	uint8_t USART_ParityControl;		/**<Enable/Disable Parity bit*/
	uint8_t USART_HWFlowControl;		/**<Use of CTS (Clear to send) or RTS (Request to send) */

}USART_Config_t;

/**
 * @brief USART Handle responsible for storing USART pointer and configuration structure
 * for the respective USARTx peripheral chosen.
 */

typedef struct{

	USART_Config_t USART_Config;		/**<structure holding USARTx configuration settings */
	USART_RegDef_t *pUSARTx;			/**<pointer for USARTx base address */
	uint8_t *pTxBuffer;					/**<pointer to location holding data to be transmitted on interrupt*/
	uint8_t *pRxBuffer;					/**<pointer to location holding received data on interrupt*/
	uint32_t TxLen;						/**<length of data to be transmitted on interrupt*/
	uint32_t RxLen;						/**<length of received data on interrupt*/
	uint8_t TxBusyState;				/**<state of interrupt handler when transmitting data*/
	uint8_t RxBusyState;				/**<state of interrupt handler when receiving data*/

}USART_Handle_t;

/**
 *  @brief Enable the peripheral clock on the desired USARTx peripheral
 *
 * @param pUSARTx pointer to the desired port (USARTx: x = 1,2,3,4,5)
 * @param EnorDi ENABLE/DISABLE
 *
 */
void USART_PeriClockControl(USART_RegDef_t *pUSARTx, uint8_t EnorDi);

/**
 * @brief Initialises the configuration of the the USARTx peripheral chosen
 * @param pUSARTHandle address of the pointer assigned to the USART handle
 */
void USART_init(USART_Handle_t *pUSARTHandle);

/**
 * @brief Clears the respective port, allowing reuse
 * @param pUSARTx pointer to the desired port
 */
void USART_DeInit(USART_Handle_t *pUSARTHandle);

/**
 * @brief Enables the peripheral once it has been configured by the user
 * @param pUSARTx pointer to the desired port
 * @param EnorDi ENABLE/DISABLE
 */
void USART_PeripheralControl(USART_RegDef_t *pUSARTx, uint8_t EnorDi);

/**
 * @brief Sends data, using pUSARTHandle configurations
 *
 * @param pUSARTHandle address of the pointer assigned to the USART handle
 * @param pTxBuffer pointer to memory address of data to be sent
 * @param len length of data to be sent
 */
void USART_SendData(USART_Handle_t *pUSARTHandle,uint8_t *pTxBuffer, uint32_t len);

/**
 * @brief Receives data, using pUSARTHandle configurations
 *
 * @param pUSARTHandle address of the pointer assigned to the USART handle
 * @param pRxBuffer pointer to memory address of data, where received data will be stored
 * @param len length of data to be received
 */
void USART_ReceiveData(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t len);

/**
 * @brief Sends data on interrupt, using pUSARTHandle configurations
 *
 * @param pUSARTHandle address of the pointer assigned to the USART handle
 * @param pTxBuffer pointer to memory address of data to be sent
 * @param len length of data to be sent
 */
uint8_t USART_SendDataIT(USART_Handle_t *pUSARTHandle,uint8_t *pTxBuffer, uint32_t len);

/**
 * @brief Receives data on interrupt, using pUSARTHandle configurations
 *
 * @param pUSARTHandle address of the pointer assigned to the USART handle
 * @param pRxBuffer pointer to memory address of data, where received data will be stored
 * @param len length of data to be received
 */
uint8_t USART_ReceiveDataIT(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t len);

/**
 * @brief Enables a designated interrupt for the desired USARTx periphal to be used.
 *
 * @param IRQNumber	External interrupt identifier (to be found in device header file)
 * @param EnorDi ENABLE/DISABLE
 */
void USART_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);

/**
 * @brief Enables corresponding IRQ for the NVIC
 *
 * @param IRQNumber	External interrupt identifier (to be found in device header file)
 * @param IRQPriority Corresponding NVIC number for the External interrupt identfier (0 having the highest priority)
 */
void USART_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);

/**
 * @brief Enables corresponding ISR on the given USARTx peripheral
 *
 * @param pUSARTHandle address of the pointer assigned to the USART handle
 * @param IRQNumber	External interrupt identifier (to be found in device header file)
 */
void USART_IRQHandling(USART_Handle_t *pUSARTHandle);


#endif /* INC_STM32F407XX_USART_DRIVERS_H_ */
