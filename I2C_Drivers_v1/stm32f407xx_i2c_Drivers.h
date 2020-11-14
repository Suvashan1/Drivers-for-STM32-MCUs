

/**
 * @file stm32f407xx_i2c_Drivers.h
 * @author Suvashan Pillay
 * @brief Header file for I2C Drivers
 *
 */

/**
 * \mainpage File Description
 *
 * STM32F407xx/STM32F405xx are popular families of microcontrollers used in product development and manufacturing. The following
 * drivers provide an introductory insight to the microcontroller family, in terms of I2C initialisation and usage. No
 * Hardware Abstraction Layer (HAL) has been used. The STM32F407VGTX DISC-1 board had been used for application
 * implementation, along with the ST-LINK (ST-LINK GDB SERVER) debugger. To use the following code, one has to also
 * utilize the stm32f407xx_GPIO_Drivers.h library for GPIO configuration of peripherals.
 *
 */

#ifndef INC_STM32F407XX_I2C_DRIVERS_H_
#define INC_STM32F407XX_I2C_DRIVERS_H_

//#include "stm32f407xx.h"
//#include "stm32f407xx_RCC_Driver.h"


//Flags
#define I2C_SB_FLAG				(1 << I2C_SR1_SB)
#define I2C_TXE_FLAG 			(1 << I2C_SR1_TXE)
#define I2C_RXNE_FLAG			(1 << I2C_SR1_RXNE)
#define I2C_ADDR_FLAG			(1 << I2C_SR1_ADDR)
#define I2C_BTF_FLAG			(1 << I2C_SR1_BTF)
#define I2C_STOPF_FLAG			(1 << I2C_SR1_STOPF)
#define I2C_BERR_FLAG			(1 << I2C_SR1_BERR)
#define I2C_ARLO_FLAG			(1 << I2C_SR1_ARLO)
#define I2C_AF_FLAG				(1 << I2C_SR1_AF)
#define I2C_OVR_FLAG			(1 << I2C_SR1_OVR)

//Flag used to determine status of data transfer
#define I2C_READY		0
#define I2C_BUSY_IN_RX	1
#define I2C_BUSY_IN_TX  2

//Enabling/Disabling repeated starts of transmission/reception of data
#define I2C_SR_DISABLE		RESET
#define I2C_SR_ENABLE		SET

//Mode selection
#define I2C_SCL_SPEED_SM 	100000
#define I2C_SCL_SPEED_FM4K 	400000
#define I2C_SCL_SPEED_FM2K  200000

//Speed macros
#define I2C_SM		0				//standard mode
#define I2C_FM		1				//Fast mode

//ACK enable and disable
#define I2C_ACK_DISABLE		0
#define I2C_ACK_ENABLE		1

//FM Duty cycle for fast-mode
#define I2C_DUTYCYCLE2		0
#define I2C_DUTYCYCLE16_9	1

//I2C application event macros
#define I2C_EV_TX_COMPLETE		0
#define I2C_EV_RX_COMPLETE		1
#define I2C_EV_STOP				2

//I2C event error macros
#define I2C_ERROR_BERR  		3
#define I2C_ERROR_ARLO  		4
#define I2C_ERROR_AF    		5
#define I2C_ERROR_OVR  		 	6
#define I2C_ERROR_TIMEOUT 		7
#define I2C_EV_DATA_REQ			8
#define I2C_EV_DATA_RCV			9

/**
 *\section
 *
 * @brief Configuration parameters for pI2Cx , x = 1,2,3
 */

typedef struct{
	uint8_t	I2C_Device_Address;				/**<Address for configured device*/
	uint8_t	I2C_SCL_Speed;					/**<Clock speed*/
	uint8_t	I2C_ACK_Control;				/**<Enabling/Disabling ACK control*/
	uint8_t	I2C_FMDuty_Cycle;				/**<Duty cycle control if fast mode is chosen*/
}I2C_Config_t;

/**
 * @brief I2C Handle responsible for storing I2C pointer and configuration structure
 * for the respective I2Cx peripheral chosen.
 */
typedef struct{
	I2C_RegDef_t	*pI2Cx;					/**<pointer for I2Cx base address*/
	I2C_Config_t 	I2C_Config;				/**<configuration structure for I2Cx configuration settings*/
	uint8_t 		*pTxBuffer;				/**<buffer to hold data to be transmitted*/
	uint8_t 		*pRxBuffer;				/**<buffer to hold data when reception occurs*/
	uint32_t 		Txlen;					/**<length of data to be transmitted*/
	uint32_t 		Rxlen;					/**<length of data to be received*/
	uint8_t 		TxRxState;				/**<configuring Rx OR Tx state*/
	uint8_t 		DevAddr;				/**<configuration of device address when the host device is in interrupt mode*/
	uint32_t 		RxSize;					/**<configuring Rx OR Tx state*/
	uint8_t 		Sr;						/**<enabling/disabling a repeated start*/

}I2C_Handle_t;




/**
 * @brief Initialises the configuration of the the I2Cx peripheral chosen
 * @param pI2CHandle address of the pointer assigned to the I2C handle
 */
void I2C_init(I2C_Handle_t *pI2CHandle);
/**
 * @brief Clears the respective port, allowing reuse
 * @param pI2Cx pointer to the desired port
 */
void I2C_DeInit(I2C_RegDef_t *pI2Cx);
/**
 *  @brief Enable the peripheral clock on the desired I2Cx peripheral
 *
 * @param pI2Cx pointer to the desired port (I2Cx: x = 1,2,3)
 * @param EnorDi ENABLE/DISABLE
 *
 */
void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);
/**
 * @brief Enables the peripheral once it has been configured by the user
 * @param pI2Cx pointer to the desired port
 * @param EnorDi ENABLE/DISABLE
 */
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi);

/**
 * @brief Sends data, using pI2CHandle configurations
 *
 * @param pI2CHandle address of the pointer assigned to the I2C handle
 * @param pTxBuffer pointer to memory address of data to be sent
 * @param len length of data to be sent
 * @param SlaveAddress address of assigned slave device address
 * @param SR Enabling/Disabling repeated starts
 */
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t len, uint8_t SlaveAddress, uint8_t Sr);
/**
 * @brief Receives data, using pI2CHandle configurations
 *
 * @param pI2CHandle address of the pointer assigned to the I2C handle
 * @param pRxBuffer pointer to memory address where data will be stored after reception
 * @param len length of data to be received
 * @param SlaveAddress address of assigned slave device address
 * @param SR Enabling/Disabling repeated starts
 */
void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint8_t len, uint8_t SlaveAddress, uint8_t Sr);

/**
 * @brief Enables ACK control of the I2Cx configured
 * @param pI2Cx pointer to the desired port
 * @param EnorDi ENABLE/DISABLE
 */
void ManageACKing(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);

/**
 * @brief Enables a designated interrupt for the desired I2Cx periphal to be used.
 *
 * @param IRQNumber	External interrupt identifier (to be found in device header file)
 * @param EnorDi ENABLE/DISABLE
 */
void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
/**
 * @brief Enables corresponding IRQ for the NVIC
 *
 * @param IRQNumber	External interrupt identifier (to be found in device header file)
 * @param IRQPriority Corresponding NVIC number for the External interrupt identfier (0 having the highest priority)
 */
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
/**
 * @brief Enables corresponding event IRQ for the given I2C handle assigned
 *
 * @param pI2CHandle address of the pointer assigned to the I2C handle
 *
 */
void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle);
/**
 * @brief Enables corresponding error IRQ for the given I2C handle assigned
 *
 * @param pI2CHandle address of the pointer assigned to the I2C handle
 *
 */
void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle);
/**
 * @brief Sends data on interrupt, using pI2CHandle configurations
 *
 * @param pI2CHandle address of the pointer assigned to the I2C handle
 * @param pTxBuffer pointer to memory address of data to be sent
 * @param len length of data to be received
 * @param SlaveAddress address of assigned slave device address
 * @param SR Enabling/Disabling repeated starts
 *
 * @return I2C_BUSY_IN_TX or I2C_READY
 */
uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t len, uint8_t SlaveAddress, uint8_t Sr);
/**
 * @brief Receives data on interrupt, using pI2CHandle configurations
 *
 * @param pI2CHandle address of the pointer assigned to the I2C handle
 * @param pRxBuffer pointer to memory address where data will be stored after reception
 * @param len length of data to be received
 * @param SlaveAddress address of assigned slave device address
 * @param SR Enabling/Disabling repeated starts
 *
 * @return I2C_BUSY_IN_RX or I2C_READY
 */
uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint8_t len, uint8_t SlaveAddress, uint8_t Sr);
/**
 * @brief Discontinues reception of the data when an error occurs
 * @param pI2CHandle address of the pointer assigned to the I2C handle
 */
void I2C_CloseReceiveData(I2C_Handle_t *pI2CHandle);
/**
 * @brief Discontinues sending data when an error occurs
 * @param pI2CHandle address of the pointer assigned to the I2C handle
 */
void I2C_CloseSendData(I2C_Handle_t *pI2CHandle);
/**
 * @brief Clears the respective port, allowing reuse
 *
 * @param pI2Cx pointer to the desired port
 * @param data 8-bit data to be sent by slave device
 */
void I2C_SlaveSendData(I2C_RegDef_t *pI2Cx, uint8_t data);

/**
 * @brief Receives data, where the configured device is the slave
 *
 * @param pI2Cx pointer to the desired port
 *
 * @return The received data
 */
uint8_t I2C_SlaveReceiveData(I2C_RegDef_t *pI2Cx);
/**
 * @brief Starts the I2C transmission.
 *
 * @param pI2Cx pointer to the desired port
 *
 * @note This function should be used, as an event callback for errors
 *
 */
void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx);
/**
 * @brief Hard - Stops the I2C transmission.
 *
 * @param pI2Cx pointer to the desired port
 *
 * @note This function should be used, as an event callback for errors
 *
 */
void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx);

/**
 * @brief Allows the user to assign events which should occur for errors or internal I2C events.
 *
 * @param pI2Cx pointer to the desired port
 *
 */
void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle,uint8_t AppEv);
/**
 * @brief Retrieves the status of a flag that can be set in the status register for various events
 *
 * @param pI2Cx pointer to the desired port
 * @param FlagName Flag of the SR register
 *
 */
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx , uint32_t FlagName);
/**
 * @brief Disables the control bits for, error and event, interrupts
 *
 * @param pI2Cx pointer to the desired port
 * @param EnorDi ENABLE/DISABLE
 *
 */
void I2C_SlaveEnableDisableCallbackEvents(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);

#endif /* INC_STM32F407XX_I2C_DRIVERS_H_ */
