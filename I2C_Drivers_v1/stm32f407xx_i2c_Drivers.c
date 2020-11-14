/*
 * stm32f407xx_i2c_Drivers.c
 *
 *  Created on: 18 Sep 2020
 *      Author: Suvashan
 */


#include "stm32f407xx.h"

/* Calculation of the AHB prescaler, followed by calculation of APB1 prescaler, has to be done to find the clock value
according to clock tree */

void I2C_CloseReceiveData(I2C_Handle_t *pI2CHandle){
	//clear control bits of the interrupts, to prevent any RXNE and TXE events
	pI2CHandle -> pI2Cx -> CR2 &= ~(1 << I2C_CR2_ITBUFEN);
	pI2CHandle -> pI2Cx -> CR2 &= ~(1 << I2C_CR2_ITEVTEN);

	pI2CHandle -> TxRxState = I2C_READY;
	pI2CHandle -> pRxBuffer = NULL;
	pI2CHandle -> Rxlen = 0;
	pI2CHandle -> RxSize = 0;
	if (pI2CHandle -> I2C_Config.I2C_ACK_Control == I2C_ACK_ENABLE){
	ManageACKing(pI2CHandle -> pI2Cx, ENABLE);
	}
}

void I2C_CloseSendData(I2C_Handle_t *pI2CHandle){

	pI2CHandle -> pI2Cx -> CR2 &= ~(1 << I2C_CR2_ITBUFEN);
	pI2CHandle -> pI2Cx -> CR2 &= ~(1 << I2C_CR2_ITEVTEN);

	pI2CHandle -> TxRxState = I2C_READY;
	pI2CHandle -> pTxBuffer = NULL;
	pI2CHandle -> Txlen = 0;

}



void ManageACKing(I2C_RegDef_t *pI2Cx, uint8_t EnorDi){
	if (EnorDi == ENABLE){
		pI2Cx -> CR1 |= (1 << I2C_CR1_ACK);
	}
	else {
		pI2Cx -> CR1 &= ~(1 << I2C_CR1_ACK);
	}
}

static void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle){

	uint32_t dummyRead;
	//check device mode
	if (pI2CHandle -> pI2Cx -> SR2 & (1 << I2C_SR2_MSL)){ //indicates device is in master mode
		if(pI2CHandle -> TxRxState == I2C_BUSY_IN_RX){
			if(pI2CHandle -> RxSize == 1){								//this performs the same task as the receiveData API, for Rx length = 1
				//disable ACK
				ManageACKing(pI2CHandle -> pI2Cx, DISABLE);
				//clear ADDR flag (reading SR1 and SR2)
				dummyRead = pI2CHandle -> pI2Cx -> SR1;
				dummyRead = pI2CHandle -> pI2Cx -> SR2;
				(void)dummyRead;
			}
		}else
		{
			dummyRead = pI2CHandle -> pI2Cx -> SR1;
			dummyRead = pI2CHandle -> pI2Cx -> SR2;
			(void)dummyRead;
		}
	}else
	{
		//indicates device is in slave mode
		dummyRead = pI2CHandle -> pI2Cx -> SR1;
		dummyRead = pI2CHandle -> pI2Cx -> SR2;
		(void)dummyRead;

	}
}

static void I2C_SendSlaveAddrW(I2C_RegDef_t *pI2Cx, uint8_t slaveAddr){

	slaveAddr = slaveAddr << 1;			//Shift slave address by one bit TO accommadate r/w bit
	slaveAddr &= ~(1);					//the first bit must be cleared
	pI2Cx -> DR = slaveAddr;
}

static void I2C_SendSlaveAddrR(I2C_RegDef_t *pI2Cx, uint8_t slaveAddr){

	slaveAddr = slaveAddr << 1;			//Shift slave address by one bit TO accommadate r/w bit
	slaveAddr |= 1;						//the first bit must be set
	pI2Cx -> DR = slaveAddr;
}

void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx){ pI2Cx -> CR1 |= (1 << I2C_CR1_STOP); }

void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx){ pI2Cx -> CR1 |= (1 << I2C_CR1_START); }




uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx , uint32_t FlagName)
{
	if(pI2Cx->SR1 & FlagName)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}


void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pI2Cx->CR1 |= (1 << I2C_CR1_PE);
		//pI2cBaseAddress->CR1 |= I2C_CR1_PE_Bit_Mask;
	}else
	{
		pI2Cx->CR1 &= ~(1 << 0);
	}

}

void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{

	if(EnorDi == ENABLE)
	{
		if(pI2Cx == I2C1)
		{
			I2C1_PCLK_EN();
		}else if (pI2Cx == I2C2)
		{
			I2C2_PCLK_EN();
		}else if (pI2Cx == I2C3)
		{
			I2C3_PCLK_EN();
		}
	}
}

void I2C_init(I2C_Handle_t *pI2CHandle){

	I2C_PeriClockControl(pI2CHandle -> pI2Cx, ENABLE);

	uint32_t temp = 0;
	//CR1 register configurations
	temp  |= pI2CHandle -> I2C_Config.I2C_ACK_Control << 10;
	pI2CHandle -> pI2Cx -> CR1 = temp;

	//CR2 register configurations
	//configure FREQ field of CR2
	temp = 0;
	temp |= RCC_GetPClk()/ 1000000U; 			//the 6 bit integer should be assigned, not the integer value of the clock
	pI2CHandle -> pI2Cx -> CR2 = (temp & 0x3F);

	//store slave address in OAR1 register
	temp = 0;
	temp |= pI2CHandle -> I2C_Config.I2C_Device_Address << 1;
	temp |= (1 << 14);
	pI2CHandle -> pI2Cx -> OAR1 = temp;


	//configure CCR register, and mode configurations
	uint16_t ccr_value = 0;
	temp = 0;
	/*for standard mode, duty cycle is 50%, T(high) = T(low) of sclk.
			Tsclk = 2 * CCR * T(pclk)
			CCR = F(pclk)/ 2 * F(sclk)
			*/
	if (pI2CHandle -> I2C_Config. I2C_SCL_Speed <= I2C_SCL_SPEED_SM){

		ccr_value = (RCC_GetPClk() / (2 * pI2CHandle -> I2C_Config.I2C_SCL_Speed));
		temp |= (ccr_value & 0xFFF); //the first 12 bits must be masked as that forms the CCR value
	}
	else{
		//fast mode
		temp |= (1 << 15);
		temp |= (pI2CHandle->I2C_Config.I2C_FMDuty_Cycle << 14);
		if (pI2CHandle -> I2C_Config.I2C_FMDuty_Cycle == I2C_DUTYCYCLE2){

			ccr_value = (RCC_GetPClk() / (3 * pI2CHandle -> I2C_Config. I2C_SCL_Speed));
		}else{

			ccr_value = (RCC_GetPClk() / (25 * pI2CHandle -> I2C_Config. I2C_SCL_Speed));
		}
		temp |= (ccr_value & 0xFFF);	//the first 12 bits must be masked as that forms the CCR value
	}

	pI2CHandle -> pI2Cx -> CCR  = temp;

	//TRise calculation
	if(pI2CHandle -> I2C_Config.I2C_SCL_Speed <= I2C_SCL_SPEED_SM){

		temp = (RCC_GetPClk() / 1000000U) + 1;

	}else
	{
		temp = ((RCC_GetPClk() * 300) / 1000000U) + 1;
	}

	pI2CHandle -> pI2Cx -> TRISE = (temp & 0x3F);
}


void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t len, uint8_t SlaveAddress, uint8_t Sr){

	//1. Generate START condition
	I2C_GenerateStartCondition(pI2CHandle -> pI2Cx);

	//2.confirm start generation by checking SB flag in SR1. (If SB is still clear, SCL will be strtched low)
	while (!(I2C_GetFlagStatus(pI2CHandle -> pI2Cx, I2C_SB_FLAG)));

	//3. Send the address of slave with r/wn bit set to w(0)
	I2C_SendSlaveAddrW(pI2CHandle -> pI2Cx, SlaveAddress);

	//4. Confirm address is sent by checking ADDR in SR1 register
	while (!(I2C_GetFlagStatus(pI2CHandle -> pI2Cx, I2C_ADDR_FLAG)));

	//5. Clear ADDR flag
	I2C_ClearADDRFlag(pI2CHandle);

	//6.send data until len = 0 (data register is empty)
	while (len > 0){
		while (!(I2C_GetFlagStatus(pI2CHandle -> pI2Cx, I2C_TXE_FLAG)));
		pI2CHandle -> pI2Cx -> DR = *pTxBuffer;
		pTxBuffer++;
		len--;
	}

	while (!(I2C_GetFlagStatus(pI2CHandle -> pI2Cx, I2C_TXE_FLAG)));
	while (!(I2C_GetFlagStatus(pI2CHandle -> pI2Cx, I2C_BTF_FLAG)));

	if (Sr == I2C_SR_DISABLE){
	I2C_GenerateStopCondition(pI2CHandle -> pI2Cx);
	}
}

void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint8_t len, uint8_t SlaveAddress, uint8_t Sr){

	//1. Generate START condition
	I2C_GenerateStartCondition(pI2CHandle -> pI2Cx);

	//2. confirm start generation by checking if SB_Flag is set
	while (!(I2C_GetFlagStatus(pI2CHandle -> pI2Cx, I2C_SB_FLAG)));

	//3. send slave address with
	I2C_SendSlaveAddrR(pI2CHandle -> pI2Cx, SlaveAddress);

	//4. wait until ADDR flag is set to ensure address phase is complete
	while(!(I2C_GetFlagStatus(pI2CHandle -> pI2Cx, I2C_SR1_ADDR)));

	//5. if len = 1:
	if(len == 1){

		//disable ACKing
		ManageACKing(pI2CHandle -> pI2Cx, I2C_ACK_DISABLE);

		//clear ADDR flag
		I2C_ClearADDRFlag(pI2CHandle);

		//wait til RXNE becomes one
		while(!(I2C_GetFlagStatus(pI2CHandle -> pI2Cx, I2C_RXNE_FLAG)));

		if (Sr == I2C_SR_DISABLE){
			I2C_GenerateStopCondition(pI2CHandle -> pI2Cx);
			}

		*pRxBuffer = pI2CHandle -> pI2Cx -> DR;


	}

	if(len > 1){

		//clear ADDR flag to allow data reception
		I2C_ClearADDRFlag(pI2CHandle);

		for(uint32_t i = len; i > 0; i--){

			// wait until RXNE becomes 1
			while(!(I2C_GetFlagStatus(pI2CHandle -> pI2Cx, I2C_RXNE_FLAG)));

			if (len == 2){

				//clear ACKing
				ManageACKing(pI2CHandle -> pI2Cx, I2C_ACK_DISABLE);

				//generate STOP condition
				if (Sr == I2C_SR_DISABLE){
					I2C_GenerateStopCondition(pI2CHandle -> pI2Cx);
					}
			}
			//read the data into buffer
			*pRxBuffer = pI2CHandle -> pI2Cx -> DR;
			//increment buffer address
			pRxBuffer++;
		}
	}

	if(pI2CHandle -> I2C_Config. I2C_ACK_Control == I2C_ACK_ENABLE){

	ManageACKing(pI2CHandle -> pI2Cx, I2C_ACK_ENABLE);

	}

}


uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t len, uint8_t SlaveAddress, uint8_t Sr){

	uint8_t busystate = pI2CHandle-> TxRxState;

		if( (busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))
		{
			pI2CHandle->pTxBuffer = pTxBuffer;
			pI2CHandle->Txlen = len;
			pI2CHandle->TxRxState = I2C_BUSY_IN_TX;
			pI2CHandle->DevAddr = SlaveAddress;
			pI2CHandle->Sr = Sr;

			//Generate START Condition
			I2C_GenerateStartCondition(pI2CHandle -> pI2Cx);

			I2C_GetFlagStatus(pI2CHandle -> pI2Cx, I2C_SB_FLAG);


			//enable ITBUFEN Control Bit
			pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITBUFEN);

			//enable ITEVFEN Control Bit
			pI2CHandle -> pI2Cx -> CR2 |= (1 << I2C_CR2_ITEVTEN);

			//enable ITERREN Control Bit
			pI2CHandle -> pI2Cx -> CR2 |= (1 << I2C_CR2_ITERREN);

		}

		return busystate;

	}

uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint8_t len, uint8_t SlaveAddress, uint8_t Sr){

	uint8_t busystate = pI2CHandle->TxRxState;

		if( (busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))
		{
			pI2CHandle->pRxBuffer = pRxBuffer;
			pI2CHandle->Rxlen = len;
			pI2CHandle->TxRxState = I2C_BUSY_IN_RX;
			pI2CHandle->RxSize = len; //Rxsize is used in the ISR code to manage the data reception
			pI2CHandle->DevAddr = SlaveAddress;
			pI2CHandle->Sr = Sr;

			//Generate START Condition
			I2C_GenerateStartCondition(pI2CHandle -> pI2Cx);


			//enable ITBUFEN Control Bit
			pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITBUFEN);

			//enable ITEVFEN Control Bit
			pI2CHandle -> pI2Cx -> CR2 |= (1 << I2C_CR2_ITEVTEN);

			//enable ITERREN Control Bit
			pI2CHandle -> pI2Cx -> CR2 |= (1 << I2C_CR2_ITERREN);
		}

		return busystate;
}


void I2C_SlaveSendData(I2C_RegDef_t *pI2Cx, uint8_t data){
	pI2Cx -> DR = data;
}

uint8_t I2C_SlaveReceiveData(I2C_RegDef_t *pI2Cx){
	return (uint8_t)pI2Cx -> DR;
}

void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle)//interrupt handling for both master and slave mode of a device
{
	uint32_t temp1, temp2, temp3;

	//Below, the bits of the I2C CR2 and SR1 are being read. The following action allows the bits
	//to be read whether it is set or not. By ANDing the bit with the bit set itself, we can read
	// one or zero. 1&0 = 0, 1&1 = 1. This lets the temp variable read the value of the bit only since
	//I2C_CR2_ITEVFEN is the only bit set in that register, so all other bits are masked.

	temp1 = pI2CHandle -> pI2Cx -> CR2 & (1 << I2C_CR2_ITEVTEN);
	temp2 = pI2CHandle -> pI2Cx -> CR2 & (1 << I2C_CR2_ITBUFEN);


	//1. Handle for interrupt generated by SB event, SB FLAG is only applicable in Master mode
	temp3 = pI2CHandle -> pI2Cx -> SR1 & (1 << I2C_SR1_SB);
	if(temp1 && temp3){
																			//SB flag is set, so we may execute address phase, which we have read and write functions for.
		if (pI2CHandle -> TxRxState == I2C_BUSY_IN_TX){						//The device's state of busy in RX or TX from the, will determine reading ot writing slave address.

			I2C_SendSlaveAddrW(pI2CHandle -> pI2Cx, pI2CHandle -> DevAddr);

		}else if (pI2CHandle -> TxRxState == I2C_BUSY_IN_RX){

			I2C_SendSlaveAddrR(pI2CHandle -> pI2Cx, pI2CHandle -> DevAddr);
		}

	}


	//2.Handle for interrupt generated by ADDR event. This event for a master, is when the address has
	//been sent. This event for the slave, is when the address sent by the master matches its own address
	temp3 = pI2CHandle -> pI2Cx -> SR1 & (1 << I2C_SR1_ADDR);
	if(temp1 && temp3){//ADDR flag is set, clock is stretched. It's needed to clear ADDR flag

		I2C_ClearADDRFlag(pI2CHandle);

		}


	//3. Handle for interrupt generated by BTF (byte transfer finished) flag
	temp3 = pI2CHandle -> pI2Cx -> SR1 & (1 << I2C_SR1_BTF);
	//BTF flag is set. When BTF and TXE is set, shift reg and DR are empty. When BTF and RXNE is set, shift reg and DR are full. Any comms of data is completed when flags are set.
	//BTF flag can be used
	if(temp1 && temp3){

				if(pI2CHandle-> TxRxState == I2C_BUSY_IN_TX){

					if(pI2CHandle -> pI2Cx -> SR1 & (1 << I2C_SR1_TXE)){

						if (pI2CHandle -> Txlen == 0)
						{
						//We can close the data transmission, if repeated start isnt enabled
						if (pI2CHandle -> Sr == I2C_SR_DISABLE){
						I2C_GenerateStopCondition(pI2CHandle -> pI2Cx);
						}
						//reset member elements of handle structure
						I2C_CloseSendData(pI2CHandle);
						//notify application about transmission complete
						I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_TX_COMPLETE);
						}
					}
				}

				else if(pI2CHandle -> TxRxState == I2C_BUSY_IN_RX){

					;
				}
			}

	//4.Handle for interrupt generated by STOPF event
	temp3 = pI2CHandle -> pI2Cx -> SR1 & (1 << I2C_SR1_STOPF);	//this flag will only be set in when device is in slave mode
	if(temp1 && temp3){

					//when STOPF flag is set, according to ref manual, it must be cleared by reading SR1, and writing to CR1
				pI2CHandle -> pI2Cx -> CR1 |= 0x0000;

				//Notify application that STOP is detected
				I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_STOP);

				}

	//5.Handle for interrupt generated by TXE event
	temp3 = pI2CHandle -> pI2Cx -> SR1 & (1 << I2C_SR1_TXE);	//if this flag is set, it shows DR is empty and a byte must be received into the DR to transmit
	if(temp1 && temp2 && temp3)
	{								//basically, implement data transmission
		if(pI2CHandle -> pI2Cx -> SR2 & (1 << I2C_SR2_MSL))
		{	//only if device is in master mode as this is for MASTER-INTERRUPTS ONLY
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
		{
			if(pI2CHandle -> Txlen > 0)
			{
				//1. load data into DR
				pI2CHandle -> pI2Cx -> DR = *(pI2CHandle -> pTxBuffer);
				//2.decrement  TxLen
				pI2CHandle -> Txlen--;
				//3. increment buffer address
				pI2CHandle -> pTxBuffer++;
		  }
		}
	  }else{
		  //config for slave mode when TXE is set
		  if(pI2CHandle -> pI2Cx -> SR2 & (1<<I2C_SR2_TRA))			//if this flag is set, that means the slave is in transmitter mode
		  {
		  I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_DATA_REQ);
		  }
	  }

	}

	//6.Handle for interrupt generated by RXNE event
	temp3 = pI2CHandle -> pI2Cx -> SR1 & (1 << I2C_SR1_RXNE);
		if(temp1 && temp2 && temp3)
		{
			if(pI2CHandle -> pI2Cx -> SR2 & (1 << I2C_SR2_MSL))
			{		//need to check if device is in master mode, because this is for interrupt for the master only
				if(pI2CHandle -> TxRxState == I2C_BUSY_IN_RX)
				{
					if(pI2CHandle -> RxSize == 1){

						*pI2CHandle -> pRxBuffer = pI2CHandle -> pI2Cx -> DR;
						 pI2CHandle->Rxlen--;
					}
				if(pI2CHandle -> RxSize == 1){

					if(pI2CHandle -> Rxlen == 2){
						ManageACKing(pI2CHandle -> pI2Cx, DISABLE);
					}

					*pI2CHandle-> pRxBuffer = pI2CHandle-> pI2Cx -> DR;
					pI2CHandle -> pRxBuffer++;
					pI2CHandle->Rxlen--;

				}

				if(pI2CHandle->Rxlen == 0){
					//close data reception and notify application. generate stop condtion, close I2C-RX, notify application
					if(pI2CHandle-> Sr == I2C_SR_DISABLE)
						I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

					I2C_CloseReceiveData(pI2CHandle);
					I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_RX_COMPLETE);
				}
			}
		}
			else
			{
				if(!(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_TRA))){	//make sure device is a slave in receiver mode
					I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_DATA_RCV);
				}
			}
	}
}
void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle)
{

	uint32_t temp1,temp2;

    //Know the status of  ITERREN control bit in the CR2
	temp2 = (pI2CHandle->pI2Cx->CR2) & ( 1 << I2C_CR2_ITERREN);


	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1<< I2C_SR1_BERR);
	if(temp1  && temp2 )
	{
		//This is Bus error


		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_BERR);


	   I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_BERR);
	}


	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_ARLO );
	if(temp1  && temp2)
	{
		//This is arbitration lost error


		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_ARLO);


		I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_ARLO);
	}



	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_AF);
	if(temp1  && temp2)
	{
		//This is ACK failure error


		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_AF);


		I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_AF);
	}


	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_OVR);
	if(temp1  && temp2)
	{
		//This is Overrun/underrun


		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_OVR);


		I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_OVR);
	}


	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_TIMEOUT);
	if(temp1  && temp2)
	{
		//This is Time out error


		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_TIMEOUT);


		I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_TIMEOUT);
	}

}

void I2C_SlaveEnableDisableCallbackEvents(I2C_RegDef_t *pI2Cx, uint8_t EnorDi){

	if(EnorDi == ENABLE){
		pI2Cx -> CR2 |= (1 << I2C_CR2_ITEVTEN);
		pI2Cx -> CR2 |= (1 << I2C_CR2_ITBUFEN);
		pI2Cx -> CR2 |= (1 << I2C_CR2_ITERREN);
	}
	else{
		pI2Cx -> CR2 &= ~(1 << I2C_CR2_ITEVTEN);
		pI2Cx -> CR2 &= ~(1 << I2C_CR2_ITBUFEN);
		pI2Cx -> CR2 &= ~(1 << I2C_CR2_ITERREN);
	}

}

void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{

	if(EnorDi == ENABLE)
	{
		if(IRQNumber <= 31)
		{
			//program ISER0 register
			*NVIC_ISER0 |= ( 1 << IRQNumber );

		}else if(IRQNumber > 31 && IRQNumber < 64 ) //32 to 63
		{
			//program ISER1 register
			*NVIC_ISER1 |= ( 1 << (IRQNumber % 32) );
		}
		else if(IRQNumber >= 64 && IRQNumber < 96 )
		{
			//program ISER2 register //64 to 95
			*NVIC_ISER3 |= ( 1 << (IRQNumber % 64) );
		}
	}else
	{
		if(IRQNumber <= 31)
		{
			//program ICER0 register
			*NVIC_ICER0 |= ( 1 << IRQNumber );
		}else if(IRQNumber > 31 && IRQNumber < 64 )
		{
			//program ICER1 register
			*NVIC_ICER1 |= ( 1 << (IRQNumber % 32) );
		}
		else if(IRQNumber >= 6 && IRQNumber < 96 )
		{
			//program ICER2 register
			*NVIC_ICER3 |= ( 1 << (IRQNumber % 64) );
		}
	}

}

void I2C_IRQPriorityConfig(uint8_t IRQNumber,uint32_t IRQPriority)
{
	//1. first lets find out the ipr register
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section  = IRQNumber %4 ;

	uint8_t shift_amount = ( 8 * iprx_section) + ( 8 - NO_PR_BITS_IMPLEMENTED) ;

	*(  NVIC_PR_BASE_ADDR + iprx ) |=  ( IRQPriority << shift_amount );

}
