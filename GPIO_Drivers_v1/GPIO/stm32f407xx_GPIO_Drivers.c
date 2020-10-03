/*
 * stm32f407xx_GPIO_Drivers.c
 *
 *  Created on: Sep 1, 2020
 *      Author: Suvashan
 */


#include "stm32f407xx.h"


void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_EN();
		}else if (pGPIOx == GPIOB)
		{
			GPIOB_PCLK_EN();
		}else if (pGPIOx == GPIOC)
		{
			GPIOC_PCLK_EN();
		}else if (pGPIOx == GPIOD)
		{
			GPIOD_PCLK_EN();
		}else if (pGPIOx == GPIOE)
		{
			GPIOE_PCLK_EN();
		}else if (pGPIOx == GPIOF)
		{
			GPIOF_PCLK_EN();
		}else if (pGPIOx == GPIOG)
		{
			GPIOG_PCLK_EN();
		}else if (pGPIOx == GPIOH)
		{
			GPIOH_PCLK_EN();
		}else if (pGPIOx == GPIOI)
		{
			GPIOI_PCLK_EN();
		}
	}
	else
	{
		//TODO
	}
}

void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	 GPIO_PeriClockControl(pGPIOHandle->pGPIOx, ENABLE);
	 uint32_t temp=0;


	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{
		//the non interrupt mode
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber ) );
		pGPIOHandle->pGPIOx->MODER &= ~( 0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER |= temp; //setting
	}

	else{
		if (pGPIOHandle -> GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT){
			//falling edge
			EXTI -> FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);		//set bit in FTSR corresponding to pin number of interrupt
			EXTI -> RTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);		//make sure to clear same bit in RTSR

		}else if(pGPIOHandle -> GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT){
			//rising edge
			EXTI -> RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI -> FTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		}else if(pGPIOHandle -> GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT){		//rising and falling edge

			EXTI -> RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);		//both edges are set
			EXTI -> FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		}

		//Configure GPIO port for SYSCFG_EXTI Control register, helps identify the port that interrupt is on
		uint8_t temp1 = pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber / 4;
		uint8_t temp2 = pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber % 4;
		uint8_t PortCode = GPIO_BASEADDR_TO_CODE(pGPIOHandle -> pGPIOx);
		SYSCFG_PCLK_EN();
		SYSCFG -> EXTICR[temp1] |= PortCode << (temp2 * 4);

		//then enable exti interrupt delivery  using interrupt mask register (IMR)
		EXTI -> IMR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	}

	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << ( 2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) );
		pGPIOHandle->pGPIOx->OSPEEDR &= ~( 0x3 << ( 2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); //clearing
		pGPIOHandle->pGPIOx->OSPEEDR |= temp;

		//3. configure the pupd settings
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << ( 2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) );
		pGPIOHandle->pGPIOx->PUPDR &= ~( 0x3 << ( 2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); //clearing
		pGPIOHandle->pGPIOx->PUPDR |= temp;


		//4. configure the optype
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );
		pGPIOHandle->pGPIOx->OTYPER &= ~( 0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //clearing
		pGPIOHandle->pGPIOx->OTYPER |= temp;

		//5. configure the alt functionality
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN)
		{
			//configure the alt function registers.
			uint8_t temp1, temp2;

			temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
			temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber  % 8;
			pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF << ( 4 * temp2 ) ); //clearing
			pGPIOHandle->pGPIOx->AFR[temp1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << ( 4 * temp2 ) );
		}
}

void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
	if(pGPIOx == GPIOA)
	{
		GPIOA_REG_RESET();
	}else if (pGPIOx == GPIOB)
	{
		GPIOB_REG_RESET();
	}else if (pGPIOx == GPIOC)
	{
		GPIOC_REG_RESET();
	}else if (pGPIOx == GPIOD)
	{
		GPIOD_REG_RESET();
	}else if (pGPIOx == GPIOE)
	{
		GPIOE_REG_RESET();
	}else if (pGPIOx == GPIOF)
	{
		GPIOF_REG_RESET();
	}else if (pGPIOx == GPIOG)
	{
		GPIOG_REG_RESET();
	}else if (pGPIOx == GPIOH)
	{
		GPIOH_REG_RESET();
	}else if (pGPIOx == GPIOI)
	{
		GPIOI_REG_RESET();
	}

}

uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
   uint8_t value;

   value = (uint8_t )((pGPIOx->IDR  >> PinNumber) & 0x00000001 ) ;	//read from the IDR register of the peripheral
   //and mask other bits

   return value;
}

uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
	uint16_t value;

	value = (uint16_t)pGPIOx->IDR;

	return value;
}

void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value)
{

	if(Value == GPIO_PIN_SET)
	{
		//write 1 to the output data register at the bit field corresponding to the pin number
		pGPIOx->ODR |= ( 1 << PinNumber);
	}else
	{
		//write 0
		pGPIOx->ODR &= ~( 1 << PinNumber);
	}
}

void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value)
{
	pGPIOx->ODR  = Value;
}

void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	pGPIOx->ODR  ^= ( 1 << PinNumber);
}

void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority){

	//1. find IPR register
	uint8_t iprx = IRQNumber/4;
	uint8_t iprx_section = IRQNumber % 4;
	uint8_t shiftamount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASE_ADDR + iprx) |= (IRQPriority << shiftamount);

}


void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi){

	if (EnorDi == ENABLE){
		if (IRQNumber <= 31){
			//program ISER0 register
			*NVIC_ISER0 |= (1<< IRQNumber);
		}else if (IRQNumber > 31 && IRQNumber <64){
			//program ISER1 register
			*NVIC_ISER1 |= (1<< (IRQNumber % 32));

		}else if (IRQNumber >= 64 && IRQNumber < 96){
			//program ISER2 register
			*NVIC_ISER2 |= (1<< (IRQNumber % 64));
		}
	}
	else {
		if (IRQNumber <= 31){
			//program ICR0 register
			*NVIC_ICER0 |= (1<< IRQNumber);
		}else if (IRQNumber > 31 && IRQNumber < 64){
			//program ICER1 register
			*NVIC_ICER1 |= (1<< (IRQNumber % 32));
		}else if (IRQNumber >= 64 && IRQNumber < 96){
			//program ICER2 register
			*NVIC_ICER2 |= (1<< (IRQNumber % 64));
		}
	}
}
void GPIO_IRQHandling(uint8_t PinNumber){
	//clear pending register on the EXTI for the corresponding pin
	if(EXTI -> PR & (1 << PinNumber)){

		EXTI -> PR |= (1<<PinNumber);
	}

}
