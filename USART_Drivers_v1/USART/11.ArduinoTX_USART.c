/*
 * 11.ArduinoTX_USART.c
 *
 *  Created on: 01 Oct 2020
 *      Author: Suvashan
 */


/* 
USART2 may not be used in this example code, if the STM32F407VG-DISC1 board is used, as that USART peripheral
is used by ST-LINK during debugging.
*/
   	

#include "stm32f407xx.h"
#include <stdio.h>
#include <string.h>

char message[1024] = "USART TX TESTING...\n\r";

USART_Handle_t USARTp;

void USART4_INIT(void){

	USARTp.pUSARTx = USART1;
	USARTp.USART_Config.USART_Baud = USART_STD_BAUD_115200;
	USARTp.USART_Config.USART_HWFlowControl = USART_HW_FLOW_CTRL_NONE;
	USARTp.USART_Config.USART_Mode = USART_MODE_ONLY_TX;
	USARTp.USART_Config.USART_ParityControl = USART_PARITY_DISABLE;
	USARTp.USART_Config.USART_StopBits = USART_STOPBITS_1;
	USARTp.USART_Config.USART_WordLength = USART_WORDLEN_8BITS;

	USART_init(&USARTp);

}

void delay(void){
	for(uint32_t i = 0; i < 500000/2; i++);
}

void GPIO_ButtonInit(void)
{
	GPIO_Handle_t GPIOBtn;

	//this is btn gpio configuration
	GPIOBtn.pGPIOx = GPIOA;
	GPIOBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	GPIOBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GPIOBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIOBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_Init(&GPIOBtn);

}

void USART_GPIO(void){

	GPIO_Handle_t USARTgpio ;
	USARTgpio.pGPIOx = GPIOB;
	USARTgpio.GPIO_PinConfig.GPIO_PinAltFunMode = 7;
	USARTgpio.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	USARTgpio.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	USARTgpio.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
	USARTgpio.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;

	USARTgpio.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;
	GPIO_Init(&USARTgpio);		//TX pin

	USARTgpio.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_7;
	GPIO_Init(&USARTgpio);		//RX pin


}




int main(void){

	GPIO_ButtonInit();

	USART_GPIO();

	USART4_INIT();

	USART_PeripheralControl(USART1, ENABLE);

	while(1){

	while(!(GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0)));

	delay();

	USART_SendData(&USARTp, (uint8_t*)message, strlen(message));

	}

	return 0;
}
