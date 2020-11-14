/*
 * stm32f407xx_RCC_Driver.c
 *
 *  Created on: 01 Oct 2020
 *      Author: Suvashan
 */

#include "stm32f407xx.h"

uint16_t AHB_Prescaler[8] = {2, 4, 8, 16, 32, 64, 128, 256};
uint16_t APB1_Prescaler[4] = {2, 4, 8, 16};

uint32_t RCC_GetPClk(void){

	uint8_t clksource, temp, ahbp, apb1p;
	uint32_t pclk, systemclk;

	//Read the contents of the CFGR register (bit 2 and 3) of the RCC, to determine the clock, and mask other bits
	clksource = (RCC -> CFGR >> 2) & 0x3;

	if(clksource == 0){
		systemclk = 16000000;
	}
	else if (clksource == 1){
		systemclk = 8000000;
	}
	else if (clksource == 2){
		systemclk = RCC_GetPLLClock();
	}

	//find AHB prescaler, by reading bits 7:4 in CFGR register
	temp = ((RCC -> CFGR >> 4) & 0xF);		//temp may be 8,9,10,11,12,13,14,15
	if (temp < 8){
		ahbp = 1;
	}else
	{
		ahbp = AHB_Prescaler[temp-8];
	}

	//find APB1 prescaler, by reading 12:10 in CFGR register
	temp = ((RCC -> CFGR >> 10) & 0x7);
	if (temp < 4){
		apb1p = 1;

	}else {

		apb1p = APB1_Prescaler[temp - 4];
	}

	pclk = (systemclk/ahbp)/apb1p;
	return pclk;
}

uint32_t RCC_GetPClk2(void){

	uint32_t SystemClock=0,tmp,pclk2;
	uint8_t clk_src = ( RCC->CFGR >> 2) & 0X3;

		uint8_t ahbp,apb2p;

		if(clk_src == 0)
		{
			SystemClock = 16000000;
		}else
		{
			SystemClock = 8000000;
		}
		tmp = (RCC->CFGR >> 4 ) & 0xF;

		if(tmp < 0x08)
		{
			ahbp = 1;
		}else
		{
	       ahbp = AHB_Prescaler[tmp-8];
		}

		tmp = (RCC->CFGR >> 13 ) & 0x7;
		if(tmp < 0x04)
		{
			apb2p = 1;
		}else
		{
			apb2p = APB1_Prescaler[tmp-4];
		}

		pclk2 = (SystemClock / ahbp )/ apb2p;

		return pclk2;

}

uint32_t  RCC_GetPLLClock()
{

	return 0;
}
