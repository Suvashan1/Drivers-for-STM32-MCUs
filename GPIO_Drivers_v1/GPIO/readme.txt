/*

The following files form a part of a simple GPIO library written for STM32F407xx/STM32F405xx microcontrollers. To implement the following code, an
STM32F07VGTX-DISC1 development board had been used, along with the onboard ST-LINK(ST-LINK GDB Server) debugger and STM32CubeIDE. 
These drivers do not use any HAL (Hardware abstraction layer) of ST Microelectronics and were implemented entirely from ground up. 
Assistance for processor details, microcontroller-specific registers and peripherals were taken from the reference manual of the 
development board and, datasheet of the STM32F407xx/STM32F405xx. The library also includes interrupt handling.

The header and source files (stm32f407xx_GPIO_Drivers.h & stm32f407xx_GPIO_Drivers.c) may be placed in the "inc" and "src" folders of your 
projects in the IDE (Eclipse, STM32CubeIDE or Keil) respectively. The "stm32f407xx.h" file is a device-specific header file responsible for peripheral 
addresses, bus information and macros. This file must be included in the main source file, of your application. 

A simple example application, to toggle the onboard LED on the STM32F407VGTX discovery board using an external button connected. This application
utilizes a host of the available functionality of this library. 

To provide ease of use for this library, a doxygen HTML help file has been generated. The file clearly documents the code, in terms of macros, 
function defintions and usage, and parameters for those functions. 

This library has also been used to develop drivers for SPI, I2C and USART peripherals. These libraries will be added to the repository in due course 

*/