/*

The following files form a part of a simple USART library written for STM32F407xx/STM32F405xx microcontrollers. To implement the following code, an
STM32F07VGTX-DISC1 development board had been used, along with the onboard ST-LINK(ST-LINK GDB Server) debugger and STM32CubeIDE. 
These drivers do not use any HAL (Hardware abstraction layer) of ST Microelectronics and were implemented entirely from ground up. 
Assistance for processor details, microcontroller-specific registers and peripherals were taken from the reference manual of the 
development board and, datasheet of the STM32F407xx/STM32F405xx. The library also includes interrupt handling.

The header and source files (stm32f407xx_USART_Drivers.h & stm32f407xx_USART_Drivers.c) may be placed in the "inc" and "src" folders of your 
projects in the IDE (Eclipse, STM32CubeIDE or Keil) respectively. The "stm32f407xx.h" file is a device-specific header file responsible for peripheral 
addresses, bus information and macros. This file must be included in the main source file, of your application. The included RCC header and source files 
must also be placed in your created-project's "include" and "source" folders. These files are not for the user to apply in his/her application, but are used
for peripheral clock configurations, for the USART peripherals

A simple example application, involving transmission of data to an Arduino uno was used to test the non-interrupt routines and provides a guide for the user.
The aforementioned arduino file has been attached for the user.

To provide ease of use for this library, a doxygen HTML help file has been generated. The file clearly documents the code, in terms of macros, 
function defintions and usage, and parameters for those functions. 

This library has also been used to develop drivers for SPI, I2C peripherals. These libraries will be added to the repository in due course 

*/
