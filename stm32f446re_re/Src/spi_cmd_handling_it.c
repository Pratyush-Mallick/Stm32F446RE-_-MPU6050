/*
 * spi_cmd_handling_it.c
 *
 *  Created on: 29-Apr-2020
 *      Author: Asus
 */


#include "stm32f446xx.h"
#include <string.h>
#include <stdio.h>

SPI_Handle_t SPI2handle;


uint8_t RcvBuff[100];

uint8_t ReadByte;

uint8_t RxContFlag = RESET;

extern void initialise_monitor_handles();

//command codes
#define COMMAND_LED_CTRL      		0x50
#define COMMAND_SENSOR_READ      	0x51
#define COMMAND_LED_READ      		0x52
#define COMMAND_PRINT      			0x53
#define COMMAND_ID_READ      		0x54

#define LED_ON     1
#define LED_OFF    0

//arduino analog pins
#define ANALOG_PIN0 	0
#define ANALOG_PIN1 	1
#define ANALOG_PIN2 	2
#define ANALOG_PIN3 	3
#define ANALOG_PIN4 	4

//arduino led

#define LED_PIN  13

void delay(void)
{
	for(uint32_t i = 0 ; i < 500000/2 ; i ++);
}

/*
 *  PB 12 ---> SPI2_NSS
 *  PB 13 ---> SPI2_SCK
 *  PB 14 ---> SPI2_MISO
 *  PB 15 ---> SPI2_MOSI
 *
 *  Alternate Functionality Mode - 15
 *  AF15
 */

void SPI2_GPIOInit()
{
	GPIO_Handle_t SPIPins;   // GPIO Pins for SPI handle

	// Port and Pin Configurations
	SPIPins.pGPIOx = GPIOB;
	SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
	SPIPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	// Selecting the SPI Pins
	// NSS
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GPIO_Init(&SPIPins);

	// SCLK
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GPIO_Init(&SPIPins);

	// MISO
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
	GPIO_Init(&SPIPins);

	// MOSI
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
	GPIO_Init(&SPIPins);
}

void SPI2_Init()
{
	SPI_Handle_t pSPI2Handle;

	pSPI2Handle.pSPIx = SPI2;
	pSPI2Handle.SPI_Config.SPI_DriverMode = SPI_DEVICE_MODE_MASTER;
	pSPI2Handle.SPI_Config.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	pSPI2Handle.SPI_Config.SPI_ClockSpeed = SPI_SCLK_SPEED_DIV8;   //fsclk is without prescaler is 16MHZ
	pSPI2Handle.SPI_Config.SPI_DFF = SPI_DFF_8BITS;
	pSPI2Handle.SPI_Config.SPI_CPHA = SPI_CPHA_LOW;
	pSPI2Handle.SPI_Config.SPI_CPOL = SPI_CPOL_LOW;
	pSPI2Handle.SPI_Config.SPI_SSM = SPI_SSM_DI; // Hardware Slave Management is enabled for NSS pin

	SPI_Init(&pSPI2Handle);

}

void GPIO_ButtonInit(void)
{
	GPIO_Handle_t GpioBtn;
	memset(&GpioBtn, 0 , sizeof(GpioBtn));

	GpioBtn.pGPIOx = GPIOC;
	GpioBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GpioBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GpioBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	//GpioBtn.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_;
	GpioBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;

	GPIO_Init(&GpioBtn);

}

uint8_t SPI_VerifyResponse(uint8_t ackbyte)
{
	if( ackbyte == 0xF5)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}
int main(void)
{
	GPIO_ButtonInit();

	//this function is used to initialize the GPIO pins to behave as SPI2 pins
	SPI2_GPIOInit();

	//This function is used to initialize the SPI2 peripheral parameters
	SPI2_Init();

	/*
	* making SSOE 1 does NSS output enable.
	* The NSS pin is automatically managed by the hardware.
	* i.e when SPE=1 , NSS will be pulled to low
	* and NSS pin will be high when SPE=0
	*/
	SPI_SSOEConfig(SPI2,ENABLE);

	SPI_IRQInterruptConfig(IRQ_NO_SPI2,ENABLE);

	//wait till button is pressed
	while( GPIO_ReadFromInputPin(GPIOC,GPIO_PIN_NO_13) );

	//to avoid button de-bouncing related issues 200ms of delay
	delay();


	//enable the SPI2 peripheral
	SPI_PeripheralControl(SPI2,ENABLE);

	RxContFlag = SET;

	while(RxContFlag == SET)
	{
	   while ( ! (SPI_ReceiveDataIT(&SPI2handle,&ReadByte,1) == SPI_READY) );
	}


	//lets confirm SPI is not busy
	while( SPI_GetFlagStatus(SPI2,SPI_BUSY_FLAG) );

	//Disable the SPI2 peripheral
	SPI_PeripheralControl(SPI2,DISABLE);

	return 0;
}


void SPI2_IRQHandler(void)
{

	SPI_IRQHandling(&SPI2handle);
}



void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle,uint8_t AppEv)
{
	static uint32_t i =0;
	static uint8_t  rcv_start = 0;
	if(AppEv == SPI_EVENT_RX_CMPLT)
	{
		if(ReadByte == 0XF1)
		{
			rcv_start = 1;
		}else
		{
			if(rcv_start)
			{
				if(ReadByte == '\r')
				{
					RxContFlag = RESET;
					rcv_start =0;
					RcvBuff[i++] = ReadByte; //place the \r
					i=0;
				}else
				{
					RcvBuff[i++] = ReadByte;

				}
			}
		}


	}

}


