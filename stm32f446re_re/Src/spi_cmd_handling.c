/*
 * spi_cmd_handling.c
 *
 *  Created on: 28-Apr-2020
 *      Author: Asus
 */


/*
 * spi_txonly_arduino.c
 *
 *  Created on: 27-Apr-2020
 *      Author: Asus
 */


#include "stm32f446xx.h"
#include <string.h>
#include <stdio.h>

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
	uint8_t dummy_write = 0xff;
	uint8_t dummy_read;
    initialise_monitor_handles();
	//Button Press Configuration Function
	GPIO_ButtonInit();

	// This Function is used to initialize GPIO pins to behave as SPI Pins
	SPI2_GPIOInit();

	//This Function is used to initialize SPI peripherals
	SPI2_Init();

	/*
	 * Making SSOE 1 enables NSS Output
	 * The NSS pin is automatically managed by the hardware
	 * i.e SPI = 1, NSS will be pulled low
	 * and NSS pin will be high when SPE =0
	 */
	SPI_SSOEConfig(SPI2, ENABLE);

	while(1)
	{
		//wait till button is pressed
		while( GPIO_ReadFromInputPin(GPIOC,GPIO_PIN_NO_13) );

		//to avoid button de-bouncing related issues 200ms of delay
		delay();

		//enable the SPI2 peripheral
		SPI_PeripheralControl(SPI2,ENABLE);

	    //1. CMD_LED_CTRL  	<pin no(1)>     <value(1)>

		uint8_t commandcode = COMMAND_LED_CTRL;
		uint8_t ackbyte;
		uint8_t args[2];

		//send command
		SPI_SendData(SPI2,&commandcode,1);

		//do dummy read to clear off the RXNE
		SPI_ReceiveData(SPI2,&dummy_read,1);

		//Send some dummy bits (1 byte) fetch the response from the slave
		SPI_SendData(SPI2,&dummy_write,1);

		//read the ack byte received
		SPI_ReceiveData(SPI2,&ackbyte,1);

		if( SPI_VerifyResponse(ackbyte))
		{
			args[0] = LED_PIN;
			args[1] = LED_ON;

			//send arguments
			SPI_SendData(SPI2,args,2);
			printf("COMMAND_LED_CTRL Executed\n");
		}
		//end of COMMAND_LED_CTRL

		//2. CMD_SENOSR_READ   <analog pin number(1) >

		//wait till button is pressed
		while( GPIO_ReadFromInputPin(GPIOC,GPIO_PIN_NO_13) );

		//to avoid button de-bouncing related issues 200ms of delay
		delay();

		commandcode = COMMAND_SENSOR_READ;

		//send command
		SPI_SendData(SPI2,&commandcode,1);

		//do dummy read to clear off the RXNE
		SPI_ReceiveData(SPI2,&dummy_read,1);

		//Send some dummy byte to fetch the response from the slave
		SPI_SendData(SPI2,&dummy_write,1);

		//read the ack byte received
		SPI_ReceiveData(SPI2,&ackbyte,1);

		if( SPI_VerifyResponse(ackbyte))
		{
			args[0] = ANALOG_PIN0;

			//send arguments
			SPI_SendData(SPI2,args,1); //sending one byte of

			//do dummy read to clear off the RXNE
			SPI_ReceiveData(SPI2,&dummy_read,1);

			//insert some delay so that slave can ready with the data
			delay();

			//Send some dummy bits (1 byte) fetch the response from the slave
			SPI_SendData(SPI2,&dummy_write,1);

			uint8_t analog_read;
			SPI_ReceiveData(SPI2,&analog_read,1);
			printf("COMMAND_SENSOR_READ %d\n",analog_read);
		}

		//3.  CMD_LED_READ 	 <pin no(1) >

		//wait till button is pressed
		while( GPIO_ReadFromInputPin(GPIOC,GPIO_PIN_NO_13) );

		//to avoid button de-bouncing related issues 200ms of delay
		delay();

		commandcode = COMMAND_LED_READ;

		//send command
		SPI_SendData(SPI2,&commandcode,1);

		//do dummy read to clear off the RXNE
		SPI_ReceiveData(SPI2,&dummy_read,1);

		//Send some dummy byte to fetch the response from the slave
		SPI_SendData(SPI2,&dummy_write,1);

		//read the ack byte received
		SPI_ReceiveData(SPI2,&ackbyte,1);

		if( SPI_VerifyResponse(ackbyte))
		{
			args[0] = LED_PIN;

			//send arguments
			SPI_SendData(SPI2,args,1); //sending one byte of

			//do dummy read to clear off the RXNE
			SPI_ReceiveData(SPI2,&dummy_read,1);

			//insert some delay so that slave can ready with the data
			delay();

			//Send some dummy bits (1 byte) fetch the response from the slave
			SPI_SendData(SPI2,&dummy_write,1);

			uint8_t led_status;
			SPI_ReceiveData(SPI2,&led_status,1);
			printf("COMMAND_READ_LED %d\n",led_status);

		}

		//4. CMD_PRINT 		<len(2)>  <message(len) >

		//wait till button is pressed
		while( GPIO_ReadFromInputPin(GPIOC,GPIO_PIN_NO_13) );

		//to avoid button de-bouncing related issues 200ms of delay
		delay();

		commandcode = COMMAND_PRINT;

		//send command
		SPI_SendData(SPI2,&commandcode,1);

		//do dummy read to clear off the RXNE
		SPI_ReceiveData(SPI2,&dummy_read,1);

		//Send some dummy byte to fetch the response from the slave
		SPI_SendData(SPI2,&dummy_write,1);

		//read the ack byte received
		SPI_ReceiveData(SPI2,&ackbyte,1);

		uint8_t message[] = "Hello ! How are you ??";
		if( SPI_VerifyResponse(ackbyte))
		{
			args[0] = strlen((char*)message);

			//send arguments
			SPI_SendData(SPI2,args,1); //sending length

			//send message
			SPI_SendData(SPI2,message,args[0]);

			printf("COMMAND_PRINT Executed \n");

		}

		//5. CMD_ID_READ
		//wait till button is pressed
		while( GPIO_ReadFromInputPin(GPIOC,GPIO_PIN_NO_13) );

		//to avoid button de-bouncing related issues 200ms of delay
		delay();

		commandcode = COMMAND_ID_READ;

		//send command
		SPI_SendData(SPI2,&commandcode,1);

		//do dummy read to clear off the RXNE
		SPI_ReceiveData(SPI2,&dummy_read,1);

		//Send some dummy byte to fetch the response from the slave
		SPI_SendData(SPI2,&dummy_write,1);

		//read the ack byte received
		SPI_ReceiveData(SPI2,&ackbyte,1);

		uint8_t id[11];
		uint32_t i=0;
		if( SPI_VerifyResponse(ackbyte))
		{
			//read 10 bytes id from the slave
			for(  i = 0 ; i < 10 ; i++)
			{
				//send dummy byte to fetch data from slave
				SPI_SendData(SPI2,&dummy_write,1);
				SPI_ReceiveData(SPI2,&id[i],1);
			}

			id[11] = '\0';

			printf("COMMAND_ID : %s \n",id);

		}

		//lets confirm SPI is not busy
		while( SPI_GetFlagStatus(SPI2,SPI_BUSY_FLAG) );

		//Disable the SPI2 peripheral
		SPI_PeripheralControl(SPI2, DISABLE);
	}

	return 0;
}


