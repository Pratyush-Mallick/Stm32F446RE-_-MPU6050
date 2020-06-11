/*
 * spi_txonly_arduino.c
 *
 *  Created on: 27-Apr-2020
 *      Author: Asus
 */


#include "stm32f446xx.h"
#include <string.h>

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
	SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	// Selecting the SPI Pins
	// NSS
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GPIO_Init(&SPIPins);

	// SCLK
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GPIO_Init(&SPIPins);

	// MISO
	//SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
	//GPIO_Init(&SPIPins);

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

int main(void)
{
	char user_data[] = "Hello World";

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
		while(GPIO_ReadFromInputPin(GPIOC, 13));

		delay();

		//This Function is used to initialize SSI bit in SPI_CR1 register
		//SPI_SSIConfig(SPI2, ENABLE);

		//Enable the SPI2 peripheral
		SPI_PeripheralControl(SPI2, ENABLE);

		// First send the data length to the slave
		uint8_t data_len = strlen(user_data);
		SPI_SendData(SPI2, &data_len, 1);

		// Send DATA.
		SPI_SendData(SPI2,(uint8_t*)user_data, data_len);

		// Wait till the SPI is not busy
		while(SPI_GetFlagStatus(SPI2, SPI_BUSY_FLAG));

		//Disable the SPI2 peripheral
		SPI_PeripheralControl(SPI2, DISABLE);
	}

	return 0;
}


