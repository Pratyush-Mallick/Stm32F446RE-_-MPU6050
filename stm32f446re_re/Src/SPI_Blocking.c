/*
 * SPI_Blocking.c
 *
 *  Created on: 25-Apr-2020
 *      Author: Asus
 */

#include "stm32f446xx.h"
#include <string.h>
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
	pSPI2Handle.SPI_Config.SPI_ClockSpeed = SPI_SCLK_SPEED_DIV2;
	pSPI2Handle.SPI_Config.SPI_DFF = SPI_DFF_8BITS;
	pSPI2Handle.SPI_Config.SPI_CPHA = SPI_CPHA_LOW;
	pSPI2Handle.SPI_Config.SPI_CPOL = SPI_CPOL_LOW;
	pSPI2Handle.SPI_Config.SPI_SSM = SPI_SSM_EN;

	SPI_Init(&pSPI2Handle);

}

int main(void)
{
  char user_data[] = "Hello World";

	// This Function is used to initialize GPIO pins to behave as SPI Pins
	SPI2_GPIOInit();

	//This Function is used to initialize SPI peripherals
	SPI2_Init();

	//This Function is used to initialize SSI bit in SPI_CR1 register
	SPI_SSIConfig(SPI2, ENABLE);

	//Enable the SPI2 peripheral
	SPI_PeripheralControl(SPI2, ENABLE);

    // Send DATA.
	while(1)
	{
	SPI_SendData(SPI2,(uint8_t*)user_data, strlen(user_data));
	}

	return 0;
}
