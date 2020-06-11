/*
 * i2c_master_tx_testing.c
 *
 *  Created on: 01-Jun-2020
 *      Author: Asus
 */

#include "stm32f446xx.h"
#include <string.h>
#include <stdio.h>

#define MY_ADDR    0x61
#define SlaveAddr  0x68
I2C_Handle_t I2C1_Handle;

// Some data
uint8_t some_data[] = "We are testing I2C master tx\n ";

void delay(void)
{
	for(uint32_t i = 0 ; i < 500000/2 ; i ++);
}


/*
 *  PB 8 -> SCL
 *  PB 9 -> SDA
 *
 */
void I2C1_GPIOInit()
{
	GPIO_Handle_t I2CPin;

	// Port and Pin Configurations
	I2CPin.pGPIOx = GPIOB;
	I2CPin.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	I2CPin.GPIO_PinConfig.GPIO_PinAltFunMode = 4;
	I2CPin.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
	I2CPin.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	I2CPin.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	// SCL Config
	I2CPin.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_8;
	GPIO_Init(&I2CPin);

	// SDA Config
	I2CPin.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_9;
	GPIO_Init(&I2CPin);

}

void I2C1_Init()
{
	I2C1_Handle.pI2Cx = I2C1;

	I2C1_Handle.I2C_Config.I2C_ACKControl = I2C_ACK_ENABLE;
	I2C1_Handle.I2C_Config.I2C_DeviceAddress = MY_ADDR;
	I2C1_Handle.I2C_Config.I2C_FMDutyCycle = I2C_FM_DUTY_2;
	I2C1_Handle.I2C_Config.I2C_SCLSpeed = I2C_SCL_SPEED_SM;

	I2C_Init(&I2C1_Handle);
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


int main()
{
	//GPIO Initialization
	GPIO_ButtonInit();

    // I2C SCL and SDA Pin selection
	I2C1_GPIOInit();

	//Initializing the I2C1 configuration
	I2C1_Init();

	//Initialize Peripheral Control
	I2C_PeripheralControl(I2C1, ENABLE);



	while(1)
	{

	// Wait for button Press
	while(GPIO_ReadFromInputPin(GPIOC,GPIO_PIN_NO_13));

	// To avoid debouncing a delay of 200 ms
	delay();

	// Send Some Data to the slave
	I2C_MasterSendData(&I2C1_Handle,some_data,strlen((char*)some_data),SlaveAddr,0);

	}
}
