
/*
 * i2c_master_tx_testing.c
 *
 *  Created on: 01-Jun-2020
 *      Author: Asus
 */

#include "stm32f446xx.h"
#include <string.h>
#include <stdio.h>

#define SlaveAddr  0x69
#define MY_ADDR     SlaveAddr
I2C_Handle_t I2C1_Handle;

//Flag variable
uint8_t rxComplt = RESET;

// Some data
uint8_t tx_buffer[32] = "Slave Mode Testing..";



extern void initialise_monitor_handles(void);

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
	initialise_monitor_handles();

	//GPIO Initialization
	GPIO_ButtonInit();

    // I2C SCL and SDA Pin selection
	I2C1_GPIOInit();

	//Initializing the I2C1 configuration
	I2C1_Init();

	// I2C IRQ Configurations
	I2C_IRQInterruptConfig(IRQ_NO_I2C1_EV, ENABLE);
	I2C_IRQInterruptConfig(IRQ_NO_I2C1_ER, ENABLE);

	I2C_SlaveEnableDisableCallBackEvents(I2C1,ENABLE);

	//Initialize Peripheral Control
	I2C_PeripheralControl(I2C1, ENABLE);

	// Ack bit is enabled after PE = 1
	I2C_ManageAcking(I2C1_Handle.pI2Cx, I2C_ACK_ENABLE);


	while(1);

}


void I2C1_EV_IRQHandler(void)
{
	I2C_IRQHandling_EV(&I2C1_Handle);
}

void I2C1_ER_IRQHandler(void)
{
	I2C_IRQHandling_ER(&I2C1_Handle);
}

void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle,uint8_t AppEv)
{
	static uint8_t commandcode = 0;
	static uint8_t cnt = 0;
	if(AppEv == I2C_EV_DATA_REQ)
	{
		// Master wants some data slave has to send it
		if(commandcode == 0x51)
		{
			I2C_SlaveSendData(pI2CHandle->pI2Cx, strlen((char*)tx_buffer));
		}
		else if(commandcode == 0x52)
		{
			I2C_SlaveSendData(pI2CHandle->pI2Cx,tx_buffer[cnt++]);
		}
	}
	else if(AppEv == I2C_EV_DATA_RCV)
	{
		// Data is waiting for the slave to read . Slave has to read it
		commandcode = I2C_SlaveReceiveData(pI2CHandle->pI2Cx);
	}
	else if(AppEv == I2C_ERROR_AF)
	{
		// This happens only when slave is transmitting
		// Master has sent the NACK . So Slave should understand that
		//master doesn't need more data
		commandcode = 0xff;
		cnt = 0;

	}
	else if(AppEv == I2C_EV_STOP)
	{
		// This happens only during Slave Reception
		// So Master has ended the I2C communication with the Slave.
	}
}
