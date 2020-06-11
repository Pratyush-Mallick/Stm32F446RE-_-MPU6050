/*
 * i2c_master_rx_testing.c
 *
 *  Created on: 03-Jun-2020
 *      Author: Asus
 */


/*
 * i2c_master_tx_testing.c
 *
 *  Created on: 01-Jun-2020
 *      Author: Asus
 */

#include "stm32f446xx.h"
#include "MPU_6050.h"
#include <string.h>
#include <stdio.h>

extern void initialise_monitor_handles(void);

#define MY_ADDR    0x61
#define SlaveAddr  0x68
//I2C_Handle_t I2C1_Handle;

//Flag variable
//uint8_t rxComplt = RESET;

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
	//I2C1_Handle.DevAddr = SlaveAddr;

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

	//Initialize Peripheral Control
	I2C_PeripheralControl(I2C1, ENABLE);

	// Ack bit is enabled after PE = 1
	I2C_ManageAcking(I2C1_Handle.pI2Cx, I2C_ACK_ENABLE);

/*	uint8_t buff[32];
	uint8_t first_tx = 0x51;
	uint8_t second_tx = 0x52;
	uint8_t len;*/


	printf("Application is Running \n");

	MPU6050_SetFullScaleAccelRange(MPU6050_ACCEL_FS_2);

	MPU6050_Initialize();

	while(1)
	{

	// Wait for button Press
	//while(GPIO_ReadFromInputPin(GPIOC,GPIO_PIN_NO_13));

	// To avoid debouncing a delay of 200 ms
	delay();

	//while(I2C_MasterSendDataIT(&I2C1_Handle,&first_tx,1,SlaveAddr,I2C_ENABLE_SR) != I2C_READY);

	// Send Some Data to the slave
	//while(I2C_MasterReceiveDataIT(&I2C1_Handle,&len,1,SlaveAddr,I2C_ENABLE_SR) != I2C_READY);


	//while(I2C_MasterSendDataIT(&I2C1_Handle,&second_tx,1,SlaveAddr,I2C_ENABLE_SR) != I2C_READY);

	//while(I2C_MasterReceiveDataIT(&I2C1_Handle,buff,len,SlaveAddr,I2C_DISABLE_SR) != I2C_READY);

	//rxComplt = RESET;

	// Wait till Rx completes
	//while(rxComplt != SET);

	printf("Data : %02x",MPU6050_GetFullScaleGyroRange());

	//rxComplt = RESET;

	}
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
     if(AppEv == I2C_EV_TX_CMPLT)
     {
    	 printf("Tx is completed \n");
     }
     else if (AppEv == I2C_EV_RX_CMPLT)
     {
    	 printf("Rx is completed \n");
    	 rxComplt = SET;
     }
     else if (AppEv == I2C_ERROR_AF)
     {
    	 printf("Error : Ack failure\n");
    	 //in master ack failure happens when slave fails to send ack for the byte
    	 //sent from the master.
    	 I2C_CloseSendData(pI2CHandle);

    	 //generate the stop condition to release the bus
    	 I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

    	 //Hang in infinite loop
    	 while(1);
     }
}
