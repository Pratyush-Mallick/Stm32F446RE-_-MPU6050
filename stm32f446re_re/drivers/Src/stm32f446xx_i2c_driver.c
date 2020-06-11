/*
 * stm32f446xx_i2c_driver.c
 *
 *  Created on: 08-May-2020
 *      Author: Asus
 */

#include "stm32f446xx_i2c_driver.h"
uint16_t AHB_Prescaler[8] = { 2,4,8,16,32,64,128,256};
uint8_t  APB1_Prescaler[4] = { 2,4,8,16};

// Private Function Definition
static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx);
static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *pI2Cx,uint8_t SlaveAddr);
static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr);
static void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle);

static void I2C_MasterHandleTXEInterrupt(I2C_Handle_t *pI2Chandle);
static void I2C_MasterHandleRXNEInterrupt(I2C_Handle_t *pI2CHandle);

static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->I2C_CR1 |= 1 << I2C_CR1_START ;
}

static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *pI2Cx,uint8_t SlaveAddr)
{
	SlaveAddr = SlaveAddr << 1; // Space for R/W bit
	SlaveAddr &= ~(1);          // Slave Address  + R/W bit
	pI2Cx->I2C_DR = SlaveAddr;
}

static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr)
{
	SlaveAddr = SlaveAddr << 1;
	SlaveAddr |= 1; //SlaveAddr is Slave address + r/nw bit=1
	pI2Cx->I2C_DR = SlaveAddr;
}

static void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle)
{
	uint32_t dummy_read;
	// Check for device in Master Mode or Slave Mode
	if(pI2CHandle->pI2Cx->I2C_SR2 & ( 1 << I2C_SR2_MSL))
	{
		// Device is in Master Mode
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
		{
			if(pI2CHandle->RxSize == 1)
			{
				// First Disable the ACK
				I2C_ManageAcking(pI2CHandle->pI2Cx, DISABLE);

				// Clear ADDR Flag ( read SR1 , Read SR2)
				dummy_read = pI2CHandle->pI2Cx->I2C_SR1;
				dummy_read = pI2CHandle->pI2Cx->I2C_SR2;
				(void)dummy_read;
			}
		}

		else
		{
			// Clear ADDR Flag ( read SR1 , Read SR2)
			dummy_read = pI2CHandle->pI2Cx->I2C_SR1;
			dummy_read = pI2CHandle->pI2Cx->I2C_SR2;
			(void)dummy_read;
		}
	}

	else
	{
		// Device is in Slave Mode
		// Clear ADDR Flag ( read SR1 , Read SR2)
		dummy_read = pI2CHandle->pI2Cx->I2C_SR1;
		dummy_read = pI2CHandle->pI2Cx->I2C_SR2;
		(void)dummy_read;
	}

}

void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->I2C_CR1 |= 1 << I2C_CR1_STOP ;
}


/*********************************************************************
 * @fn      		  - I2C_PeripheralControl
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -
 */

void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		pI2Cx->I2C_CR1 |= ( 1 << I2C_CR1_PE);
	}
	else
	{
		pI2Cx->I2C_CR1 &= ~( 1 << I2C_CR1_PE);
	}
}

/*********************************************************************
 * @fn      		  - I2C_PeriClockControl
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -
 */

void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pI2Cx == I2C1)
		{
			I2C1_PCLK_EN();
		}else if (pI2Cx == I2C2)
		{
			I2C2_PCLK_EN();
		}else if (pI2Cx == I2C3)
		{
			I2C3_PCLK_EN();
		}
	}
	else
	{
		if(pI2Cx == I2C1)
		{
			I2C1_PCLK_DI();
		}else if (pI2Cx == I2C2)
		{
			I2C2_PCLK_DI();
		}else if (pI2Cx == I2C3)
		{
			I2C3_PCLK_DI();
		}
	}
}

uint8_t RCC_PLLOutputCLock()
{
	return 0;
}


uint32_t RCC_GetPCLK1Value(void)
{
	uint32_t pclk1,SystemClk;

	uint8_t clk_src,temp, ahb_pre,apb1_pre;
	// Here we are reading the bits CFGR register to know the source of clock
	clk_src = (RCC->CFGR >> 2) & 0x3;

	if(clk_src == 0)
	{
		// System Clock is HSI
		SystemClk = 16000000;
	}
	else if (clk_src == 1)
	{
		// System CLock is HSE
		SystemClk = 8000000;
	}
	else if (clk_src == 2)
	{
		// System CLock is PLL
		SystemClk = RCC_PLLOutputCLock();
	}
	// Here we are reading the bits CFGR register to know the values of AHB1
	temp = (RCC->CFGR >> 4) & 0xF;

	// Refer to the reference manual
	// if AHB Bits of CGFR is less than 8 it has no Prescaler
	if(temp < 8)
	{
		ahb_pre = 1;
	}

	else
	{
		ahb_pre = AHB_Prescaler[temp - 8];
	}
	// Here we are reading the bits CFGR register to know the values of APB1
	temp = (RCC->CFGR >> 10) & 0x7;

	// Refer to the reference manual
	// if APB Bits of CGFR is less than 4 it has no Prescaler
	if(temp < 4)
	{
		apb1_pre = 1;
	}

	else
	{
		apb1_pre = APB1_Prescaler[temp - 4];
	}

	pclk1 = (SystemClk / ahb_pre) / apb1_pre;
	return pclk1;
}
/*********************************************************************
 * @fn      		  - I2C_Init
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -
 */

void I2C_Init(I2C_Handle_t *pI2CHandle)
{
	// Enable the clock for I2C peripheral
	I2C_PeriClockControl(pI2CHandle->pI2Cx, ENABLE);

	// ACK control Bit
	uint32_t tempreg =0;
	tempreg |= pI2CHandle->I2C_Config.I2C_ACKControl << 10 ;

	// Configure the FREQ Field of CR2 Register
	tempreg = 0;
	tempreg |= RCC_GetPCLK1Value()/1000000U;
	pI2CHandle->pI2Cx->I2C_CR2 = (tempreg & 0x3F) ;

	// Program the Device own address
	tempreg = 0;
	tempreg |= pI2CHandle->I2C_Config.I2C_DeviceAddress << 1;
	tempreg |= 1 << 14; // As instructed by the reference manual to do so
	pI2CHandle->pI2Cx->I2C_OAR1 = tempreg ;

	// CCR Calculations
	uint16_t ccr_value = 0;
	tempreg = 0;
	if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
	{
		// The mode is standard mode
		ccr_value =( RCC_GetPCLK1Value() / (2 * pI2CHandle->I2C_Config.I2C_SCLSpeed) );
		tempreg |= (ccr_value & 0xFFF);
	}
	else
	{
		// the mode is fast mode
		tempreg |= (1 << 15);
		tempreg |= (pI2CHandle->I2C_Config.I2C_FMDutyCycle << 14) ;

		if(pI2CHandle->I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY_2)
		{
			ccr_value =( RCC_GetPCLK1Value() / (3 * pI2CHandle->I2C_Config.I2C_SCLSpeed) );
		}
		else
		{
			ccr_value =( RCC_GetPCLK1Value() / (25 * pI2CHandle->I2C_Config.I2C_SCLSpeed) );
		}
		tempreg |= (ccr_value & 0xFFF);
	}
	pI2CHandle->pI2Cx->I2C_CCR = tempreg ;

	// TRISE Configuration
	if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
	{
		// The mode is standard mode
		tempreg =( RCC_GetPCLK1Value() / 1000000U) + 1;
	}
	else
	{
		// Fast Mode Trise configuration
		tempreg =( (  RCC_GetPCLK1Value() * 300) / 1000000U) + 1;
	}
	pI2CHandle->pI2Cx->I2C_TRISE = (tempreg & 0x3F) ;
}

/*********************************************************************
 * @fn      		  - I2C_DeInit
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -
 */

void I2C_DeInit(I2C_RegDef_t *pI2Cx)
{
	    if(pI2Cx == I2C1)
		{
	    	I2C1_REG_RESET();
		}
	    else if (pI2Cx == I2C2)
		{
	    	I2C2_REG_RESET();
		}
	    else if (pI2Cx == I2C3)
		{
	    	I2C3_REG_RESET();
		}
}

/*********************************************************************
 * @fn      		  - I2C_MasterSendData
 *
 * @brief             - Transfer the data buffer accordign to the sequence
 *
 * @param[in]         - handle to I2C
 * @param[in]         - data buffer
 * @param[in]         - Length of the data
 * @param[in]         - Slave Address
 * @param[in]         - Repeated Start Bit
 *
 * @return            - NONE
 *
 * @Note              - Refer to the data transfer sequence in the reference manual
 */

void I2C_MasterSendData(I2C_Handle_t *pI2CHandle,uint8_t *pTxbuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr)
{
	//1. Generate the start condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	//2. confirm that start generation is completed by checking the SB flag in the SR1
	//   Note: Until SB is cleared SCL will be stretched (pulled to LOW)
	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB));

	//3. Send the address of the slave with r/nw bit set to w(0) (total 8 bits )
	I2C_ExecuteAddressPhaseWrite(pI2CHandle->pI2Cx,SlaveAddr);

	//4. Confirm that address phase is completed by checking the ADDR flag in the SR1
	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR));

	//5. clear the ADDR flag according to its software sequence
	//   Note: Until ADDR is cleared SCL will be stretched (pulled to LOW)
	I2C_ClearADDRFlag(pI2CHandle);

	//6. Send data until the Length becomes 0
	while(Len > 0)
	{
		while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE));
		pI2CHandle->pI2Cx->I2C_DR = *pTxbuffer;
		pTxbuffer++;
		Len--;
	}

	//7. when Len becomes zero wait for TXE=1 and BTF=1 before generating the STOP condition
	//   Note: TXE=1 , BTF=1 , means that both SR and DR are empty and next transmission should begin
	//   when BTF=1 SCL will be stretched (pulled to LOW)
	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE));

	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_BTF));

	//8. Generate STOP condition and master need not to wait for the completion of stop condition.
	//   Note: generating STOP, automatically clears the BTF
	if(Sr == I2C_ENABLE_SR)
	{
		I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
	}
}


/*********************************************************************
 * @fn      		  - I2C_ReceiveData
 *
 * @brief             - Transfer the data buffer accordign to the sequence
 *
 * @param[in]         - handle to I2C
 * @param[in]         - data buffer
 * @param[in]         - Length of the data
 * @param[in]         - Slave Address
 * @param[in]         - Repeated Start Bit
 *
 * @return            - NONE
 *
 * @Note              - Refer to the data transfer sequence in the reference manual
 */

void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle,uint8_t *pRxBuffer, uint8_t Len, uint8_t SlaveAddr,uint8_t Sr)
{
	// 1. Generate the Start Condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	// 2. Confirm the start condition is completed by checking the SB Flag in SR1
	// Note: Until SB is cleared SCL will be stretched (pulled to LOW)
	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB));

	// 3. Send the Address of the Slave with r/nw bit set to R(1) (total 8 bits)
	I2C_ExecuteAddressPhaseRead(pI2CHandle->pI2Cx, SlaveAddr);

	// 4. Wait until address phase is completed by checking the ADDR Flag in the SR1
	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR));


	//procedure to read only 1 byte from slave
	if(Len == 1)
	{
		//Disable Acking
		I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_DISABLE);

		//clear the ADDR flag
		I2C_ClearADDRFlag(pI2CHandle);

		//wait until  RXNE becomes 1
		while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RXNE));

		if(Sr == I2C_ENABLE_SR)
		{
		//generate STOP condition
		I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
		}

		//read data in to buffer
		*pRxBuffer = pI2CHandle->pI2Cx->I2C_DR;

	}


    //procedure to read data from slave when Len > 1
	if(Len > 1)
	{
		//clear the ADDR flag
		I2C_ClearADDRFlag(pI2CHandle);

		//read the data until Len becomes zero
		for ( uint32_t i = Len ; i > 0 ; i--)
		{
			//wait until RXNE becomes 1
			while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RXNE));

			if(i == 2) //if last 2 bytes are remaining
			{
				//Disable Acking
				I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_DISABLE);

				if(Sr == I2C_ENABLE_SR)
				{
				//generate STOP condition
				//if(Sr == I2C_DISABLE_SR )
				I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
				}

			}

			//read the data from data register in to buffer
			*pRxBuffer = pI2CHandle->pI2Cx->I2C_DR;

			//increment the buffer address
			pRxBuffer++;

		}

	}

	//re-enable ACKing
	if(pI2CHandle->I2C_Config.I2C_ACKControl == I2C_ACK_ENABLE)
	{
		I2C_ManageAcking(pI2CHandle->pI2Cx,I2C_ACK_ENABLE);
	}
}



void I2C_ManageAcking(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi)
{
	if(EnOrDi == I2C_ACK_ENABLE)
	{
		// Enable the ACK
		pI2Cx->I2C_CR1 |= ( 1 << I2C_CR1_ACK);
	}
	else
	{
		// Disable the ack
		pI2Cx->I2C_CR1 &= ~( 1 << I2C_CR1_ACK);
	}
}


/*********************************************************************
 * @fn      		  - I2C_IRQInterruptConfig
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -

 */
void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{

	if(EnorDi == ENABLE)
	{
		if(IRQNumber <= 31)
		{
			//program ISER0 register
			*NVIC_ISER0 |= ( 1 << IRQNumber );

		}else if(IRQNumber > 31 && IRQNumber < 64 ) //32 to 63
		{
			//program ISER1 register
			*NVIC_ISER1 |= ( 1 << (IRQNumber % 32) );
		}
		else if(IRQNumber >= 64 && IRQNumber < 96 )
		{
			//program ISER2 register //64 to 95
			*NVIC_ISER3 |= ( 1 << (IRQNumber % 64) );
		}
	}else
	{
		if(IRQNumber <= 31)
		{
			//program ICER0 register
			*NVIC_ICER0 |= ( 1 << IRQNumber );
		}else if(IRQNumber > 31 && IRQNumber < 64 )
		{
			//program ICER1 register
			*NVIC_ICER1 |= ( 1 << (IRQNumber % 32) );
		}
		else if(IRQNumber >= 6 && IRQNumber < 96 )
		{
			//program ICER2 register
			*NVIC_ICER3 |= ( 1 << (IRQNumber % 64) );
		}
	}

}


/*********************************************************************
 * @fn      		  - I2C_IRQPriorityConfig
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -

 */
void I2C_IRQPriorityConfig(uint8_t IRQNumber,uint32_t IRQPriority)
{
	//1. first lets find out the ipr register
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section  = IRQNumber %4 ;

	uint8_t shift_amount = ( 8 * iprx_section) + ( 8 - NO_PR_BITS_IMPLEMNENTED );

	*(  NVIC_PR_BASE_ADDR + iprx ) |=  ( IRQPriority << shift_amount );

}

/*********************************************************************
 * @fn      		  - I2C_MasterSendDataIT
 *
 * @brief             - Transfer the data buffer in Non-Blocking Mode
 *
 * @param[in]         - handle to I2C
 * @param[in]         - data buffer
 * @param[in]         - Length of the data
 * @param[in]         - Slave Address
 * @param[in]         - Reapeated Start Bit
 *
 * @return            - success or error
 *
 * @Note              - Refer to the data transfer sequence in the reference manual
 */

uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle,uint8_t *pTxbuffer, uint32_t Len, uint8_t SlaveAddr,uint8_t Sr)
{
	uint8_t busystate = pI2CHandle->TxRxState;

		if( (busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))
		{
			pI2CHandle->pTxBuffer = pTxbuffer;
			pI2CHandle->TxLen = Len;
			pI2CHandle->TxRxState = I2C_BUSY_IN_TX;
			pI2CHandle->DevAddr = SlaveAddr;
			pI2CHandle->Sr = Sr;

			//Implement code to Generate START Condition
			I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

			//Implement the code to enable ITBUFEN Control Bit
			pI2CHandle->pI2Cx->I2C_CR2 |= ( 1 << I2C_CR2_ITBUFEN);

			//Implement the code to enable ITEVTEN Control Bit
			pI2CHandle->pI2Cx->I2C_CR2 |= ( 1 << I2C_CR2_ITEVTEN);

			//Implement the code to enable ITERREN Control Bit
			pI2CHandle->pI2Cx->I2C_CR2 |= ( 1 << I2C_CR2_ITERREN);

		}

		return busystate;

}

/*********************************************************************
 * @fn      		  - I2C_MasterSendDataIT
 *
 * @brief             - Transfer the data buffer in Non-Blocking Mode
 *
 * @param[in]         - handle to I2C
 * @param[in]         - data buffer
 * @param[in]         - Length of the data
 * @param[in]         - Slave Address
 * @param[in]         - Reapeated Start Bit
 *
 * @return            - success or error
 *
 * @Note              - Refer to the data transfer sequence in the reference manual
 */

uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle,uint8_t *pRxbuffer, uint8_t Len, uint8_t SlaveAddr,uint8_t Sr)
{
	uint8_t busystate = pI2CHandle->TxRxState;

	if( (busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))
	{
		pI2CHandle->pRxBuffer = pRxbuffer;
		pI2CHandle->RxLen = Len;
		pI2CHandle->TxRxState = I2C_BUSY_IN_RX;
		pI2CHandle->RxSize = Len; //Rxsize is used in the ISR code to manage the data reception
		pI2CHandle->DevAddr = SlaveAddr;
		pI2CHandle->Sr = Sr;

		//Implement code to Generate START Condition
		I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

		//Implement the code to enable ITBUFEN Control Bit
		pI2CHandle->pI2Cx->I2C_CR2 |= ( 1 << I2C_CR2_ITBUFEN);

		//Implement the code to enable ITEVTEN Control Bit
		pI2CHandle->pI2Cx->I2C_CR2 |= ( 1 << I2C_CR2_ITEVTEN);

		//Implement the code to enable ITERREN Control Bit
		pI2CHandle->pI2Cx->I2C_CR2 |= ( 1 << I2C_CR2_ITERREN);

	}

	return busystate;
}


void I2C_IRQHandling_EV(I2C_Handle_t *pI2Chandle)
{
	//Interrupt handling for both master and slave mode of a device

	uint32_t temp1,temp2,temp3;

	// Check if the interrupt event bits are Set in CR2 Register
	temp1 = pI2Chandle->pI2Cx->I2C_CR2 & ( 1 << I2C_CR2_ITEVTEN);

	temp2 = pI2Chandle->pI2Cx->I2C_CR2 & ( 1 << I2C_CR2_ITBUFEN);


	temp3 = pI2Chandle->pI2Cx->I2C_SR1 & ( 1 << I2C_SR1_SB);
	//1. Handle For interrupt generated by SB event
	//	Note : SB flag is only applicable in Master mode
	if(temp1 && temp3)
	{
		//The interrupt is generated because of SB: Start bit (Master mode) event
		//This block will not be executed in slave mode because for slave SB is always zero
		//In this block lets executed the address phase

		if(pI2Chandle->TxRxState == I2C_BUSY_IN_TX)
		{
			I2C_ExecuteAddressPhaseWrite(pI2Chandle->pI2Cx, pI2Chandle->DevAddr);
		}
		else if(pI2Chandle->TxRxState == I2C_BUSY_IN_RX)
		{
			I2C_ExecuteAddressPhaseRead(pI2Chandle->pI2Cx, pI2Chandle->DevAddr);
		}
	}


	temp3 = pI2Chandle->pI2Cx->I2C_SR1 & ( 1 << I2C_SR1_ADDR);
	//2. Handle For interrupt generated by ADDR event
	//Note : When master mode : Address is sent
	//		 When Slave mode   : Address matched with own address
	if(temp1 && temp3)
	{
		// interrupt is generated because of ADDR: Address sent (master mode)/matched (slave mode) Bit
		I2C_ClearADDRFlag(pI2Chandle);

	}


	temp3 = pI2Chandle->pI2Cx->I2C_SR1 & ( 1 << I2C_SR1_BTF);
	//3. Handle For interrupt generated by BTF(Byte Transfer Finished) event
	if(temp1 && temp3)
	{
		// interrupt is generated because of BTF: Byte transfer finished Event

		if(pI2Chandle->TxRxState == I2C_BUSY_IN_TX)
		{
			// Make sure that TXE is also Set
			if( pI2Chandle->pI2Cx->I2C_SR1 & ( 1 << I2C_SR1_TXE))
			{
				// BTF , TXE = 1
				if(pI2Chandle->TxLen == 0)
				{
					// 1. Generate Stop Condition
					if(pI2Chandle->Sr == I2C_DISABLE_SR)
					{
						I2C_GenerateStopCondition(pI2Chandle->pI2Cx);
					}

					// 2. Reset all member elements of handle Structure
					I2C_CloseSendData(pI2Chandle);

					// 3. Notify the application about transmission complete
					I2C_ApplicationEventCallback(pI2Chandle, I2C_EV_TX_CMPLT);
				}
			}

		}
		else if(pI2Chandle->TxRxState == I2C_BUSY_IN_RX)
		{
			;
		}

	}


	temp3 = pI2Chandle->pI2Cx->I2C_SR1 & ( 1 << I2C_SR1_STOPF);
	//4. Handle For interrupt generated by STOPF event
	// Note : Stop detection flag is applicable only slave mode . For master this flag will never be set
	//The below code block will not be executed by the master since STOPF will not set in master mode4
	if(temp1 && temp3)
	{
		// Stop Flag is Set
		//Clear the STOPF ( i.e 1) read SR1 2) Write to CR1 )
		// Read SR1 s already done in the above statement (temp3 =....) Hence need no to do again
		pI2Chandle->pI2Cx->I2C_CR1 |= 0x0000;   // This will not affect the contents of CR1 register

		// Notify the application that STOP is detected
		I2C_ApplicationEventCallback(pI2Chandle, I2C_EV_STOP);
	}


	temp3 = pI2Chandle->pI2Cx->I2C_SR1 & ( 1 << I2C_SR1_TXE);
	//5. Handle For interrupt generated by TXE event
	if(temp1 && temp2 && temp3)
	{
		// Check for device mode
		if(pI2Chandle->pI2Cx->I2C_SR2 & (1 << I2C_SR2_MSL))
		{	// TXE Flag is Set
			// We have to do data transmission
			if(pI2Chandle->TxRxState == I2C_BUSY_IN_TX)
			{
				I2C_MasterHandleTXEInterrupt(pI2Chandle);
			}
		}
		else
		{
			//slave
			//make sure that the slave is really in transmitter mode
		    if(pI2Chandle->pI2Cx->I2C_SR2 & ( 1 << I2C_SR2_TRA))
		    {
		    	I2C_ApplicationEventCallback(pI2Chandle,I2C_EV_DATA_REQ);
		    }
		}
	}


	temp3 = pI2Chandle->pI2Cx->I2C_SR1 & ( 1 << I2C_SR1_RXNE);
	//6. Handle For interrupt generated by RXNE event
	if(temp1 && temp2 && temp3)
	{
		// Check for Device Mode
		if(pI2Chandle->pI2Cx->I2C_SR2 & ( 1 << I2C_SR2_MSL))
		{
			// Device is Master
			// RXNE Flag is Set
			if(pI2Chandle->TxRxState == I2C_BUSY_IN_RX)
			{
				I2C_MasterHandleRXNEInterrupt(pI2Chandle);
			}
		}
		else
		{
			//slave
			//make sure that the slave is really in receiver mode
			if(!(pI2Chandle->pI2Cx->I2C_SR2 & ( 1 << I2C_SR2_TRA)))
			{
				I2C_ApplicationEventCallback(pI2Chandle,I2C_EV_DATA_RCV);
			}
		}
	}
}

static void I2C_MasterHandleTXEInterrupt(I2C_Handle_t *pI2Chandle)
{
	if(pI2Chandle->TxLen > 0)
	{
		// 1. Load the data into DR
		pI2Chandle->pI2Cx->I2C_DR = *(pI2Chandle->pTxBuffer);

		// 2. Decrement the length
		pI2Chandle->TxLen--;

		// 3. Increment the buffer address pointer
		pI2Chandle->pTxBuffer++;
	}
}

static void I2C_MasterHandleRXNEInterrupt(I2C_Handle_t *pI2CHandle)
{
	// We have to do data reception
	if(pI2CHandle->RxSize == 1)
	{
		*pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->I2C_DR ;
		 pI2CHandle->RxLen--;
	}

	if(pI2CHandle->RxSize > 1)
	{
		if(pI2CHandle->RxLen == 2)
		{
			// Clear Ack Bit
			I2C_ManageAcking(pI2CHandle->pI2Cx, DISABLE);
		}

		// Read DR
		*pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->I2C_DR ;
		 pI2CHandle->pRxBuffer++;
		 pI2CHandle->RxLen--;
	}


	if(pI2CHandle->RxLen == 0)
	{
		// Close the I2C Data Reception & Notify the application

		// 1. Generate the Stop Condition
		if(pI2CHandle->Sr == I2C_DISABLE_SR)
		{
			I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
		}

		// 2. Close the I2C rx
		I2C_CloseReceiveData(pI2CHandle);

		// 3. Notify the application
		I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_RX_CMPLT);
	}
}



/*********************************************************************
 * @fn      		  - I2C_ER_IRQHandling
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              - Complete the code also define these macros in the driver
						header file
						#define I2C_ERROR_BERR  3
						#define I2C_ERROR_ARLO  4
						#define I2C_ERROR_AF    5
						#define I2C_ERROR_OVR   6
						#define I2C_ERROR_TIMEOUT 7

 */

void I2C_IRQHandling_ER(I2C_Handle_t *pI2CHandle)
{

	uint32_t temp1,temp2;

    //Know the status of  ITERREN control bit in the CR2
	temp2 = (pI2CHandle->pI2Cx->I2C_CR2) & ( 1 << I2C_CR2_ITERREN);


/***********************Check for Bus error************************************/
	temp1 = (pI2CHandle->pI2Cx->I2C_SR1) & ( 1<< I2C_SR1_BERR);
	if(temp1  && temp2 )
	{
		//This is Bus error

		//Implement the code to clear the buss error flag
		pI2CHandle->pI2Cx->I2C_SR1 &= ~( 1 << I2C_SR1_BERR);

		//Implement the code to notify the application about the error
	   I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_BERR);
	}

/***********************Check for arbitration lost error************************************/
	temp1 = (pI2CHandle->pI2Cx->I2C_SR1) & ( 1 << I2C_SR1_ARLO );
	if(temp1  && temp2)
	{
		//This is arbitration lost error

		//Implement the code to clear the arbitration lost error flag
		pI2CHandle->pI2Cx->I2C_SR1 &= ~( 1 << I2C_SR1_ARLO);

		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_ARLO);

	}

/***********************Check for ACK failure  error************************************/

	temp1 = (pI2CHandle->pI2Cx->I2C_SR1) & ( 1 << I2C_SR1_AF);
	if(temp1  && temp2)
	{
		//This is ACK failure error

	    //Implement the code to clear the ACK failure error flag
		pI2CHandle->pI2Cx->I2C_SR1 &= ~( 1 << I2C_SR1_AF);

		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_AF);
	}

/***********************Check for Overrun/underrun error************************************/
	temp1 = (pI2CHandle->pI2Cx->I2C_SR1) & ( 1 << I2C_SR1_OVR);
	if(temp1  && temp2)
	{
		//This is Overrun/underrun

	    //Implement the code to clear the Overrun/underrun error flag
		pI2CHandle->pI2Cx->I2C_SR1 &= ~( 1 << I2C_SR1_OVR);

		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_OVR);
	}

/***********************Check for Time out error************************************/
	temp1 = (pI2CHandle->pI2Cx->I2C_SR1) & ( 1 << I2C_SR1_TIMEOUT);
	if(temp1  && temp2)
	{
		//This is Time out error

	    //Implement the code to clear the Time out error flag
		pI2CHandle->pI2Cx->I2C_SR1 &= ~( 1 << I2C_SR1_TIMEOUT);

		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_TIMEOUT);
	}

}



void I2C_CloseReceiveData(I2C_Handle_t *pI2CHandle)
{
	//Implement the code to disable ITBUFEN Control Bit
	pI2CHandle->pI2Cx->I2C_CR2 &= ~( 1 << I2C_CR2_ITBUFEN);

	//Implement the code to disable ITEVFEN Control Bit
	pI2CHandle->pI2Cx->I2C_CR2 &= ~( 1 << I2C_CR2_ITEVTEN);

	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->pRxBuffer = NULL;
	pI2CHandle->RxLen = 0;
	pI2CHandle->RxSize = 0;

	if(pI2CHandle->I2C_Config.I2C_ACKControl == I2C_ACK_ENABLE)
	{
		I2C_ManageAcking(pI2CHandle->pI2Cx,ENABLE);
	}

}

void I2C_CloseSendData(I2C_Handle_t *pI2CHandle)
{
	//Implement the code to disable ITBUFEN Control Bit
	pI2CHandle->pI2Cx->I2C_CR2 &= ~( 1 << I2C_CR2_ITBUFEN);

	//Implement the code to disable ITEVFEN Control Bit
	pI2CHandle->pI2Cx->I2C_CR2 &= ~( 1 << I2C_CR2_ITEVTEN);


	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->pTxBuffer = NULL;
	pI2CHandle->TxLen = 0;
}

void I2C_SlaveSendData(I2C_RegDef_t *pI2C,uint8_t data)
{
	pI2C->I2C_DR = data;
}

uint8_t I2C_SlaveReceiveData(I2C_RegDef_t *pI2C)
{
	return (uint8_t) pI2C->I2C_DR;
}

void I2C_SlaveEnableDisableCallBackEvents(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pI2Cx->I2C_CR2 |= ( 1 << I2C_CR2_ITEVTEN);
		pI2Cx->I2C_CR2 |= ( 1 << I2C_CR2_ITBUFEN);
		pI2Cx->I2C_CR2 |= ( 1 << I2C_CR2_ITERREN);
	}
	else
	{
		pI2Cx->I2C_CR2 &= ~( 1 << I2C_CR2_ITEVTEN);
		pI2Cx->I2C_CR2 &= ~( 1 << I2C_CR2_ITBUFEN);
		pI2Cx->I2C_CR2 &= ~( 1 << I2C_CR2_ITERREN);
	}
}
/*********************************************************************
 * @fn      		  - I2C_getFlagStatus
 *
 * @brief             - Check the I2C Flag Status in the status register
 *
 * @param[in]         - I2Cx- Number peripeheral
 * @param[in]         - Flag that you want to check
 *
 * @return            - NONE
 *
 * @Note              - Reading the SR1 register is part of clearing the SB Flag bit
 *
 */

uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName)
{
	if(pI2Cx->I2C_SR1 & FlagName)
	{
		return FLAG_SET;
	}

	return FLAG_RESET;
}


