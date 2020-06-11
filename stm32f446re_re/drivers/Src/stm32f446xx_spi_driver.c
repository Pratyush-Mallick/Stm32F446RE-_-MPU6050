/*
 * stm32f446xx_spi_driver.c
 *
 *  Created on: 24-Apr-2020
 *      Author: Asus
 */

#include "stm32f446xx_spi_driver.h"

static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_ove_err_interrupt_handle(SPI_Handle_t *pSPIHandle);



/*********************************************************************
 * @fn      		  - SPI_SlaveSelectInternal (SSI)
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

void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		pSPIx->SPI_CR1 |= ( 1 << SPI_CR1_SSI);
	}
	else
	{
		pSPIx->SPI_CR1 &= ~( 1 << SPI_CR1_SSI);
	}
}

/*********************************************************************
 * @fn      		  - SPI_PeripheralControl
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

void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		pSPIx->SPI_CR1 |= ( 1 << SPI_CR1_SPE);
	}
	else
	{
		pSPIx->SPI_CR1 &= ~( 1 << SPI_CR1_SPE);
	}
}

/*********************************************************************
 * @fn      		  - SPI_PeriClockControl
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

void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pSPIx == SPI1)
		{
			SPI1_PCLK_EN();
		}
		else if (pSPIx == SPI2)
		{
			SPI2_PCLK_EN();
		}
		else if (pSPIx == SPI3)
		{
			SPI3_PCLK_EN();
		}
		else if (pSPIx == SPI4)
		{
			SPI4_PCLK_EN();
		}
	}

	else
	{
		if(pSPIx == SPI1)
		{
			SPI1_PCLK_DI();
		}
		else if (pSPIx == SPI2)
		{
			SPI2_PCLK_DI();
		}
		else if (pSPIx == SPI3)
		{
			SPI3_PCLK_DI();
		}
		else if (pSPIx == SPI4)
		{
			SPI4_PCLK_DI();
		}
	}
}



/*********************************************************************
 * @fn      		  - SPI_Init
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

void SPI_Init(SPI_Handle_t *pSPIHandle)
{
	uint32_t tempreg = 0;

    // Enable the Peripheral Clock
	SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);

	// 1. Configure the Device Mode
	tempreg |= pSPIHandle->SPI_Config.SPI_DriverMode << SPI_CR1_MSTR;

	// 2. Configure the SPI Bus
	if(pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_FD)
	{
		// BIDI_Mode should be cleared
        tempreg &= ~(1 << SPI_CR1_BIDIMODE);
	}

	else if(pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_HD)
	{
		// BIDI_Mode should be set
		tempreg |= (1 << SPI_CR1_BIDIMODE);
	}

	else if(pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY)
	{
		// BIDI_Mode should be set
		tempreg |= (1 << SPI_CR1_BIDIMODE);
		// RXONLY Bit should be set
		tempreg |= (1 << SPI_CR1_RXONLY);
	}

	// 3. Configure the SPI Serial Clock Speed (Baud Rate)
	tempreg |= pSPIHandle->SPI_Config.SPI_ClockSpeed << SPI_CR1_BR;

	// 4. Configure the Data Frame Format ( DFF)
	tempreg |= pSPIHandle->SPI_Config.SPI_DFF << SPI_CR1_DFF;

	// 5. Configure the Clock Polarity (CPOL)
	tempreg |= pSPIHandle->SPI_Config.SPI_CPOL << SPI_CR1_CPOL;

	// 6. Configure the Clock Phase (CPHA)
	tempreg |= pSPIHandle->SPI_Config.SPI_CPHA << SPI_CR1_CPHA;

	// 7. Configure the Slave Select Management
	tempreg |= pSPIHandle->SPI_Config.SPI_SSM << SPI_CR1_SSM;

	pSPIHandle->pSPIx->SPI_CR1 = tempreg;

}



/*********************************************************************
 * @fn      		  - SPI_DeInit
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

void SPI_DeInit(SPI_RegDef_t *pSPIx)
{
	    if(pSPIx == SPI1)
		{
			SPI1_REG_RESET();
		}
	    else if (pSPIx == SPI2)
		{
			SPI2_REG_RESET();
		}
	    else if (pSPIx == SPI3)
		{
			SPI3_REG_RESET();
		}
	    else if (pSPIx == SPI4)
		{
			SPI4_REG_RESET();
		}
}

uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName)
{
	if(pSPIx->SPI_SR & FlagName)
	{
		return FLAG_SET;
	}

	return FLAG_RESET;
}

/*********************************************************************
 * @fn      		  - SPI_SendData
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              - This is blocking call
 */

void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer,uint32_t Len)
{
	while( Len > 0)
	{
		// Wait until TXE Flag is SET.
		while(SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG) == FLAG_RESET);

		// Check the DFF bit in CR1
		if(pSPIx->SPI_CR1 & (1 << 11))
		{
			// 16 Bit DFF
			// Load the data into data register
			pSPIx->SPI_DR = *((uint16_t*)pTxBuffer);
			Len = Len - 2;
			// Increment the pointer address to next 16 bits.
			(uint16_t*)pTxBuffer++;
		}

		else
		{
			// 8 Bit DFF
			pSPIx->SPI_DR = *pTxBuffer;
			Len --;
			pTxBuffer++;
		}
	}
}

/*********************************************************************
 * @fn      		  - SPI_SendDataIT
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              - This is non-blocking Call
 */

uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer,uint32_t Len)
{
	uint8_t state = pSPIHandle->TxState;

	if(state != SPI_BUSY_IN_TX)
	{
		// 1.Save the TxBuffer Address and Len in some global variable
		pSPIHandle->pTxBuffer = pTxBuffer;
		pSPIHandle->TxLen = Len;

		// 2. Mark the SPI State as busy so that no other code can take over
		//    same SPI peripheral until transmission is over
		pSPIHandle->TxState = SPI_BUSY_IN_TX;

		// 3. Enable the TXEIE control bit to get interrupt whenever TXE flag
		//    is set in Status Register ( SR )
		pSPIHandle->pSPIx->SPI_CR2 |= (1 << SPI_CR2_TXEIE);

		// 4. Data Transmission will be handled by SPU ISR
	}

	return state;

}



/*********************************************************************
 * @fn      		  - SPI_ReceiveData
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              - This is a blocking Call
 */

void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer,uint32_t Len)
{
	while( Len > 0)
		{
			// Wait until RXE Flag is SET i.e the buffer is filled.
			while(SPI_GetFlagStatus(pSPIx, SPI_RXNE_FLAG) == FLAG_RESET);

			// Check the DFF bit in CR1
			if(pSPIx->SPI_CR1 & (1 << SPI_CR1_DFF))
			{
				// 16 Bit DFF
				// Read Data from the data register
				*((uint16_t*)pRxBuffer) = pSPIx->SPI_DR ;
				Len = Len - 2;
				// Increment the pointer address to next 16 bits.
				(uint16_t*)pRxBuffer++;
			}

			else
			{
				// 8 Bit DFF
				*pRxBuffer = pSPIx->SPI_DR;
				Len --;
				pRxBuffer++;
			}
		}
}

/*********************************************************************
 * @fn      		  - SPI_ReceiveData
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              - This is a non-blocking Call
 */


uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer,uint32_t Len)
{
	uint8_t state = pSPIHandle->RxState;

	if(state != SPI_BUSY_IN_RX)
	{
		// 1.Save the TxBuffer Address and Len in some global variable
		pSPIHandle->pRxBuffer = pRxBuffer;
		pSPIHandle->RxLen = Len;

		// 2. Mark the SPI State as busy so that no other code can take over
		//    same SPI peripheral until transmission is over
		pSPIHandle->RxState = SPI_BUSY_IN_RX;

		// 3. Enable the TXEIE control bit to get interrupt whenever TXE flag
		//    is set in Status Register ( SR )
		pSPIHandle->pSPIx->SPI_CR2 |= (1 << SPI_CR2_RXNEIE);

		// 4. Data Transmission will be handled by SPU ISR
	}

	return state;

}


/*********************************************************************
 * @fn      		  - SPI_SSOEConfig
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

void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		pSPIx->SPI_CR2 |= (1 << SPI_CR2_SSOE);
	}

	else
	{
		pSPIx->SPI_CR2 &= ~(1 << SPI_CR2_SSOE);
	}
}


/*********************************************************************
 * @fn      		  - SPI_IRQInterruptConfig
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

void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(IRQNumber <= 31)
		{
			// Program ISER0 Register
			*NVIC_ISER0 |= (1 << IRQNumber);
		}
		else if(IRQNumber > 31 && IRQNumber < 64)
		{
			// Program ISER1 Register
			*NVIC_ISER1 |= (1 << (IRQNumber % 32));
		}
		else if(IRQNumber >= 64 && IRQNumber < 96)
		{
			// Program ISER2 Register
			*NVIC_ISER2 |= (1 << (IRQNumber % 64));
		}
	}

	else
	{
		if(IRQNumber <= 31)
		{
			// Program ISER0 Register
			*NVIC_ICER0 |= (1 << IRQNumber);
		}
		else if(IRQNumber > 31 && IRQNumber < 64)
		{
			// Program ISER1 Register
			*NVIC_ICER1 |= (1 << (IRQNumber % 32));
		}
		else if(IRQNumber >= 64 && IRQNumber < 96)
		{
			// Program ISER2 Register
			*NVIC_ICER2 |= (1 << (IRQNumber % 64));
		}
	}
}


/*********************************************************************
 * @fn      		  - SPI_IRQPriorityConfig
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

void SPI_IRQPriorityConfig(uint8_t IRQNumber,uint32_t IRQPriority)
{
	//1. first lets find out the IPR register
	uint8_t iprx = IRQNumber / 4;
	uint8_t ipr_section = IRQNumber % 4;
	uint8_t shift_amount = (8 * ipr_section) + (8 - NO_PR_BITS_IMPLEMNENTED);
	*(NVIC_PR_BASE_ADDR + iprx ) |= ( IRQPriority << shift_amount);
}


/*********************************************************************
 * @fn      		  - SPI_IRQHandling
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

void SPI_IRQHandling(SPI_Handle_t *pHandle)
{
	uint8_t temp1, temp2;
	temp1 = pHandle->pSPIx->SPI_SR & ( 1 << SPI_SR_TXE);
	temp2 = pHandle->pSPIx->SPI_CR2 & ( 1 << SPI_CR2_TXEIE);

	if( temp1 && temp2)
	{
		// Handle TXE
		spi_txe_interrupt_handle(pHandle);
	}

	// Check for RXNE
	temp1 = pHandle->pSPIx->SPI_SR & ( 1 << SPI_SR_RXNE);
	temp2 = pHandle->pSPIx->SPI_CR2 & ( 1 << SPI_CR2_RXNEIE);

	if( temp1 && temp2)
	{
		// Handle TXE
		spi_rxne_interrupt_handle(pHandle);
	}

	// Check for OVER
	temp1 = pHandle->pSPIx->SPI_SR & ( 1 << SPI_SR_OVR);
	temp2 = pHandle->pSPIx->SPI_CR2 & ( 1 << SPI_CR2_ERRIE);

	if( temp1 && temp2)
	{
		// Handle TXE
		spi_ove_err_interrupt_handle(pHandle);
	}
}

// Some Helper Function Implementation

static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	// Check the DFF bit in CR1
	if(pSPIHandle->pSPIx->SPI_CR1 & (1 << 11))
	{
		// 16 Bit DFF
		// Load the data into data register
		pSPIHandle->pSPIx->SPI_DR = *((uint16_t*)pSPIHandle->pTxBuffer);
		pSPIHandle->TxLen = pSPIHandle->TxLen - 2;
		// Increment the pointer address to next 16 bits.
		(uint16_t*)pSPIHandle->pTxBuffer++;
	}

	else
	{
		// 8 Bit DFF
		pSPIHandle->pSPIx->SPI_DR = *pSPIHandle->pTxBuffer;
		pSPIHandle->TxLen --;
		pSPIHandle->pTxBuffer++;
	}

	if(!pSPIHandle->TxLen)
	{
		// Txe Len is zero , so close the SPI Transmission and inform the application that
		// TX is over. This prevents interrupts from setting up of TXE flag.
		SPI_CloseTransmission(pSPIHandle);
		SPI_ApplicationEventCallback(pSPIHandle,SPI_EVENT_TX_CMPLT);
	}
}
static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	//do rxing as per the dff
		if(pSPIHandle->pSPIx->SPI_CR1 & ( 1 << 11))
		{
			//16 bit
			*((uint16_t*)pSPIHandle->pRxBuffer) = (uint16_t) pSPIHandle->pSPIx->SPI_DR;
			pSPIHandle->RxLen -= 2;
			pSPIHandle->pRxBuffer++;
			pSPIHandle->pRxBuffer++;

		}else
		{
			//8 bit
			*(pSPIHandle->pRxBuffer) = (uint8_t) pSPIHandle->pSPIx->SPI_DR;
			pSPIHandle->RxLen--;
			pSPIHandle->pRxBuffer++;
		}

		if(! pSPIHandle->RxLen)
		{
			//reception is complete
			SPI_CloseReception(pSPIHandle);
			SPI_ApplicationEventCallback(pSPIHandle,SPI_EVENT_RX_CMPLT);
		}
}

static void spi_ove_err_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	uint8_t temp ;
	// Clear Over Bit
	if( pSPIHandle->TxState != SPI_BUSY_IN_TX )
	{
		SPI_ClearOVRFlag(pSPIHandle->pSPIx);
	}
	(void)temp;
	// 2. Inform the Application
	SPI_ApplicationEventCallback(pSPIHandle,SPI_EVENT_OVR_ERR);
}


void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->SPI_CR2 &= ~(1 << SPI_CR2_TXEIE);
    pSPIHandle->pTxBuffer = NULL;
    pSPIHandle->TxLen = 0;
    pSPIHandle->TxState = SPI_READY;
}


void SPI_CloseReception(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->SPI_CR2 &= ~(1 << SPI_CR2_RXNEIE);
    pSPIHandle->pRxBuffer = NULL;
    pSPIHandle->RxLen = 0;
    pSPIHandle->RxState = SPI_READY;
}

void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx)
{
	uint8_t temp;
	temp = pSPIx->SPI_DR;
	temp = pSPIx->SPI_SR;
	(void)temp;
}

__weak void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle,uint8_t Event)
{
	// This is weak implementation and the application may override this function
}
