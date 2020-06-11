/*
 * stm32f446xx_spi_driver.h
 *
 *  Created on: 24-Apr-2020
 *      Author: Asus
 */

#ifndef INC_STM32F446XX_SPI_DRIVER_H_
#define INC_STM32F446XX_SPI_DRIVER_H_

#include "stm32f446xx.h"

/*
 * @SPI__MODES
 * SPI possible modes
 */

#define SPI_DEVICE_MODE_MASTER                1
#define SPI_DEVICE_MODE_SLAVE                 0

/*
 * @SPI__BUS_CONFIG
 * SPI possible Bus Configuration :-
 * FD - Full Duplex
 * HD - Half Duplex
 * S_TxOnly- Simplex Transmit only
 * S_RxOnly- Simplex Receive only
 *
 */

#define SPI_BUS_CONFIG_FD                     1
#define SPI_BUS_CONFIG_HD                     2
#define SPI_BUS_CONFIG_SIMPLEX_RXONLY         3

/*
 * @SPI__CLK_SPEED
 * SPI possible BAUD RATE
 */

#define SPI_SCLK_SPEED_DIV2                   0
#define SPI_SCLK_SPEED_DIV4                   1
#define SPI_SCLK_SPEED_DIV8                   2
#define SPI_SCLK_SPEED_DIV16                  3
#define SPI_SCLK_SPEED_DIV32                  5
#define SPI_SCLK_SPEED_DIV64                  5
#define SPI_SCLK_SPEED_DIV128                 6
#define SPI_SCLK_SPEED_DIV256                 7


/*
 * @SPI__DFF
 * SPI possible Data Frame Format
 * Default : 8 Bits
 */

#define SPI_DFF_8BITS                         0
#define SPI_DFF_16BITS                        1

/*
 * @SPI_CPOL
 * Clock Polarity
 * 0: CK to 0 when idle
 * 1: CK to 1 when idle
 * Note: This bit should not be changed when communication is ongoing
 */

#define SPI_CPOL_HIGH                         1
#define SPI_CPOL_LOW                          0

/*
 * @SPI_CPHA
 * Clock Phase
 * 0: The first clock transition is the first data capture edge
 * 1: The second clock transition is the first data capture edge
 */

#define SPI_CPHA_HIGH                         1
#define SPI_CPHA_LOW                          0

/*
 * @SPI_SSM
 * Software Slave Management
 * Default : SSM_SF
 */

#define SPI_SSM_EN                           1
#define SPI_SSM_DI                           0

/*
 *  SPI related Status Flag Definitions
 */

#define SPI_RXNE_FLAG            (1 << SPI_SR_RXNE)
#define SPI_TXE_FLAG             (1 << SPI_SR_TXE)
#define SPI_BUSY_FLAG            (1 << SPI_SR_BSY)

/*
 * Possible SPI Application States
 */

#define SPI_READY                       0
#define SPI_BUSY_IN_RX                  1
#define SPI_BUSY_IN_TX                  2

/*
 * Possible SPI Application events
 */

#define SPI_EVENT_TX_CMPLT   1
#define SPI_EVENT_RX_CMPLT   2
#define SPI_EVENT_OVR_ERR    3
#define SPI_EVENT_CRC_ERR    4



/*
 * This is a Configuration structure for a SPI
 */
typedef struct
{
	uint8_t SPI_DriverMode;
	uint8_t SPI_BusConfig;
	uint8_t SPI_ClockSpeed;
	uint8_t SPI_DFF;
	uint8_t SPI_CPOL;
	uint8_t SPI_CPHA;
	uint8_t SPI_SSM;
}SPI_Config_t;

/*
 * This is a Handle structure for a SPI
 */

typedef struct
{
	SPI_RegDef_t  *pSPIx;       	/*!< This holds the base address of the SPIx  >*/
	SPI_Config_t  SPI_Config;       /*!< This holds SPI configuration settings    >*/
	     uint8_t  *pTxBuffer;       /*!< To store the app. Tx Buffer address      >*/
	     uint8_t  *pRxBuffer;       /*!< To store the app. Rx Buffer address      >*/
	    uint32_t  TxLen;            /*!< To store Tx Len                          >*/
	    uint32_t  RxLen;            /*!< To store Rx Len                          >*/
	    uint8_t   TxState;          /*!< To store Tx State                        >*/
	    uint8_t   RxState;          /*!< To store Rx State                        >*/
}SPI_Handle_t;


/******************************************************************************************
 *								APIs supported by this driver
 *		 For more information about the APIs check the function definitions
 ******************************************************************************************/

/*
 *   Peripheral Clock Setup
 */

void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);


/*
 *   Init and De-init
 */

void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);

/*
 *   Data Send and Receive
 */

void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer,uint32_t Len);
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer,uint32_t Len);

/*
 *   Interrupt Based Data Send and Receive
 */

uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer,uint32_t Len);
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer,uint32_t Len);

/*
 *   IRQ Configuration & ISR Handling
 */

void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t *pHandle);

/*
 *  Other Peripheral Control API's
 */

void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName);
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi);
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi);
void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx);
void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle);
void SPI_CloseReception(SPI_Handle_t *pSPIHandle);

/*
 *  Application Event CallBack
 */

void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle,uint8_t Event);

#endif /* INC_STM32F446XX_SPI_DRIVER_H_ */
