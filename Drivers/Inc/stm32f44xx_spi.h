/*
 * stm32f44xx_spi.h
 *
 *  Created on: Apr 20, 2025
 *      Author: katog
 */

#ifndef INC_STM32F44XX_SPI_H_
#define INC_STM32F44XX_SPI_H_

#include "stm32f4xx.h"
#include "stm32f44xx_timer.h"

#define SPI_SLAVE_MDOE			(0)
#define SPI_MASTER_MODE 		(1)

#define SPI_MAX_TIMEOUT			(1000000000)

typedef enum {

	SPI_DEVICE_MODE_MSTR,
	SPI_DEVICE_MODE_SLAVE,

}SPI_DEVICE_MODE_t;
typedef enum {

	SPI_MODE_FULL_DPLX,
	SPI_MODE_HALF_DPLX,
	SPI_MODE_SMPLX_TX,
	SPI_MODE_SMPLX_RX

}SPI_MODE_t;

typedef enum {

	SPI_BAUD_DIV2,
	SPI_BAUD_DIV4,
	SPI_BAUD_DIV8,
	SPI_BAUD_DIV16,
	SPI_BAUD_DIV32,
	SPI_BAUD_DIV64,
	SPI_BAUD_DIV128,
	SPI_BAUD_DIV256

}SPI_BAUD_t;

typedef enum {

	SPI_DFF_8BITS,
	SPI_DFF_16BITS

}SPI_DFF_SIZE_t;

typedef enum {

	SPI_CPOL_HI,
	SPI_CPOL_LO,

}SPI_CPOL_STATE_t;

typedef enum {

	SPI_CPHA_HI,
	SPI_CPHA_LO

}SPI_CPHA_STATE_t;

typedef enum {
	SPI_READY,
	SPI_BUSY_IN_TX,
	SPI_BUSY_IN_RX,
	SPI_BUSY,
	SPI_OK
}SPI_BUSY_STATE_t;

typedef enum
{
	SPI_TX_CMPLT,
	SPI_RX_CMPLT,
	SPI_OVR_OCCUR

}SPI_APP_EV_t;

typedef struct {

	SPI_DEVICE_MODE_t SPI_DEVICE_MODE;
	SPI_BAUD_t SPI_BAUD_RATE;
	SPI_MODE_t SPI_MODE;
	SPI_DFF_SIZE_t SPI_DFF_SIZE;
	SPI_CPOL_STATE_t SPI_CPOL_STATE;
	SPI_CPHA_STATE_t SPI_CPHA_STATE;
	SPI_TypeDef* pSPIx;
	SPI_BUSY_STATE_t SPI_BUSY_STATE;
	FunctionalState SPI_SSM_STATE;
	uint8_t* RxBuffer;
	uint8_t* TxBuffer;
	uint32_t Txlen;
	uint32_t Rxlen;

}SPI_Handle_t;



/**
 * @brief Function to enable or disable the peripheral clock of SPI
 * @param
 */
ErrorStatus SPI_PCLK_CTRL(SPI_TypeDef* pSPIx, FunctionalState ENorDI);

/**
 * @brief Function to enable or disable the SPI peripheral
 * @param
 */
void SPI_PERI_CTRL(SPI_TypeDef* pSPIx, FunctionalState ENorDI);

/**
 * @brief Function to initialize SPI peripheral with user configurations
 * @param
 */
ErrorStatus SPI_INIT(SPI_Handle_t* SPI_HANDLE);

/**
 * @brief Function to initialize SPI peripheral with user configurations
 * @param
 */
ErrorStatus SPI_DeINIT(SPI_Handle_t* SPI_HANDLE);


/**
 * @brief Function to send data in Master mode
 * @param
 */
void SPI_SEND_DATA(SPI_TypeDef* pSPIx, uint8_t* buffer, uint32_t len);

/**
 * @brief Function to receive data in Master mode
 * @param
 */
void SPI_RCV_DATA(SPI_TypeDef* pSPIx, uint8_t* buffer, uint32_t len);

/**
 * @brief Function to receive data
 * @param
 */
SPI_BUSY_STATE_t SPI_RCV_DATA_IT(SPI_Handle_t* SPI_HANDLE, uint8_t* buffer, uint32_t len);

/**
 * @brief Function to send data using interrupt
 * @param
 */
SPI_BUSY_STATE_t SPI_SEND_DATA_IT(SPI_Handle_t* SPI_HANDLE, uint8_t* buffer, uint32_t len);


/**
 * @brief Function
 * @param
 */
void SSOE_BIT_CONTROL(SPI_TypeDef* pSPIx, FunctionalState ENorDI);

/**
 * @brief Function
 * @param
 */
void SPI_IRQ_CFG(uint8_t IRQ_NUMBER, FunctionalState ENorDI);

/**
 * @brief
 * @param
 */
void SPI_IRQ_HANDLE(SPI_Handle_t* SPI_HANDLE);

/**
 * @brief Function
 * @param
 */
__attribute__((weak)) void SPI_APP_CLKBK(SPI_Handle_t* SPI_HANDLE, SPI_APP_EV_t SPI_APPEV);
#endif /* INC_STM32F44XX_SPI_H_ */
