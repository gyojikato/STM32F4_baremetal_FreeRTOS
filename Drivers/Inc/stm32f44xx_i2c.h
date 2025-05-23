/*
 * stm32f44xx_i2c.h
 *
 *  Created on: May 3, 2025
 *      Author: katog
 */

#ifndef INC_STM32F44XX_I2C_H_
#define INC_STM32F44XX_I2C_H_


#include "stm32f4xx.h"
#include "stm32f44xx_rcc.h"
#include "stm32f44xx_timer.h"
#include <stdlib.h>

typedef enum {
	I2C_APPEV_CLLBCK_TX_CMPLT,
	I2C_APPEV_CLLBCK_RX_CMPLT,
	I2C_APPEV_CLLBCK_BERR_OCCUR,
	I2C_APPEV_CLLBCK_ARLO_OCCUR,
	I2C_APPEV_CLLBCK_AF_OCCUR,
	I2C_APPEV_CLLBCK_OVR_OCCUR,
	I2C_APPEV_CLLBCK_PECERR_OCCUR,
	I2C_APPEV_CLLBCK_TIMEOUT_OCCUR
}I2C_APPEV_CLLBCK_t;

typedef enum {
	I2C_STATUS_READY,
	I2C_STATUS_TX_BUSY,
	I2C_STATUS_RX_BUSY,
	I2C_BUSY,
	I2C_OK
}I2C_STATUS_t;

typedef enum {
	I2C_ERROR_TIMEOUT,
	I2C_SUCCESS
}I2C_ERROR_STATUS_t;

typedef enum {

	I2C_SPEED_SM = 100000,
	I2C_SPEED_FM = 400000

}I2C_SPEED_t;

typedef enum {
	I2C_FM_DUTY_MODE_2,
	I2C_FM_DUTY_MODE_16_9
}I2C_FM_DUTY_MODE;
typedef struct {
	I2C_SPEED_t I2C_SPD;
	I2C_TypeDef* pI2Cx;
	I2C_FM_DUTY_MODE I2C_FM_DUTY_MODE;
	uint8_t DEVICE_ADDR;
	uint8_t* TxBuffer;
	uint8_t* RxBuffer;
	uint32_t TxSize;
	uint32_t RxSize;
	uint32_t TxLen;
	uint32_t RxLen;
	uint8_t SlaveAddress;
	I2C_STATUS_t I2C_STATUS;
	uint8_t I2C_RPT_STRT;



}I2C_Handle_t;

/**
 * @brief
 * @param
 */
void I2C_PCLK_CTRL(I2C_TypeDef* pI2Cx, uint8_t ENorDI);

/**
 * @brief Function to initialize I2C peripheral with user configurations
 * @param
 */
void I2C_INIT(I2C_Handle_t* I2C_HANDLE);

/**
 * @brief
 * @param
 */

void I2C_GEN_START_CONDITION(I2C_TypeDef* pI2Cx);

/**
 * @brief
 * @param
 */

void I2C_GEN_STOP_CONDITION(I2C_TypeDef* pI2Cx);

/**
 * @brief
 * @param
 */
void I2C_ACK_CTRL(I2C_TypeDef* pI2Cx, uint8_t ENorDI);

/**
 * @brief
 * @param
 */
void I2C_PERI_CTRL(I2C_TypeDef* pI2Cx, uint8_t ENorDI);

/**
 * @brief
 * @param
 */
void I2C_CLEAR_ADDR(I2C_TypeDef* pI2Cx);

/**
 * @brief
 * @param
 */
void I2C_CLEAR_STOPF(I2C_TypeDef* pI2Cx);


/**
 * @brief
 * @param
 */
void I2C_SEND_ADDR(I2C_TypeDef* pI2Cx, uint8_t Slave_Addr, uint8_t RW);

/**
 * @brief
 * @param
 */
I2C_ERROR_STATUS_t I2C_MSTR_SEND_DATA(I2C_TypeDef* pI2Cx, uint8_t Slave_Addr, uint8_t* TxBuffer, uint32_t len, uint8_t Rpt_Strt);

/**
 * @brief
 * @param
 */
I2C_ERROR_STATUS_t I2C_MSTR_RECEIVE_DATA(I2C_TypeDef* pI2Cx, uint8_t Slave_Addr, uint8_t* RxBuffer, uint32_t len, uint8_t Rpt_Strt);

/**
 * @brief
 * @param
 */
I2C_ERROR_STATUS_t I2C_SLAVE_SEND_DATA(I2C_TypeDef* pI2Cx, uint8_t* TxBuffer,  uint32_t len);

/**
 * @brief
 * @param
 */
I2C_ERROR_STATUS_t I2C_SLAVE_RECEIVE_DATA(I2C_TypeDef* pI2Cx, uint8_t* RxBuffer,  uint32_t len);

/**
 * @brief
 * @param
 */
I2C_STATUS_t I2C_MSTR_SEND_DATA_IT(I2C_Handle_t* I2C_HANDLE, uint8_t Slave_Addr, uint8_t* TxBuffer, uint32_t len, uint8_t Rpt_Strt);

/*
 * @brief
 * @param
 */
I2C_STATUS_t I2C_MSTR_RECEIVE_DATA_IT(I2C_Handle_t* I2C_HANDLE, uint8_t Slave_Addr, uint8_t* RxBuffer, uint32_t len, uint8_t Rpt_Strt);
/**
 * @brief
 * @param
 */
I2C_STATUS_t I2C_SLAVE_SEND_DATA_IT(I2C_Handle_t* I2C_HANDLE, uint8_t* TxBuffer, uint32_t len);

/**
 * @brief
 * @param
 */
I2C_STATUS_t I2C_SLAVE_RECEIVE_DATA_IT(I2C_Handle_t* I2C_HANDLE, uint8_t* RxBuffer, uint32_t len);


/**
 * @brief
 * @param
 */
void I2C_ITEVT_HANDLE(I2C_Handle_t* I2C_HANDLE);

/**
 * @brief
 * @param
 */
void I2C_ITERR_HANDLE(I2C_Handle_t* I2C_HANDLE);

/**
 * @brief
 * @param
 */
void I2C_IRQ_CFG(uint8_t IRQ_NUMBER, uint8_t ENorDI);

__attribute__((weak)) void I2C_APPEV_CLLBCK(I2C_APPEV_CLLBCK_t APP_EV);
#endif /* INC_STM32F44XX_I2C_H_ */
