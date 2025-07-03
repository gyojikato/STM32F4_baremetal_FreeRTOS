/*
 * stm32f44xx_i2c.c
 *
 *  Created on: May 3, 2025
 *      Author: katog
 */


#include "stm32f44xx_i2c.h"

#define I2C_MAX_TIMEOUT			(0xFFFFF000)
#define I2C_WRITE				(0)
#define I2C_READ				(1)




// static functions start here
static I2C_ERROR_STATUS_t I2C_WAIT_STATUS_SR1(I2C_TypeDef* pI2Cx, uint32_t FLG)
{
	uint32_t prev_tick = DELAY_TICK();
	while(!(pI2Cx->SR1 & FLG)){
		if(DELAY_TICK() - prev_tick  > I2C_MAX_TIMEOUT){
			return I2C_ERROR_TIMEOUT;
		}
	}
	return I2C_SUCCESS;
}


static void close_rx(I2C_Handle_t* I2C_HANDLE)
{
	I2C_HANDLE->RxLen = 0;
	I2C_HANDLE->RxSize = 0;
	I2C_HANDLE->RxBuffer = NULL;
	I2C_HANDLE->I2C_STATUS = I2C_STATUS_READY;
	I2C_HANDLE->pI2Cx->CR2 &= ~(I2C_CR2_ITBUFEN_Msk);
	I2C_HANDLE->pI2Cx->CR2 &= ~(I2C_CR2_ITEVTEN_Msk);
	I2C_HANDLE->pI2Cx->CR2 &= ~(I2C_CR2_ITERREN_Msk);
}
static void close_tx(I2C_Handle_t* I2C_HANDLE)
{
	I2C_HANDLE->TxLen = 0;
	I2C_HANDLE->TxSize = 0;
	I2C_HANDLE->TxBuffer = NULL;
	I2C_HANDLE->I2C_STATUS = I2C_STATUS_READY;
	I2C_HANDLE->pI2Cx->CR2 &= ~(I2C_CR2_ITBUFEN_Msk);
	I2C_HANDLE->pI2Cx->CR2 &= ~(I2C_CR2_ITEVTEN_Msk);
	I2C_HANDLE->pI2Cx->CR2 &= ~(I2C_CR2_ITERREN_Msk);
}

// static functions end here
/**
 * @brief
 * @param
 */
void I2C_PCLK_CTRL(I2C_TypeDef* pI2Cx, uint8_t ENorDI)
{
	if(ENorDI == ENABLE){
		if(pI2Cx == I2C1){
			RCC->APB1ENR |= (RCC_APB1ENR_I2C1EN_Msk);
		}
		else if(pI2Cx == I2C2){
			RCC->APB1ENR |= (RCC_APB1ENR_I2C2EN_Msk);
		}
		else if(pI2Cx == I2C3){
			RCC->APB1ENR |= (RCC_APB1ENR_I2C3EN_Msk);
		}
	}
	else {
		if(pI2Cx == I2C1){
			RCC->APB1ENR &= ~(RCC_APB1ENR_I2C1EN_Msk);
		}
		else if(pI2Cx == I2C2){
			RCC->APB1ENR &= ~(RCC_APB1ENR_I2C2EN_Msk);
		}
		else if(pI2Cx == I2C3){
			RCC->APB1ENR &= ~(RCC_APB1ENR_I2C3EN_Msk);
		}
	}
}

/**
 * @brief Function to initialize I2C peripheral with user configurations
 * @param
 */
void I2C_INIT(I2C_Handle_t* I2C_HANDLE)
{
	I2C_PCLK_CTRL(I2C_HANDLE->pI2Cx, ENABLE);

	uint32_t APB1_CLK = RCC_GET_APB1_CLK();
	uint8_t CR2_FREQ_VALUE = (uint8_t)(APB1_CLK / 1000000);

	// configure the peripheral input clock;
	I2C_HANDLE->pI2Cx->CR2 &= ~(I2C_CR2_FREQ_Msk);
	I2C_HANDLE->pI2Cx->CR2 |= CR2_FREQ_VALUE;

	uint16_t CCR_VALUE;

	// I2C FREQ configurations
	if(I2C_HANDLE->I2C_SPD <= I2C_SPEED_SM){
		// I2C Master mode selection (SM)
		I2C_HANDLE->pI2Cx->CCR &= ~(I2C_CCR_FS_Msk);
		// maximum TRISE of standard mode is 1000ns
		I2C_HANDLE->pI2Cx->TRISE |= (CR2_FREQ_VALUE + 1) & 0x3F;

		CCR_VALUE = (uint16_t)(APB1_CLK / (2 * I2C_HANDLE->I2C_SPD));


	}
	else if(I2C_HANDLE->I2C_SPD > I2C_SPEED_SM && I2C_HANDLE->I2C_SPD <= I2C_SPEED_FM){
		// I2C Master mode selection (FM)
		I2C_HANDLE->pI2Cx->CCR |= (I2C_CCR_FS_Msk);
		// maximum TRISE of fastmode is 300ns
		I2C_HANDLE->pI2Cx->TRISE |= ((300 * (1000 / CR2_FREQ_VALUE)) + 1) & 0x3F;

		if(I2C_HANDLE->I2C_FM_DUTY_MODE == I2C_FM_DUTY_MODE_2){
			I2C_HANDLE->pI2Cx->CR2 &= ~(I2C_CCR_DUTY_Msk);
			CCR_VALUE = (uint16_t)(APB1_CLK / (3 * I2C_HANDLE->I2C_SPD));

		}
		else {
			I2C_HANDLE->pI2Cx->CR2 |= (I2C_CCR_DUTY_Msk);
			CCR_VALUE = (uint16_t)(APB1_CLK / (25 * I2C_HANDLE->I2C_SPD));

		}
	}
	else {
		return;
	}
	I2C_HANDLE->pI2Cx->CCR = CCR_VALUE & 0x3FF;


}

/**
 * @brief
 * @param
 */

void I2C_GEN_START_CONDITION(I2C_TypeDef* pI2Cx)
{
	pI2Cx->CR1 |= (I2C_CR1_START_Msk);
}

/**
 * @brief
 * @param
 */

void I2C_GEN_STOP_CONDITION(I2C_TypeDef* pI2Cx)
{
	pI2Cx->CR1 |= (I2C_CR1_STOP_Msk);
}

/**
 * @brief
 * @param
 */
void I2C_ACK_CTRL(I2C_TypeDef* pI2Cx, uint8_t ENorDI)
{
	if(ENorDI == ENABLE){
		pI2Cx->CR1 |= (I2C_CR1_ACK_Msk);
	}
	else {
		pI2Cx->CR1 &= ~(I2C_CR1_ACK_Msk);
	}
}


/**
 * @brief
 * @param
 */
void I2C_PERI_CTRL(I2C_TypeDef* pI2Cx, uint8_t ENorDI)
{
	if(ENorDI == ENABLE){
		pI2Cx->CR1 |= (I2C_CR1_PE_Msk);
	}
	else {
		pI2Cx->CR1 &= ~(I2C_CR1_PE_Msk);
	}
}

/**
 * @brief
 * @param
 */
void I2C_CLEAR_ADDR(I2C_TypeDef* pI2Cx)
{
	uint32_t dummy_read;
	dummy_read = pI2Cx->SR1;
	dummy_read = pI2Cx->SR2;
	(void)dummy_read;
}

/**
 * @brief
 * @param
 */
void I2C_CLEAR_STOPF(I2C_TypeDef* pI2Cx)
{
	uint32_t dummy_read;
	dummy_read = pI2Cx->SR1;
	pI2Cx->CR1 |= 0x0000;
	(void)dummy_read;
}

/**
 * @brief
 * @param
 */
void I2C_SEND_ADDR(I2C_TypeDef* pI2Cx, uint8_t Slave_Addr, uint8_t RW)
{
	if(RW == I2C_WRITE){
		pI2Cx->DR = ((Slave_Addr << 0x01) & ~(0x01));
	}
	else if(RW == I2C_READ){
		pI2Cx->DR = ((Slave_Addr << 0x01) | (0x01));
	}
}


/**
 * @brief
 * @param
 */
I2C_ERROR_STATUS_t I2C_MSTR_SEND_DATA(I2C_TypeDef* pI2Cx, uint8_t Slave_Addr, uint8_t* TxBuffer, uint32_t len, uint8_t Rpt_Strt)
{
	// Acts as a master after sending the start condition
	I2C_GEN_START_CONDITION(pI2Cx);

	// After start condition, wait until SB flag is set in the SR1 register
	if(I2C_WAIT_STATUS_SR1(pI2Cx, I2C_SR1_SB_Msk) == I2C_ERROR_TIMEOUT){
		return I2C_ERROR_TIMEOUT;
	}

	// Clear SB bit by reading SR1 and sending Slave address to the DR register
	I2C_SEND_ADDR(pI2Cx, Slave_Addr, I2C_WRITE);

	// Wait for ADDR flag to be set and then clear the ADDR flag
	if(I2C_WAIT_STATUS_SR1(pI2Cx, I2C_SR1_ADDR_Msk) == I2C_ERROR_TIMEOUT){
		return I2C_ERROR_TIMEOUT;
	}

	// clear the ADDR flag
	I2C_CLEAR_ADDR(pI2Cx);


	while(len > 0)
	{
		if(I2C_WAIT_STATUS_SR1(pI2Cx, I2C_SR1_TXE_Msk) == I2C_ERROR_TIMEOUT){
			return I2C_ERROR_TIMEOUT;
		}
		pI2Cx->DR = *TxBuffer;
		TxBuffer++;
		len--;
	}

	if(I2C_WAIT_STATUS_SR1(pI2Cx, I2C_SR1_TXE_Msk) == I2C_ERROR_TIMEOUT){
		return I2C_ERROR_TIMEOUT;
	}

	if(I2C_WAIT_STATUS_SR1(pI2Cx, I2C_SR1_BTF_Msk) == I2C_ERROR_TIMEOUT){
		return I2C_ERROR_TIMEOUT;
	}

	if(Rpt_Strt == DISABLE){
		I2C_GEN_STOP_CONDITION(pI2Cx);
	}
	return I2C_SUCCESS;
}


/**
 * @brief
 * @param
 */
I2C_ERROR_STATUS_t I2C_MSTR_RECEIVE_DATA(I2C_TypeDef* pI2Cx, uint8_t Slave_Addr, uint8_t* RxBuffer, uint32_t len, uint8_t Rpt_Strt)
{
	I2C_ACK_CTRL(pI2Cx, ENABLE);
	// Acts as a master after sending the start condition
	I2C_GEN_START_CONDITION(pI2Cx);

	// After start condition, wait until SB flag is set in the SR1 register
	if(I2C_WAIT_STATUS_SR1(pI2Cx, I2C_SR1_SB_Msk) == I2C_ERROR_TIMEOUT){
		return I2C_ERROR_TIMEOUT;
	}

	// Clear SB bit by reading SR1 and sending Slave address to the DR register
	I2C_SEND_ADDR(pI2Cx, Slave_Addr, I2C_READ);

	// Wait for ADDR flag to be set and then clear the ADDR flag
	if(I2C_WAIT_STATUS_SR1(pI2Cx, I2C_SR1_ADDR_Msk) == I2C_ERROR_TIMEOUT){
		return I2C_ERROR_TIMEOUT;
	}

	// case of 1 byte to be received
	if(len == 1){
		// NACK is sent before reception of byte
		I2C_ACK_CTRL(pI2Cx, DISABLE);

		// clear the ADDR flag
		I2C_CLEAR_ADDR(pI2Cx);

		if(Rpt_Strt == DISABLE){
			I2C_GEN_STOP_CONDITION(pI2Cx);
		}

		if(I2C_WAIT_STATUS_SR1(pI2Cx, I2C_SR1_RXNE_Msk) == I2C_ERROR_TIMEOUT){
			return I2C_ERROR_TIMEOUT;
		}
		*RxBuffer = pI2Cx->DR;
		len--;

		return I2C_SUCCESS;

	}

	// clear the ADDR flag
	I2C_CLEAR_ADDR(pI2Cx);

	while(len > 0){

		if(len == 1){
			I2C_ACK_CTRL(pI2Cx, DISABLE);
			if(Rpt_Strt == DISABLE){
				I2C_GEN_STOP_CONDITION(pI2Cx);
			}
		}

		if(I2C_WAIT_STATUS_SR1(pI2Cx, I2C_SR1_RXNE_Msk) == I2C_ERROR_TIMEOUT){
			return I2C_ERROR_TIMEOUT;
		}
		*RxBuffer = pI2Cx->DR;
		RxBuffer++;
		len--;
	}

	return I2C_SUCCESS;
}

/**
 * @brief
 * @param
 */
I2C_ERROR_STATUS_t I2C_SLAVE_SEND_DATA(I2C_TypeDef* pI2Cx, uint8_t* TxBuffer,  uint32_t len)
{
	// wait until address bit is set, meaning slave is selected
	while(!(pI2Cx->SR1 & I2C_SR1_ADDR_Msk));
	I2C_CLEAR_ADDR(pI2Cx);

	while(len > 0){
		if(I2C_WAIT_STATUS_SR1(pI2Cx, I2C_SR1_TXE_Msk) == I2C_ERROR_TIMEOUT){
			return I2C_ERROR_TIMEOUT;
		}
		pI2Cx->DR = *TxBuffer;
		TxBuffer++;
		len--;
	}

	if(I2C_WAIT_STATUS_SR1(pI2Cx, I2C_SR1_AF_Msk) == I2C_ERROR_TIMEOUT){
		return I2C_ERROR_TIMEOUT;
	}

	// clears the AF bit after receiving a NACK
	pI2Cx->SR1 &= ~(I2C_SR1_AF_Msk);

	return I2C_SUCCESS;
}

/**
 * @brief
 * @param
 */
I2C_ERROR_STATUS_t I2C_SLAVE_RECEIVE_DATA(I2C_TypeDef* pI2Cx, uint8_t* RxBuffer,  uint32_t len)
{
	// slave waits for ADDR flag to be set indicating master is talking to the correct slave
	while(!(pI2Cx->SR1 & I2C_SR1_ADDR_Msk));
	I2C_CLEAR_ADDR(pI2Cx);

	while(len > 0){
		if(I2C_WAIT_STATUS_SR1(pI2Cx, I2C_SR1_RXNE_Msk) == I2C_ERROR_TIMEOUT){
			return I2C_ERROR_TIMEOUT;
		}
		*RxBuffer = pI2Cx->DR;
		RxBuffer++;
		len--;
	}

	if(I2C_WAIT_STATUS_SR1(pI2Cx, I2C_SR1_STOPF_Msk) == I2C_ERROR_TIMEOUT){
		return I2C_ERROR_TIMEOUT;
	}
	// STOPF bit is set after successful reception from the master, clear to free SPI
	I2C_CLEAR_STOPF(pI2Cx);

	return I2C_SUCCESS;
}

/**
 * @brief
 * @param
 */
I2C_STATUS_t I2C_MSTR_SEND_DATA_IT(I2C_Handle_t* I2C_HANDLE, uint8_t Slave_Addr, uint8_t* TxBuffer, uint32_t len, uint8_t Rpt_Strt)
{
	if(I2C_HANDLE->I2C_STATUS == I2C_STATUS_READY)
	{
		I2C_HANDLE->SlaveAddress = Slave_Addr;
		I2C_HANDLE->TxBuffer = TxBuffer;
		I2C_HANDLE->TxLen = len;
		I2C_HANDLE->TxSize = len;
		I2C_HANDLE->I2C_RPT_STRT = Rpt_Strt;
		I2C_HANDLE->I2C_STATUS = I2C_STATUS_TX_BUSY;

		I2C_GEN_START_CONDITION(I2C_HANDLE->pI2Cx);
		I2C_HANDLE->pI2Cx->CR2 |= (I2C_CR2_ITEVTEN_Msk);
		I2C_HANDLE->pI2Cx->CR2 |= (I2C_CR2_ITERREN_Msk);
		I2C_HANDLE->pI2Cx->CR2 |= (I2C_CR2_ITBUFEN_Msk);

		return I2C_OK;
	}
	else {
		return I2C_BUSY;
	}
}

/**
 * @brief
 * @param
 */
I2C_STATUS_t I2C_MSTR_RECEIVE_DATA_IT(I2C_Handle_t* I2C_HANDLE, uint8_t Slave_Addr, uint8_t* RxBuffer, uint32_t len, uint8_t Rpt_Strt)
{
	if(I2C_HANDLE->I2C_STATUS == I2C_STATUS_READY){
		I2C_HANDLE->SlaveAddress = Slave_Addr;
		I2C_HANDLE->RxBuffer = RxBuffer;
		I2C_HANDLE->RxLen = len;
		I2C_HANDLE->RxSize = len;
		I2C_HANDLE->I2C_RPT_STRT = Rpt_Strt;
		I2C_HANDLE->I2C_STATUS = I2C_STATUS_RX_BUSY;

		I2C_ACK_CTRL(I2C_HANDLE->pI2Cx, ENABLE);
		I2C_GEN_START_CONDITION(I2C_HANDLE->pI2Cx);
		I2C_HANDLE->pI2Cx->CR2 |= (I2C_CR2_ITEVTEN_Msk);
		I2C_HANDLE->pI2Cx->CR2|= (I2C_CR2_ITERREN_Msk);
		I2C_HANDLE->pI2Cx->CR2 |= (I2C_CR2_ITBUFEN_Msk);

		return I2C_OK;
	}
	else {
		return I2C_BUSY;
	}
}


/**
 * @brief
 * @param
 */
I2C_STATUS_t I2C_SLAVE_SEND_DATA_IT(I2C_Handle_t* I2C_HANDLE, uint8_t* TxBuffer, uint32_t len)
{
	if(I2C_HANDLE->I2C_STATUS == I2C_STATUS_READY)
	{
		I2C_HANDLE->TxBuffer = TxBuffer;
		I2C_HANDLE->TxLen = len;
		I2C_HANDLE->TxSize = len;
		I2C_HANDLE->I2C_STATUS = I2C_STATUS_TX_BUSY;

		I2C_HANDLE->pI2Cx->CR2 |= (I2C_CR2_ITEVTEN_Msk);
		I2C_HANDLE->pI2Cx->CR2 |= (I2C_CR2_ITERREN_Msk);
		I2C_HANDLE->pI2Cx->CR2 |= (I2C_CR2_ITBUFEN_Msk);

		return I2C_OK;
	}
	else {
		return I2C_BUSY;
	}
}

/**
 * @brief
 * @param
 */
I2C_STATUS_t I2C_SLAVE_RECEIVE_DATA_IT(I2C_Handle_t* I2C_HANDLE, uint8_t* RxBuffer, uint32_t len)
{
	if(I2C_HANDLE->I2C_STATUS == I2C_STATUS_READY){
		I2C_HANDLE->RxBuffer = RxBuffer;
		I2C_HANDLE->RxLen = len;
		I2C_HANDLE->RxSize = len;
		I2C_HANDLE->I2C_STATUS = I2C_STATUS_RX_BUSY;

		I2C_ACK_CTRL(I2C_HANDLE->pI2Cx, ENABLE);
		I2C_HANDLE->pI2Cx->CR2 |= (I2C_CR2_ITEVTEN_Msk);
		I2C_HANDLE->pI2Cx->CR2|= (I2C_CR2_ITERREN_Msk);
		I2C_HANDLE->pI2Cx->CR2 |= (I2C_CR2_ITBUFEN_Msk);

		return I2C_OK;
	}
	else {
		return I2C_BUSY;
	}
}

/**
 * @brief
 * @param
 */
void I2C_ITEVT_HANDLE(I2C_Handle_t* I2C_HANDLE)
{
	if(!(I2C_HANDLE->pI2Cx->CR2 & (I2C_CR2_ITEVTEN_Msk))){
		// I2C interrupt is not generated by an event proceed to error handle
		return;
	}

	uint32_t tempreg;
	tempreg = I2C_HANDLE->pI2Cx->SR1;
	if(tempreg & I2C_SR1_SB_Msk){
		if(I2C_HANDLE->I2C_STATUS == I2C_STATUS_TX_BUSY){

			I2C_SEND_ADDR(I2C_HANDLE->pI2Cx, I2C_HANDLE->SlaveAddress, I2C_WRITE);

		}
		else if(I2C_HANDLE->I2C_STATUS == I2C_STATUS_RX_BUSY){

			I2C_SEND_ADDR(I2C_HANDLE->pI2Cx, I2C_HANDLE->SlaveAddress, I2C_READ);

		}
	}
	if(tempreg & I2C_SR1_ADDR_Msk){
		if(I2C_HANDLE->I2C_STATUS == I2C_STATUS_TX_BUSY){

			I2C_CLEAR_ADDR(I2C_HANDLE->pI2Cx);

		}
		else if(I2C_HANDLE->I2C_STATUS == I2C_STATUS_RX_BUSY){

			if((I2C_HANDLE->RxSize == 1) && (I2C_HANDLE->pI2Cx->SR2 & I2C_SR2_MSL_Msk)){
				I2C_ACK_CTRL(I2C_HANDLE->pI2Cx, DISABLE);
				I2C_GEN_STOP_CONDITION(I2C_HANDLE->pI2Cx);
			}
			I2C_CLEAR_ADDR(I2C_HANDLE->pI2Cx);
		}
	}
	if(tempreg & I2C_SR1_BTF_Msk)
	{
		if(I2C_HANDLE->I2C_STATUS == I2C_STATUS_TX_BUSY){
			I2C_GEN_STOP_CONDITION(I2C_HANDLE->pI2Cx);
			close_tx(I2C_HANDLE);
			I2C_APPEV_CLLBCK(I2C_APPEV_CLLBCK_TX_CMPLT);
		}

	}
	if(tempreg & I2C_SR1_STOPF_Msk){
		if(!(I2C_HANDLE->pI2Cx->SR1 & I2C_SR2_MSL_Msk)){
			I2C_CLEAR_STOPF(I2C_HANDLE->pI2Cx);
			close_rx(I2C_HANDLE);
		}
	}
	if(tempreg & I2C_SR1_TXE_Msk && I2C_HANDLE->I2C_STATUS == I2C_STATUS_TX_BUSY){
		if(I2C_HANDLE->TxLen > 0){
			I2C_HANDLE->pI2Cx->DR = *(I2C_HANDLE->TxBuffer);
			I2C_HANDLE->TxBuffer++;
			I2C_HANDLE->TxLen--;
		}
	}
	if(tempreg & I2C_SR1_RXNE_Msk && I2C_HANDLE->I2C_STATUS == I2C_STATUS_RX_BUSY){

		if(I2C_HANDLE->RxLen == 2){
			// check if device is running on master
			if(I2C_HANDLE->pI2Cx->SR2 & I2C_SR2_MSL_Msk){

				I2C_ACK_CTRL(I2C_HANDLE->pI2Cx, DISABLE);
				if(I2C_HANDLE->I2C_RPT_STRT == DISABLE){

					I2C_GEN_STOP_CONDITION(I2C_HANDLE->pI2Cx);
				}
			}
		}
		*(I2C_HANDLE->RxBuffer) = I2C_HANDLE->pI2Cx->DR;
		I2C_HANDLE->RxBuffer++;
		I2C_HANDLE->RxLen--;

		if(I2C_HANDLE->RxLen == 0){
			I2C_APPEV_CLLBCK(I2C_APPEV_CLLBCK_RX_CMPLT);
			close_rx(I2C_HANDLE);
		}
	}

}

/**
 * @brief
 * @param
 */
void I2C_ITERR_HANDLE(I2C_Handle_t* I2C_HANDLE)
{
	if(!(I2C_HANDLE->pI2Cx->CR2 & I2C_CR2_ITERREN_Msk)){
		// I2C interrupt is not due to error since I2C_CR2_ITERREN is not set
		return;
	}
	uint32_t tempreg;
	tempreg = I2C_HANDLE->pI2Cx->SR1;
	if(tempreg & I2C_SR1_BERR_Msk){
		I2C_HANDLE->pI2Cx->SR1 &= ~(I2C_SR1_BERR_Msk);
		I2C_APPEV_CLLBCK(I2C_APPEV_CLLBCK_BERR_OCCUR);
	}
	if(tempreg & I2C_SR1_ARLO_Msk){
		I2C_HANDLE->pI2Cx->SR1 &= ~(I2C_SR1_ARLO_Msk);
		I2C_APPEV_CLLBCK(I2C_APPEV_CLLBCK_ARLO_OCCUR);
	}
	if(tempreg & I2C_SR1_AF_Msk){
		I2C_HANDLE->pI2Cx->SR1 &= ~(I2C_SR1_AF_Msk);
		if(I2C_HANDLE->pI2Cx->SR2 & I2C_SR2_MSL_Msk)
		{
			I2C_GEN_STOP_CONDITION(I2C_HANDLE->pI2Cx);

		}
		close_tx(I2C_HANDLE);
		I2C_APPEV_CLLBCK(I2C_APPEV_CLLBCK_AF_OCCUR);

	}
	if(tempreg & I2C_SR1_OVR_Msk){
		I2C_HANDLE->pI2Cx->SR1 &= ~(I2C_SR1_OVR_Msk);
		I2C_APPEV_CLLBCK(I2C_APPEV_CLLBCK_OVR_OCCUR);
	}
	if(tempreg & I2C_SR1_PECERR_Msk){
		I2C_HANDLE->pI2Cx->SR1 &= ~(I2C_SR1_PECERR_Msk);
		I2C_APPEV_CLLBCK(I2C_APPEV_CLLBCK_PECERR_OCCUR);
	}
	if(tempreg & I2C_SR1_TIMEOUT_Msk){
		I2C_HANDLE->pI2Cx->SR1 &= ~(I2C_SR1_TIMEOUT_Msk);
		I2C_APPEV_CLLBCK(I2C_APPEV_CLLBCK_TIMEOUT_OCCUR);
	}

}

/**
 * @brief
 * @param
 */
void I2C_IRQ_CFG(uint8_t IRQ_NUMBER, uint8_t ENorDI)
{
	if(ENorDI == ENABLE){
		NVIC_EnableIRQ(IRQ_NUMBER);
	}
	else if(ENorDI == DISABLE){
		NVIC_DisableIRQ(IRQ_NUMBER);
	}
}

__attribute__((weak)) void I2C_APPEV_CLLBCK(I2C_APPEV_CLLBCK_t APP_EV){

}
