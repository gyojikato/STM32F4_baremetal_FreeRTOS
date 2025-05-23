/*
 * stm32f44xx_spi.c
 *
 *  Created on: Apr 20, 2025
 *      Author: katog
 */


#include "stm32f44xx_spi.h"
#include <stddef.h>


/* HELPER FUNCTIONS START HERE * */
/**
 * @brief Function to get the state of specific flags in the status register
 * @param
 */
static ErrorStatus SPI_WAIT_FLG_STATUS(SPI_TypeDef* pSPIx, uint32_t FLG)
{
	uint32_t prev_tick = GET_CURR_TICK();
	while(!(pSPIx->SR & FLG)){

		if((GET_CURR_TICK() - prev_tick) > SPI_MAX_TIMEOUT){
			return ERROR;
		}
	}
	return SUCCESS;
}

/**
 * @brief
 * @param
 */
static void txe_handle(SPI_Handle_t* SPI_HANDLE)
{
	if(SPI_HANDLE->SPI_DFF_SIZE == SPI_DFF_8BITS){
		SPI_HANDLE->pSPIx->DR = *(SPI_HANDLE->TxBuffer);
		SPI_HANDLE->TxBuffer++;
	}
	else if(SPI_HANDLE->SPI_DFF_SIZE == SPI_DFF_16BITS){
		SPI_HANDLE->pSPIx->DR = *(uint16_t*)SPI_HANDLE->TxBuffer;
		SPI_HANDLE->TxBuffer++;
		SPI_HANDLE->TxBuffer++;
	}
	SPI_HANDLE->Txlen--;

	if(SPI_HANDLE->Txlen == 0){
		SPI_HANDLE->TxBuffer = NULL;
		SPI_HANDLE->SPI_BUSY_STATE = SPI_READY;
		SPI_HANDLE->pSPIx->CR2 &= ~(SPI_CR2_TXEIE_Msk);
		SPI_APP_CLKBK(SPI_HANDLE, SPI_TX_CMPLT);
	}
}

/**
 * @brief
 * @param
 */
static void rxne_handle(SPI_Handle_t* SPI_HANDLE)
{

	if(SPI_HANDLE->SPI_DFF_SIZE == SPI_DFF_8BITS){
		*(SPI_HANDLE->RxBuffer) = SPI_HANDLE->pSPIx->DR;
		SPI_HANDLE->RxBuffer++;
	}
	else if(SPI_HANDLE->SPI_DFF_SIZE == SPI_DFF_16BITS){
		*(uint16_t*)SPI_HANDLE->RxBuffer = SPI_HANDLE->pSPIx->DR;
		SPI_HANDLE->RxBuffer++;
		SPI_HANDLE->RxBuffer++;
	}
	SPI_HANDLE->Rxlen--;

	if(SPI_HANDLE->Rxlen == 0){
		SPI_HANDLE->RxBuffer = NULL;
		SPI_HANDLE->SPI_BUSY_STATE = SPI_READY;
		SPI_HANDLE->pSPIx->CR2 &= ~(SPI_CR2_RXNEIE_Msk);
		SPI_APP_CLKBK(SPI_HANDLE, SPI_RX_CMPLT);
	}

}

static void ovr_handle(SPI_Handle_t* SPI_HANDLE)
{
	uint32_t dummy_read;
	dummy_read = SPI_HANDLE->pSPIx->DR;
	dummy_read = SPI_HANDLE->pSPIx->SR;
	(void)dummy_read;
	SPI_APP_CLKBK(SPI_HANDLE, SPI_OVR_OCCUR);
}

/* HELPER FUNCTIONS END HERE */
/**
 * @brief Function to enable or disable the peripheral clock of SPI
 * @param
 */
ErrorStatus SPI_PCLK_CTRL(SPI_TypeDef* pSPIx, FunctionalState ENorDI)
{
	if(ENorDI == ENABLE){
		if(pSPIx == SPI1){
			RCC->APB2ENR |= (RCC_APB2ENR_SPI1EN_Msk);
		}
		else if(pSPIx == SPI2){
			RCC->APB1ENR |= (RCC_APB1ENR_SPI2EN_Msk);
		}
		else if(pSPIx == SPI3){
			RCC->APB1ENR |= (RCC_APB1ENR_SPI3EN_Msk);
		}
		else if(pSPIx == SPI4){
			RCC->APB2ENR |= (RCC_APB2ENR_SPI4EN_Msk);
		}
		else {
			return ERROR;
		}
	}
	else if(ENorDI == DISABLE){
		if(pSPIx == SPI1){
				RCC->APB2ENR &= ~(RCC_APB2ENR_SPI1EN_Msk);
			}
			else if(pSPIx == SPI2){
				RCC->APB1ENR &= ~(RCC_APB1ENR_SPI2EN_Msk);
			}
			else if(pSPIx == SPI3){
				RCC->APB1ENR &= ~(RCC_APB1ENR_SPI3EN_Msk);
			}
			else if(pSPIx == SPI4){
				RCC->APB2ENR &= ~(RCC_APB2ENR_SPI4EN_Msk);
			}
			else {
				return ERROR;
			}
	}
	return SUCCESS;
}

void SPI_PERI_CTRL(SPI_TypeDef* pSPIx, FunctionalState ENorDI)
{
	if(ENorDI == ENABLE){
		pSPIx->CR1 |= (SPI_CR1_SPE_Msk);
	}
	else {
		while(pSPIx->SR >> (SPI_SR_BSY_Pos) & 0x01);
		pSPIx->CR1 &= ~(SPI_CR1_SPE_Msk);
	}
}

/**
 * @brief Function to initialize SPI peripheral with user configurations
 * @param
 */
ErrorStatus SPI_INIT(SPI_Handle_t* SPI_HANDLE)
{
	uint32_t tempreg;
	// SPI_CR1 configurations
	// SPI Baud configurations

	if(SPI_PCLK_CTRL(SPI_HANDLE->pSPIx, ENABLE) == ERROR){
		return ERROR;
	}

	tempreg = SPI_HANDLE->pSPIx->CR1;
	if(SPI_HANDLE->SPI_BAUD_RATE == SPI_BAUD_DIV2){
		tempreg &= ~(SPI_CR1_BR_Msk);
	}
	else if(SPI_HANDLE->SPI_BAUD_RATE <= SPI_BAUD_DIV256){
		MODIFY_REG(tempreg, SPI_CR1_BR_Msk, SPI_HANDLE->SPI_BAUD_RATE << SPI_CR1_BR_Pos);
	}
	else {
		return ERROR;
	}

	// Master selection
	if(SPI_HANDLE->SPI_DEVICE_MODE == SPI_DEVICE_MODE_SLAVE){
		tempreg &= ~(SPI_CR1_MSTR_Msk);
	}
	else if(SPI_HANDLE->SPI_DEVICE_MODE == SPI_DEVICE_MODE_MSTR){
		tempreg |= (SPI_CR1_MSTR_Msk);
	}
	// SPI communications mode configuration
	if((SPI_HANDLE->SPI_MODE == SPI_MODE_FULL_DPLX) ||
		(SPI_HANDLE->SPI_MODE == SPI_MODE_SMPLX_TX)){
		tempreg &= ~(SPI_CR1_BIDIMODE_Msk);
		tempreg &= ~(SPI_CR1_RXONLY_Msk);
	}
	else if (SPI_HANDLE->SPI_MODE == SPI_MODE_HALF_DPLX){
		tempreg |= (SPI_CR1_BIDIMODE_Msk);
		tempreg &= ~(SPI_CR1_RXONLY_Msk);
	}
	else if(SPI_HANDLE->SPI_MODE == SPI_MODE_SMPLX_RX){
		tempreg &= ~(SPI_CR1_BIDIMODE_Msk);
		tempreg |= (SPI_CR1_RXONLY_Msk);
	}
	else {
		return ERROR;
	}

	// SPI_CPOL configurations
	if(SPI_HANDLE->SPI_CPOL_STATE == SPI_CPOL_LO){
		tempreg &= ~(SPI_CR1_CPOL_Msk);
	}
	else if(SPI_HANDLE->SPI_CPOL_STATE == SPI_CPOL_HI){
		tempreg |= (SPI_CR1_CPOL_Msk);
	}
	else {
		return ERROR;
	}

	// SPI_CPHA configurations
	if(SPI_HANDLE->SPI_CPHA_STATE == SPI_CPHA_HI){
		tempreg |= (SPI_CR1_CPHA_Msk);
	}
	else if(SPI_HANDLE->SPI_CPHA_STATE == SPI_CPHA_LO){
		tempreg &= ~(SPI_CR1_CPHA_Msk);
	}
	else {
		return ERROR;
	}

	//SPI data frame format configurations
	if(SPI_HANDLE->SPI_DFF_SIZE == SPI_DFF_8BITS){
		tempreg  &= ~(SPI_CR1_DFF_Msk);
	}
	else if(SPI_HANDLE->SPI_DFF_SIZE == SPI_DFF_16BITS){
		tempreg  |= (SPI_CR1_DFF_Msk);
	}
	else {
		return ERROR;
	}

	// software slave management configuration
	if(SPI_HANDLE->SPI_SSM_STATE == ENABLE){
		tempreg |= (SPI_CR1_SSM_Msk);
		/*SPI_HANDLE->pSPIx->CR2 &= ~(SPI_CR2_SSOE_Msk);*/
	}
	else {
		// if no software slave management, SSOE bit should be set for SPI to work properly
		tempreg &= ~(SPI_CR1_SSM_Msk);
		/*SPI_HANDLE->pSPIx->CR2 |= (SPI_CR2_SSOE_Msk);*/
	}
	SPI_HANDLE->pSPIx->CR1 = tempreg;

	return SUCCESS;
}

/**
 * @brief Function to send data
 * @param
 */
void SPI_SEND_DATA(SPI_TypeDef* pSPIx, uint8_t* buffer, uint32_t len)
{
	while(len > 0){
		while(!(pSPIx->SR & (SPI_SR_TXE_Msk)));
		if(pSPIx->CR1 & SPI_CR1_DFF_Msk){
			pSPIx->DR = *(uint16_t*)buffer;
			buffer++;
			buffer++;
		}
		else {
			pSPIx->DR = *buffer;
			buffer++;
		}
		len--;
	}
}

/**
 * @brief Function to receive data
 * @param
 */
void SPI_RCV_DATA(SPI_TypeDef* pSPIx, uint8_t* buffer, uint32_t len)
{
	while(len > 0){
		if(SPI_WAIT_FLG_STATUS(pSPIx, SPI_SR_RXNE_Msk) == ERROR){
			// timeout error occured
			return;
		}
		if(pSPIx->CR1 & SPI_CR1_DFF_Msk){
			*(uint16_t*)buffer = pSPIx->DR;
			buffer++;
			buffer++;
		}
		else {
			*buffer = pSPIx->DR;
			buffer++;
		}
		len--;
	}
}

/**
 * @brief Function to send data using interrupt
 * @param
 */
SPI_BUSY_STATE_t SPI_SEND_DATA_IT(SPI_Handle_t* SPI_HANDLE, uint8_t* buffer, uint32_t len)
{
	if(SPI_HANDLE->SPI_BUSY_STATE == SPI_READY){

		SPI_HANDLE->TxBuffer = buffer;
		SPI_HANDLE->Txlen = len;
		SPI_HANDLE->SPI_BUSY_STATE = SPI_BUSY_IN_TX;
		SPI_HANDLE->pSPIx->CR2 |= (SPI_CR2_TXEIE_Msk);

		return SPI_OK;
	}
	else {
		return SPI_BUSY;
	}
}

/**
 * @brief Function to receive data
 * @param
 */
SPI_BUSY_STATE_t SPI_RCV_DATA_IT(SPI_Handle_t* SPI_HANDLE, uint8_t* buffer, uint32_t len)
{
	if(SPI_HANDLE->SPI_BUSY_STATE == SPI_READY){

		SPI_HANDLE->RxBuffer = buffer;
		SPI_HANDLE->Rxlen = len;
		SPI_HANDLE->SPI_BUSY_STATE = SPI_BUSY_IN_RX;
		SPI_HANDLE->pSPIx->CR2 |= (SPI_CR2_RXNEIE_Msk);

		return SPI_OK;
	}
	else {
		return SPI_BUSY;
	}
}

void SPI_IRQ_HANDLE(SPI_Handle_t* SPI_HANDLE)
{
	uint32_t tempreg_SR = (SPI_HANDLE->pSPIx->SR & SPI_SR_TXE_Msk);
	uint32_t tempreg_CR2 = (SPI_HANDLE->pSPIx->CR2 & SPI_CR2_TXEIE_Msk);
	if(tempreg_SR && tempreg_CR2){
		txe_handle(SPI_HANDLE);
	}

	tempreg_SR = (SPI_HANDLE->pSPIx->SR & SPI_SR_RXNE_Msk);
	tempreg_CR2 = (SPI_HANDLE->pSPIx->CR2 & SPI_CR2_RXNEIE_Msk);
	if(tempreg_SR && tempreg_CR2){
		rxne_handle(SPI_HANDLE);
	}

	tempreg_SR = (SPI_HANDLE->pSPIx->SR & SPI_SR_OVR_Msk);
	tempreg_CR2 = (SPI_HANDLE->pSPIx->CR2 & SPI_CR2_ERRIE_Msk);
	if(tempreg_SR && tempreg_CR2)
	{
		ovr_handle(SPI_HANDLE);
	}
}


/**
 * @brief Function to enable or disable the SSOE bit in SPI_CR2 to avoid MODF errors
 * @param
 */
void SSOE_BIT_CONTROL(SPI_TypeDef* pSPIx, FunctionalState ENorDI)
{
	if(ENorDI == ENABLE){
		pSPIx->CR2 |= (SPI_CR2_SSOE_Msk);
	}
	else {
		pSPIx->CR2 &= ~(SPI_CR2_SSOE_Msk);
	}
}

/**
 * @brief
 * @param
 */
void SPI_IRQ_CFG(uint8_t IRQ_NUMBER, FunctionalState ENorDI)
{
	if(ENorDI == ENABLE){
		NVIC_EnableIRQ(IRQ_NUMBER);
	}
	else if(ENorDI == DISABLE){
		NVIC_DisableIRQ(IRQ_NUMBER);
	}
}
__attribute__((weak)) void SPI_APP_CLKBK(SPI_Handle_t* SPI_HANDLE, SPI_APP_EV_t SPI_APPEV)
{

}
