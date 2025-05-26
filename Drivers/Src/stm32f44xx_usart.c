/*
 * stm32f44xx_usart.c
 *
 *  Created on: May 11, 2025
 *      Author: katog
 */

#include "stm32f44xx_usart.h"

#define GET_USART_PCLK_VALUE(pUSARTx) 	(pUSARTx == USART1 || pUSARTx == USART6) ? RCC_GET_APB2_CLK() : RCC_GET_APB1_CLK()
static void usart_close_tx(USART_Handle_t* USART_HANDLE){

	USART_HANDLE->TX_len = 0;
	USART_HANDLE->TxBuffer = NULL;
	USART_HANDLE->USART_TX_STATUS = USART_STATUS_READY;
	USART_HANDLE->pUSARTx->CR1 &= ~(USART_CR1_TCIE_Msk);
	USART_HANDLE->pUSARTx->CR1 &= ~(USART_CR1_TXEIE_Msk);

}
static void usart_close_rx(USART_Handle_t* USART_HANDLE){
	USART_HANDLE->RX_len = 0;
	USART_HANDLE->RxBuffer = NULL;
	USART_HANDLE->USART_RX_STATUS = USART_STATUS_READY;
	if(USART_HANDLE->pUSARTx->CR1 & USART_CR1_PCE_Msk)
	{
		USART_HANDLE->pUSARTx->CR1 &= ~(USART_CR1_PEIE_Msk);
	}

	USART_HANDLE->pUSARTx->CR1 &= ~(USART_CR1_RXNEIE_Msk);

}

static void USART_BRR_CALC(USART_TypeDef* pUSARTx, USART_BAUD_RATE_t BaudRate)
{
	uint8_t OVR8_value = (pUSARTx->CR1 >> USART_CR1_OVER8_Pos) & 0x01;

	uint32_t PCLK_value = GET_USART_PCLK_VALUE(pUSARTx);

	uint16_t USARTDIV = ((25 * PCLK_value) / ((2 - OVR8_value) * BaudRate * 2));

	uint16_t DIV_mantissa = (USARTDIV / 100);

	uint16_t DIV_frac = (USARTDIV - (DIV_mantissa * 100));

	if(OVR8_value){
		pUSARTx->BRR = (DIV_mantissa << 4) | ((DIV_frac / 16) & 0x0F);
	}
	else {
		pUSARTx->BRR = (DIV_mantissa << 4) | ((DIV_frac / 8) & 0x07);
	}
}

void USART_PCLK_CTRL(USART_TypeDef* pUSARTx, uint8_t ENorDI)
{
	if(ENorDI == ENABLE){
		if(pUSARTx == USART1){
			RCC->APB2ENR |= (RCC_APB2ENR_USART1EN_Msk);
		}
		else if(pUSARTx == USART2){
			RCC->APB1ENR |= (RCC_APB1ENR_USART2EN_Msk);
		}
		else if(pUSARTx == USART3){
			RCC->APB1ENR |= (RCC_APB1ENR_USART3EN_Msk);
		}
		else if(pUSARTx == UART4){
			RCC->APB1ENR |= (RCC_APB1ENR_UART4EN_Msk);
		}
		else if(pUSARTx == UART5){
			RCC->APB1ENR |= (RCC_APB1ENR_UART5EN_Msk);
		}
		else if(pUSARTx == USART6){
			RCC->APB2ENR |= (RCC_APB2ENR_USART6EN_Msk);
		}
	}
	else {
		if(pUSARTx == USART1){
			RCC->APB2ENR &= ~(RCC_APB2ENR_USART1EN_Msk);
		}
		else if(pUSARTx == USART2){
			RCC->APB1ENR &= ~(RCC_APB1ENR_USART2EN_Msk);
		}
		else if(pUSARTx == USART3){
			RCC->APB1ENR&= ~(RCC_APB1ENR_USART3EN_Msk);
		}
		else if(pUSARTx == UART4){
			RCC->APB1ENR &= ~(RCC_APB1ENR_UART4EN_Msk);
		}
		else if(pUSARTx == UART5){
			RCC->APB1ENR &= ~(RCC_APB1ENR_UART5EN_Msk);
		}
		else if(pUSARTx == USART6){
			RCC->APB2ENR &= ~(RCC_APB2ENR_USART6EN_Msk);
		}
	}
}

void USART_INIT(USART_Handle_t* USART_HANDLE)
{
	uint32_t tempreg = 0;
	USART_PCLK_CTRL(USART_HANDLE->pUSARTx, ENABLE);

	if(USART_HANDLE->USART_MODE <= USART_MODE_RXTX){
		tempreg |= (USART_HANDLE->USART_MODE << USART_CR1_RE_Pos);
	}

	if(USART_HANDLE->USART_OVER_VAL <= USART_OVER_8){
		tempreg |= (USART_HANDLE->USART_OVER_VAL << USART_CR1_OVER8_Pos);
	}

	if(USART_HANDLE->USART_WORDLEN <= USART_WORD_LEN_9BITS){
		tempreg |= (USART_HANDLE->USART_WORDLEN << USART_CR1_M_Pos);
	}
	if(!(USART_HANDLE->USART_PARITY_CTRL == USART_PARITY_DISABLE)){
		tempreg |= (USART_CR1_PCE_Msk);
	}
	if(USART_HANDLE->USART_PARITY_CTRL == USART_PARITY_ODD){
		tempreg |= (USART_CR1_PS_Msk);
	}
	USART_HANDLE->pUSARTx->CR1 = tempreg;



	USART_BRR_CALC(USART_HANDLE->pUSARTx, USART_HANDLE->USART_BAUDRATE);

}

void USART_DE_INIT(USART_TypeDef* pUSARTx);

void USART_CTRL(USART_TypeDef* pUSARTx, uint8_t ENorDI)
{
	if(ENorDI == ENABLE){
		pUSARTx->CR1 |= (USART_CR1_UE_Msk);
	}
	else {
		pUSARTx->CR1 &= ~(USART_CR1_UE_Msk);
	}
}

void USART_SEND_DATA(USART_TypeDef* pUSARTx, uint8_t* TxBuffer, uint32_t len)
{
	uint8_t FrameLen = ((pUSARTx->CR1 >> USART_CR1_M_Pos) & 0x01);

	uint8_t ParityEn = ((pUSARTx->CR1 >> USART_CR1_PCE_Pos) & 0x01);

	while(len > 0){
		while(!(pUSARTx->SR & USART_SR_TXE_Msk));
		if(FrameLen == USART_WORD_LEN_8BITS){
			if(ParityEn){
				pUSARTx->DR = (*TxBuffer & 0x7F);
			}
			else {
				pUSARTx->DR = *TxBuffer;
			}
			TxBuffer++;
		}
		else if(FrameLen == USART_WORD_LEN_9BITS){
			if(ParityEn){
				pUSARTx->DR = *TxBuffer;
				TxBuffer++;
			}
			else {
				pUSARTx->DR = *((uint16_t*)TxBuffer) & 0x01FF;
				TxBuffer++;
				TxBuffer++;
			}
		}
		len--;
	}

	while(!(pUSARTx->SR & USART_SR_TC_Msk));
	pUSARTx->SR &= ~(USART_SR_TC_Msk);
}

void USART_RECEIVE_DATA(USART_TypeDef* pUSARTx, uint8_t* RxBuffer, uint32_t len)
{
	uint8_t FrameLen = ((pUSARTx->CR1 >> USART_CR1_M_Pos) & 0x01);

		uint8_t ParityEn = ((pUSARTx->CR1 >> USART_CR1_PCE_Pos) & 0x01);

		while(len > 0){
			while(!(pUSARTx->SR & USART_SR_RXNE_Msk));
			if(FrameLen == USART_WORD_LEN_8BITS){
				if(ParityEn){
					*RxBuffer = pUSARTx->DR & 0x7F;
				}
				else {
					*RxBuffer = pUSARTx->DR;
				}
				RxBuffer++;
			}
			else if(FrameLen == USART_WORD_LEN_9BITS){
				if(ParityEn){
					*RxBuffer =	pUSARTx->DR;
					RxBuffer++;
				}
				else {
					*((uint16_t*)RxBuffer) = pUSARTx->DR & 0x01FF;
					RxBuffer++;
					RxBuffer++;
				}
			}
			len--;
		}
}


USART_STATUS_t USART_SEND_DATA_IT(USART_Handle_t* USART_HANDLE, uint8_t* TxBuffer, uint32_t len)
{
	if(USART_HANDLE->USART_TX_STATUS == USART_STATUS_READY){
		USART_HANDLE->TX_len = len;
		USART_HANDLE->TxBuffer = TxBuffer;
		USART_HANDLE->USART_TX_STATUS = USART_STATUS_TX_BUSY;

		USART_HANDLE->pUSARTx->CR1 |= (USART_CR1_TCIE_Msk);
		USART_HANDLE->pUSARTx->CR1 |= (USART_CR1_TXEIE_Msk);

		return USART_OK;
	}
	else {
		return USART_BUSY;

	}
}

USART_STATUS_t USART_RECEIVE_DATA_IT(USART_Handle_t* USART_HANDLE, uint8_t* RxBuffer, uint32_t len)
{
	if(USART_HANDLE->USART_RX_STATUS == USART_STATUS_READY){
		USART_HANDLE->RX_len = len;
		USART_HANDLE->RxBuffer = RxBuffer;
		USART_HANDLE->USART_RX_STATUS = USART_STATUS_RX_BUSY;

		if(USART_HANDLE->pUSARTx->CR1 & USART_CR1_PCE_Msk)
		{
			USART_HANDLE->pUSARTx->CR1 |= (USART_CR1_PEIE_Msk);
		}

		USART_HANDLE->pUSARTx->CR1 |= (USART_CR1_RXNEIE_Msk);
		return USART_OK;
	}
	else {
		return USART_BUSY;
	}
}

void USART_IRQ_HANDLE(USART_Handle_t* USART_HANDLE)
{
	uint32_t tempreg = USART_HANDLE->pUSARTx->SR;

	if(tempreg & USART_SR_TC_Msk){
		if(USART_HANDLE->TX_len == 0 && USART_HANDLE->USART_TX_STATUS == USART_STATUS_TX_BUSY){
			usart_close_tx(USART_HANDLE);
			USART_APPEV_CLLBCK(USART_TX_SUCCESS);
		}
	}

	tempreg = USART_HANDLE->pUSARTx->SR;
	if(tempreg & USART_SR_TXE_Msk && USART_HANDLE->USART_TX_STATUS == USART_STATUS_TX_BUSY){
		if(USART_HANDLE->TX_len > 0){
			if(USART_HANDLE->USART_WORDLEN == USART_WORD_LEN_9BITS){
				if(USART_HANDLE->USART_PARITY_CTRL != USART_PARITY_DISABLE){
					USART_HANDLE->pUSARTx->DR = *USART_HANDLE->TxBuffer;
					USART_HANDLE->TxBuffer++;
				}
				else {
					USART_HANDLE->pUSARTx->DR = (*(uint16_t*)USART_HANDLE->TxBuffer) & 0x01FF;
					USART_HANDLE->TxBuffer++;
					USART_HANDLE->TxBuffer++;

				}
			}
			else {
				// transmitted data is 8 bit length
				if(USART_HANDLE->USART_PARITY_CTRL != USART_PARITY_DISABLE){
					USART_HANDLE->pUSARTx->DR = *USART_HANDLE->TxBuffer & 0x7F;
				}
				else {
					USART_HANDLE->pUSARTx->DR = *USART_HANDLE->TxBuffer;
				}
				USART_HANDLE->TxBuffer++;
			}
			USART_HANDLE->TX_len--;
		}
	}

	tempreg = USART_HANDLE->pUSARTx->SR;
	if(tempreg & USART_SR_RXNE_Msk && USART_HANDLE->USART_RX_STATUS == USART_STATUS_RX_BUSY){
		if(USART_HANDLE->RX_len > 0){
			if(USART_HANDLE->USART_WORDLEN == USART_WORD_LEN_9BITS){
				if(USART_HANDLE->USART_PARITY_CTRL != USART_PARITY_DISABLE){
					*USART_HANDLE->RxBuffer = USART_HANDLE->pUSARTx->DR;
					USART_HANDLE->TxBuffer++;
				}
				else {
					(*(uint16_t*)USART_HANDLE->RxBuffer) = USART_HANDLE->pUSARTx->DR  & 0x01FF;
					USART_HANDLE->RxBuffer++;
					USART_HANDLE->RxBuffer++;

				}
			}
			else {
				// transmitted data is 8 bit length
				if(USART_HANDLE->USART_PARITY_CTRL != USART_PARITY_DISABLE){
					*USART_HANDLE->RxBuffer = USART_HANDLE->pUSARTx->DR & 0x7F;
				}
				else {
					*USART_HANDLE->RxBuffer = USART_HANDLE->pUSARTx->DR;
				}
				USART_HANDLE->RxBuffer++;
			}
			USART_HANDLE->RX_len--;

			if(USART_HANDLE->RX_len == 0){
				usart_close_rx(USART_HANDLE);
				USART_APPEV_CLLBCK(USART_RX_SUCCESS);
			}
		}
		else {
			usart_close_rx(USART_HANDLE);
			USART_APPEV_CLLBCK(USART_RX_SUCCESS);
		}
	}

	tempreg = USART_HANDLE->pUSARTx->SR;
	if((tempreg & USART_SR_PE_Msk) && (tempreg && USART_SR_RXNE_Msk) && USART_HANDLE->USART_PARITY_CTRL != USART_PARITY_DISABLE){
		// clear the Parity Error flag
		uint8_t dummy_read = USART_HANDLE->pUSARTx->DR;
		(void)dummy_read;
		USART_APPEV_CLLBCK(USART_PE_OCCUR);
	}
}


void USART_IRQ_CFG(uint8_t IRQ_NUMBER, uint8_t ENorDI)
{
	if(ENorDI == ENABLE){
		NVIC_EnableIRQ(IRQ_NUMBER);
	}
	else if(ENorDI == DISABLE){
		NVIC_DisableIRQ(IRQ_NUMBER);
	}
}

__attribute__((weak)) void USART_APPEV_CLLBCK(USART_APPEV_CLLBCK_t APP_EV){

}
