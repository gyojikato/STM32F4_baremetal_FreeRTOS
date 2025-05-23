/*
 * stm32f44xx_rcc.c
 *
 *  Created on: May 3, 2025
 *      Author: katog
 */

#include "stm32f44xx_rcc.h"


#define HSE_VALUE    ((uint32_t)25000000) /*!< Default value of the External oscillator in Hz */

#define HSI_VALUE    ((uint32_t)16000000) /*!< Value of the Internal oscillator in Hz*/

#define HSI			(0x00)
#define HSE			(0x01)
#define PLL_P		(0x02)
#define PLL_R		(0x03)

static uint32_t GET_SYS_CLOCK()
{
	uint8_t CFGR_SW;

	// get the selected system clock
	CFGR_SW = (RCC->CFGR & RCC_CFGR_SW_Msk);

	switch(CFGR_SW) {

		case HSI:
			return HSI_VALUE;

		case HSE:
			return HSE_VALUE;

		case PLL_P:
			return 0;

		case PLL_R:
			return 0;
		default:
			return 0;
	}
}

/**
 * @brief Function to get the current AHB pre-scaler value by determining the configured HPRE bits
 * @param
 */
static uint16_t GET_AHB_PSCL_VALUE(){
	uint16_t AHB_PRESCALER_VALUES[] = {2, 4, 8, 16, 64, 128, 256, 512};
	uint8_t HPRE_VALUE = (RCC->CFGR >> RCC_CFGR_HPRE_Pos) & 0x0F;
		if(HPRE_VALUE < 0x08){
			return 1;
		}
		else {
			return AHB_PRESCALER_VALUES[HPRE_VALUE % 8];
		}
}
/**
 * @brief Function to get the current clock value of APB1 bus
 * @param
 */
uint32_t RCC_GET_APB1_CLK()
{
	uint32_t CORE_CLK;
	uint16_t AHB_PSCL;
	uint8_t  APB1_PRESCALER_VALUES[] = {2, 4, 8, 16};
	uint8_t  APB1_PSCL;

	// get the current system clock
	CORE_CLK = GET_SYS_CLOCK();

	// get the current pre scaler value for AHB
	AHB_PSCL = GET_AHB_PSCL_VALUE();

	uint8_t PPRE1_VALUE = (RCC->CFGR >> RCC_CFGR_PPRE1_Pos) & 0x07;
	if(PPRE1_VALUE < 0x04){
		APB1_PSCL = 1;
	}
	else {
		APB1_PSCL = APB1_PRESCALER_VALUES[PPRE1_VALUE % 4];
	}

	// corresponding clock pre-scalers are applied to the core clock
	return CORE_CLK / (uint32_t)AHB_PSCL / (uint32_t)APB1_PSCL;
}

uint32_t RCC_GET_APB2_CLK()
{
	uint32_t CORE_CLK;
	uint16_t AHB_PSCL;
	uint8_t  APB2_PRESCALER_VALUES[] = {2, 4, 8, 16};
	uint8_t  APB2_PSCL;

	// get the current system clock
	CORE_CLK = GET_SYS_CLOCK();

	// get the current pre scaler value for AHB
	AHB_PSCL = GET_AHB_PSCL_VALUE();

	uint8_t PPRE1_VALUE = (RCC->CFGR >> RCC_CFGR_PPRE2_Pos) & 0x07;
	if(PPRE1_VALUE < 0x04){
		APB2_PSCL = 1;
	}
	else {
		APB2_PSCL = APB2_PRESCALER_VALUES[PPRE1_VALUE % 4];
	}

	// corresponding clock pre-scalers are applied to the core clock
	return CORE_CLK / (uint32_t)AHB_PSCL / (uint32_t)APB2_PSCL;
}
