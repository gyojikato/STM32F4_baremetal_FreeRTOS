/*
 * gpio.c
 *
 *  Created on: Apr 16, 2025
 *      Author: katog
 */
#include "stm32f44xx_gpio.h"


/* HELPER FUNCTIONS START HERE */
/**
 * @brief Helper function to clear a bit before setting a bit
 * @param
 */
static void SetBit(uint32_t *REG, uint16_t bit_pos, uint8_t mask,  uint8_t clear_mask)
{
	MODIFY_REG(*REG, (clear_mask << bit_pos), (mask << bit_pos));

}

/**
 * @brief Function to initialize EXTI configurations
 * @param
 */
static uint8_t EXTI_INIT(GPIO_TypeDef* pGPIOx, EXTI_Config_t* EXTI_HANDLE, uint8_t PIN_NUMBER)
{
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN_Msk;
	if(pGPIOx == GPIOA){
		SYSCFG->EXTICR[PIN_NUMBER / 4] &= ~(0x0F << ((PIN_NUMBER % 4) * 4));
	}
	else if(pGPIOx == GPIOB){
		SYSCFG->EXTICR[PIN_NUMBER / 4] |= (0x01 << ((PIN_NUMBER % 4) * 4));
	}
	else if(pGPIOx == GPIOC){
		SYSCFG->EXTICR[PIN_NUMBER / 4] |= (0x02 << ((PIN_NUMBER % 4) * 4));
	}
	else if(pGPIOx == GPIOD){
		SYSCFG->EXTICR[PIN_NUMBER / 4] |= (0x03 << ((PIN_NUMBER % 4) * 4));
	}
	else {
		return ERROR;
	}

	// EMR register configurations
	if(EXTI_HANDLE->EXTI_EMR_SET == DISABLE){
		EXTI->EMR &= ~(1 << PIN_NUMBER);
	}
	else {
		EXTI->EMR |= (1 << PIN_NUMBER);
	}
	// IMR register configurations
	if(EXTI_HANDLE->EXTI_IMR_SET == DISABLE){
		EXTI->IMR &= ~(1 << PIN_NUMBER);
	}
	else {
		EXTI->IMR |= (1 << PIN_NUMBER);
	}

	// enable syscfg peripheral clock
	// trigger selection configurations
	if(EXTI_HANDLE->EXTI_RTSR_SEL == ENABLE){
		EXTI->FTSR &= ~(1 << PIN_NUMBER);
		EXTI->RTSR |= (1 << PIN_NUMBER);
	}
	else if(EXTI_HANDLE->EXTI_FTSR_SEL == ENABLE){
		EXTI->RTSR &= ~(1 << PIN_NUMBER);
		EXTI->FTSR |= (1 << PIN_NUMBER);
	}
	else {
		EXTI->FTSR &= ~(1 << PIN_NUMBER);
		EXTI->RTSR &= ~(1 << PIN_NUMBER);
	}

	return SUCCESS;
}
/* FUNCTION DEFINITIONS STARTS HERE */

/**
 * @brief Function to initialize GPIO peripheral using user configurations
 * @param
 */
uint8_t GPIO_INIT(GPIO_Handle_t* GPIO_Handle)
{
	uint32_t tempreg;

	GPIO_PCLK_CTRL(GPIO_Handle->pGPIOx, ENABLE);
	if(GPIO_Handle->GPIO_PIN_NUMBER > 15){
		return ERROR;
	}
	// pin mode configurations
	tempreg = GPIO_Handle->pGPIOx->MODER;
	if(GPIO_Handle->GPIO_MODE == GPIO_PIN_MODE_INPUT){
		// clear the MODEn bits of the set position for input mode
		tempreg &= ~(0x03 << (GPIO_Handle->GPIO_PIN_NUMBER * 2));
		if(GPIO_Handle->EXTI_CFG.EXTI_IMR_SET || GPIO_Handle->EXTI_CFG.EXTI_EMR_SET){
			if(EXTI_INIT(GPIO_Handle->pGPIOx, &GPIO_Handle->EXTI_CFG, GPIO_Handle->GPIO_PIN_NUMBER) == ERROR){
				return ERROR;
			}
		}
	}
	else if(GPIO_Handle->GPIO_MODE <= GPIO_PIN_MODE_ANALOG){
		// set MODEn bits of the set position for desired mode except for input mode
		SetBit(&tempreg, (GPIO_Handle->GPIO_PIN_NUMBER * 2), GPIO_Handle->GPIO_MODE, 0x03);

	}
	else {
		return ERROR;
	}
	GPIO_Handle->pGPIOx->MODER = tempreg;

	// afio mode configurations
	if(GPIO_Handle->GPIO_MODE == GPIO_PIN_MODE_AFIO){

		// selects which AFR register is to be modified
		tempreg = GPIO_Handle->pGPIOx->AFR[GPIO_Handle->GPIO_PIN_NUMBER / 8];

		// get the proper bit position due to AFRH and AFRL
		uint8_t bit_pos = GPIO_Handle->GPIO_PIN_NUMBER - (8 * (GPIO_Handle->GPIO_PIN_NUMBER > 7 ? 1 : 0));

		if(GPIO_Handle->GPIO_AFIO_MODE == AFIO_MODE_0){
			// clear AFRn bits to for afio mode 0
			tempreg &= ~(0x0F << bit_pos);
		}
		else if(GPIO_Handle->GPIO_AFIO_MODE <= AFIO_MODE_15){
			// set AFRn bits to desired AFIO mode
			SetBit(&tempreg, bit_pos * 4, GPIO_Handle->GPIO_AFIO_MODE, 0x0F);
		}
		else {
			return ERROR;
		}
		GPIO_Handle->pGPIOx->AFR[GPIO_Handle->GPIO_PIN_NUMBER / 8] = tempreg;
	}

	// output speed and type configurations
	if(GPIO_Handle->GPIO_MODE != GPIO_PIN_MODE_INPUT){

		// output pin speed configurations
		tempreg = GPIO_Handle->pGPIOx->OSPEEDR;
		if(GPIO_Handle->GPIO_OUTPUT_SPD == GPIO_OUT_LOW_SPD){
			// clear the ODRn bits to set the output as low speed
			tempreg &= ~(3 << (GPIO_Handle->GPIO_PIN_NUMBER * 2));
		}
		else if(GPIO_Handle->GPIO_OUTPUT_SPD <= GPIO_OUT_HIGH_SPD){
			// set output speed to the ODRn bits to desired output speed except for low speed
			SetBit(&tempreg, (GPIO_Handle->GPIO_PIN_NUMBER * 2), GPIO_Handle->GPIO_OUTPUT_SPD, 0x03);
		}
		else {
			return ERROR;
		}
		GPIO_Handle->pGPIOx->OSPEEDR = tempreg;

		// output type configurations
		tempreg = GPIO_Handle->pGPIOx->OTYPER;
		if(GPIO_Handle->GPIO_OUTPUT_TYPE == GPIO_OUT_PP){
			// clear the OTYPERn bit to set as push pull
			tempreg &= ~(1 << (GPIO_Handle->GPIO_PIN_NUMBER * 2));
		}
		else if(GPIO_Handle->GPIO_OUTPUT_TYPE == GPIO_OUT_OD){
			// set the OTYPERn bit to set as open drain
			SetBit(&tempreg, (GPIO_Handle->GPIO_PIN_NUMBER), 0x01, 0x01);
		}
		else {
			return ERROR;
		}
		GPIO_Handle->pGPIOx->OTYPER = tempreg;
	}

	// configuration for pull-up/pull down register
	tempreg = GPIO_Handle->pGPIOx->PUPDR;
	if(GPIO_Handle->GPIO_PUPPD == GPIO_NO_PUPD){
		// clear the PUPDRn bit to use pull up resistor
		tempreg &= ~(1 << (GPIO_Handle->GPIO_PIN_NUMBER * 2));
	}
	else if(GPIO_Handle->GPIO_PUPPD <= GPIO_PULL_DOWN){
		// set the PUPDRn bibt to use the pull down resistor
		SetBit(&tempreg, (GPIO_Handle->GPIO_PIN_NUMBER * 2), 0x01, 0x01);
	}
	else {
		return ERROR;
	}
	GPIO_Handle->pGPIOx->PUPDR = tempreg;
	return SUCCESS;
}

/**
 * @brief Function to de-initialize GPIO peripheral using user configurations
 * @param
 */
void GPIO_DeINIT(GPIO_Handle_t* GPIO_Handle)
{

}

/**
 * @brief Function to enable/disable the peripheral clock for GPIO
 * @param
 */
void GPIO_PCLK_CTRL(GPIO_TypeDef* pGPIOx, FunctionalState ENorDI)
{
	if(ENorDI == ENABLE){
		if(pGPIOx == GPIOA){
			RCC->AHB1ENR |= (RCC_AHB1ENR_GPIOAEN_Msk);
		}
		else if(pGPIOx == GPIOB){
			RCC->AHB1ENR |= (RCC_AHB1ENR_GPIOBEN_Msk);
		}
		else if(pGPIOx == GPIOC){
			RCC->AHB1ENR |= (RCC_AHB1ENR_GPIOCEN_Msk);
		}
		else if(pGPIOx == GPIOD){
			RCC->AHB1ENR |= (RCC_AHB1ENR_GPIODEN_Msk);
		}
	}
	else if(ENorDI == DISABLE){
		if(pGPIOx == GPIOA){
			RCC->AHB1ENR &= ~(RCC_AHB1ENR_GPIOAEN_Msk);
		}
		else if(pGPIOx == GPIOB){
			RCC->AHB1ENR &= ~(RCC_AHB1ENR_GPIOBEN_Msk);
		}
		else if(pGPIOx == GPIOC){
			RCC->AHB1ENR &= ~(RCC_AHB1ENR_GPIOCEN_Msk);
		}
		else if(pGPIOx == GPIOD){
			RCC->AHB1ENR &= ~(RCC_AHB1ENR_GPIODEN_Msk);
		}
	}
}

/**
 * @brief Function to read the status of the selected pin number of a gpio peripheral
 * @param
 */
uint8_t GPIO_READ_INPUT_PIN(GPIO_TypeDef* pGPIOx, uint8_t PIN_NUMBER)
{
	return (pGPIOx->IDR >> PIN_NUMBER) & 0x01;
}

/**
 * @brief Function to read the status of the selected port of gpio peripheral
 * @param
 */
uint16_t GPIO_READ_INPUT_PORT(GPIO_TypeDef* pGPIOx)
{
	return pGPIOx->IDR & 0xFFFF;
}
/**
 * @brief Function to write 1/0 to a selected pin of a gpio peripheral
 * @param
 */
void GPIO_WRITE_OUTPUT_PIN(GPIO_TypeDef* pGPIOx, uint8_t PIN_NUMBER, FlagStatus value)
{
	if(PIN_NUMBER > 15){
		return;
	}

	if(value == RESET){
		pGPIOx->ODR &= ~(1 << PIN_NUMBER);
	}
	else if(value == SET){
		pGPIOx->ODR |= (1 << PIN_NUMBER);
	}
}

/**
 * @brief Function to write to the 16 bit port of a gpio peripheral
 * @param
 */
void GPIO_WRITE_OUTPUT_PORT(GPIO_TypeDef* pGPIOx, uint16_t value)
{
	pGPIOx->ODR &= ~(0xFFFF);
	pGPIOx->ODR = value & 0xFFFF;
}

/**
 * @brief Function to toggle the target pin ON/OFF
 * @param
 */
void GPIO_TOGGLE_PIN(GPIO_TypeDef* pGPIOx, uint8_t PIN_NUMBER)
{
	if(PIN_NUMBER > 15){
		return;
	}
	pGPIOx->ODR ^= (1 << PIN_NUMBER);
}

/**
 * @brief Function to enable or disable the NVIC IRQ
 * @param
 */
void GPIO_IRQ_CFG(uint8_t IRQ_NUMBER, FunctionalState ENorDI)
{
	if(ENorDI == ENABLE){
		NVIC_EnableIRQ(IRQ_NUMBER);
	}
	else if(ENorDI == DISABLE){
		NVIC_DisableIRQ(IRQ_NUMBER);
	}
}
/**
 * @brief Function to handle the pending bit register in EXTI
 * @param
 */
void GPIO_IRQ_HANDLING(uint8_t PIN_NUMBER){
	if(PIN_NUMBER > 15){
		return;
	}
	if(EXTI->PR & (1 << PIN_NUMBER)){
		EXTI->PR |= (1 << PIN_NUMBER);
	}
}
