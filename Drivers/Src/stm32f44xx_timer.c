/*
 * stm32f44xx_timer.c
 *
 *  Created on: Apr 21, 2025
 *      Author: katog
 */

#include "stm32f44xx_timer.h"

#define TIM_CLK_FREQ			(1000000)
#define TIMER_TIMEOUT			(16000) // 1 ms timeout value assuming 16Mhz system clock


/* General purpose TIMER6 is reserved only to be used for DELAY functionalities
 * The following must be set at all times
 * Prescaler value : 16 - 1 1Mhz Clock frequency
 * Auto Reload Register value : to achieve micro second clock ticks, in accordance to 1Mhz Clock frequency
 * Control Register : default value
 *
 * */

/**
 * @brief Function to initialize delays
 * @param
 * @caveat: TIMER6 is reserved to set as the main source of delay function
 */
uint8_t DELAY_INIT()
{
	// enable tim6 peripheral clock
	RCC->APB1ENR |= RCC_APB1ENR_TIM6EN_Msk;
	// configure tim6 to default values
	TIM6->CR1 = 0;
	// set the maximum value for Auto reload register
	TIM6->ARR = 0xFFFF;
	// APB1 bus clock runs on 16 Mhz, prescaler of 16 - 1 to achieve 1us delay
	TIM6->PSC = 0x1F - 1;

	// reinitialize the timer and load the pre-load values
	TIM6->EGR |= (TIM_EGR_UG_Msk);

	// wait to confirm the update of registers
	uint32_t temp_cnt = 0;
	while(TIM6->SR & TIM_SR_UIF_Msk){
		if(temp_cnt > TIMER_TIMEOUT) return 0; // possible to lock up
	}
	// enable count for timer
	TIM6->CR1 |= (TIM_CR1_CEN_Msk);
	return 1;
}

void DELAY_us(uint32_t delay)
{
	TIM6->CNT = 0;
	while(TIM6->CNT < delay);
}

void DELAY_ms(uint32_t delay)
{
	while(delay-=1){
		TIMER_uS_delay(1000);
	}
}

uint32_t DELAY_TICK(void)
{
	return TIM6->CNT;
}
/* Since PWM functions are also under TIMER peripherals,
 * PWM functionalities are also included in TIMER driver */

/* ALL PWM RELATED STARTS HERE */
/**
 * @brief Function to initialize PWM output
 * @param
 * @caveat: suggested to use TIM2~TIM5 or TIM9~TIM14 for the option of max 32-bit ARR value
 * 			the timer clocks are all set to 1MHz by the pre-scaler for easier PWM configuration
 */
void PWM_INIT(PWM_Handle_t* TIM_HANDLE)
{
	uint32_t ARR_VAL;

	uint32_t CCR_VAL;
	// enable peripheral clock on APB1 bus
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN_Msk;

	// timer clock set to 1Mhz frequency, whereas system core clock is at 16MHz
	TIM_HANDLE->pTIMx->PSC = 15;

	// CR1 set to default values
	TIM_HANDLE->pTIMx->CR1 = 0;

	// set the frequency to  100hz
	ARR_VAL = TIM_CLK_FREQ / TIM_HANDLE->PWM_FREQUENCY;
	TIM_HANDLE->pTIMx->ARR = ARR_VAL - 1;

	// set desired duty cycle 0~100%
	CCR_VAL = (ARR_VAL * TIM_HANDLE->PWM_DUTY_CYCLE) / 100;
	TIM_HANDLE->pTIMx->CCR[TIM_HANDLE->PWM_CHANNEL] = CCR_VAL;

	// PWM active high and off when reaches compare value OCxM bit
	TIM_HANDLE->pTIMx->CCMR[TIM_HANDLE->PWM_CHANNEL / 2] |= (0x06 << (4 + (8 * (TIM_HANDLE->PWM_CHANNEL % 2))));

	// Pre-load for CCR is enabled for smoother PWM transactions OCxPE bit
	TIM_HANDLE->pTIMx->CCMR[TIM_HANDLE->PWM_CHANNEL / 2] |= (1 << (3 + (8 * (TIM_HANDLE->PWM_CHANNEL % 2))));

	// configuration of OC active state CCxP bit
	TIM_HANDLE->pTIMx->CCER |= (TIM_HANDLE->PWM_OC_ACTIVE_STATE << (1 + (4 * TIM_HANDLE->PWM_CHANNEL)));

	// reinitialize the timer and load the pre-load values
	TIM2->EGR |= (TIM_EGR_UG_Msk);

	// wait to confirm the update of registers
	while(TIM2->SR & TIM_SR_UIF_Msk);

	TIM2->CR1 |= TIM_CR1_CEN_Msk;

}

/**
 * @brief Function to enable or disable a PWM channel
 * @param
 */
void PWM_CH_CTRL(TIM_TypeDef* pTIMx, PWM_CH_SEL_t CH, FunctionalState ENorDI)
{
	// enable/disable OC bit
	if(ENorDI == ENABLE){
		pTIMx->CCER |= (1 << (4 * CH));
	}
	else {
		pTIMx->CCER &= ~(1 << (4 * CH));
	}
}

/**
 * @brief Function to dynamically change the pwm output on runtime
 * @param
 */
void PWM_SET_DC_FQ(TIM_TypeDef* pTIMx, PWM_CH_SEL_t CH, uint8_t DUTY_CYCLE, uint32_t FREQUENCY)
{
	uint32_t ARR_VAL;
	uint32_t CCR_VAL;

	// set to desired frequency
	ARR_VAL = (TIM_CLK_FREQ / FREQUENCY);
	pTIMx->ARR = ARR_VAL - 1;

	// set to desired duty cycle
	CCR_VAL = ((ARR_VAL * DUTY_CYCLE) / 100);
	pTIMx->CCR[CH] = CCR_VAL;

	pTIMx->CNT = 0;
}

/* ALL PWM RELATED ENDS HERE */
