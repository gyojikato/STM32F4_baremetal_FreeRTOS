/*
 * stm32f44xx_timer.h
 *
 *  Created on: Apr 21, 2025
 *      Author: katog
 */

#ifndef INC_STM32F44XX_TIMER_H_
#define INC_STM32F44XX_TIMER_H_

#include "stm32f4xx.h"

typedef enum {
	PWM_CH_SEL_CH1,
	PWM_CH_SEL_CH2,
	PWM_CH_SEL_CH3,
	PWM_CH_SEL_CH4
}PWM_CH_SEL_t;

typedef enum {
	PWM_MODE_1 = 0x06,
	PWM_MODE_2
}PWM_MODE_t;

typedef enum {
	PWM_OC_ACTIVE_HIGH,
	PWM_OC_ACTIVE_LOW
}PWM_OC_ACTIVE_STATE_t;

typedef struct {
	uint32_t PWM_FREQUENCY;
	uint8_t PWM_DUTY_CYCLE;
	PWM_CH_SEL_t PWM_CHANNEL;
	PWM_MODE_t PWM_MODE;
	PWM_OC_ACTIVE_STATE_t PWM_OC_ACTIVE_STATE;
	TIM_TypeDef* pTIMx;

}PWM_Handle_t;

uint8_t DELAY_INIT();
void DELAY_us(uint32_t delay);
void DELAY_ms(uint32_t delay);
uint32_t DELAY_TICK(void);

/**
 * @brief Function to enable or disable a PWM channel
 * @param
 */
void PWM_INIT(PWM_Handle_t* TIM_HANDLE);

/**
 * @brief Function to enable or disable a PWM channel
 * @param
 */
void PWM_CH_CTRL(TIM_TypeDef* pTIMx, PWM_CH_SEL_t CH, FunctionalState ENorDI);

/**
 * @brief Function to dynamically change the pwm output on runtime
 * @param
 */
void PWM_SET_DC_FQ(TIM_TypeDef* pTIMx, PWM_CH_SEL_t CH, uint8_t DUTY_CYCLE, uint32_t FREQUENCY);


#endif /* INC_STM32F44XX_TIMER_H_ */
