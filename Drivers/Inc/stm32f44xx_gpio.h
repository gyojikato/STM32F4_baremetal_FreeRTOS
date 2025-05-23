/*
 * gpio.h
 *
 *  Created on: Apr 16, 2025
 *      Author: katog
 */

#ifndef INC_STM32F44XX_GPIO_H_
#define INC_STM32F44XX_GPIO_H_


#include "stm32f4xx.h"


#define GPIO_PIN_NUMBER_0				(0)
#define GPIO_PIN_NUMBER_1				(1)
#define GPIO_PIN_NUMBER_2				(2)
#define GPIO_PIN_NUMBER_3				(3)
#define GPIO_PIN_NUMBER_4				(4)
#define GPIO_PIN_NUMBER_5				(5)
#define GPIO_PIN_NUMBER_6				(6)
#define GPIO_PIN_NUMBER_7				(7)
#define GPIO_PIN_NUMBER_8				(8)
#define GPIO_PIN_NUMBER_9				(9)
#define GPIO_PIN_NUMBER_10				(10)
#define GPIO_PIN_NUMBER_11				(11)
#define GPIO_PIN_NUMBER_12				(12)
#define GPIO_PIN_NUMBER_13				(13)
#define GPIO_PIN_NUMBER_14				(14)
#define GPIO_PIN_NUMBER_15				(15)

#define AFIO_MODE_0						(0) // SYS
#define AFIO_MODE_1						(1) // TIM1/2
#define AFIO_MODE_4						(4) // I2C1/2/3/4
#define AFIO_MODE_5						(5) // SPI1/2/3/4
#define AFIO_MODE_7						(7) // SPI2/3 USART1/2/3 USART4/5
#define AFIO_MODE_8						(8) // USART6 USART4/5
#define AFIO_MODE_15					(15) // SYS

typedef enum {
	GPIO_PIN_MODE_INPUT,
	GPIO_PIN_MODE_OUTPUT,
	GPIO_PIN_MODE_AFIO,
	GPIO_PIN_MODE_ANALOG
}GPIO_PIN_MODE_t;

typedef enum {

	GPIO_OUT_LOW_SPD,
	GPIO_OUT_MED_SPD,
	GPIO_OUT_FST_SPD,
	GPIO_OUT_HIGH_SPD,

}GPIO_OUT_SPD_t;

typedef enum {

	GPIO_OUT_PP,
	GPIO_OUT_OD

}GPIO_OTYPE_t;

typedef enum {

	GPIO_NO_PUPD,
	GPIO_PULL_UP,
	GPIO_PULL_DOWN

}GPIO_PUPPD_t;


typedef struct {

	FunctionalState EXTI_IMR_SET;
	FunctionalState EXTI_EMR_SET;
	FunctionalState EXTI_FTSR_SEL;
	FunctionalState EXTI_RTSR_SEL;

}EXTI_Config_t;


// GPIO Handle for user configuration
typedef struct {
	GPIO_PIN_MODE_t GPIO_MODE;
	GPIO_OUT_SPD_t GPIO_OUTPUT_SPD;
	uint8_t GPIO_PIN_NUMBER;
	GPIO_OTYPE_t GPIO_OUTPUT_TYPE;
	GPIO_PUPPD_t GPIO_PUPPD;
	uint8_t	GPIO_AFIO_MODE;
	EXTI_Config_t EXTI_CFG;
	GPIO_TypeDef* pGPIOx;
}GPIO_Handle_t;



/* Function Declarations */

/**
 * @brief Function to initialize GPIO peripheral using user configurations
 * @param
 */
uint8_t GPIO_INIT(GPIO_Handle_t* GPIO_Handle);

/**
 * @brief Function to de-initialize GPIO peripheral using user configurations
 * @param
 */
void GPIO_DeINIT(GPIO_Handle_t* GPIO_Handle);

/**
 * @brief Function to enable/disable the peripheral clock for GPIO
 * @param
 */
void GPIO_PCLK_CTRL(GPIO_TypeDef* pGPIOx, FunctionalState ENorDI);

/**
 * @brief Function to read the status of the selected pin number of a gpio peripheral
 * @param
 */
uint8_t GPIO_READ_INPUT_PIN(GPIO_TypeDef* pGPIOx, uint8_t PIN_NUMBER);

/**
 * @brief Function to read the status of the selected port of gpio peripheral
 * @param
 */
uint16_t GPIO_READ_INPUT_PORT(GPIO_TypeDef* pGPIOx);

/**
 * @brief Function to write 1/0 to a selected pin of a gpio peripheral
 * @param
 */
void GPIO_WRITE_OUTPUT_PIN(GPIO_TypeDef* pGPIOx, uint8_t PIN_NUMBER, FlagStatus value);

/**
 * @brief Function to write to the 16 bit port of a gpio peripheral
 * @param
 */
void GPIO_WRITE_OUTPUT_PORT(GPIO_TypeDef* pGPIOx, uint16_t value);

/**
 * @brief Function to toggle the target pin ON/OFF
 * @param
 */
void GPIO_TOGGLE_PIN(GPIO_TypeDef* pGPIOx, uint8_t PIN_NUMBER);

/**
 * @brief Function to enable or disable the NVIC IRQ
 * @param
 */
void GPIO_IRQ_CFG(uint8_t IRQ_NUMBER, FunctionalState ENorDI);

/**
 * @brief Function
 * @param
 */
void GPIO_IRQ_HANDLING(uint8_t PIN_NUMBER);



#endif /* INC_STM32F44XX_GPIO_H_ */
