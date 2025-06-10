/*
 * main.c
 *
 *  Created on: Jun 2, 2025
 *      Author: katog
 */

#include <string.h>
#include "stm32f44xx_gpio.h"
#include "stm32f44xx_timer.h"
#include "stm32f44xx_i2c.h"
#include "DHT22.h"
#include "SH1106.h"
#include "DS1307.h"
#include "FreeRTOS.h"
#include "task.h"

/* GLOBAL VARIABLES */
static DHT22_Handle_t 	DIGICLOCK_DHT22_HANDLE;
static DHT22_FUNC_t 	DIGICLOCK_DHT22_WRAPPER;
static GPIO_Handle_t 	DIGICLOCK_DHT22_PIN;

static GPIO_Handle_t	DIGICLOCK_I2C1_BUS_PINS;
static I2C_Handle_t		DIGICLOCK_I2C1_BUS_HANDLE;

SH1106_Handle_t 		DIGICLOCK_SH1106_HANDLE;
SH1106_Comms_t			DIGICLOCK_SH1106_WRAPPER;

DS1307_Handle_t			DIGICLOCK_DS1307_HANDLE;

/* wrapper functions for DHT22 */
void DIGICLOCK_DHT22_WRITE(uint8_t value)
{
	GPIO_WRITE_OUTPUT_PIN(GPIOC, GPIO_PIN_NUMBER_9, value);
}

// wrapper function for reading from dht22 pin
uint8_t DIGICLOCK_DHT22_READ(void)
{
	return GPIO_READ_INPUT_PIN(GPIOC, GPIO_PIN_NUMBER_9);
}

// wrapper function for setting pin mode of dht22 pin
void DIGICLOCK_DHT22_PINMODE(DHT22_PIN_MODE_t PIN_MODE)
{
	if(PIN_MODE == DHT22_PIN_MODE_INPUT){
		DIGICLOCK_DHT22_PIN.GPIO_MODE = GPIO_PIN_MODE_INPUT;
	}
	else if(PIN_MODE == DHT22_PIN_MODE_OUTPUT){
		DIGICLOCK_DHT22_PIN.GPIO_MODE = GPIO_PIN_MODE_OUTPUT;
	}
	GPIO_INIT(&DIGICLOCK_DHT22_PIN);
}

void DIGICLOCK_DHT22_DELAY_uS(uint32_t delay)
{
	// using hardware timer for the source of micro second delays since
	// FreeRTOS doesnt have delay source for micro seconds
	DELAY_us(delay);
}
// this function handles timeout conditions for the dht22 library
uint32_t DIGICLOCK_DHT22_GET_TICKS(void)
{
	return DELAY_TICK();
}

/* DHT22 SETUP CODE */
void DIGILOCK_DHT22_PIN_INITS(GPIO_Handle_t* DHT22_GPIO_HANDLE)
{
	DHT22_GPIO_HANDLE->GPIO_MODE 		= GPIO_PIN_MODE_INPUT;
	DHT22_GPIO_HANDLE->GPIO_PUPPD		= GPIO_NO_PUPD;
	DHT22_GPIO_HANDLE->GPIO_OUTPUT_TYPE = GPIO_OUT_PP;
	DHT22_GPIO_HANDLE->GPIO_OUTPUT_SPD	= GPIO_OUT_FST_SPD;
	DHT22_GPIO_HANDLE->pGPIOx 			= GPIOC;
	DHT22_GPIO_HANDLE->GPIO_PIN_NUMBER	= GPIO_PIN_NUMBER_9;

	GPIO_INIT(DHT22_GPIO_HANDLE);
}

/* I2C1 MAIN BUS SETUP CODE */
void I2C_BUS_PINS_INIT(GPIO_Handle_t* I2C_PIN_HANDLE)
{
	I2C_PIN_HANDLE->GPIO_AFIO_MODE 		= AFIO_MODE_5;
	I2C_PIN_HANDLE->GPIO_MODE 			= GPIO_PIN_MODE_AFIO;
	I2C_PIN_HANDLE->GPIO_OUTPUT_SPD 	= GPIO_OUT_FST_SPD;
	I2C_PIN_HANDLE->GPIO_OUTPUT_TYPE 	= GPIO_OUT_OD;
	I2C_PIN_HANDLE->GPIO_PUPPD			= GPIO_NO_PUPD;
	I2C_PIN_HANDLE->pGPIOx				= GPIOB;
	// PB8 SCL
	I2C_PIN_HANDLE->GPIO_PIN_NUMBER		= GPIO_PIN_NUMBER_8;
	GPIO_INIT(I2C_PIN_HANDLE);

	// PB9 SDA
	I2C_PIN_HANDLE->GPIO_PIN_NUMBER		= GPIO_PIN_NUMBER_9;
	GPIO_INIT(I2C_PIN_HANDLE);

}
void I2C_BUS_INIT(I2C_Handle_t* I2C_HANDLE){
	I2C_HANDLE->DEVICE_ADDR 	 = 0x69;
	I2C_HANDLE->I2C_FM_DUTY_MODE = I2C_FM_DUTY_MODE_2;
	I2C_HANDLE->I2C_SPD			 = I2C_SPEED_FM;
	I2C_HANDLE->I2C_STATUS		 = I2C_STATUS_READY;

	I2C_INIT(I2C_HANDLE);
}


/* SH1106 SETUP CODE */
// sh1106 wrapper functions
uint8_t SH1106_I2C_WRITE(uint8_t Slave_Addr, uint8_t* TxBuffer, uint32_t len, uint8_t Rpt_Strt)
{
	TimeOut_t xTimeOut;

	TickType_t ticks_to_wait = pdMS_TO_TICKS(300);

	while(I2C_MSTR_SEND_DATA_IT(&DIGICLOCK_I2C1_BUS_HANDLE, Slave_Addr, TxBuffer, len, Rpt_Strt) == I2C_BUSY){
		if(xTaskCheckForTimeOut(&xTimeOut, &ticks_to_wait) == pdTRUE){
			// timeout happened
			// call application callback in main side
			return 0;
		}
		vTaskDelay(5); // 5 ticks to give way to other tasks while checking timeout occurence
	}
	return 1;
}

/* DS1307 SETUP CODE */
// ds1307 wrapper functions
uint8_t DS1307_I2C_TRANSMIT(uint8_t Slave_Addr, uint8_t reg_addr, uint8_t value)
{
	TimeOut_t xTimeOut;
	TickType_t ticks_to_wait = pdMS_TO_TICKS(300);

	uint8_t temp_tx_buffer[2];
	temp_tx_buffer[0] = reg_addr;
	temp_tx_buffer[1] = value;
	vTaskSetTimeOutState(&xTimeOut);
	while(I2C_MSTR_SEND_DATA_IT(&DIGICLOCK_I2C1_BUS_HANDLE, Slave_Addr, temp_tx_buffer, 2, DISABLE) == I2C_BUSY){
		if(xTaskCheckForTimeOut(&xTimeOut, &ticks_to_wait) == pdTRUE){
			// timeout happened
			// call application callback in main side
			return 0;
		}
		vTaskDelay(5);
	}
	return 1;
}

uint8_t DS1307_I2C_RECEIVE(uint8_t Slave_Addr, uint8_t reg_addr, uint8_t* RxBuffer)
{
	TimeOut_t xTimeOut;
	TickType_t ticks_to_wait = pdMS_TO_TICKS(300);
	vTaskSetTimeOutState(&xTimeOut);
	while(I2C_MSTR_SEND_DATA_IT(&DIGICLOCK_I2C1_BUS_HANDLE, Slave_Addr, &reg_addr, 1, ENABLE) == I2C_BUSY){
		if(xTaskCheckForTimeOut(&xTimeOut, &ticks_to_wait) == pdTRUE){
			// timeout happened
			// call application callback in main side
			return 0;
		}
		vTaskDelay(5);
	}
	vTaskSetTimeOutState(&xTimeOut);
	while(I2C_MSTR_RECEIVE_DATA_IT(&DIGICLOCK_I2C1_BUS_HANDLE, Slave_Addr, RxBuffer, 1, DISABLE) == I2C_BUSY){
		if(xTaskCheckForTimeOut(&xTimeOut, &ticks_to_wait) == pdTRUE){
			// timeout happened
			// call application callback in main side
			return 0;
		}
	}
	return 1;
}
/* GPIO BUTTON SETUP CODE */
void GPIO_BUTTONS_INIT()
{
	GPIO_Handle_t GPIO_BUTTONS;
	GPIO_BUTTONS.GPIO_MODE		 = GPIO_PIN_MODE_INPUT;
	GPIO_BUTTONS.GPIO_PUPPD 	 = GPIO_NO_PUPD;
	GPIO_BUTTONS.GPIO_PIN_NUMBER = GPIO_PIN_NUMBER_11;
	GPIO_INIT(&GPIO_BUTTONS);

	GPIO_BUTTONS.GPIO_PIN_NUMBER = GPIO_PIN_NUMBER_12;
	GPIO_INIT(&GPIO_BUTTONS);




}
int main(){

	DIGICLOCK_DHT22_WRAPPER.DHT22_DELAY_us 	= 	DIGICLOCK_DHT22_DELAY_uS;
	DIGICLOCK_DHT22_WRAPPER.DHT22_PINMODE 	= 	DIGICLOCK_DHT22_PINMODE;
	DIGICLOCK_DHT22_WRAPPER.DHT22_PIN_READ 	= 	DIGICLOCK_DHT22_READ;
	DIGICLOCK_DHT22_WRAPPER.DHT22_PIN_WRITE =	DIGICLOCK_DHT22_WRITE;
	DIGICLOCK_DHT22_WRAPPER.get_ticks		=	DIGICLOCK_DHT22_GET_TICKS;

	DELAY_INIT();
	DIGILOCK_DHT22_PIN_INITS(&DIGICLOCK_DHT22_PIN);
	I2C_BUS_PINS_INIT(&DIGICLOCK_I2C1_BUS_PINS);
	I2C_BUS_INIT(&DIGICLOCK_I2C1_BUS_HANDLE);

	DHT22_INIT(&DIGICLOCK_DHT22_HANDLE, &DIGICLOCK_DHT22_WRAPPER);
	SH1106_Init(&DIGICLOCK_SH1106_HANDLE, &DIGICLOCK_SH1106_WRAPPER);

}

void I2C1_
