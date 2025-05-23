/*
 * main.c
 *
 *  Created on: Apr 16, 2025
 *      Author: katog
 */
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include "stm32f44xx_usart.h"
#include "stm32f44xx_gpio.h"
#include "FreeRTOS.h"
#include "task.h"

/*************  GLOBAL VARIABLES START HERE **********/

static TaskHandle_t task1_handle = NULL;

static USART_Handle_t USART2_HANDLE;

static uint8_t USART_CLR_TO_SEND = 1;

/*************  GLOBAL VARIABLES END HERE **********/

/************* PERIPHERAL INITS START HERE ********************/
extern void initialise_monitor_handles(void);

void USART_PINS_INIT()
{
	GPIO_Handle_t USART2_PIN;
	USART2_PIN.GPIO_MODE = GPIO_PIN_MODE_AFIO;
	USART2_PIN.GPIO_AFIO_MODE = AFIO_MODE_7;
	USART2_PIN.GPIO_OUTPUT_TYPE = GPIO_OUT_PP;
	USART2_PIN.GPIO_PUPPD = GPIO_NO_PUPD;
	USART2_PIN.pGPIOx = GPIOA;
	USART2_PIN.GPIO_OUTPUT_SPD = GPIO_OUT_FST_SPD;

	// USART TX
	USART2_PIN.GPIO_PIN_NUMBER = GPIO_PIN_NUMBER_2;
	GPIO_INIT(&USART2_PIN);

	// USART RX
	USART2_PIN.GPIO_PIN_NUMBER = GPIO_PIN_NUMBER_3;
	GPIO_INIT(&USART2_PIN);
}

void USART1_INIT(USART_Handle_t* USART2_HANDLE)
{

	USART2_HANDLE->STOPBIT_LEN = USART_STOPBIT_LEN_1;
	USART2_HANDLE->USART_MODE = USART_MODE_RXTX;
	USART2_HANDLE->USART_BAUDRATE = USART_BAUD_RATE_9600;
	USART2_HANDLE->USART_OVER_VAL = USART_OVER_8;
	USART2_HANDLE->USART_WORDLEN = USART_WORD_LEN_8BITS;
	USART2_HANDLE->USART_PARITY_CTRL = USART_PARITY_DISABLE;
	USART2_HANDLE->USART_STATUS = USART_STATUS_READY;
	USART2_HANDLE->pUSARTx = USART2;

	USART_INIT(USART2_HANDLE);
}
/************* PERIPHERAL INITS END HERE ********************/

/************* FreeRTOS Tasks START HERE *******************/
/*void taskMonitor(void *pvParameters){
	for(;;){
		vTaskList(monitor_buffer);
		printf("%s", monitor_buffer);
		vTaskDelay(2000 / portTICK_PERIOD_MS);
	}
}*/

void task1_RX(void *pvParameters)
{
	while(1){
		static uint8_t msg[100];
		static uint8_t msg1[] = "taenamo\r\n";
		if(USART2_HANDLE.USART_STATUS == USART_STATUS_READY && USART_CLR_TO_SEND){
			USART_CTRL(USART2, ENABLE);
			USART_IRQ_CFG(USART2_IRQn, ENABLE);
			USART_SEND_DATA_IT(&USART2_HANDLE, msg1, sizeof(msg1)/sizeof(uint8_t));
			/*USART_RECEIVE_DATA_IT(&USART2_HANDLE, msg, 2);*/
			USART_CLR_TO_SEND = 0;
		}
		vTaskDelay(1000 / portTICK_PERIOD_MS);
	}
}

void task2_print(void *pvParameters)
{
	while(1){

		vTaskDelay(1000 / portTICK_PERIOD_MS);
	}
}
/************* FreeRTOS Tasks END HERE *******************/

int main()
{
/*
	initialise_monitor_handles();

	printf("semi hosting enabled\n");*/
	USART_PINS_INIT();
	USART1_INIT(&USART2_HANDLE);

	/*xTaskCreate(taskMonitor, "taskMonitor", 1048, NULL, 0, NULL);*/
	xTaskCreate(task1_RX, "task1_RX", 2048, NULL, 4, &task1_handle);
	/*xTaskCreate(task2_print, "task2_print", 1024, NULL, 3, &task2_handle);*/

	vTaskStartScheduler();
	while(1);


}


void SysTick_Handler(void) {
    xPortSysTickHandler();  // FreeRTOS tick handler
}

void USART2_IRQHandler(void){
	USART_IRQ_HANDLE(&USART2_HANDLE);
}

void USART_APPEV_CLLBCK(USART_APPEV_CLLBCK_t APP_EV){
	if(APP_EV == USART_TX_SUCCESS){
		USART_CLR_TO_SEND = 1;
		USART_IRQ_CFG(USART2_IRQn, DISABLE);
		USART_CTRL(USART2, ENABLE);
	}
	if(APP_EV == USART_RX_SUCCESS){
		USART_CLR_TO_SEND = 1;
		USART_IRQ_CFG(USART2_IRQn, DISABLE);
		USART_CTRL(USART2, ENABLE);
	}
}
