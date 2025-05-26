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
#include "queue.h"

/******************** MACRO DEFINITIONS START HERE *****************/

/*#define SEMIHOSTING_ENABLE*/
#define QUEUE_MAX_LEN			100U

typedef enum {
	USART_RX_READY_FOR_NEW_STRING,
	USART_RXING_CURR_STRING,
	USART_RX_TO_PROCESS
}USART_RX_STR_STATE_ts;
/******************** MACRO DEFINITIONS END HERE *****************/


/*************  GLOBAL VARIABLES START HERE **********/

static TaskHandle_t task1_handle = NULL;

static TaskHandle_t task2_handle = NULL;

static USART_Handle_t USART2_HANDLE;

static USART_RX_STR_STATE_ts USART_rx_in_progress = USART_RX_READY_FOR_NEW_STRING;

static uint8_t USART_BUFFER;

static QueueHandle_t USART_MSG_QUEUE;

static uint8_t MSG_LEN = 0;

static BaseType_t xHigherPriorityTaskWoken = pdFALSE;
/*************  GLOBAL VARIABLES END HERE **********/

/************* PERIPHERAL INITS START HERE ********************/

#ifdef SEMIHOSTING_ENABLE

extern void initialise_monitor_handles(void);

#endif
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

void USART2_INIT(USART_Handle_t* USART2_HANDLE)
{

	USART2_HANDLE->STOPBIT_LEN = USART_STOPBIT_LEN_1;
	USART2_HANDLE->USART_MODE = USART_MODE_RXTX;
	USART2_HANDLE->USART_BAUDRATE = USART_BAUD_RATE_9600;
	USART2_HANDLE->USART_OVER_VAL = USART_OVER_8;
	USART2_HANDLE->USART_WORDLEN = USART_WORD_LEN_8BITS;
	USART2_HANDLE->USART_PARITY_CTRL = USART_PARITY_DISABLE;
	USART2_HANDLE->USART_RX_STATUS = USART_STATUS_READY;
	USART2_HANDLE->USART_TX_STATUS = USART_STATUS_READY;
	USART2_HANDLE->pUSARTx = USART2;

	USART_INIT(USART2_HANDLE);
}
/************* PERIPHERAL INITS END HERE ********************/

/************* FreeRTOS Tasks START HERE *******************/

void task1_RX(void *pvParameters)
{
	while(1){
		if(USART_rx_in_progress == USART_RX_READY_FOR_NEW_STRING){
			USART_CTRL(USART2, ENABLE);
			USART_IRQ_CFG(USART2_IRQn, ENABLE);
			uint8_t mcu_shell[] = "\r\nkatog>";
			USART_SEND_DATA(USART2, mcu_shell, sizeof(mcu_shell)/sizeof(mcu_shell[0]));

			USART_RECEIVE_DATA_IT(&USART2_HANDLE, &USART_BUFFER, 1);
			USART_rx_in_progress = USART_RXING_CURR_STRING;
		}
		vTaskDelay(50 / portTICK_PERIOD_MS);
	}
}

void task2_print(void *pvParameters)
{
	while(1){
		if(USART_rx_in_progress == USART_RX_TO_PROCESS){

			uint8_t *task2_msg_buffer = pvPortMalloc(MSG_LEN * sizeof(USART_BUFFER));

			uint8_t delay_str[] = "DELAY";


			
			if(strncmp((const char*)delay_str, (const char*)task2_msg_buffer, strlen((const char*)delay_str)) == 0){
				// process and retrieve the value of the delay
			}
			vPortFree(task2_msg_buffer);
			MSG_LEN = 0;
			USART_rx_in_progress = USART_RX_READY_FOR_NEW_STRING;
		}
		vTaskDelay(50/ portTICK_PERIOD_MS);
	}
}
/************* FreeRTOS Tasks END HERE *******************/

int main()
{


#ifdef SEMIHOSTING_ENABLE
	initialise_monitor_handles();
	printf("semi hosting enabled\n");
#endif

	USART_PINS_INIT();
	USART2_INIT(&USART2_HANDLE);

	USART_MSG_QUEUE = xQueueCreate(QUEUE_MAX_LEN, sizeof(uint8_t));
	if(USART_MSG_QUEUE == NULL){
#ifdef SEMIHOSTING_ENABLE
		printf("failed to create USART_MSG_QUEUE\n");
#endif
		NVIC_SystemReset();
	}
	if(xTaskCreate(task1_RX, "task1_RX", 2048, NULL, 2, &task1_handle) == errCOULD_NOT_ALLOCATE_REQUIRED_MEMORY){
#ifdef SEMIHOSTING_ENABLE
		printf("failed to create task1_RX\n");
#endif
	}

	if(xTaskCreate(task2_print, "task2_print", 2048, NULL, 3, &task2_handle) == errCOULD_NOT_ALLOCATE_REQUIRED_MEMORY){
#ifdef SEMIHOSTING_ENABLE
		printf("failed to create task2_print\n");
#endif
	}





	vTaskStartScheduler();
	for(;;);

}

void USART2_IRQHandler(void){
	USART_IRQ_HANDLE(&USART2_HANDLE);
}

void USART_APPEV_CLLBCK(USART_APPEV_CLLBCK_t APP_EV){
	if(APP_EV == USART_RX_SUCCESS){

		if(MSG_LEN < QUEUE_MAX_LEN && USART_BUFFER != '\r'){
			MSG_LEN++;
			xQueueSendFromISR(USART_MSG_QUEUE, &USART_BUFFER, &xHigherPriorityTaskWoken);
			USART_SEND_DATA_IT(&USART2_HANDLE, &USART_BUFFER, 1);
		}
		else {
			uint8_t carriage_ch[] = "\r\n";
			USART_SEND_DATA_IT(&USART2_HANDLE, carriage_ch, 2);
			USART_BUFFER = '\0';
			MSG_LEN++;
			xQueueSendFromISR(USART_MSG_QUEUE, &USART_BUFFER, &xHigherPriorityTaskWoken);
		}
	}
	if(APP_EV == USART_TX_SUCCESS){
		if(USART_BUFFER == '\r'){
			USART_CTRL(USART2, DISABLE);
			USART_IRQ_CFG(USART2_IRQn, DISABLE);
			USART_rx_in_progress = USART_RX_TO_PROCESS;

			if(xHigherPriorityTaskWoken == pdTRUE){
				portYIELD_FROM_ISR(xHigherPriorityTaskWoken); // yield to task 2 since it manages data processing
			}
		}
		else {
			USART_RECEIVE_DATA_IT(&USART2_HANDLE, &USART_BUFFER, 1);
		}
	}
}

void SysTick_Handler(void) {
    xPortSysTickHandler();  // FreeRTOS tick handler
}

