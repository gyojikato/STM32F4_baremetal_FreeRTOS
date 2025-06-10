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

typedef struct {
	uint8_t msg[QUEUE_MAX_LEN];
	uint32_t MSG_SIZE;
}USART2_MSG_BUFFER_t;
/******************** MACRO DEFINITIONS END HERE *****************/


/*************  GLOBAL VARIABLES START HERE **********/
static USART2_MSG_BUFFER_t USART2_BUFFER;

static TaskHandle_t task1_handle = NULL;

static TaskHandle_t task2_handle = NULL;

static TaskHandle_t task3_handle = NULL;

static USART_Handle_t USART2_HANDLE;

static USART_RX_STR_STATE_ts USART_rx_in_progress = USART_RX_READY_FOR_NEW_STRING;

static QueueHandle_t USART_MSG_QUEUE;

static QueueHandle_t LED_TOGGLE_DELAY_QUEUE;


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
	USART2_HANDLE->USART_BAUDRATE = USART_BAUD_RATE_460800;
	USART2_HANDLE->USART_OVER_VAL = USART_OVER_8;
	USART2_HANDLE->USART_WORDLEN = USART_WORD_LEN_8BITS;
	USART2_HANDLE->USART_PARITY_CTRL = USART_PARITY_DISABLE;
	USART2_HANDLE->USART_RX_STATUS = USART_STATUS_READY;
	USART2_HANDLE->USART_TX_STATUS = USART_STATUS_READY;
	USART2_HANDLE->pUSARTx = USART2;

	USART_INIT(USART2_HANDLE);
}

void GPIO_LED_INIT()
{
	GPIO_Handle_t GPIO_LED;
	GPIO_LED.GPIO_MODE = GPIO_PIN_MODE_OUTPUT;
	GPIO_LED.GPIO_OUTPUT_SPD = GPIO_OUT_FST_SPD;
	GPIO_LED.GPIO_OUTPUT_TYPE = GPIO_OUT_PP;
	GPIO_LED.GPIO_PUPPD = GPIO_NO_PUPD;
	GPIO_LED.pGPIOx = GPIOA;
	GPIO_LED.GPIO_PIN_NUMBER = GPIO_PIN_NUMBER_5;
	GPIO_INIT(&GPIO_LED);
}
/************* PERIPHERAL INITS END HERE ********************/

/************* FreeRTOS Tasks START HERE *******************/

void task1_RX(void *pvParameters)
{
	while(1){
		if(xTaskNotifyWait(0x0000, 0xFFFFFFFF, NULL, 50/portTICK_PERIOD_MS) == pdTRUE){
			memset(&USART2_BUFFER,0,sizeof(USART2_MSG_BUFFER_t));
			USART_CTRL(USART2, ENABLE);
			USART_IRQ_CFG(USART2_IRQn, ENABLE);
			uint8_t mcu_shell[] = "\r\nkatog>";
			USART_SEND_DATA(USART2, mcu_shell, sizeof(mcu_shell)/sizeof(mcu_shell[0]));

			USART_RECEIVE_DATA_IT(&USART2_HANDLE, &USART2_BUFFER.msg[USART2_BUFFER.MSG_SIZE], 1);
			/*USART_rx_in_progress = USART_RXING_CURR_STRING;*/
		}

	}
}

void task2_print(void *pvParameters)
{
	while(1){
		if(xTaskNotifyWait(0x0000, 0xFFFFFFFF, NULL, 50/portTICK_PERIOD_MS) == pdTRUE){

			USART2_MSG_BUFFER_t *task2_msg_buffer = (USART2_MSG_BUFFER_t*)pvPortMalloc(sizeof(USART2_MSG_BUFFER_t));
			if(task2_msg_buffer != NULL){

				if(xQueueReceive(USART_MSG_QUEUE, task2_msg_buffer, 50 / portTICK_PERIOD_MS) == pdPASS){

					if(strncmp("HELP", (const char*)task2_msg_buffer->msg, 4) == 0){
						uint8_t cmd_help[] = "Available commands: \r"
								"Help 			   - Displays All Available Commands \r"
								"DELAY 			   - Adjusts the Toggle Delay of the on-board LED (DELAY is followed by ms Value, e.g. DELAY 1000)\r"
								"LED ON/OFF/TOGGLE - Manually set the state of on-board LED\r";
						USART_CTRL(USART2, ENABLE);
						USART_SEND_DATA(USART2, cmd_help, sizeof(cmd_help)/sizeof(cmd_help[0]));

					}
					else if(strncmp("PAKYU", (const char*)task2_msg_buffer->msg, 5) == 0){
						// PAKYU string matches

						uint8_t pakyu_str_to_print[] = "    .\r\n"
											".........../¯/)..........(\\¯\\\r\n"
											"........./..//............\\\\..\\\r\n"
											"......../..//..............\\\\..\\\r\n"
											"....../¯/../¯\\........../¯ `\\..\\¯\\\r\n"
											"..././../../.../|_......_|..\\..\\..\\..\\\r\n"
											"(.(..(....(..(/.)..)..(..(.\\..)...).).)\r\n"
											".\\...........\\/../.....\\. \\/............/\r\n"
											"..\\............./.......\\............./\r\n"
											"...\\...........(.........).........../\r\n";
						USART_CTRL(USART2, ENABLE);
						USART_SEND_DATA(USART2, pakyu_str_to_print, sizeof(pakyu_str_to_print)/sizeof(pakyu_str_to_print[0]));
					}
					else if(strncmp("DELAY", (const char*)task2_msg_buffer->msg, 5) == 0){
						// perform atoi() after receiving string DELAY XXXXnnn;
						// send to another task the delay which controls the delay of led blink
						uint32_t DELAY = atoi((const char*)(&task2_msg_buffer->msg[5]));
						xQueueSend(LED_TOGGLE_DELAY_QUEUE, &DELAY, 10 / portTICK_PERIOD_MS);

					}
					else if(strncmp("GET", (const char*)task2_msg_buffer->msg, 3) == 0){
						int size =  xPortGetFreeHeapSize();
						char heap_buffer_msg[50] = "FREE HEAP: ";

						// convert size to string and store the buffer
						itoa(size, &heap_buffer_msg[11], 10);

						uint8_t len = strlen(heap_buffer_msg);

						heap_buffer_msg[len] = '\r';

						USART_CTRL(USART2, ENABLE);
						USART_SEND_DATA(USART2, (uint8_t*)heap_buffer_msg, (uint32_t)sizeof(heap_buffer_msg)/sizeof(heap_buffer_msg[0]));
						USART_CTRL(USART2, DISABLE);

					}
					/*USART_rx_in_progress = USART_RX_READY_FOR_NEW_STRING;*/
					xTaskNotifyGive(task1_handle);

				}
			}
			vPortFree(task2_msg_buffer);

		}
	}
}
void task3_led(void *pvParameters)
{

	while(1){
		static uint32_t LED_TOGGLE_DELAY = 500;
		xQueueReceive(LED_TOGGLE_DELAY_QUEUE, &LED_TOGGLE_DELAY, 0);
		GPIO_TOGGLE_PIN(GPIOA, GPIO_PIN_NUMBER_5);
		vTaskDelay(LED_TOGGLE_DELAY / portTICK_PERIOD_MS);
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
	GPIO_LED_INIT();
	// set interrupt priorities lower (numerically higher for NVIC) than 5 due to FreeRTOS kernel being interrupted accidentally

	NVIC_SetPriority(USART2_IRQn, 7);

	LED_TOGGLE_DELAY_QUEUE = xQueueCreate(2, sizeof(uint32_t));
	if(LED_TOGGLE_DELAY_QUEUE == NULL){
#ifdef SEMIHOSTING_ENABLE
		printf("failed to create LED_TOGGLE_DELAY_QUEUE\n");
#endif
		NVIC_SystemReset();
	}

	// create the queue to have atleast twice the size of message buffer struct
	USART_MSG_QUEUE = xQueueCreate(2, sizeof(USART2_MSG_BUFFER_t));
	if(USART_MSG_QUEUE == NULL){
#ifdef SEMIHOSTING_ENABLE
		printf("failed to create USART_MSG_QUEUE\n");
#endif
		NVIC_SystemReset();
	}
	if(xTaskCreate(task1_RX, "task1_RX", 2048, NULL, 3, &task1_handle) == errCOULD_NOT_ALLOCATE_REQUIRED_MEMORY){
#ifdef SEMIHOSTING_ENABLE
		printf("failed to create task1_RX\n");
#endif
	}

	if(xTaskCreate(task2_print, "task2_print", 2048, NULL, 4, &task2_handle) == errCOULD_NOT_ALLOCATE_REQUIRED_MEMORY){
#ifdef SEMIHOSTING_ENABLE
		printf("failed to create task2_print\n");
#endif
	}

	if(xTaskCreate(task3_led, "task3_led", 2048, NULL, 2, &task3_handle) == errCOULD_NOT_ALLOCATE_REQUIRED_MEMORY){
#ifdef SEMIHOSTING_ENABLE
		printf("failed to create task3_led\n");
#endif
	}

	xTaskNotifyGive(task1_handle); // let task1 to run from the start




	vTaskStartScheduler();
	for(;;);

}

void USART2_IRQHandler(void){
	USART_IRQ_HANDLE(&USART2_HANDLE);
}

void USART_APPEV_CLLBCK(USART_APPEV_CLLBCK_t APP_EV){
	if(APP_EV == USART_RX_SUCCESS){

		if(USART2_BUFFER.MSG_SIZE < QUEUE_MAX_LEN && USART2_BUFFER.msg[USART2_BUFFER.MSG_SIZE] != '\r'){

			USART_SEND_DATA_IT(&USART2_HANDLE, &USART2_BUFFER.msg[USART2_BUFFER.MSG_SIZE++], 1);
		}
		else {
			uint8_t carriage_ch[] = "\r\n";
			USART_SEND_DATA_IT(&USART2_HANDLE, carriage_ch, 2);

		}
	}
	if(APP_EV == USART_TX_SUCCESS){
		if(USART2_BUFFER.msg[USART2_BUFFER.MSG_SIZE] == '\r' || USART2_BUFFER.MSG_SIZE >= QUEUE_MAX_LEN){

			BaseType_t xHigherPriorityTaskWoken = pdFALSE;

			USART2_BUFFER.msg[USART2_BUFFER.MSG_SIZE] = '\0'; // terminate the end of the string properly
			USART_CTRL(USART2, DISABLE);
			USART_IRQ_CFG(USART2_IRQn, DISABLE);



			// After the message buffer is filled by the string input of the user, it is sent to the queue

			xQueueSendFromISR(USART_MSG_QUEUE, &USART2_BUFFER, &xHigherPriorityTaskWoken);

			/*USART_rx_in_progress = USART_RX_TO_PROCESS;*/

			vTaskNotifyGiveFromISR(task2_handle, &xHigherPriorityTaskWoken);


			if(xHigherPriorityTaskWoken == pdTRUE){
				portYIELD_FROM_ISR(xHigherPriorityTaskWoken); // yield to task 2 since it manages data processing
			}
		}
		else {
			USART_RECEIVE_DATA_IT(&USART2_HANDLE, &USART2_BUFFER.msg[USART2_BUFFER.MSG_SIZE], 1);
		}
	}
}

void SysTick_Handler(void) {
    xPortSysTickHandler();  // FreeRTOS tick handler
}

