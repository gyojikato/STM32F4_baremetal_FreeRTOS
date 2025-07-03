/*
 * main.c
 *
 *  Created on: 29 Jun 2025
 *      Author: katog
 */

#include "main.h"

typedef struct {
	char temp_buf[20];
	char hum_buf[20];
}DHT22_BUFF_t;

I2C_Handle_t  I2C_DCLK_BUS_h;
GPIO_Handle_t I2C_DCLK_PIN_h;
GPIO_Handle_t DHT22_PIN_h;

SH1106_Handle_t SH1106_DCLK_h;
SH1106_Comms_t  SH1106_comms_h;

DS1307_Handle_t DS1307_DCLK_h;

DHT22_Handle_t DHT22_DCLK_h;
DHT22_FUNC_t DHT22_WRAPPER_h;

SemaphoreHandle_t xI2C_BUS_Smphr;
SemaphoreHandle_t xTIME_DATA_Smphr;
SemaphoreHandle_t xDATE_DATA_Smphr;
QueueHandle_t xDHT22_DATA_Queue;

TaskHandle_t xTask1;
TaskHandle_t xTask2;
TaskHandle_t xTask3;
TaskHandle_t xTask4;

/* I2C BUS INIT */
void I2C_DCLK_BUS_INIT(I2C_Handle_t* i2c_h);
/* I2C PINS INIT */
void I2C_DCLK_BUS_GPIO_INIT(GPIO_Handle_t* gpio_h);
/* DHT22 PIN INIT */
void DIGILOCK_DHT22_PIN_INITS(GPIO_Handle_t* DHT22_GPIO_HANDLE);


/* SH1106 WRAPPER FUNCTIONS */
uint8_t SH1106_I2C_WRITE(uint8_t Slave_Addr, uint8_t* TxBuffer, uint32_t len, uint8_t Rpt_Str);

/* DS1307 WRAPPER FUNCTIONS */
uint8_t DS1307_I2C_WRITE(uint8_t Slave_Addr, uint8_t reg_addr, uint8_t value);
uint8_t DS1307_I2C_READ(uint8_t Slave_Addr, uint8_t reg_addr, uint8_t* RxBuffer, uint32_t len);

/* DHT22 WRAPPER FUNCTIONS */
void DHT22_WRITE_PIN(uint8_t value);

uint8_t DHT22_READ_PIN(void);

void DHT22_PIN_MODE(DHT22_PIN_MODE_t PIN_MODE);

void DHT22_us_DELAY(uint32_t delay);

uint32_t DHTT22_CURR_TICK(void);

/* FreeRTOS Tasks */
void vTaskDCLKInit(void* pvParameters);

void vTaskGetTime(void* pvParameters);

void vTaskPrintOled(void* pvParameters);

void vTaskGetDHT22(void* pvParameters);


int main()
{
	uint8_t task_status = 0;

	if(xTaskCreate(vTaskDCLKInit,
			"vTaskDCLKInit",
			1024,
			NULL,
			5,
			&xTask1) == pdFALSE){
		task_status += 2;
	}

	if(xTaskCreate(vTaskGetTime,
			"vTaskGetTime",
			1024,
			NULL,
			4,
			&xTask2) == pdFALSE){
		task_status += 3;
	}

	if(xTaskCreate(vTaskPrintOled,
			"vTaskPrintOled",
			1024,
			NULL,
			3,
			&xTask3) == pdFALSE){
		task_status += 4;
	}

	if(xTaskCreate(vTaskGetDHT22,
			"vTaskGetDHT22",
			1024,
			NULL,
			2,
			&xTask4) == pdFALSE){
		task_status += 10;
	}

	vTaskStartScheduler();
}

/* Helper Functions */
static void I2C1_irq_cfg(uint8_t ENorDI)
{
	I2C_IRQ_CFG(I2C1_ER_IRQn, ENorDI);
	I2C_IRQ_CFG(I2C1_EV_IRQn, ENorDI);
}

static uint8_t cmp_time(const DS1307_TIME_t time_a, const DS1307_TIME_t time_b )
{
	return (time_a.HOURS == time_b.HOURS && time_a.HOUR_FORMAT == time_b.HOUR_FORMAT &&
			time_a.MINUTES == time_b.MINUTES && time_a.SECONDS == time_b.SECONDS);
}

static void oled_screen_update()
{
	I2C_PERI_CTRL(I2C1, ENABLE);
	SH1106_UPDATE_SCRN(&SH1106_DCLK_h, SH1106_I2C_ADDR);
	I2C_PERI_CTRL(I2C1, DISABLE);
}

static uint8_t check_day_reset(const DS1307_TIME_t ds1307_time_data){
	if( ds1307_time_data.HOUR_FORMAT == DS1307_12H_FORMAT_AM ){
		if( ds1307_time_data.HOURS == 12 &&
		   ds1307_time_data.MINUTES == 00 &&
		   ds1307_time_data.SECONDS == 00 ){

			return 1;
		}
	}
	else if( ds1307_time_data.HOUR_FORMAT == DS1307_24H_FORMAT ){
		if( ds1307_time_data.HOURS == 00 &&
		   ds1307_time_data.MINUTES == 00 &&
		   ds1307_time_data.SECONDS == 00 ){
			return 1;
		}
	}
	return 0;
}

/* FreeRTOS Tasks */
void vTaskDCLKInit(void* pvParameters)
{
	char initial_date_buffer[20];

	DS1307_DCLK_h.I2C_RECEIVE = DS1307_I2C_READ;
	DS1307_DCLK_h.I2C_TRANSMIT = DS1307_I2C_WRITE;

	SH1106_comms_h.SH1106_WRITE = SH1106_I2C_WRITE;

	DHT22_WRAPPER_h.DHT22_PIN_WRITE = DHT22_WRITE_PIN;
	DHT22_WRAPPER_h.DHT22_PIN_READ = DHT22_READ_PIN;
	DHT22_WRAPPER_h.DHT22_PINMODE = DHT22_PIN_MODE;
	DHT22_WRAPPER_h.DHT22_DELAY_us = DHT22_us_DELAY;
	DHT22_WRAPPER_h.get_ticks = DHTT22_CURR_TICK;

	NVIC_SetPriority(I2C1_ER_IRQn, 7);
	NVIC_SetPriority(I2C1_EV_IRQn, 6);

	DELAY_INIT();
	DIGILOCK_DHT22_PIN_INITS(&DHT22_PIN_h);
	I2C_DCLK_BUS_GPIO_INIT(&I2C_DCLK_PIN_h);
	I2C_DCLK_BUS_INIT(&I2C_DCLK_BUS_h);
	DHT22_INIT(&DHT22_DCLK_h, &DHT22_WRAPPER_h);
	I2C_PERI_CTRL(I2C1, ENABLE);
	taskENTER_CRITICAL();
	SH1106_Init(&SH1106_DCLK_h, &SH1106_comms_h);
	taskEXIT_CRITICAL();
	I2C_PERI_CTRL(I2C1, DISABLE);

	xI2C_BUS_Smphr = xSemaphoreCreateBinary();

	xTIME_DATA_Smphr = xSemaphoreCreateBinary();

	xDATE_DATA_Smphr = xSemaphoreCreateBinary();

	xDHT22_DATA_Queue = xQueueCreate(2, sizeof(DHT22_BUFF_t));



	/* TODO Initialize Date values here and send to oled buffer*/
	I2C_PERI_CTRL(I2C1, ENABLE);
	I2C1_irq_cfg(ENABLE);
	DS1307_GET_DATE_IT(&DS1307_DCLK_h, DS1307_SLAVE_ADDR);
	if(xSemaphoreTake(xDATE_DATA_Smphr, pdMS_TO_TICKS(1000)) == pdTRUE){
		DS1307_CONVERT_RAW_DATE(&DS1307_DCLK_h.DS1307_DATE_HANDLE, DS1307_DCLK_h.raw_date);
		DateToString(&DS1307_DCLK_h.DS1307_DATE_HANDLE, initial_date_buffer, sizeof(initial_date_buffer));
		SH1106_GotoXY(&SH1106_DCLK_h, 50, 0); // coordinate position should be same as the date update event in vTaskPrintOled
		SH1106_Puts(&SH1106_DCLK_h, initial_date_buffer, &Font_7x10, SH1106_COLOR_BLACK);
	}

	xSemaphoreGive(xI2C_BUS_Smphr);

	vTaskDelete(NULL);

}


void vTaskGetTime(void* pvParameters)
{
	while(1){
		if(xSemaphoreTake(xI2C_BUS_Smphr, pdMS_TO_TICKS(0)) == pdTRUE){
			I2C_PERI_CTRL(I2C1, ENABLE);
			I2C1_irq_cfg(ENABLE);
			DS1307_GET_TIME_IT(&DS1307_DCLK_h, DS1307_SLAVE_ADDR);
		}
		vTaskDelay(pdMS_TO_TICKS(100));
	}
	vTaskDelete(NULL);
}

void vTaskPrintOled(void* pvParameters)
{
	while(1){

		uint8_t toled_scrn_update_ev = 0;
		DS1307_TIME_t tds1307_time_h;
		static DS1307_TIME_t tds1307_time_h_old;
		uint8_t ttraw_time[raw_time_len];
		char ttime_buffer[20];

		DHT22_BUFF_t tdht22_data_buff;

		if(xSemaphoreTake(xTIME_DATA_Smphr, pdMS_TO_TICKS(0)) == pdTRUE)
		{
			memcpy(ttraw_time, DS1307_DCLK_h.raw_time, sizeof(ttraw_time)/sizeof(ttraw_time[0]));
			DS1307_CONVERT_RAW_TIME(&tds1307_time_h, ttraw_time);
			if(cmp_time(tds1307_time_h,  tds1307_time_h_old) == 0){

				if(check_day_reset(tds1307_time_h) == 1){

					if(xSemaphoreTake(xI2C_BUS_Smphr, pdMS_TO_TICKS(100)) == pdTRUE){
						I2C_PERI_CTRL(I2C1, ENABLE);
						I2C1_irq_cfg(ENABLE);

						DS1307_GET_DATE_IT(&DS1307_DCLK_h, DS1307_SLAVE_ADDR);
					}

					if(xSemaphoreTake(xDATE_DATA_Smphr, pdMS_TO_TICKS(50)) == pdTRUE){
						uint8_t traw_date[raw_date_len];
						char tdate_bufer[20];
						DS1307_DATE_t tds1307_date_h;


						memcpy(traw_date, DS1307_DCLK_h.raw_date, sizeof(traw_date)/sizeof(traw_date[0]));
						DS1307_CONVERT_RAW_DATE(&tds1307_date_h, traw_date);
						DateToString(&tds1307_date_h, tdate_bufer, sizeof(tdate_bufer));

						SH1106_GotoXY(&SH1106_DCLK_h, 50, 0);
						SH1106_Puts(&SH1106_DCLK_h, tdate_bufer, &Font_7x10, SH1106_COLOR_BLACK);

					}

				}

				TimeToString(&tds1307_time_h, ttime_buffer, sizeof(ttime_buffer));
				SH1106_GotoXY(&SH1106_DCLK_h, 50, 50);
				SH1106_Puts(&SH1106_DCLK_h, ttime_buffer, &Font_7x10, SH1106_COLOR_BLACK);
				toled_scrn_update_ev = 1;

			}
			tds1307_time_h_old = tds1307_time_h;
		}

		if(xQueueReceive(xDHT22_DATA_Queue, &tdht22_data_buff, 0) == pdTRUE){

			SH1106_GotoXY(&SH1106_DCLK_h, 0, 25);
			SH1106_Puts(&SH1106_DCLK_h, tdht22_data_buff.temp_buf, &Font_7x10, SH1106_COLOR_BLACK);
			SH1106_GotoXY(&SH1106_DCLK_h, 50, 25);
			SH1106_Puts(&SH1106_DCLK_h, tdht22_data_buff.hum_buf, &Font_7x10, SH1106_COLOR_BLACK);

			toled_scrn_update_ev = 1;
		}

		if(toled_scrn_update_ev == 1){
			if(xSemaphoreTake(xI2C_BUS_Smphr, pdMS_TO_TICKS(100)) == pdTRUE){
				/*vTaskSuspendAll();*/
				taskENTER_CRITICAL();	/*FIXME: taskENTER_CRITICAL should be replaced */
				oled_screen_update();	/*FIXME: move OLED to DMA*/
				taskEXIT_CRITICAL();
				/*xTaskResumeAll();*/
				xSemaphoreGive(xI2C_BUS_Smphr);
			}
		}

		vTaskDelay(pdMS_TO_TICKS(50));
	}
	vTaskDelete(NULL);
}

void vTaskGetDHT22(void* pvParameters)
{
	while(1){

		DHT22_BUFF_t dht22_buffer_t;

		if(xSemaphoreTake(xI2C_BUS_Smphr, pdMS_TO_TICKS(100)) == pdTRUE)
		{
			if(DHT22_GET_TEMP_HUM(&DHT22_DCLK_h) == 1){
				int8_t integ_part;
				uint8_t frac_part;
				integ_part = (int8_t)(DHT22_DCLK_h.temperature);
				frac_part = (uint8_t)((DHT22_DCLK_h.temperature - integ_part) * 10);

				snprintf(dht22_buffer_t.temp_buf, sizeof(dht22_buffer_t.temp_buf), "T:%d.%dC", integ_part, frac_part);
				/*TODO Continue this shit */

				integ_part = (uint8_t)DHT22_DCLK_h.humidity;
				frac_part = (uint8_t)((DHT22_DCLK_h.humidity - integ_part) * 10);
				snprintf(dht22_buffer_t.hum_buf, sizeof(dht22_buffer_t.hum_buf), "H:%d.%d%%", integ_part, frac_part);

				xQueueSend(xDHT22_DATA_Queue, &dht22_buffer_t, pdMS_TO_TICKS(100));

			}
			xSemaphoreGive(xI2C_BUS_Smphr);
		}
		vTaskDelay(pdMS_TO_TICKS(2000));
	}
}

/* I2C BUS INIT */
void I2C_DCLK_BUS_INIT(I2C_Handle_t* i2c_h)
{
	i2c_h->DEVICE_ADDR 			= 0x69;
	i2c_h->I2C_SPD 				= I2C_SPEED_SM;
	i2c_h->I2C_FM_DUTY_MODE 	= I2C_FM_DUTY_MODE_2;
	i2c_h->pI2Cx 				= I2C1;
	i2c_h->I2C_STATUS 			= I2C_STATUS_READY;
	I2C_INIT(i2c_h);
}
/* I2C PINS INIT */
void I2C_DCLK_BUS_GPIO_INIT(GPIO_Handle_t* gpio_h)
{
	gpio_h->GPIO_AFIO_MODE 		= AFIO_MODE_4;
	gpio_h->GPIO_MODE 			= GPIO_PIN_MODE_AFIO;
	gpio_h->GPIO_OUTPUT_SPD 	= GPIO_OUT_FST_SPD;
	gpio_h->GPIO_OUTPUT_TYPE 	= GPIO_OUT_OD;
	gpio_h->GPIO_PUPPD 			= GPIO_NO_PUPD;
	gpio_h->pGPIOx 				= GPIOB;

	// PB8 SCL
	gpio_h->GPIO_PIN_NUMBER 	= GPIO_PIN_NUMBER_8;
	GPIO_INIT(gpio_h);

	// PB9 SDA
	gpio_h->GPIO_PIN_NUMBER 	= GPIO_PIN_NUMBER_9;
	GPIO_INIT(gpio_h);
}

/* DHT22 PIN INIT */
void DIGILOCK_DHT22_PIN_INITS( GPIO_Handle_t* DHT22_GPIO_HANDLE )
{
	DHT22_GPIO_HANDLE->GPIO_MODE 		= GPIO_PIN_MODE_INPUT;
	DHT22_GPIO_HANDLE->GPIO_PUPPD		= GPIO_NO_PUPD;
	DHT22_GPIO_HANDLE->GPIO_OUTPUT_TYPE = GPIO_OUT_PP;
	DHT22_GPIO_HANDLE->GPIO_OUTPUT_SPD	= GPIO_OUT_FST_SPD;
	DHT22_GPIO_HANDLE->pGPIOx 			= GPIOC;
	DHT22_GPIO_HANDLE->GPIO_PIN_NUMBER	= GPIO_PIN_NUMBER_9;

	GPIO_INIT( DHT22_GPIO_HANDLE );
}


/* SH1106 WRAPPER FUNCTION */
uint8_t SH1106_I2C_WRITE(uint8_t Slave_Addr, uint8_t* TxBuffer, uint32_t len, uint8_t Rpt_Strt)
{
	return I2C_MSTR_SEND_DATA(I2C_DCLK_BUS_h.pI2Cx, Slave_Addr, TxBuffer, len, Rpt_Strt);
}

/* DS1307 WRAPPER FUNCTIONS */
uint8_t DS1307_I2C_WRITE(uint8_t Slave_Addr, uint8_t reg_addr, uint8_t value)
{
	uint8_t txbuffer[2];
	txbuffer[0] = reg_addr;
	txbuffer[1] = value;
	TimeOut_t xTimeOut;
	TickType_t ticks_to_wait = pdMS_TO_TICKS(100);
	vTaskSetTimeOutState(&xTimeOut);
	while(I2C_MSTR_SEND_DATA_IT(&I2C_DCLK_BUS_h, Slave_Addr, txbuffer, sizeof(txbuffer)/sizeof(txbuffer[0]), DISABLE) == I2C_BUSY){
		if(xTaskCheckForTimeOut(&xTimeOut, &ticks_to_wait) == pdTRUE){
			return 0;
		}
		vTaskDelay(pdMS_TO_TICKS(10));
	}
	return 1;
}
uint8_t DS1307_I2C_READ(uint8_t Slave_Addr, uint8_t reg_addr, uint8_t* RxBuffer, uint32_t len)
{
	TimeOut_t xTimeOut;
	TickType_t ticks_to_wait = pdMS_TO_TICKS(300);
	vTaskSetTimeOutState(&xTimeOut);
	while(I2C_MSTR_SEND_DATA_IT(&I2C_DCLK_BUS_h, Slave_Addr, &reg_addr, 1, ENABLE) == I2C_BUSY){
		if(xTaskCheckForTimeOut(&xTimeOut, &ticks_to_wait) == pdTRUE){

			return 0;
		}
		vTaskDelay(pdMS_TO_TICKS(10));
	}

	vTaskSetTimeOutState(&xTimeOut);
	while(I2C_MSTR_RECEIVE_DATA_IT(&I2C_DCLK_BUS_h, Slave_Addr, RxBuffer, len, DISABLE) == I2C_BUSY){
		if(xTaskCheckForTimeOut(&xTimeOut, &ticks_to_wait) == pdTRUE){

			return 0;
		}
		vTaskDelay(pdMS_TO_TICKS(10));
	}

	return 1;
}

/* DHT22 WRAPPER FUNCTIONS */
void DHT22_WRITE_PIN(uint8_t value)
{
	GPIO_WRITE_OUTPUT_PIN(GPIOC, GPIO_PIN_NUMBER_9, value);
}

uint8_t DHT22_READ_PIN(void)
{
	return GPIO_READ_INPUT_PIN(GPIOC, GPIO_PIN_NUMBER_9);
}

void DHT22_PIN_MODE(DHT22_PIN_MODE_t PIN_MODE)
{
	if(PIN_MODE == DHT22_PIN_MODE_INPUT){
		DHT22_PIN_h.GPIO_MODE = GPIO_PIN_MODE_INPUT;
	}
	else {
		DHT22_PIN_h.GPIO_MODE = GPIO_PIN_MODE_OUTPUT;
	}
	GPIO_INIT(&DHT22_PIN_h);
}

void DHT22_us_DELAY(uint32_t delay)
{
	DELAY_us(delay);
}

uint32_t DHTT22_CURR_TICK(void)
{
	return DELAY_TICK();
}

void SysTick_Handler(void)
{
    xPortSysTickHandler();  // FreeRTOS tick handler
}

void I2C1_EV_IRQHandler(void)
{
	I2C_ITEVT_HANDLE(&I2C_DCLK_BUS_h);
}

void I2C1_ER_IRQHandler(void)
{
	I2C_ITERR_HANDLE(&I2C_DCLK_BUS_h);
}

void I2C_APPEV_CLLBCK(uint8_t value)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	if(value == I2C_APPEV_CLLBCK_RX_CMPLT){
		if(DS1307_DCLK_h.DS1307_IT_STATUS == DS1307_IT_GET_TIME_BUSY){
			I2C1_irq_cfg(DISABLE);
			I2C_PERI_CTRL(I2C1, DISABLE);
			DS1307_DCLK_h.DS1307_IT_STATUS = DS1307_IT_FREE;
			xSemaphoreGiveFromISR(xTIME_DATA_Smphr, &xHigherPriorityTaskWoken);
			xSemaphoreGiveFromISR(xI2C_BUS_Smphr, &xHigherPriorityTaskWoken);
		}
		else if(DS1307_DCLK_h.DS1307_IT_STATUS == DS1307_IT_GET_DATE_BUSY){
			I2C1_irq_cfg(DISABLE);
			I2C_PERI_CTRL(I2C1, DISABLE);
			DS1307_DCLK_h.DS1307_IT_STATUS = DS1307_IT_FREE;
			xSemaphoreGiveFromISR(xDATE_DATA_Smphr, &xHigherPriorityTaskWoken);
			xSemaphoreGiveFromISR(xI2C_BUS_Smphr, &xHigherPriorityTaskWoken);
		}

		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
	}
	if(value == I2C_APPEV_CLLBCK_AF_OCCUR || value == I2C_APPEV_CLLBCK_ARLO_OCCUR ||
			value == I2C_APPEV_CLLBCK_BERR_OCCUR || value == I2C_APPEV_CLLBCK_OVR_OCCUR ||
			value == I2C_APPEV_CLLBCK_PECERR_OCCUR || value == I2C_APPEV_CLLBCK_TIMEOUT_OCCUR){
		xSemaphoreGiveFromISR(xI2C_BUS_Smphr, &xHigherPriorityTaskWoken);
	}
}
