/*
 * DHT22.c
 *
 *  Created on: Jun 4, 2025
 *      Author: katog
 */


#include "DHT22.h"
#include "stm32f44xx_gpio.h"

#define LOW			(0)
#define HIGH 		(1)

#define DHT22_MAX_TIMEOUT				(3000) // max timeout value of 3ms
#define DHT22_BUFFER_MAX_LEN			(5)

typedef enum {
	DHT22_OK,
	DHT22_ERROR,
	DHT22_TIMEOUT_ERROR
}DHT22_STATUS_t;

/* HELPER FUNCTIONS START HERE */

/**
 * @brief
 * @param
 */
static void DHT22_DELAY_ms(DHT22_FUNC_t* DHT22_FUNC, uint32_t delay)
{
	while(delay--){
		DHT22_FUNC->DHT22_DELAY_us(1000);
	}
}

/**
 * @brief
 * @param
 */
static DHT22_STATUS_t dht22_gen_start(DHT22_FUNC_t* DHT22_FUNC)
{
	DHT22_FUNC->DHT22_PINMODE(DHT22_PIN_MODE_OUTPUT);
	// mcu pulls down bus signal high for atleast 1ms
	DHT22_FUNC->DHT22_PIN_WRITE(LOW);
	// delay for 1 ms
	DHT22_DELAY_ms(DHT22_FUNC, 1);

	// mcu then pulls up bus signal and wait for dht22 to pull the signal down
	// we can just set the pin to input mode to let the bus signal be pulled high and be released
	// and let the sensor control the bus
	DHT22_FUNC->DHT22_PINMODE(DHT22_PIN_MODE_INPUT);

	// wait for the sensor to send a response signal
	uint32_t start_tick = DHT22_FUNC->get_ticks();
	while(DHT22_FUNC->DHT22_PIN_READ() == HIGH){

		if(DHT22_FUNC->get_ticks() - start_tick > DHT22_MAX_TIMEOUT){
			return DHT22_TIMEOUT_ERROR; // sensor failed to pull the bus low (sending response signal)
		}
	}
	return DHT22_OK;
}

/**
 * @brief
 * @param
 */
static DHT22_STATUS_t dht22_get_response(DHT22_FUNC_t* DHT22_FUNC)
{
	// check for the 70us LOW ~ HIGH response signal from sensor
	uint32_t start_tick = DHT22_FUNC->get_ticks();
	while(DHT22_FUNC->DHT22_PIN_READ() == LOW){

		if(DHT22_FUNC->get_ticks() - start_tick > DHT22_MAX_TIMEOUT){
			return DHT22_TIMEOUT_ERROR;
		}
	}

	start_tick = DHT22_FUNC->get_ticks();
	while(DHT22_FUNC->DHT22_PIN_READ() == HIGH){

		if(DHT22_FUNC->get_ticks() - start_tick > DHT22_MAX_TIMEOUT){
			return DHT22_TIMEOUT_ERROR;
		}
	}
	return DHT22_OK;
}

/**
 * @brief
 * @param
 */
static DHT22_STATUS_t dht22_get_data(uint8_t* data_buf, DHT22_FUNC_t* DHT22_FUNC)
{
	for(uint8_t idx = 0; idx < 5; idx++){
		uint8_t tempreg = 0;
		for(int8_t bit_pos = 7; bit_pos >= 0; bit_pos--){
			// signal low for 50 us
			// check signal if still high after delay of 80 us
			uint32_t start_tick = DHT22_FUNC->get_ticks();
			while(DHT22_FUNC->DHT22_PIN_READ() == LOW){
				if(DHT22_FUNC->get_ticks() - start_tick > DHT22_MAX_TIMEOUT){
					return DHT22_TIMEOUT_ERROR; // the bus locked up, exit the function
				}
			}
			DHT22_FUNC->DHT22_DELAY_us(15); // wait for the low signal before every bit
			/* DELAY SHOULD BE ADJUSTED IF THERE ARE PROBLEMS IN TIMING */

			if(DHT22_FUNC->DHT22_PIN_READ() == HIGH){ // if signal is still high, it means that the transmitted bit is 1
				tempreg |= (1 << bit_pos);
			}
			else {
				tempreg &= ~(1 << bit_pos);
			}
			start_tick = DHT22_FUNC->get_ticks();
			while(DHT22_FUNC->DHT22_PIN_READ() == HIGH){
				if(DHT22_FUNC->get_ticks() - start_tick > DHT22_MAX_TIMEOUT){
					return DHT22_TIMEOUT_ERROR; // the bus locked up, exit the function
				}
			}

		}
		data_buf[idx] = tempreg;
	}
	return DHT22_OK;
}

/**
 * @brief
 * @param
 */
static uint8_t dht22_checksum(uint8_t* data_buf, DHT22_Handle_t* DHT22_HANDLE)
{
	// store the total sum of humidity and temperature values to temporary checksum
	uint8_t temp_checksum = 0;
	for(uint8_t idx = 0; idx < (DHT22_BUFFER_MAX_LEN - 1); idx++){
		temp_checksum += data_buf[idx];
	}

	// compare the temporary checksum to the actual receievd checksum
	if(temp_checksum != data_buf[4]){
		return DHT22_ERROR; // T and H data is not overwritten
	}

	// store properly the correct values to the handle
	uint16_t temp_data;
	temp_data = ((data_buf[0] << 8) | data_buf[1]);
	DHT22_HANDLE->humidity = (float)temp_data / 10;

	temp_data = (((data_buf[2] & 0x7F) << 8) | data_buf[3]);
	DHT22_HANDLE->temperature = data_buf[2] & 0x80 ? -(float)temp_data / 10 : (float)temp_data / 10;

	return DHT22_OK;
}
/* HELPER FUNCTIONS END HERE */

/* MAIN FUNCTIONS START HERE */

/**
 * @brief
 * @param
 */
void DHT22_INIT(DHT22_Handle_t* DHT22_HANDLE, DHT22_FUNC_t *DHT22_WRAPPER_FUNCS)
{
	DHT22_HANDLE->DHT22_FUNC = *DHT22_WRAPPER_FUNCS;
}

/**
 * @brief
 * @param
 */
uint8_t DHT22_GET_TEMP_HUM(DHT22_Handle_t* DHT22_HANDLE)
{
	uint8_t data_buf[5] = {0, 0, 0, 0, 0};
	// generate start condition;
	if(dht22_gen_start(&(DHT22_HANDLE->DHT22_FUNC)) != DHT22_OK){
		return 0; // host to generate start failed
	}

	// check response from dht22
	if(dht22_get_response(&(DHT22_HANDLE->DHT22_FUNC)) != DHT22_OK){
		return 0;	// host to receive response from sensor failed
	}

	if(dht22_get_data(data_buf, &(DHT22_HANDLE->DHT22_FUNC)) != DHT22_OK){
		return 0; // failed to acquire data
	}

	if(dht22_checksum(data_buf, DHT22_HANDLE) != DHT22_OK){
		return 0; // received data didnt match the checsum part
	}

	return 1;
}

/* MAIN FUNCTIONS END HERE */
