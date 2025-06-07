/*
 * DHT22.h
 *
 *  Created on: Jun 4, 2025
 *      Author: katog
 */

#ifndef INC_DHT22_H_
#define INC_DHT22_H_

#include <stdint.h>

typedef enum {
	DHT22_PIN_MODE_INPUT,
	DHT22_PIN_MODE_OUTPUT
}DHT22_PIN_MODE_t;


typedef struct {
	void (*DHT22_PIN_WRITE)(uint8_t VALUE);
	uint8_t (*DHT22_PIN_READ)(void);
	void (*DHT22_PINMODE)(DHT22_PIN_MODE_t PIN_MODE); // PIN_MODE to be OUTPUT or INPUT
	void (*DHT22_DELAY_us)(uint32_t delay);
	uint32_t (*get_ticks)(void); // ticks should be updated on a micro-second basis

}DHT22_FUNC_t;

typedef struct {
	DHT22_FUNC_t DHT22_FUNC;
	float temperature;
	float humidity;
}DHT22_Handle_t;

// brief
void DHT22_INIT(DHT22_Handle_t* DHT22_HANDLE, DHT22_FUNC_t *DHT22_WRAPPER_FUNCS);


// brief
uint8_t DHT22_GET_TEMP_HUM(DHT22_Handle_t* DHT22_HANDLE);

#endif /* INC_DHT22_H_ */
