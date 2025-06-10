/*
 * DS1307.h
 *
 *  Created on: Jun 1, 2025
 *      Author: katog
 */

#ifndef INC_DS1307_H_
#define INC_DS1307_H_

#include <stdint.h>
#include "stm32f44xx_i2c.h"

#define DS1307_SLAVE_ADDR			(0x68)



#define SUNDAY						(0)
#define MONDAY						(1)
#define TUESDAY						(2)
#define WEDNESDAY					(3)
#define THURSDAY					(4)
#define FRIDAY						(5)
#define SATURDAY					(6)

typedef enum {
	DS1307_24H_FORMAT,
	DS1307_12H_FORMAT_AM,
	DS1307_12H_FORMAT_PM
}HOUR_FORMAT_t;


/*typedef struct {
	uint8_t	(*I2C_TRANSMIT)(uint8_t Slave_Addr, uint8_t reg_addr, uint8_t value);
	uint8_t (*I2C_RECEIVE)(uint8_t Slave_Addr, uint8_t reg_addr, uint8_t* RxBuffer);
}DS1307_I2C_COMMS_t;*/

typedef struct {
 uint8_t SECONDS;
 uint8_t MINUTES;
 uint8_t HOURS;
 HOUR_FORMAT_t HOUR_FORMAT; // 24H, 12H AM, 12H PM


}DS1307_TIME_t;

typedef struct {
 uint8_t DAY;
 uint8_t DAY_OF_THE_WEEK;
 uint8_t MONTH;
 uint8_t YEAR;

}DS1307_DATE_t;

typedef struct {

	/*DS1307_I2C_COMMS_t DS1307_COMMS_HANDLE;*/
	uint8_t	(*I2C_TRANSMIT)(uint8_t Slave_Addr, uint8_t reg_addr, uint8_t value);
	uint8_t (*I2C_RECEIVE)(uint8_t Slave_Addr, uint8_t reg_addr, uint8_t* RxBuffer);
	DS1307_DATE_t DS1307_DATE_HANDLE;
	DS1307_TIME_t DS1307_TIME_HANDLE;

}DS1307_Handle_t;

uint8_t DS137_INIT(DS1307_Handle_t* DS1307_HANDLE, uint8_t SlaveAddr);
uint8_t DS1307_SET_TIME(DS1307_Handle_t* DS1307_HANDLE, uint8_t Slave_Addr);
uint8_t DS1307_SET_DATE(DS1307_Handle_t* DS1307_HANDLE, uint8_t Slave_Addr);
uint8_t DS1307_GET_TIME(DS1307_Handle_t* DS1307_HANDLE, uint8_t Slave_Addr);
uint8_t DS1307_GET_DATE(DS1307_Handle_t* DS1307_HANDLE, uint8_t Slave_Addr);



#endif /* INC_DS1307_H_ */
