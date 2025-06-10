/*
 * DS1307.h
 *
 *  Created on: Jun 1, 2025
 *      Author: katog
 */


// DONT FORGET TO CONNECT THE MODULE TO LOGIC LEVEL SHIFTER (DS1307 runs on minimum 4.5V)
#include "DS1307.h"


#define DS1307_ADDR_SECS			(0x00)
#define DS1307_ADDR_MINUTES			(0x01)
#define DS1307_ADDR_HOURS			(0x02)
#define DS1307_ADDR_DAY				(0x03)
#define DS1307_ADDR_DATE			(0x04)
#define DS1307_ADDR_MONTH			(0x05)
#define DS1307_ADDR_YEAR			(0x06)

#define DS1307_ERROR				(0)
#define DS1307_SUCCESS				(1)

#define SECS_ADDDR_CH_MSK			(1 << 7)

/* HELPER FUNCTIONS START HERE */

/**
 * @brief
 * @param
 */
static uint8_t DecToBCD(uint8_t val)
{
	if(val > 99){
		return -1;
	}
	return ((val / 10) << 4) | (val % 10);
}

/**
 * @brief
 * @param
 */
static uint8_t BCDToDec(uint8_t val)
{
	return ((val >> 4) * 10) + (val & 0x0F);
}
/* HELPER FUNCTIONS END HERE */

/*************** MAIN FUNCTIONS START HERE *************************************/

/**
 * @brief Function that disables the oscillator of the module and subjects the module for new time or date values
 * @param
 */
uint8_t DS137_INIT(DS1307_Handle_t* DS1307_HANDLE, uint8_t SlaveAddr)
{
	uint8_t temp_rx_buffer;

	if(DS1307_HANDLE->I2C_RECEIVE(SlaveAddr, DS1307_ADDR_SECS, &temp_rx_buffer) == 0){
		return 0;
	}

	temp_rx_buffer |= SECS_ADDDR_CH_MSK;
	if(DS1307_HANDLE->I2C_TRANSMIT(SlaveAddr, DS1307_ADDR_SECS, temp_rx_buffer) == 0){
		return 0;
	}
	// disables the oscillator

	return 1;
}

/**
 * @brief
 * @param
 */
uint8_t DS1307_SET_TIME(DS1307_Handle_t* DS1307_HANDLE, uint8_t Slave_Addr){

	// call init here so osc will be paused

	// set the value for hours register
	uint8_t temp_reg = DecToBCD(DS1307_HANDLE->DS1307_TIME_HANDLE.HOURS);
	switch(DS1307_HANDLE->DS1307_TIME_HANDLE.HOUR_FORMAT) {
	case DS1307_24H_FORMAT:
		temp_reg &= ~(0x40); // clears the BIT6 of hour register to set as 24 format

	case DS1307_12H_FORMAT_AM:
		temp_reg |= (0x40);	// sets BIT6 and clears BIT5 for 12 HR format and AM
		temp_reg &= ~(0x20);

	case DS1307_12H_FORMAT_PM:
		temp_reg |= (0x40 | 0x20); // sets both BIT6 and BIT5 for 12 hr format and PM
	}

	if(DS1307_HANDLE->I2C_TRANSMIT(Slave_Addr, DS1307_ADDR_HOURS, temp_reg) == DS1307_ERROR){
		return DS1307_ERROR;
	}
	// set the value for minutes register
	if(DS1307_HANDLE->I2C_TRANSMIT(Slave_Addr, DS1307_ADDR_MINUTES, DecToBCD(DS1307_HANDLE->DS1307_TIME_HANDLE.MINUTES)) == DS1307_ERROR){
		return DS1307_ERROR;
	}
	// set the value for seconds register
	if(DS1307_HANDLE->I2C_TRANSMIT(Slave_Addr, DS1307_ADDR_SECS, DecToBCD(DS1307_HANDLE->DS1307_TIME_HANDLE.SECONDS) & ~(0x80)) == DS1307_ERROR){
		return DS1307_ERROR; // 0x80 clears the bit 7 (Clock Halt) of 0x00 register, enables the OSC
	}
	return DS1307_SUCCESS;
}


uint8_t DS1307_SET_DATE(DS1307_Handle_t* DS1307_HANDLE, uint8_t Slave_Addr)
{
	uint8_t temp_reg;
	// set day of the week
	temp_reg = DS1307_HANDLE->DS1307_DATE_HANDLE.DAY_OF_THE_WEEK;
	if(DS1307_HANDLE->I2C_TRANSMIT(Slave_Addr, DS1307_ADDR_DAY, DecToBCD(temp_reg & 0x07)) == DS1307_ERROR){ // clear the mask for
		return DS1307_ERROR;
	}

	// set the date
	temp_reg = DS1307_HANDLE->DS1307_DATE_HANDLE.DAY;
	if(DS1307_HANDLE->I2C_TRANSMIT(Slave_Addr, DS1307_ADDR_DATE, DecToBCD(temp_reg & 0x3F)) == DS1307_ERROR){ // clear the mask for
		return DS1307_ERROR;
	}

	// set the month
	temp_reg = DS1307_HANDLE->DS1307_DATE_HANDLE.MONTH;
	if(DS1307_HANDLE->I2C_TRANSMIT(Slave_Addr, DS1307_ADDR_MONTH, DecToBCD(temp_reg & 0x1F)) == DS1307_ERROR){ // clear the mask for
		return DS1307_ERROR;
	}

	// set the year
	temp_reg = DS1307_HANDLE->DS1307_DATE_HANDLE.YEAR;
	if(DS1307_HANDLE->I2C_TRANSMIT(Slave_Addr, DS1307_ADDR_YEAR, DecToBCD(temp_reg)) == DS1307_ERROR){ // clear the mask for
		return DS1307_ERROR;
	}

	return DS1307_SUCCESS;
}

/**
 * @brief
 * @param
 */
uint8_t DS1307_GET_TIME(DS1307_Handle_t* DS1307_HANDLE, uint8_t Slave_Addr){
	uint8_t tempreg;
	//receive hours value
	if(DS1307_HANDLE->I2C_RECEIVE(Slave_Addr, DS1307_ADDR_HOURS, &tempreg) == DS1307_ERROR){
		return DS1307_ERROR; // timeout error
	}
	if(tempreg & (0x01 << 6)){
		if(tempreg & (1 << 5)){
			DS1307_HANDLE->DS1307_TIME_HANDLE.HOUR_FORMAT = DS1307_12H_FORMAT_PM;
		}
		else {
			DS1307_HANDLE->DS1307_TIME_HANDLE.HOUR_FORMAT = DS1307_12H_FORMAT_AM;
		}
		DS1307_HANDLE->DS1307_TIME_HANDLE.HOURS = BCDToDec(tempreg & ~(0x03 << 5)); // clear the mask for BIT5 and BIT6, BIT4~BIT0 are 12 hour values
	}
	else {
		DS1307_HANDLE->DS1307_TIME_HANDLE.HOUR_FORMAT = DS1307_24H_FORMAT;
		DS1307_HANDLE->DS1307_TIME_HANDLE.HOURS = BCDToDec(tempreg & ~(0x01 << 6)); // clear the mask for BIT6, BIT5~BIT0 are 24 hour values
	}
	// receive minutes value
	if(DS1307_HANDLE->I2C_RECEIVE(Slave_Addr, DS1307_ADDR_MINUTES, &tempreg) == DS1307_ERROR){
		return DS1307_ERROR; // timeout error
	}
	DS1307_HANDLE->DS1307_TIME_HANDLE.MINUTES = BCDToDec(tempreg);

	// receive seconds value
	if(DS1307_HANDLE->I2C_RECEIVE(Slave_Addr, DS1307_ADDR_SECS, &tempreg) == DS1307_ERROR){
		return DS1307_ERROR; // timeout error
	}
	DS1307_HANDLE->DS1307_TIME_HANDLE.SECONDS = BCDToDec(tempreg & 0x7F);

	return DS1307_SUCCESS;
}


uint8_t DS1307_GET_DATE(DS1307_Handle_t* DS1307_HANDLE, uint8_t Slave_Addr)
{
	uint8_t temp_reg;
	// receive day of the week
	if(DS1307_HANDLE->I2C_RECEIVE(Slave_Addr, DS1307_ADDR_DAY, &temp_reg) == DS1307_ERROR){
		return DS1307_ERROR;
	}

	DS1307_HANDLE->DS1307_DATE_HANDLE.DAY_OF_THE_WEEK = BCDToDec(temp_reg & (0x07)); // clear the mask for

	// receive date
	if(DS1307_HANDLE->I2C_RECEIVE(Slave_Addr, DS1307_ADDR_DATE, &temp_reg) == DS1307_ERROR){
		return DS1307_ERROR;
	}
	DS1307_HANDLE->DS1307_DATE_HANDLE.DAY = BCDToDec(temp_reg & (0x3F)); // clear the mask for

	// receive month
	if(DS1307_HANDLE->I2C_RECEIVE(Slave_Addr, DS1307_ADDR_MONTH, &temp_reg) == DS1307_ERROR){
		return DS1307_ERROR;
	}
	DS1307_HANDLE->DS1307_DATE_HANDLE.MONTH = BCDToDec(temp_reg & (0x1F)); // clear the mask for

	// receive year
	if(DS1307_HANDLE->I2C_RECEIVE(Slave_Addr, DS1307_ADDR_MONTH, &temp_reg) == DS1307_ERROR){
		return DS1307_ERROR;
	}
	DS1307_HANDLE->DS1307_DATE_HANDLE.YEAR = BCDToDec(temp_reg);

	return DS1307_SUCCESS;
}
/*************** MAIN FUNCTIONS END HERE *************************************/

