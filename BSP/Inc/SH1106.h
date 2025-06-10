/*
 * oled.h
 *
 *  Created on: Nov 23, 2024
 *      Author: katog
 */

#include <stdint.h>
#include <stdio.h>
#include "string.h"
#include "fonts.h"

#ifndef OLED_H_
#define OLED_H_

/* I2C OLED Address */
#ifndef SH1106_I2C_ADDR
#define SH1106_I2C_ADDR            0x3C
#endif

/* SH1106 settings */
/* SH1106 width in pixels */
#ifndef SH1106_WIDTH
#define SH1106_WIDTH            128
#endif

/* SH1106 LCD height in pixels */
#ifndef SH1106_HEIGHT
#define SH1106_HEIGHT           64
#endif

#define SH1106_BUFFER_SIZE				((SH1106_WIDTH * SH1106_HEIGHT) / 8) // 8 represents the number of bit for each byte (bit == pixels)


#define DATA_STREAM					0x40				//C0 = 0, D/C = 1 last control byte, data byte for ram operation
#define DATA_BYTE					0xC0				//C0 = 1, D/C = 1 next two bytes data byte & control byte, data byte for ram operation
#define COMMAND_BYTE				0x80				//C0 = 1, D/C = 0 next two bytes data byte & control byte, data byte for command
#define COMMAND_STREAM				0x00				//C0 = 0, D/C = 0 last control byte, data byte for command operation

typedef enum {
	SH1106_COLOR_BLACK = 0x00,
	SH1106_COLOR_WHITE = 0xFF
}SH1106_Color_t;

typedef struct {
	uint8_t (*SH1106_WRITE)(uint8_t Slave_Addr, uint8_t* TxBuffer, uint32_t len, uint8_t Rpt_Strt);
}SH1106_Comms_t;

typedef struct {
	uint8_t SH1106_DISP_BUFFER[SH1106_BUFFER_SIZE];
	uint8_t current_x;
	uint8_t current_y;
	SH1106_Color_t color_fill;
	SH1106_Comms_t SH1106_COMMS;
}SH1106_Handle_t;

uint8_t SH1106_Init(SH1106_Handle_t* SH1106_HANDLE, SH1106_Comms_t* SH1106_COMMS_HANDLE);

void SH1106_FILL_BUFFER(SH1106_Handle_t* SH1106_HANDLE, SH1106_Color_t color);

void SH1106_UPDATE_SCRN(SH1106_Handle_t* SH1106_HANDLE, uint8_t Slave_Addr);

void SH1106_upload_bitmap(SH1106_Handle_t* SH1106_HANDLE, uint8_t *bitmap);

void SH1106_Toggle_Invert(SH1106_Handle_t* SH1106_HANDLE);

uint8_t SH1106_Draw_Pixel(SH1106_Handle_t* SH1106_HANDLE, uint8_t x, uint8_t y, SH1106_Color_t color);

void SH1106_Draw_Line(SH1106_Handle_t* SH1106_HANDLE, uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, SH1106_Color_t color);

void SH1106_Draw_Rectangle(SH1106_Handle_t* SH1106_HANDLE, uint16_t x, uint16_t y, uint16_t w, uint16_t h, SH1106_Color_t color);

void SH1106_Draw_Filled_Rectangle(SH1106_Handle_t* SH1106_HANDLE, uint16_t x, uint16_t y, uint16_t w, uint16_t h, SH1106_Color_t color);

void SH1106_GotoXY(SH1106_Handle_t* SH1106_HANDLE, uint16_t x, uint16_t y);

char SH1106_Putc(SH1106_Handle_t* SH1106_HANDLE, char ch, FontDef_t* Font, SH1106_Color_t color);

char SH1106_Puts(SH1106_Handle_t* SH1106_HANDLE, char* str, FontDef_t* Font, SH1106_Color_t color);


#endif /* OLED_H_ */
