#ifndef __OLED_H
#define __OLED_H			  	 

#include "io.h"

typedef struct oledConfig_s {
	ioTag_t RST;			// PC13
	ioTag_t DC;				// PB4
	ioTag_t SCL;			// PC15
	ioTag_t SDA;			// PC14
}oledConfig_t;

/* OLED Control Functions */
void OLED_WR_Byte(uint8_t dat,uint8_t cmd);
void OLED_Display_On(void);
void OLED_Display_Off(void);
void OLED_Refresh_Gram(void);
void oledInit(oledConfig_t *oledConfig);
void OLED_Clear(void);
void OLED_DrawPoint(uint8_t x, uint8_t y, uint8_t t);
void OLED_ShowChar(uint8_t x, uint8_t y, uint8_t chr, uint8_t size, uint8_t mode);
void OLED_ShowNumber(uint8_t x, uint8_t y, uint32_t num, uint8_t len, uint8_t size);
void OLED_ShowString(uint8_t x,uint8_t y,const uint8_t *p);

#endif  

	 

