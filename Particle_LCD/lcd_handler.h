/*
 * lcd_handler.h
 *
 *  Created on: 2016¦~5¤ë20¤é
 *      Author: Tyng
 */

#ifndef LCD_HANDLER_H_
#define LCD_HANDLER_H_

void LCD_Init(void);
void LCD_Clear(void);
void LCD_XY_Set(uint8_t x, uint8_t y);
void LCD_String(char *characters, int len);
int LCD_XY_Print(uint8_t x, uint8_t y, uint8_t *strb, uint16_t len);
int LCD_XY_Range_Clear(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1);
void LCD_XY_Print_SymIdx(uint8_t x, uint8_t y, int char_sym);
void LCD_XY_Print_DecNumb(uint8_t x, uint8_t y, int dec);

#endif /* LCD_HANDLER_H_ */
