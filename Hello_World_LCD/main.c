/*
 * Copyright (c) 2017, Intel Corporation
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of the Intel Corporation nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE INTEL CORPORATION OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * Author :BruceTsao Modified from  Sertek ¿c¨|¼w Senior Manager
 *Uart Sample for TX
 *
* this program is used to display   */

#include "clk.h"
#include "qm_common.h"
#include "qm_gpio.h"
#include "qm_pinmux.h"
#include "qm_pin_functions.h"
#include "lcd_handler.h"
#include "hts221_handler.h"

#include "qm_uart.h"


#define DELAY 250000UL /* 0.25 seconds. */
#define WAIT_1MSEC (DELAY)

int re_print_s1 = 0;
int re_print_s2 = 0;
int re_print_s3 = 0;
int re_print_s4 = 0;
int re_print_s5 = 0;
uint8_t linefeed[1] =  {0x0a} ;
int temp = 0;
int humid = 0;
uint8_t h0t0_l,h0t0_h,h1t0_l,h1t0_h;
uint8_t tmp;


static void pin_mux_setup()
{
// Mux out STDOUT_UART TX/RX pins and enable input for RX.

	qm_pmux_select(QM_PIN_ID_17, QM_PIN_17_FN_UART1_RXD);
	qm_pmux_select(QM_PIN_ID_16, QM_PIN_16_FN_UART1_TXD);
	qm_pmux_input_en(QM_PIN_ID_17, true);

}


int uint2str(unsigned int  no, uint8_t *p)
{
	int ret = 0 ;
//	int tmp  = 0 ;
	if (no <10)
	{
		*p = 0x30 + no ;
		ret = 1 ;
	}
	else if (no <100)
	{

		*(p+1) = 0x30 + (no % 10) ;
		*p = 0x30 + (int)(no/10) ;
		ret = 2 ;

	}else if (no <1000)
	{
		*(p+2) = 0x30 + (no % 10) ;
		*(p+1) = 0x30 + (int)((no/100) / 10) ;
		*p = 0x30 + (int)(no/100) ;
		ret = 3 ;

	}else if (no <10000)
	{
		*(p+3) = 0x30 + (no % 10) ;
		*(p+2) = 0x30 + (int)((no % 100) %10) ;
		*(p+1) = 0x30 + (int)((no % 100) / 10) ;
		*p = 0x30 + (int)(no/1000) ;
		ret = 4 ;

	}
	return ret ;
}



void SensorData_Print(uint8_t x, uint8_t y, int dec)
{
  uint8_t str5[2] = "%";
  if (re_print_s1 || re_print_s2 || re_print_s3 || re_print_s4)
    LCD_XY_Range_Clear(9, y, 13, y);

  if (re_print_s1 && (y == 0))
    re_print_s1 = 0;
  else if (re_print_s2 && (y == 1))
    re_print_s2 = 0;
  else if (re_print_s3 && (y == 2))
  {
    LCD_XY_Print_SymIdx(13, 2, 0x64);
    re_print_s3 = 0;
  }
  else if (re_print_s4 && (y == 3))
  {
    LCD_XY_Print(13, 3, str5, 1);
    re_print_s4 = 0;
  }
  else if (re_print_s5 && (y == 4))
    re_print_s5 = 0;

  if (((dec / 100) > 0) && (y == 0))
    re_print_s1 = 1;
  else if (((dec / 100) > 0) && (y == 1))
    re_print_s2 = 1;
  else if (((dec / 100) > 0) && (y == 2))
    re_print_s3 = 1;
  else if (((dec / 100) > 0) && (y == 3))
    re_print_s4 = 1;
  else if (((dec / 100) > 0) && (y == 4))
    re_print_s5 = 1;
  LCD_XY_Print_DecNumb(x, y, dec);
}

void lcd_update(int s3, int s4)
{
#if 0
	uint8_t null_str[] = "-";

  if (s3)
    SensorData_Print(9, 2, s3);
  else
    LCD_XY_Print(9, 2, null_str, 1);

  if (s4)
    SensorData_Print(9, 3, s4);
  else
    LCD_XY_Print(9, 3, null_str, 1);

#endif
  SensorData_Print(9, 2, s3);
  SensorData_Print(9, 3, s4);
}

int main(void)
{
	QM_PUTS("hello, world");
	int strlen = 0;


	  qm_uart_config_t uart1_cfg ;
	qm_gpio_port_config_t aon_cfg;
	pin_mux_setup();


	    uart1_cfg.baud_divisor = QM_UART_CFG_BAUD_DL_PACK(0, 17, 6);
	  uart1_cfg.line_control = QM_UART_LC_8N1;
	  uart1_cfg.hw_fc = false;

	qm_gpio_set_config(QM_AON_GPIO_0, &aon_cfg);
	qm_uart_set_config(QM_UART_1, &uart1_cfg);

	//uint8_t uart1_message[] = "PM2.5:";
	uint8_t uart1_message1[20] ;


	LCD_Init();


	uint8_t str0[] = "MakerPro";
	uint8_t str1[13] = "Temperature:";
	uint8_t str2[9] = "Humidity:";
	uint8_t str3[5] = "Temp:";
	uint8_t str4[6] = "Humid:";


	LCD_XY_Print(0, 0, str0, 13);
	clk_sys_udelay(DELAY*5);

	LCD_XY_Print(0, 2, str1, 9);
	LCD_XY_Print(0, 3, str2, 9);
	lcd_update(0, 0);

	HTS221_Init();

	while(1){

	    temp = HTS221_Tempurature_Read();
	    humid = HTS221_Humidity_Read();

	    lcd_update(temp, humid);

	  qm_uart_write_buffer(QM_UART_1, str3, sizeof(str3));

		 strlen = uint2str(temp, &uart1_message1[0]) ;
		 qm_uart_write_buffer(QM_UART_1, uart1_message1, strlen) ;


		  qm_uart_write_buffer(QM_UART_1, linefeed, 1);


		 qm_uart_write_buffer(QM_UART_1, str4, sizeof(str4));

		 strlen = uint2str(humid, &uart1_message1[0]) ;
		 qm_uart_write_buffer(QM_UART_1, uart1_message1, strlen) ;

		    clk_sys_udelay(DELAY*10);
	}

	return 0;
}
