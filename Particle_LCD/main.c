/*
 * Copyright (c) 2016, Intel Corporation
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

#include "qm_common.h"
#include "ppd42nj.h"
#include "lcd_handler.h"
#include "qm_gpio.h"
#include "qm_pinmux.h"

qm_gpio_port_config_t LED_cfg;

int re_print_s1 = 0;
int re_print_s2 = 0;
int re_print_s3 = 0;
int re_print_s4 = 0;
int re_print_s5 = 0;

uint32_t concentration ;
uint8_t pm25 = 0;

void SensorData_Print(uint8_t x, uint8_t y, int dec);
/*uint8_t comp_sensor_data(uint8_t *sensorData, uint8_t *upd_data);*/

/* Hello world example app */
int main(void)
{
	QM_PRINTF("hello, world\n");


	 /* initialize GPIO PIN Status */

	LCD_Init();
	PPD42NJ_Init();

	uint8_t str0[9] = "   Sertek";
	uint8_t str1[11] = "Wait 30 sec";
	uint8_t str2[9] = "PM2.5   :";
	uint8_t str3[9] = "TIME Sec:";
	uint8_t str4[9] = "  Green  ";
	uint8_t str5[9] = "  Yellow ";
	uint8_t str6[9] = "  Red    ";
	uint8_t str7[9] = "  Purple ";

	LCD_XY_Print(0, 0, str0, 9);
	LCD_XY_Print(0, 1, str1, 11);
	LCD_XY_Print(0, 2, str3, 9);
	LCD_XY_Print(0, 3, str2, 9);

	while(1){

		concentration = PPD42NJ_Read();
		pm25 = concentration ;
		QM_PRINTF("concentration: %d\n", concentration);
		LCD_XY_Range_Clear(9, 3, 12, 3);
		LCD_XY_Print_DecNumb(9, 3, concentration);
		//SensorData_Print(9, 3, concentration);

		if (concentration < 36) {

			LCD_XY_Range_Clear(2, 4, 12, 4);
			LCD_XY_Print(0, 4, str4, 9);

		}
		else if (35 < concentration && concentration < 54 ) {

			LCD_XY_Range_Clear(2, 4, 12, 4);
			LCD_XY_Print(0, 4, str5, 9);

		}
		else if (53 < concentration && concentration < 71 ) {

					LCD_XY_Range_Clear(2, 4, 12, 4);
					LCD_XY_Print(0, 4, str6, 9);


		}
		else if (concentration > 70 ) {

					LCD_XY_Range_Clear(2, 4, 12, 4);
					LCD_XY_Print(0, 4, str7, 9);

		}

	}

	return 0;
}

void SensorData_Print(uint8_t x, uint8_t y, int dec)
{
  uint8_t str5[2] = "%";
  if (re_print_s1 || re_print_s2 || re_print_s3 || re_print_s4)
    LCD_XY_Range_Clear(9, y, 13, y);

  if (re_print_s1 && (y == 0))
  {

    re_print_s1 = 0;
  }
  else if (re_print_s2 && (y == 1))
  {

    re_print_s2 = 0;
  }
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

  if (((dec / 100) > 0) && (y == 0))
    re_print_s1 = 1;
  else if (((dec / 100) > 0) && (y == 1))
    re_print_s2 = 1;
  else if (((dec / 100) > 0) && (y == 2))
    re_print_s3 = 1;
  else if (((dec / 100) > 0) && (y == 3))
    re_print_s4 = 1;
  LCD_XY_Print_DecNumb(x, y, dec);
}

