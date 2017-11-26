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

 */

#include "qm_pinmux.h"
#include "qm_pin_functions.h"

#include "qm_gpio.h"

#include "clk.h"
#include "qm_common.h"

#include "qm_uart.h"


#define DELAY 250000UL /* 0.25 seconds. */
/* Define aon3 for UART mux pin*/
#define AON3 3
/* Wait time (us). */
#define WAIT_1MSEC (1000)




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


int main(void)
{
	int strlen = 0;


	  qm_uart_config_t uart1_cfg ;
	qm_gpio_port_config_t aon_cfg;



	pin_mux_setup();


	    uart1_cfg.baud_divisor = QM_UART_CFG_BAUD_DL_PACK(0, 17, 6);
	  uart1_cfg.line_control = QM_UART_LC_8N1;
	  uart1_cfg.hw_fc = false;

	aon_cfg.direction = BIT(AON3);		   /* Set AON3 pins as output. */

	qm_gpio_set_config(QM_AON_GPIO_0, &aon_cfg);
	qm_uart_set_config(QM_UART_1, &uart1_cfg);

	uint8_t uart1_message[] = "PM2.5:";
	uint8_t uart1_message1[20] ;


	while(1){

		clk_sys_udelay(WAIT_1MSEC);

				for (int i = 50 ; i <120; i++)
				{
					strlen = uint2str(i, &uart1_message1[0]) ;
					 qm_uart_write_buffer(QM_UART_1, uart1_message, sizeof(uart1_message));
					 qm_uart_write_buffer(QM_UART_1, uart1_message1, strlen) ;
					   clk_sys_udelay(DELAY*5);
				}


	}

	return 0;
}



