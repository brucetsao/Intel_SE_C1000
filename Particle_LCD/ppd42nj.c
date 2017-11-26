/*
 * PDD42NJ.c

 *
 *  Created on: Nov 28, 2016
 *      Author: R1977
 */

#include "qm_gpio.h"
#include "clk.h"
#include <x86intrin.h>
#include "qm_pinmux.h"
#include "qm_common.h"
#include "ppd42nj.h"
#include "lcd_handler.h"

/* Local Defines */
#define get_ticks() _rdtsc()
#define PIN_PPD42 QM_PIN_ID_0
#define PIN_MUX_FN (QM_PMUX_FN_0)

qm_gpio_port_config_t gpio_cfg;
qm_gpio_state_t gpio_state;
uint32_t duration;
uint32_t lowpulseoccupancy = 0;
uint32_t starttime;
uint32_t starttime1;
uint32_t sampletime_ms = 30000; /* sampling time 30 sec for each data update*/
uint32_t ticks_per_us = 32;
uint32_t ratio = 0;
uint32_t concentration = 0;
uint32_t delaytime;
uint32_t tsec=0;

uint32_t pulseIn(uint8_t pin, bool level);
uint32_t millis();

/**
 * PDD42NJ initiation.
 * PIN_PPD42 init to digital input pin
 *
 * @return
 */
void PPD42NJ_Init(void){

	qm_pmux_select(PIN_PPD42, PIN_MUX_FN);
	gpio_cfg.direction = ~(BIT(PIN_PPD42));
	qm_gpio_set_config(QM_GPIO_0, &gpio_cfg);

}

/**
 * PDD42NJ read function.
 * Read from PIN_PPD42 every 30 sec
 * Lo Pulse Width = Approximately 1ms to 300ms
 * Lo Pulse Occupancy Ratio (%) = Lo Pulse Occupancy time / unit time (30sec)
 * @return
 */
uint64_t PPD42NJ_Read(void){

	starttime = millis();//get the current time for PDD42 cycle time
	starttime1 = millis();//get the current time for timer

	while(1){
		duration = pulseIn(PIN_PPD42, 0);
		lowpulseoccupancy = lowpulseoccupancy + duration;

		if ((millis() - starttime1) > 1000) {
			//QM_PRINTF("time sec: %d\n", tsec++);
			LCD_XY_Range_Clear(9, 2, 11, 2);
			LCD_XY_Print_DecNumb(9, 2, tsec++);
			starttime1 = millis(); /*reset starttime1 for next sec */
		}

		if ((millis() - starttime) > sampletime_ms) // Run if loop after the loop has run for sampletime_ms
		{
		  ratio = (lowpulseoccupancy * 1000+(sampletime_ms >> 1)) / sampletime_ms; // ratios of lowpulseoccupancy and sampletims_ms in percentage
		  	  	  	  	  	  	  	  	  	  	  	  	  	  	  	  	  	  	  	  /* (sampletime_ms >> 1) add 1/2 to give correct rounding */
		  concentration = 56100*ratio*ratio*ratio/1000000000UL-6920*ratio*ratio/1000000UL+1560*ratio/1000;  // using spec sheet curve

		  QM_PRINTF("low pulse width per us: %d\n", lowpulseoccupancy);
		  QM_PRINTF("ratio: %d\n", ratio);
		  QM_PRINTF("concentration: %d\n", concentration);
		  // reset counters for next loop
		  lowpulseoccupancy = 0;
		  starttime = millis(); // reset starttime as the total time the program has been running
		  duration = 0;
		  tsec =0;
		  return concentration;
		}
	}

}

uint32_t pulseIn(uint8_t pin, bool level){


	/*uint32_t ticks_per_us = clk_sys_get_ticks_per_us();*/
	uint32_t timeout = ticks_per_us * 1;
	uint32_t count = 0;
	uint32_t width = 0; /*keep initialization out of time critical area*/


	//QM_PUTS("Wait for any previous pulse to end");
	/* Wait for any previous pulse to end */
	do {
		if (count++ >= timeout) /*  */
			return 0;
		/* Busy loop to check pin status */
		qm_gpio_read_pin(QM_GPIO_0, PIN_PPD42, &gpio_state);

	} while (gpio_state == level);


	//QM_PUTS("Wait for the pulse to start");
	/* Wait for the pulse to start */
	do {
		if (count++ >= timeout)
			return 0;
		/* Busy loop to check pin status */
		qm_gpio_read_pin(QM_GPIO_0, PIN_PPD42, &gpio_state);
	} while (gpio_state != level);


	/*QM_PUTS("Wait for the pulse to stop");*/

	/* Wait for the pulse to stop */
	do {
		/* Busy loop to check pin status */
		qm_gpio_read_pin(QM_GPIO_0, PIN_PPD42, &gpio_state);
		width++;
	} while (gpio_state == level);


	width = (width*108+2)/1000/ticks_per_us; /* return with ms*/
	return width;
}

uint32_t millis() {

	return get_ticks()/(ticks_per_us*1000);
}

