/*
 * hts221_handler.c
 */
#include "clk.h"
#include "qm_common.h"
#include "qm_i2c.h"
#include "qm_pinmux.h"
#include "qm_pin_functions.h"
#include "hts221_handler.h"
#include <string.h>
/* #include <math.h> */

#define READ_LEN 23
#define I2C_MAX_RETRY_COUNT 5

uint8_t H0_rH_x2, H1_rH_x2, T0_degC_x8, T1_degC_x8, T0_msb, T1_msb;
int16_t H0_T0_OUT, H1_T0_OUT, T0_OUT, T1_OUT;
int16_t temperatureCount = 0, humidityCount = 0;  /* variables to hold raw HTS221 temperature and humidity values */
float HTS221_humidity, HTS221_temperature;

static void HTS221_Calibration(void)
{
  /* Read calibrations data */
  H0_rH_x2 = HTS221_Sensor_1byte_Read(HTS221_CALIB_0);
  H1_rH_x2 = HTS221_Sensor_1byte_Read(HTS221_CALIB_1);
  T0_degC_x8 = HTS221_Sensor_1byte_Read(HTS221_CALIB_2);
  T1_degC_x8 = HTS221_Sensor_1byte_Read(HTS221_CALIB_3);
  T0_msb = HTS221_Sensor_1byte_Read(HTS221_CALIB_5) & 0x03;
  T1_msb = (HTS221_Sensor_1byte_Read(HTS221_CALIB_5) & 0x0C) >> 2;
  H0_T0_OUT = (int16_t)(HTS221_Sensor_1byte_Read(HTS221_CALIB_7) << 8 | HTS221_Sensor_1byte_Read(HTS221_CALIB_6));
  H1_T0_OUT = (int16_t)(HTS221_Sensor_1byte_Read(HTS221_CALIB_B) << 8 | HTS221_Sensor_1byte_Read(HTS221_CALIB_A));
  T0_OUT = ((int16_t)HTS221_Sensor_1byte_Read(HTS221_CALIB_D) << 8 | HTS221_Sensor_1byte_Read(HTS221_CALIB_C));
  T1_OUT = ((int16_t)HTS221_Sensor_1byte_Read(HTS221_CALIB_F) << 8 | HTS221_Sensor_1byte_Read(HTS221_CALIB_E));
}

char HTS221_Init(void)
{
  qm_i2c_config_t cfg;
  uint8_t ctrl01 = 0, ctrl03 = 0, av_conf = 0;

  qm_pmux_select(QM_PIN_ID_22, QM_PIN_22_FN_I2C1_SCL);
  qm_pmux_select(QM_PIN_ID_23, QM_PIN_23_FN_I2C1_SDA);
  clk_periph_enable(CLK_PERIPH_CLK | CLK_PERIPH_I2C_M1_REGISTER);
  /* Configure I2C */
  cfg.address_mode = QM_I2C_7_BIT;
  cfg.mode = QM_I2C_MASTER;
  cfg.speed = QM_I2C_SPEED_FAST;

  qm_i2c_set_config(QM_I2C_1, &cfg);
  if (HTS221_Sensor_1byte_Read(HTS221_WHOAMI_REG) == (uint8_t)0xBC)
  {
    /* fill 0x3F to 0x10*/
    av_conf = HTS221_Sensor_1byte_Read(HTS221_AVCONF_REG);
    if (av_conf == 0)
    {
       HTS221_Sensor_1byte_Write(HTS221_AVCONF_REG, av_conf | 0x3F);
    }
    /* fill CTRLREG 00 0x81 */
	ctrl01 = HTS221_Sensor_1byte_Read(HTS221_CTRLREG01_REG);
	if (ctrl01 == 0)
	{
      HTS221_Sensor_1byte_Write(HTS221_CTRLREG01_REG, ctrl01 | 0x85);
      ctrl01 = HTS221_Sensor_1byte_Read(HTS221_CTRLREG01_REG);
/*
	  ctrl02 = HTS221_Sensor_1byte_Read(HTS221_CTRLREG02_REG);
      HTS221_Sensor_1byte_Write(HTS221_CTRLREG02_REG, ctrl02 | 0x01);
      ctrl02 = HTS221_Sensor_1byte_Read(HTS221_CTRLREG02_REG);
 */
	}
	ctrl03 = HTS221_Sensor_1byte_Read(HTS221_CTRLREG03_REG);
	if (ctrl03 == 0)
	{

	}
	HTS221_Calibration();
    return 1;
  }
  return 0;
}

float HTS221_Tempurature_Read(void)
{
  uint8_t temp_l = 0, temp_h = 0;
  uint8_t status = 0;

  status = HTS221_Sensor_1byte_Read(HTS221_STATUS_REG);
  if (!(status & 0x01))
    return 0;

  temp_l = HTS221_Sensor_1byte_Read(HTS221_TEMP_L_REG);
  temp_h = HTS221_Sensor_1byte_Read(HTS221_TEMP_H_REG);

  temperatureCount = (int16_t)(((temp_h << 8) & 0xff00) | temp_l);
  uint16_t outT0 = (uint16_t)(( T0_msb << 8) | T0_degC_x8);
  uint16_t outT1 = (uint16_t)(( T1_msb << 8) | T1_degC_x8);
  HTS221_temperature = (((float)temperatureCount - (float)T0_OUT)/((float)T1_OUT - (float)T0_OUT))*((float)outT1 - (float)outT0)/8. + (float) outT0/8.;
  return HTS221_temperature;
}


float HTS221_Humidity_Read(void)
{
  uint8_t humi_l = 0, humi_h = 0;
  int16_t humidity_t;
  uint8_t status = 0;
  status = HTS221_Sensor_1byte_Read(HTS221_STATUS_REG);
  if (!(status & 0x02))
    return 0;

  humi_l = HTS221_Sensor_1byte_Read(HTS221_HUMI_L_REG);
  humi_h = HTS221_Sensor_1byte_Read(HTS221_HUMI_H_REG);
  humidityCount = (int16_t)((humi_h << 8) & 0xff00) | humi_l;
  humidity_t = (((float)humidityCount - (float)H0_T0_OUT)/((float)H1_T0_OUT - (float)H0_T0_OUT))*((float)H1_rH_x2 - (float)H0_rH_x2)/2. + (float) H0_rH_x2/2.;

  return humidity_t;
}

uint8_t HTS221_Sensor_1byte_Write(uint8_t reg, uint8_t data)
{
	int rc;
	qm_i2c_status_t status;
	uint8_t w_data[2];
	w_data[0] = reg;
	w_data[1] = data;
	rc = qm_i2c_master_write(QM_I2C_1, SLAVE_ADDR, w_data, 2, true, &status);
	if (rc != 0) {
		return rc;
	}
	return 0;
}

uint8_t HTS221_Sensor_1byte_Read(uint8_t reg)
{
	int rc;
	qm_i2c_status_t status;
	uint8_t w_data[2];
	uint8_t r_data[2];


	memset(r_data, 0, 2);
	w_data[0] = reg;

	rc = qm_i2c_master_write(QM_I2C_1, SLAVE_ADDR, w_data, 1, false, &status);
	if (rc != 0) {
		return rc;
  	}

  /* Master read */
	rc = qm_i2c_master_read(QM_I2C_1, SLAVE_ADDR, r_data, 1, true, &status);
	if (rc != 0) {
		return rc;
	}

    return r_data[0];

}


