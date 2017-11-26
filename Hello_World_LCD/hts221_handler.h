/*
 * lcd_handler.h
 *
 *  Created on: 2016¦~5¤ë20¤é
 *      Author: Tyng
 */

char HTS221_Init(void);
uint8_t HTS221_Sensor_1byte_Read(uint8_t reg);
uint8_t HTS221_Sensor_1byte_Write(uint8_t reg, uint8_t data);
float HTS221_Humidity_Read(void);
float HTS221_Tempurature_Read(void);

#define SLAVE_ADDR 0x5F
#define HTS221_WHOAMI_REG 0x0F
#define HTS221_AVCONF_REG 0x10
#define HTS221_CTRLREG01_REG 0x20
#define HTS221_CTRLREG02_REG 0x21
#define HTS221_CTRLREG03_REG 0x22
#define HTS221_STATUS_REG  0x27
#define HTS221_HUMI_L_REG  0x28
#define HTS221_HUMI_H_REG  0x29
#define HTS221_TEMP_L_REG  0x2A
#define HTS221_TEMP_H_REG  0x2B

#define HUM_DECIMAL_DIGITS                  (2)
#define TEMP_DECIMAL_DIGITS                 (2)
/*
#define HTS221_WHO_AM_I          0x0F   // should be 0xBC
#define HTS221_AV_CONF           0x10
#define HTS221_CTRL_REG1         0x20
#define HTS221_CTRL_REG2         0x21
#define HTS221_CTRL_REG3         0x22
#define HTS221_STATUS_REG        0x27
#define HTS221_HUMIDITY_OUT_L    0x28
#define HTS221_HUMIDITY_OUT_H    0x29
#define HTS221_TEMP_OUT_L        0x2A
#define HTS221_TEMP_OUT_H        0x2B
*/
#define HTS221_CALIB_0           0x30  /* CALIB 0 - F are read only! */
#define HTS221_CALIB_1           0x31
#define HTS221_CALIB_2           0x32
#define HTS221_CALIB_3           0x33
#define HTS221_CALIB_4           0x34
#define HTS221_CALIB_5           0x35
#define HTS221_CALIB_6           0x36
#define HTS221_CALIB_7           0x37
#define HTS221_CALIB_8           0x38
#define HTS221_CALIB_9           0x39
#define HTS221_CALIB_A           0x3A
#define HTS221_CALIB_B           0x3B
#define HTS221_CALIB_C           0x3C
#define HTS221_CALIB_D           0x3D
#define HTS221_CALIB_E           0x3E
#define HTS221_CALIB_F           0x3F


