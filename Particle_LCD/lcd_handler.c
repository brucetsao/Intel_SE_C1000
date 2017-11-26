/*
 * lcd_handler.c
 */
#include "qm_common.h"
#include "qm_gpio.h"
#include "qm_spi.h"
#include "qm_pinmux.h"
#include "qm_pin_functions.h"
#include "qm_interrupt.h"
#include "clk.h"
#include "string.h"
#include "ASCII.h"
#include "lcd_handler.h"

/* Local Defines */
#define SPI_CLOCK_DIV (256)
#define LCD_CMD    (0)
#define LCD_DATA   (1)
#define LCD_X     (84)
#define LCD_MAX_X (14)
#define LCD_Y     (48)
#define LCD_MAX_Y (6)
#define BUFFERSIZE (LCD_X*LCD_Y)
#define PIN_BACKLIGHT 1  /* QM_PIN_ID_1 */
#define PIN_RESET 30 /* QM_PIN_ID_34 */
#define PIN_DC 20   /* QM_PIN_ID_54 */


qm_spi_config_t cfg;
qm_spi_transfer_t polled_xfer_desc;
qm_spi_status_t status;
qm_gpio_port_config_t LCD_gpioA_cfg;

/*static uint8_t tx_buff[BUFFERSIZE];*/

void lcd_private_write(uint8_t DC, uint8_t *data, uint16_t len);


void LcdCharacter(char character)
{
  uint8_t str_buf[10];
  int index = 0;
  str_buf[index] = 0x00;
  for ( index = 0; index < 5; index++)
  {
    str_buf[index + 1] =  ASCII[character - 0x20][index];
  }
  str_buf[index + 1] = 0x00;
  lcd_private_write(LCD_DATA, str_buf, 6);
}

void LCD_Clear(void)
{
  int index = 0;
  uint8_t clean_buf[BUFFERSIZE / 8];
  for (index = 0; index < BUFFERSIZE / 8; index++)
  {
	  clean_buf[index] = 0;
  }
  lcd_private_write(LCD_DATA, clean_buf, index);
  uint8_t set_xy[] = {0x80, 0x40};
  lcd_private_write(LCD_CMD, set_xy, 2);
}

void lcd_private_write(uint8_t DC, uint8_t *data, uint16_t len)
{
#if 0
  int i = 0;
  for (i = 0; i < BUFFERSIZE; i ++)
    tx_buff[i] = 0;
  for (i = 0; i < len; i++)
    tx_buff[i] = data[i];
#endif
  polled_xfer_desc.tx = data;
  polled_xfer_desc.tx_len = len;
  if (DC) /* Data mode */
    qm_gpio_set_pin(QM_GPIO_0, PIN_DC);
  else /* Command Mode */
    qm_gpio_clear_pin(QM_GPIO_0, PIN_DC);

  qm_spi_transfer(QM_SPI_MST_0, &polled_xfer_desc, &status);
}

void LCD_String(char *characters, int len)
{
  int idx = 0;
  while (*characters)
  {
    LcdCharacter(characters[idx]);
    idx++;
    if (idx >= len)
      break;
  }
}

void LCD_XY_Set(uint8_t x, uint8_t y)
{
  uint8_t set_xy[2];
  set_xy[0] = 0x80 | (x * 6);
  set_xy[1] = 0x40 | y;
  lcd_private_write(LCD_CMD, set_xy, 2);
}

int LCD_XY_Range_Clear(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1)
{
  int idx = 0;
  int x_clear_len = 0;
  int y_clear_len = 0;
  uint8_t clean_byte[6];

  memset(clean_byte, 0, 6);
  /* out of x,y length */
  if (x1 - x0 > LCD_MAX_X || y1 - y0 > LCD_MAX_Y)
	return -1;

  x_clear_len = x1 - x0;
  y_clear_len = y1 - y0;
  LCD_XY_Set(x0, y0);
  for (idx = 0; idx < x_clear_len; idx++)
  {
    LCD_XY_Set(x0 + idx, y0);
    lcd_private_write(LCD_DATA, clean_byte, 6);
  }
  for (idx = 0; idx < y_clear_len; idx++)
  {
    LCD_XY_Set(x0, y0 + idx);
    lcd_private_write(LCD_DATA, clean_byte, 6);
  }
  return 0;
}

int LCD_XY_Print(uint8_t x, uint8_t y, uint8_t *strb, uint16_t len)
{
  if (x > LCD_MAX_X)
	  return -1;
  if (y > LCD_MAX_Y)
	  return -1;

  LCD_XY_Set(x, y);
  LCD_String((char *)strb, len);
  return 0;
}

void LCD_XY_Print_DecNumb(uint8_t x, uint8_t y, int dec)
{
  int i = 0;
  int numbs = 0;
  int t = 0, j = 0;
  uint8_t StrBuf[4];
  if (dec >= 1000 && dec < 10000)
  {
    numbs = 4;
    t = 1000;
  }
  else if (dec >= 100 && dec < 1000)
  {
    numbs = 3;
    t = 100;
  }
  else if (dec >= 10 && dec < 100)
  {
    numbs = 2;
    t = 10;
  }
  else if (dec < 10 && dec >= 0)
  {
    numbs = 1;
    t = 1;
  }
  else
	return;
  for (i = 0; i < numbs; i++)
  {
    StrBuf[i] = ((dec - j) / t) + 0x30;
    j = (dec / t) * t;
    t /= 10;
  }
  LCD_XY_Set(x, y);
  StrBuf[numbs + 1] = 0;
  LCD_String((char *)StrBuf, numbs);
}

void LCD_XY_Print_SymIdx(uint8_t x, uint8_t y, int char_sym)
{
  uint8_t str_buf[10];
  int index = 0;
  str_buf[index] = 0x00;
  for ( index = 0; index < 5; index++)
    str_buf[index + 1] =  ASCII[char_sym][index];
  str_buf[index + 1] = 0x00;
  LCD_XY_Set(x, y);
  lcd_private_write(LCD_DATA, str_buf, 6);
}

void LCD_Init(void)
{
  qm_pmux_select(QM_PIN_ID_54, QM_PMUX_FN_0);         /* DC */
  qm_pmux_select(QM_PIN_ID_34, QM_PMUX_FN_1);      /* RST */
  qm_pmux_select(QM_PIN_ID_1, QM_PMUX_FN_0);  /* BK */
  LCD_gpioA_cfg.direction = BIT(PIN_DC) | BIT(PIN_RESET) | BIT(PIN_BACKLIGHT);
  qm_gpio_set_config(QM_GPIO_0, &LCD_gpioA_cfg);
  qm_pmux_select(QM_PIN_ID_52, QM_PMUX_FN_1);  /* SS2 */
  qm_pmux_select(QM_PIN_ID_55, QM_PMUX_FN_1);   /* SCK */
  qm_pmux_select(QM_PIN_ID_57, QM_PMUX_FN_1);   /* TXD */

  clk_periph_enable(CLK_PERIPH_CLK | CLK_PERIPH_SPI_M0_REGISTER);
  /*  Initialise SPI configuration */
  cfg.frame_size = QM_SPI_FRAME_SIZE_8_BIT;
  cfg.transfer_mode = QM_SPI_TMOD_TX;
  cfg.bus_mode = QM_SPI_BMODE_0;
  cfg.clk_divider = SPI_CLOCK_DIV;
  qm_spi_set_config(QM_SPI_MST_0, &cfg);

  /* initialize GPIO PIN Status */
  qm_gpio_clear_pin(QM_GPIO_0, PIN_DC);
  qm_gpio_clear_pin(QM_GPIO_0, PIN_RESET);
  qm_gpio_clear_pin(QM_GPIO_0, PIN_BACKLIGHT);

  /* Set up loopback mode by RMW directly to the ctrlr0 register.*/
  QM_SPI[QM_SPI_MST_0]->ctrlr0 |= BIT(11);

  qm_spi_slave_select(QM_SPI_MST_0, QM_SPI_SS_0);

  /* Reset LCD */
  qm_gpio_clear_pin(QM_GPIO_0, PIN_RESET);
  clk_sys_udelay(1000);
  qm_gpio_set_pin(QM_GPIO_0, PIN_RESET);

  /* Enable LCD Backlight */
  qm_gpio_set_pin(QM_GPIO_0, PIN_BACKLIGHT);
  clk_sys_udelay(1500);

  uint8_t init_cmd[] = {0x21, 0xB1, 0x04, 0x13, 0x20, 0x0C};
  lcd_private_write(LCD_CMD, init_cmd, 6);
  LCD_Clear();
}

