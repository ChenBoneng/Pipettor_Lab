#include "lcd12864.h"

#define LCD_CS_GPIO_Port    GPIOC
#define LCD_CS_Pin          GPIO_PIN_0
#define LCD_SID_GPIO_Port   GPIOC
#define LCD_SID_Pin         GPIO_PIN_1
#define LCD_SCLK_GPIO_Port  GPIOC
#define LCD_SCLK_Pin        GPIO_PIN_2
#define LCD_RST_GPIO_Port   GPIOC
#define LCD_RST_Pin         GPIO_PIN_3

#define LCD_CMD_SYNC        0xF8U
#define LCD_DATA_SYNC       0xFAU

static void LCD12864_DelayShort(void);
static void LCD12864_WriteRawByte(uint8_t value);
static void LCD12864_WriteByte(uint8_t sync, uint8_t value);
static void LCD12864_WriteCommand(uint8_t command);
static void LCD12864_WriteData(uint8_t data);

static void LCD12864_DelayShort(void)
{
  for (volatile uint32_t i = 0U; i < 300U; ++i)
  {
  }
}

static void LCD12864_WriteRawByte(uint8_t value)
{
  for (uint8_t i = 0U; i < 8U; ++i)
  {
    HAL_GPIO_WritePin(LCD_SCLK_GPIO_Port, LCD_SCLK_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LCD_SID_GPIO_Port, LCD_SID_Pin,
                      ((value & 0x80U) != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    LCD12864_DelayShort();
    HAL_GPIO_WritePin(LCD_SCLK_GPIO_Port, LCD_SCLK_Pin, GPIO_PIN_SET);
    LCD12864_DelayShort();
    value <<= 1U;
  }
}

static void LCD12864_WriteByte(uint8_t sync, uint8_t value)
{
  HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_SET);
  LCD12864_DelayShort();
  LCD12864_WriteRawByte(sync);
  LCD12864_WriteRawByte(value & 0xF0U);
  LCD12864_WriteRawByte((uint8_t)((value << 4U) & 0xF0U));
  HAL_GPIO_WritePin(LCD_SCLK_GPIO_Port, LCD_SCLK_Pin, GPIO_PIN_RESET);
  LCD12864_DelayShort();
  HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_RESET);
  HAL_Delay(2U);
}

static void LCD12864_WriteCommand(uint8_t command)
{
  LCD12864_WriteByte(LCD_CMD_SYNC, command);
}

static void LCD12864_WriteData(uint8_t data)
{
  LCD12864_WriteByte(LCD_DATA_SYNC, data);
}

void LCD12864_Init(void)
{
  HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LCD_SCLK_GPIO_Port, LCD_SCLK_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LCD_SID_GPIO_Port, LCD_SID_Pin, GPIO_PIN_RESET);

  HAL_GPIO_WritePin(LCD_RST_GPIO_Port, LCD_RST_Pin, GPIO_PIN_RESET);
  HAL_Delay(20U);
  HAL_GPIO_WritePin(LCD_RST_GPIO_Port, LCD_RST_Pin, GPIO_PIN_SET);
  HAL_Delay(50U);

  LCD12864_WriteCommand(0x30U);
  HAL_Delay(10U);
  LCD12864_WriteCommand(0x30U);
  HAL_Delay(10U);
  LCD12864_WriteCommand(0x06U);
  HAL_Delay(5U);
  LCD12864_WriteCommand(0x0CU);
  HAL_Delay(5U);
  LCD12864_WriteCommand(0x01U);
  HAL_Delay(12U);
}

void LCD12864_Clear(void)
{
  LCD12864_WriteCommand(0x30U);
  LCD12864_WriteCommand(0x01U);
  HAL_Delay(12U);
}

void LCD12864_Fill(uint8_t pattern)
{
  LCD12864_WriteCommand(0x34U);
  LCD12864_WriteCommand(0x36U);

  for (uint8_t y = 0U; y < 32U; ++y)
  {
    LCD12864_WriteCommand((uint8_t)(0x80U + y));
    LCD12864_WriteCommand(0x80U);

    for (uint8_t x = 0U; x < 32U; ++x)
    {
      LCD12864_WriteData(pattern);
    }
  }
}

void LCD12864_SetCursor(uint8_t row, uint8_t col)
{
  static const uint8_t rowAddress[4] = {0x80U, 0x90U, 0x88U, 0x98U};

  if (row > 3U)
  {
    row = 3U;
  }
  if (col > 15U)
  {
    col = 15U;
  }

  LCD12864_WriteCommand((uint8_t)(rowAddress[row] + col));
}

void LCD12864_WriteString(uint8_t row, uint8_t col, const char *text)
{
  LCD12864_SetCursor(row, col);

  while ((*text != '\0') && (col < 16U))
  {
    LCD12864_WriteData((uint8_t)*text);
    ++text;
    ++col;
  }
}
