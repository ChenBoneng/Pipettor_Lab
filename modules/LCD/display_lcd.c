#include "display_lcd.h"
#include <string.h>
#include "bsp_dwt.h"
#include "main.h"

/*
 * ST7920 串行模式引脚。
 *
 * LCD 串行空闲状态必须保持：
 * - CS   = 0；
 * - SID  = 0；
 * - SCLK = 0；
 * - RST  = 1。
 */
#define DISPLAY_LCD_CS_GPIO_PORT      GPIOC
#define DISPLAY_LCD_CS_GPIO_PIN       GPIO_PIN_0
#define DISPLAY_LCD_SID_GPIO_PORT     GPIOC
#define DISPLAY_LCD_SID_GPIO_PIN      GPIO_PIN_1
#define DISPLAY_LCD_SCLK_GPIO_PORT    GPIOC
#define DISPLAY_LCD_SCLK_GPIO_PIN     GPIO_PIN_2
#define DISPLAY_LCD_RST_GPIO_PORT     GPIOC
#define DISPLAY_LCD_RST_GPIO_PIN      GPIO_PIN_3

/*
 * ST7920 串行写入控制字。
 *
 * 串行写一个命令或数据必须连续发送 3 个字节：
 * - 写命令：0xF8 + 数据高四位 + 数据低四位；
 * - 写数据：0xFA + 数据高四位 + 数据低四位。
 */
#define DISPLAY_LCD_CMD_SYNC          0xF8U
#define DISPLAY_LCD_DATA_SYNC         0xFAU

#define DISPLAY_LCD_TEXT_COLS         8U
#define DISPLAY_LCD_LINE_BYTES        16U
#define DISPLAY_LCD_GDRAM_HALF_ROWS   32U
#define DISPLAY_LCD_GDRAM_WORDS       8U
#define DISPLAY_LCD_GDRAM_ROW_BYTES   16U
#define DISPLAY_LCD_SPACE             0x20U

/*
 * 常用指令。
 */
#define DISPLAY_LCD_CMD_CLEAR         0x01U
#define DISPLAY_LCD_CMD_HOME          0x02U
#define DISPLAY_LCD_CMD_ENTRY_MODE    0x06U
#define DISPLAY_LCD_CMD_DISPLAY_ON    0x0CU
#define DISPLAY_LCD_CMD_BASIC         0x30U
#define DISPLAY_LCD_CMD_EXT_G_OFF     0x34U
#define DISPLAY_LCD_CMD_EXT_G_ON      0x36U
#define DISPLAY_LCD_CMD_ADDR_BASE     0x80U

static uint8_t display_lcd_inited = 0U;
static uint8_t display_lcd_line_cache[DISPLAY_LCD_ROW_MAX][DISPLAY_LCD_LINE_BYTES];

static void DisplayLcd_SetIdleLevel(void);
static void DisplayLcd_DelayBit(void);
static void DisplayLcd_DelayNormalCommand(void);
static void DisplayLcd_DelayLongCommand(void);
static void DisplayLcd_WritePin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state);
static void DisplayLcd_WriteRawByte(uint8_t value);
static void DisplayLcd_WriteSerialFrame(uint8_t sync, uint8_t value);
static void DisplayLcd_WriteCommand(uint8_t command);
static void DisplayLcd_WriteData(uint8_t data);
static uint8_t DisplayLcd_IsValidRow(DisplayLcdRow_e row);
static uint8_t DisplayLcd_GetDdramAddress(DisplayLcdRow_e row, uint8_t col, uint8_t *addr);

/**
 * @brief 设置 LCD 串行总线空闲电平。
 *
 * @note PC0~PC3 的 GPIO 模式、速度和上下拉由 CubeMX 在 MX_GPIO_Init() 中配置。
 *       LCD 模块这里只负责把总线输出到 ST7920 要求的空闲电平。
 */
static void DisplayLcd_SetIdleLevel(void)
{
    DisplayLcd_WritePin(DISPLAY_LCD_CS_GPIO_PORT, DISPLAY_LCD_CS_GPIO_PIN, GPIO_PIN_RESET);
    DisplayLcd_WritePin(DISPLAY_LCD_SID_GPIO_PORT, DISPLAY_LCD_SID_GPIO_PIN, GPIO_PIN_RESET);
    DisplayLcd_WritePin(DISPLAY_LCD_SCLK_GPIO_PORT, DISPLAY_LCD_SCLK_GPIO_PIN, GPIO_PIN_RESET);
    DisplayLcd_WritePin(DISPLAY_LCD_RST_GPIO_PORT, DISPLAY_LCD_RST_GPIO_PIN, GPIO_PIN_SET);
}

/**
 * @brief 软件 SPI 位间隔。
 *
 * ST7920 在 SCLK 上升沿采样 SID。HAL_GPIO_WritePin() 本身已经有一定开销，
 * 这里再补一个极短空循环，让边沿更稳一点。
 */
static void DisplayLcd_DelayBit(void)
{
    for (volatile uint32_t i = 0U; i < 12U; i++)
    {
    }
}

/**
 * @brief 普通指令/数据等待。
 *
 * 文档建议普通指令约 72us。这里使用 DWT 等待 100us，兼顾稳定性和刷新速度。
 * AllTaskInit() 中已经先调用 DWT_Init()，再调用 DisplayLcd_Init()。
 */
static void DisplayLcd_DelayNormalCommand(void)
{
    DWT_Delay(0.000100f);
}

/**
 * @brief 清屏和归位指令等待。
 *
 * 文档建议清屏、归位至少等待 4.6ms。这里留 10ms 余量。
 */
static void DisplayLcd_DelayLongCommand(void)
{
    HAL_Delay(10U);
}

/**
 * @brief 写一个 GPIO 输出电平。
 *
 * @param port GPIO 端口。
 * @param pin GPIO 引脚。
 * @param state 输出电平。
 */
static void DisplayLcd_WritePin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state)
{
    HAL_GPIO_WritePin(port, pin, state);
}

/**
 * @brief 按 bit7 到 bit0 的顺序发送 1 个原始字节。
 *
 * @param value 要发送的原始字节。
 *
 * @note 每一位发送流程：SCLK 拉低 -> SID 放数据 -> SCLK 拉高，LCD 在上升沿采样。
 */
static void DisplayLcd_WriteRawByte(uint8_t value)
{
    for (uint8_t bit = 0U; bit < 8U; bit++)
    {
        DisplayLcd_WritePin(DISPLAY_LCD_SCLK_GPIO_PORT,
                            DISPLAY_LCD_SCLK_GPIO_PIN,
                            GPIO_PIN_RESET);
        DisplayLcd_WritePin(DISPLAY_LCD_SID_GPIO_PORT,
                            DISPLAY_LCD_SID_GPIO_PIN,
                            ((value & 0x80U) != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
        DisplayLcd_DelayBit();
        DisplayLcd_WritePin(DISPLAY_LCD_SCLK_GPIO_PORT,
                            DISPLAY_LCD_SCLK_GPIO_PIN,
                            GPIO_PIN_SET);
        DisplayLcd_DelayBit();
        value <<= 1U;
    }
}

/**
 * @brief 发送 ST7920 串行三字节帧。
 *
 * @param sync 控制字节，命令为 0xF8，数据为 0xFA。
 * @param value 原始命令或数据字节。
 *
 * @note 每发送一个命令或数据时，CS 先拉高，连续发送 3 个字节后再拉低。
 */
static void DisplayLcd_WriteSerialFrame(uint8_t sync, uint8_t value)
{
    DisplayLcd_WritePin(DISPLAY_LCD_CS_GPIO_PORT, DISPLAY_LCD_CS_GPIO_PIN, GPIO_PIN_SET);
    DisplayLcd_DelayBit();

    DisplayLcd_WriteRawByte(sync);
    DisplayLcd_WriteRawByte(value & 0xF0U);
    DisplayLcd_WriteRawByte((uint8_t)((value << 4U) & 0xF0U));

    DisplayLcd_WritePin(DISPLAY_LCD_SCLK_GPIO_PORT, DISPLAY_LCD_SCLK_GPIO_PIN, GPIO_PIN_RESET);
    DisplayLcd_WritePin(DISPLAY_LCD_SID_GPIO_PORT, DISPLAY_LCD_SID_GPIO_PIN, GPIO_PIN_RESET);
    DisplayLcd_DelayBit();
    DisplayLcd_WritePin(DISPLAY_LCD_CS_GPIO_PORT, DISPLAY_LCD_CS_GPIO_PIN, GPIO_PIN_RESET);
}

/**
 * @brief 写 1 个 LCD 命令。
 *
 * @param command ST7920 指令字节。
 */
static void DisplayLcd_WriteCommand(uint8_t command)
{
    DisplayLcd_WriteSerialFrame(DISPLAY_LCD_CMD_SYNC, command);

    if ((command == DISPLAY_LCD_CMD_CLEAR) || (command == DISPLAY_LCD_CMD_HOME))
    {
        DisplayLcd_DelayLongCommand();
    }
    else
    {
        DisplayLcd_DelayNormalCommand();
    }
}

/**
 * @brief 写 1 个 LCD 数据字节。
 *
 * @param data 待写入 DDRAM 或 GDRAM 的数据。
 */
static void DisplayLcd_WriteData(uint8_t data)
{
    DisplayLcd_WriteSerialFrame(DISPLAY_LCD_DATA_SYNC, data);
    DisplayLcd_DelayNormalCommand();
}

/**
 * @brief 判断文本行号是否合法。
 *
 * @param row 行号。
 * @return 1 表示合法；0 表示非法。
 */
static uint8_t DisplayLcd_IsValidRow(DisplayLcdRow_e row)
{
    return (row < DISPLAY_LCD_ROW_MAX) ? 1U : 0U;
}

/**
 * @brief 根据行列计算 ST7920 文本 DDRAM 地址。
 *
 * @param row 行号，取值见 DisplayLcdRow_e。
 * @param col 字位列号，范围 0~7。
 * @param addr 输出参数，用于保存 DDRAM 地址。
 * @return 1 表示计算成功；0 表示参数非法。
 */
static uint8_t DisplayLcd_GetDdramAddress(DisplayLcdRow_e row, uint8_t col, uint8_t *addr)
{
    static const uint8_t row_addr[DISPLAY_LCD_ROW_MAX] = {
        0x80U,
        0x90U,
        0x88U,
        0x98U,
    };

    if ((DisplayLcd_IsValidRow(row) == 0U) ||
        (col >= DISPLAY_LCD_TEXT_COLS) ||
        (addr == NULL))
    {
        return 0U;
    }

    *addr = (uint8_t)(row_addr[row] + col);
    return 1U;
}

void DisplayLcd_Init(void)
{
    if (display_lcd_inited != 0U)
    {
        return;
    }

    DisplayLcd_SetIdleLevel();

    /*
     * LCD 上电后先硬复位一次。RST 低电平保持 20ms，再释放并等待内部稳定。
     */
    DisplayLcd_WritePin(DISPLAY_LCD_RST_GPIO_PORT, DISPLAY_LCD_RST_GPIO_PIN, GPIO_PIN_RESET);
    HAL_Delay(20U);
    DisplayLcd_WritePin(DISPLAY_LCD_RST_GPIO_PORT, DISPLAY_LCD_RST_GPIO_PIN, GPIO_PIN_SET);
    HAL_Delay(50U);

    /*
     * ST7920 初始化序列：
     * 1. 0x30：基本指令集，唤醒 LCD；
     * 2. 0x30：再次确认基本指令集；
     * 3. 0x0C：显示开，光标关，闪烁关；
     * 4. 0x01：清屏；
     * 5. 0x06：进入点设定，写入后地址自动加 1。
     */
    DisplayLcd_WriteCommand(DISPLAY_LCD_CMD_BASIC);
    DisplayLcd_WriteCommand(DISPLAY_LCD_CMD_BASIC);
    DisplayLcd_WriteCommand(DISPLAY_LCD_CMD_DISPLAY_ON);
    DisplayLcd_WriteCommand(DISPLAY_LCD_CMD_CLEAR);
    DisplayLcd_WriteCommand(DISPLAY_LCD_CMD_ENTRY_MODE);

    memset(display_lcd_line_cache, DISPLAY_LCD_SPACE, sizeof(display_lcd_line_cache));
    display_lcd_inited = 1U;
}

void DisplayLcd_Clear(void)
{
    if (display_lcd_inited == 0U)
    {
        return;
    }

    DisplayLcd_WriteCommand(DISPLAY_LCD_CMD_BASIC);
    DisplayLcd_WriteCommand(DISPLAY_LCD_CMD_CLEAR);
    memset(display_lcd_line_cache, DISPLAY_LCD_SPACE, sizeof(display_lcd_line_cache));
}

void DisplayLcd_SetTextPosition(DisplayLcdRow_e row, uint8_t col)
{
    uint8_t addr;

    if ((display_lcd_inited == 0U) ||
        (DisplayLcd_GetDdramAddress(row, col, &addr) == 0U))
    {
        return;
    }

    DisplayLcd_WriteCommand(DISPLAY_LCD_CMD_BASIC);
    DisplayLcd_WriteCommand(addr);
}

void DisplayLcd_ShowBytes(DisplayLcdRow_e row,
                          uint8_t col,
                          const uint8_t *data,
                          uint8_t len)
{
    uint8_t max_len;
    uint8_t cache_index;

    if ((display_lcd_inited == 0U) ||
        (DisplayLcd_IsValidRow(row) == 0U) ||
        (col >= DISPLAY_LCD_TEXT_COLS) ||
        (data == NULL) ||
        (len == 0U))
    {
        return;
    }

    cache_index = (uint8_t)(col * 2U);
    max_len = (uint8_t)(DISPLAY_LCD_LINE_BYTES - cache_index);
    if (len > max_len)
    {
        len = max_len;
    }

    DisplayLcd_SetTextPosition(row, col);
    for (uint8_t i = 0U; i < len; i++)
    {
        DisplayLcd_WriteData(data[i]);
        display_lcd_line_cache[row][cache_index + i] = data[i];
    }
}

void DisplayLcd_ShowString(DisplayLcdRow_e row, uint8_t col, const char *text)
{
    if (text == NULL)
    {
        return;
    }

    DisplayLcd_ShowBytes(row, col, (const uint8_t *)text, (uint8_t)strlen(text));
}

void DisplayLcd_UpdateLine(DisplayLcdRow_e row, const uint8_t *data, uint8_t len)
{
    uint8_t line_buffer[DISPLAY_LCD_LINE_BYTES];

    if ((display_lcd_inited == 0U) ||
        (DisplayLcd_IsValidRow(row) == 0U))
    {
        return;
    }

    memset(line_buffer, DISPLAY_LCD_SPACE, sizeof(line_buffer));
    if (data != NULL)
    {
        if (len > DISPLAY_LCD_LINE_BYTES)
        {
            len = DISPLAY_LCD_LINE_BYTES;
        }
        memcpy(line_buffer, data, len);
    }

    if (memcmp(display_lcd_line_cache[row], line_buffer, DISPLAY_LCD_LINE_BYTES) == 0)
    {
        return;
    }

    DisplayLcd_SetTextPosition(row, 0U);
    for (uint8_t i = 0U; i < DISPLAY_LCD_LINE_BYTES; i++)
    {
        DisplayLcd_WriteData(line_buffer[i]);
    }

    memcpy(display_lcd_line_cache[row], line_buffer, DISPLAY_LCD_LINE_BYTES);
}

void DisplayLcd_ClearGraphic(void)
{
    if (display_lcd_inited == 0U)
    {
        return;
    }

    DisplayLcd_WriteCommand(DISPLAY_LCD_CMD_EXT_G_OFF);

    for (uint8_t y = 0U; y < DISPLAY_LCD_GDRAM_HALF_ROWS; y++)
    {
        DisplayLcd_WriteCommand((uint8_t)(DISPLAY_LCD_CMD_ADDR_BASE + y));
        DisplayLcd_WriteCommand(DISPLAY_LCD_CMD_ADDR_BASE);
        for (uint8_t x = 0U; x < DISPLAY_LCD_GDRAM_WORDS; x++)
        {
            DisplayLcd_WriteData(0x00U);
            DisplayLcd_WriteData(0x00U);
        }

        DisplayLcd_WriteCommand((uint8_t)(DISPLAY_LCD_CMD_ADDR_BASE + y));
        DisplayLcd_WriteCommand((uint8_t)(DISPLAY_LCD_CMD_ADDR_BASE + 8U));
        for (uint8_t x = 0U; x < DISPLAY_LCD_GDRAM_WORDS; x++)
        {
            DisplayLcd_WriteData(0x00U);
            DisplayLcd_WriteData(0x00U);
        }
    }

    DisplayLcd_WriteCommand(DISPLAY_LCD_CMD_EXT_G_OFF);
}

void DisplayLcd_DrawImage(const uint8_t *image)
{
    uint16_t offset;

    if ((display_lcd_inited == 0U) || (image == NULL))
    {
        return;
    }

    /*
     * 写 GDRAM 前先进入扩展指令集并关闭图形显示；写完后再打开图形显示。
     * 下半屏不是 y=32~63，而是 y=0~31、x=8~15，这是 ST7920 的内部映射规则。
     */
    DisplayLcd_WriteCommand(DISPLAY_LCD_CMD_EXT_G_OFF);

    for (uint8_t y = 0U; y < DISPLAY_LCD_GDRAM_HALF_ROWS; y++)
    {
        DisplayLcd_WriteCommand((uint8_t)(DISPLAY_LCD_CMD_ADDR_BASE + y));
        DisplayLcd_WriteCommand(DISPLAY_LCD_CMD_ADDR_BASE);
        for (uint8_t x = 0U; x < DISPLAY_LCD_GDRAM_WORDS; x++)
        {
            offset = (uint16_t)y * DISPLAY_LCD_GDRAM_ROW_BYTES + (uint16_t)x * 2U;
            DisplayLcd_WriteData(image[offset]);
            DisplayLcd_WriteData(image[offset + 1U]);
        }

        DisplayLcd_WriteCommand((uint8_t)(DISPLAY_LCD_CMD_ADDR_BASE + y));
        DisplayLcd_WriteCommand((uint8_t)(DISPLAY_LCD_CMD_ADDR_BASE + 8U));
        for (uint8_t x = 0U; x < DISPLAY_LCD_GDRAM_WORDS; x++)
        {
            offset = (uint16_t)(y + 32U) * DISPLAY_LCD_GDRAM_ROW_BYTES + (uint16_t)x * 2U;
            DisplayLcd_WriteData(image[offset]);
            DisplayLcd_WriteData(image[offset + 1U]);
        }
    }

    DisplayLcd_WriteCommand(DISPLAY_LCD_CMD_EXT_G_ON);
}
