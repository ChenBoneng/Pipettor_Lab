#ifndef DISPLAY_LCD_H
#define DISPLAY_LCD_H

#include <stdint.h>

/**
 * @file display_lcd.h
 * @brief ST7920 兼容 12864 LCD 串行驱动模块。
 *
 * 本模块使用 GPIO 软件 SPI 驱动 LCD，不使用 STM32 硬件 SPI，也不读取忙标志。
 *
 * 硬件连接：
 * - DISPLAY_LCD_CS   = PC0；
 * - DISPLAY_LCD_SID  = PC1；
 * - DISPLAY_LCD_SCLK = PC2；
 * - DISPLAY_LCD_RST  = PC3。
 *
 * @note 中文显示必须传入 GB2312 双字节编码。当前工程源文件通常是 UTF-8，
 *       因此不要直接写中文字符串字面量，否则 LCD 收到的是 UTF-8 字节，
 *       ST7920 内置中文字库无法正确显示。
 */

/**
 * @brief LCD 文本行号。
 *
 * ST7920 文本模式一共有 4 行，内部 DDRAM 地址顺序不是线性排列：
 * 第 1 行 0x80，第 2 行 0x90，第 3 行 0x88，第 4 行 0x98。
 */
typedef enum
{
    DISPLAY_LCD_ROW_1 = 0, /**< 第 1 行，DDRAM 起始地址 0x80。 */
    DISPLAY_LCD_ROW_2,     /**< 第 2 行，DDRAM 起始地址 0x90。 */
    DISPLAY_LCD_ROW_3,     /**< 第 3 行，DDRAM 起始地址 0x88。 */
    DISPLAY_LCD_ROW_4,     /**< 第 4 行，DDRAM 起始地址 0x98。 */
    DISPLAY_LCD_ROW_MAX    /**< 行数量边界值，仅用于参数检查。 */
} DisplayLcdRow_e;

/**
 * @brief 初始化 LCD 驱动和显示模块。
 *
 * @note PC0~PC3 的 GPIO 模式由 CubeMX 配置，本函数只设置 LCD 空闲电平，
 *       并执行一次 LCD 上电初始化和清屏。函数内部有初始化标志，重复调用
 *       不会反复复位或清屏，避免显示闪烁。
 */
void DisplayLcd_Init(void);

/**
 * @brief 清除文本显示区。
 *
 * @note 清屏指令耗时较长，不建议在主循环里周期性调用。稳定显示后，应只在
 *       内容变化时更新对应行或对应位置。
 */
void DisplayLcd_Clear(void);

/**
 * @brief 设置文本显示位置。
 *
 * @param row 行号，取值见 DisplayLcdRow_e。
 * @param col 字位列号，范围 0~7。一个字位可显示 1 个汉字或 2 个 ASCII 字符。
 */
void DisplayLcd_SetTextPosition(DisplayLcdRow_e row, uint8_t col);

/**
 * @brief 在指定位置写入原始显示字节。
 *
 * @param row 行号，取值见 DisplayLcdRow_e。
 * @param col 字位列号，范围 0~7。
 * @param data 待显示字节数据，可以是 ASCII，也可以是 GB2312 中文双字节。
 * @param len 待写入字节数，超过当前行剩余容量时会自动截断。
 *
 * @note 本函数只负责发送字节，不做 UTF-8 到 GB2312 的编码转换。
 */
void DisplayLcd_ShowBytes(DisplayLcdRow_e row,
                          uint8_t col,
                          const uint8_t *data,
                          uint8_t len);

/**
 * @brief 在指定位置写入 C 字符串。
 *
 * @param row 行号，取值见 DisplayLcdRow_e。
 * @param col 字位列号，范围 0~7。
 * @param text 待显示字符串。
 *
 * @note ASCII 字符串可以直接传入；中文字符串必须保证底层字节已经是 GB2312，
 *       不能直接传 UTF-8 中文字符串。
 */
void DisplayLcd_ShowString(DisplayLcdRow_e row, uint8_t col, const char *text);

/**
 * @brief 更新整行文本，内容未变化时不重复刷新。
 *
 * @param row 行号，取值见 DisplayLcdRow_e。
 * @param data 待显示字节数据，可以少于一行长度。
 * @param len 待显示字节数，超过 16 字节时会自动截断。
 *
 * @note 本函数会用空格补齐一整行，并通过内部缓存判断内容是否变化。
 *       适合主循环或任务中周期调用，避免整屏频繁清屏造成闪烁。
 */
void DisplayLcd_UpdateLine(DisplayLcdRow_e row, const uint8_t *data, uint8_t len);

/**
 * @brief 清除图形显存 GDRAM。
 *
 * @note 本函数只清 GDRAM，不清文本 DDRAM。清除结束后保持扩展指令集但关闭图形显示。
 */
void DisplayLcd_ClearGraphic(void);

/**
 * @brief 绘制一张 128x64 单色图像。
 *
 * @param image 图像缓存，大小必须为 1024 字节，排列方式为每行 16 字节，
 *              共 64 行，从屏幕左上角开始。
 *
 * @note ST7920 下半屏内部映射在 X=8~15，本函数已经按文档规则处理。
 */
void DisplayLcd_DrawImage(const uint8_t *image);

#endif //DISPLAY_LCD_H
