//
// Created by lenovo on 26-7-30.
//

#ifndef MACHINECMD_TEXT_H
#define MACHINECMD_TEXT_H

#include <stdint.h>

/**
 * @brief LCD 原始显示文案。
 *
 * ST7920 的内置中文字库按 GB2312 编码取字模，所以中文不要直接写 UTF-8
 * 字符串。这里统一用 data + len 记录待发送到 LCD 的原始字节。
 */
typedef struct
{
    const uint8_t *data; // GB2312 中文字节或 ASCII 字节
    uint8_t len;         // 字节长度，LCD 单行最大 16 字节
} MachineCmdText_s;

extern const MachineCmdText_s machine_cmd_text_title;
extern const MachineCmdText_s machine_cmd_text_init;
extern const MachineCmdText_s machine_cmd_text_eeprom;
extern const MachineCmdText_s machine_cmd_text_ready;
extern const MachineCmdText_s machine_cmd_text_system_ready;
extern const MachineCmdText_s machine_cmd_text_any_key_start;
extern const MachineCmdText_s machine_cmd_text_standby;
extern const MachineCmdText_s machine_cmd_text_conc;
extern const MachineCmdText_s machine_cmd_text_left;
extern const MachineCmdText_s machine_cmd_text_volume;
extern const MachineCmdText_s machine_cmd_text_prep_title;
extern const MachineCmdText_s machine_cmd_text_disp_title;
extern const MachineCmdText_s machine_cmd_text_bottle;
extern const MachineCmdText_s machine_cmd_text_saved;
extern const MachineCmdText_s machine_cmd_text_target;
extern const MachineCmdText_s machine_cmd_text_prep_volume;
extern const MachineCmdText_s machine_cmd_text_raw_med;
extern const MachineCmdText_s machine_cmd_text_concentration;
extern const MachineCmdText_s machine_cmd_text_start_next_confirm;
extern const MachineCmdText_s machine_cmd_text_remaining_prefix;
extern const MachineCmdText_s machine_cmd_text_disp_start;
extern const MachineCmdText_s machine_cmd_text_current_measure;
extern const MachineCmdText_s machine_cmd_text_input_over_left;
extern const MachineCmdText_s machine_cmd_text_number_input;
extern const MachineCmdText_s machine_cmd_text_start;
extern const MachineCmdText_s machine_cmd_text_prep_run;
extern const MachineCmdText_s machine_cmd_text_prep_measure;
extern const MachineCmdText_s machine_cmd_text_activity;
extern const MachineCmdText_s machine_cmd_text_wait_activity;
extern const MachineCmdText_s machine_cmd_text_activity_timeout;
extern const MachineCmdText_s machine_cmd_text_countdown;
extern const MachineCmdText_s machine_cmd_text_pause_hint;
extern const MachineCmdText_s machine_cmd_text_put_tank;
extern const MachineCmdText_s machine_cmd_text_start_ok;
extern const MachineCmdText_s machine_cmd_text_disp_run;
extern const MachineCmdText_s machine_cmd_text_pump2;
extern const MachineCmdText_s machine_cmd_text_progress;
extern const MachineCmdText_s machine_cmd_text_dont_move;
extern const MachineCmdText_s machine_cmd_text_paused_title;
extern const MachineCmdText_s machine_cmd_text_prep_paused;
extern const MachineCmdText_s machine_cmd_text_disp_paused;
extern const MachineCmdText_s machine_cmd_text_start_continue;
extern const MachineCmdText_s machine_cmd_text_reset_stop;
extern const MachineCmdText_s machine_cmd_text_clean_title;
extern const MachineCmdText_s machine_cmd_text_clean_pipe;
extern const MachineCmdText_s machine_cmd_text_pump1_run;
extern const MachineCmdText_s machine_cmd_text_waste_cup;
extern const MachineCmdText_s machine_cmd_text_manual;
extern const MachineCmdText_s machine_cmd_text_last;
extern const MachineCmdText_s machine_cmd_text_reset_exit_manual;
extern const MachineCmdText_s machine_cmd_text_on;
extern const MachineCmdText_s machine_cmd_text_off;
extern const MachineCmdText_s machine_cmd_text_key_wi;
extern const MachineCmdText_s machine_cmd_text_key_wo;
extern const MachineCmdText_s machine_cmd_text_in_tank;
extern const MachineCmdText_s machine_cmd_text_out_tank;
extern const MachineCmdText_s machine_cmd_text_needle_in;
extern const MachineCmdText_s machine_cmd_text_needle_out;
extern const MachineCmdText_s machine_cmd_text_draw_med;
extern const MachineCmdText_s machine_cmd_text_exhaust;
extern const MachineCmdText_s machine_cmd_text_remote;
extern const MachineCmdText_s machine_cmd_text_water_in;
extern const MachineCmdText_s machine_cmd_text_med_in;
extern const MachineCmdText_s machine_cmd_text_water_out;
extern const MachineCmdText_s machine_cmd_text_med_out;
extern const MachineCmdText_s machine_cmd_text_idle;
extern const MachineCmdText_s machine_cmd_text_alarm;
extern const MachineCmdText_s machine_cmd_text_fault;
extern const MachineCmdText_s machine_cmd_text_locked;
extern const MachineCmdText_s machine_cmd_text_rst_alarm;

#endif //MACHINECMD_TEXT_H
