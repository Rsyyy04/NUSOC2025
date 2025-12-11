/**
 * @file buzzer.h
 * @brief 蜂鸣器控制接口定义
 */

#ifndef BUZZER_H
#define BUZZER_H

#include <stdint.h>

// 蜂鸣模式
typedef enum {
    BEEP_SHORT,         // 短促蜂鸣 (100ms)
    BEEP_LONG,          // 长蜂鸣 (500ms)
    BEEP_DOUBLE,        // 双声蜂鸣
    BEEP_PATTERN_ALARM  // 报警模式（连续）
} beep_pattern_t;

/**
 * @brief 初始化蜂鸣器
 */
void buzzer_init(void);

/**
 * @brief 蜂鸣一次
 * @param pattern 蜂鸣模式
 */
void buzzer_beep_pattern(beep_pattern_t pattern);

/**
 * @brief 开始持续蜂鸣（寻找模式）
 */
void buzzer_start_continuous(void);

/**
 * @brief 停止蜂鸣
 */
void buzzer_stop(void);

/**
 * @brief 设置蜂鸣频率
 * @param frequency 频率 (Hz)
 */
void buzzer_set_frequency(uint16_t frequency);

#endif // BUZZER_H

