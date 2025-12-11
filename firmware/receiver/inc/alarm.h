/**
 * @file alarm.h
 * @brief 报警处理模块接口定义
 */

#ifndef ALARM_H
#define ALARM_H

#include <stdint.h>

// 报警类型
typedef enum {
    ALARM_DISCONNECTED,     // 连接断开
    ALARM_WEAK_SIGNAL,      // 信号弱
    ALARM_OUT_OF_RANGE      // 超出范围
} alarm_type_t;

/**
 * @brief 初始化报警模块
 */
void alarm_init(void);

/**
 * @brief 触发报警
 * @param type 报警类型
 * @param tag_id 标签ID
 */
void trigger_alarm(alarm_type_t type, uint8_t tag_id);

/**
 * @brief 停止报警
 */
void alarm_stop(void);

/**
 * @brief 开始连接监测
 */
void alarm_start_monitoring(void);

/**
 * @brief 停止连接监测
 */
void alarm_stop_monitoring(void);

#endif // ALARM_H

