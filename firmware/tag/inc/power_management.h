/**
 * @file power_management.h
 * @brief 电源管理接口定义
 */

#ifndef POWER_MANAGEMENT_H
#define POWER_MANAGEMENT_H

#include <stdint.h>

/**
 * @brief 初始化电源管理
 */
void power_management_init(void);

/**
 * @brief 获取电池电量
 * @return 电量百分比 (0-100%)
 */
uint8_t battery_level_get(void);

/**
 * @brief 进入低功耗模式
 */
void enter_low_power_mode(void);

/**
 * @brief 配置GPIO为低功耗
 */
void gpio_low_power_config(void);

#endif // POWER_MANAGEMENT_H
