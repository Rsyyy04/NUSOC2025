/**
 * @file main.h
 * @brief 智能物品防丢系统 - 接收器主程序头文件
 */

#ifndef MAIN_H
#define MAIN_H

#include <stdint.h>
#include <stdbool.h>

// 系统版本信息
#define SYSTEM_VERSION_MAJOR    1
#define SYSTEM_VERSION_MINOR    0
#define SYSTEM_VERSION_PATCH    0

// 应用配置
#define DEVICE_NAME             "AntiLoss_RX"
#define MAX_TAG_COUNT           8
#define RSSI_THRESHOLD_DEFAULT  -75

/**
 * @brief 获取系统初始化状态
 * @return true: 已初始化, false: 未初始化
 */
bool is_system_initialized(void);

#endif // MAIN_H

