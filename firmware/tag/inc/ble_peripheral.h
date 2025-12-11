/**
 * @file ble_peripheral.h
 * @brief BLE外设接口定义
 */

#ifndef BLE_PERIPHERAL_H
#define BLE_PERIPHERAL_H

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief 初始化BLE协议栈
 */
void ble_stack_init(void);

/**
 * @brief 初始化BLE外设
 */
void ble_peripheral_init(void);

/**
 * @brief 开始广播
 */
void advertising_start(void);

/**
 * @brief 停止广播
 */
void advertising_stop(void);

/**
 * @brief 更新广播数据
 * @param battery_level 电池电量 (0-100%)
 * @param status 状态标志
 */
void advertising_update_data(uint8_t battery_level, uint8_t status);

/**
 * @brief 获取连接状态
 * @return true: 已连接, false: 未连接
 */
bool is_connected(void);

/**
 * @brief 获取连接句柄
 * @return 连接句柄
 */
uint16_t get_conn_handle(void);

#endif // BLE_PERIPHERAL_H
