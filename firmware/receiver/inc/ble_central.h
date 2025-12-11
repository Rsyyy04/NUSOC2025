/**
 * @file ble_central.h
 * @brief BLE中心设备接口定义
 */

#ifndef BLE_CENTRAL_H
#define BLE_CENTRAL_H

#include <stdint.h>
#include <stdbool.h>

// BLE事件回调函数类型
typedef void (*ble_scan_evt_handler_t)(const uint8_t *mac_addr, const char *name, int8_t rssi);
typedef void (*ble_connect_evt_handler_t)(uint16_t conn_handle, const uint8_t *mac_addr);
typedef void (*ble_disconnect_evt_handler_t)(uint16_t conn_handle, uint8_t reason);

/**
 * @brief 初始化BLE协议栈
 */
void ble_stack_init(void);

/**
 * @brief 初始化BLE中心设备
 */
void ble_central_init(void);

/**
 * @brief 开始BLE扫描
 */
void ble_central_scan_start(void);

/**
 * @brief 停止BLE扫描
 */
void ble_central_scan_stop(void);

/**
 * @brief 连接到指定设备
 * @param mac_addr 设备MAC地址
 * @return 0: 成功, -1: 失败
 */
int ble_central_connect(const uint8_t *mac_addr);

/**
 * @brief 断开连接
 * @param conn_handle 连接句柄
 * @return 0: 成功, -1: 失败
 */
int ble_central_disconnect(uint16_t conn_handle);

/**
 * @brief 读取RSSI值
 * @param conn_handle 连接句柄
 * @return RSSI值 (dBm)
 */
int8_t ble_central_get_rssi(uint16_t conn_handle);

/**
 * @brief 发送寻找命令到标签
 * @param conn_handle 连接句柄
 * @param enable 1: 开始寻找, 0: 停止寻找
 * @return 0: 成功, -1: 失败
 */
int ble_central_send_find_command(uint16_t conn_handle, uint8_t enable);

/**
 * @brief 写入标签名称
 * @param conn_handle 连接句柄
 * @param name 标签名称
 * @return 0: 成功, -1: 失败
 */
int ble_central_write_tag_name(uint16_t conn_handle, const char *name);

/**
 * @brief 注册扫描事件回调
 */
void ble_central_register_scan_handler(ble_scan_evt_handler_t handler);

/**
 * @brief 注册连接事件回调
 */
void ble_central_register_connect_handler(ble_connect_evt_handler_t handler);

/**
 * @brief 注册断开事件回调
 */
void ble_central_register_disconnect_handler(ble_disconnect_evt_handler_t handler);

#endif // BLE_CENTRAL_H
