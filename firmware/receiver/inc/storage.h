/**
 * @file storage.h
 * @brief Flash存储管理接口定义
 */

#ifndef STORAGE_H
#define STORAGE_H

#include <stdint.h>
#include "tag_manager.h"

#define MAX_DISCONNECT_LOGS     10

// 断线记录
typedef struct {
    uint8_t tag_id;
    uint32_t timestamp;
    int8_t last_rssi;
    uint8_t reason;
} disconnect_log_t;

/**
 * @brief 初始化存储模块
 * @return 0: 成功, -1: 失败
 */
int storage_init(void);

/**
 * @brief 保存标签信息
 * @return 0: 成功, -1: 失败
 */
int storage_save_tags(void);

/**
 * @brief 加载标签信息
 * @return 0: 成功, -1: 失败
 */
int storage_load_tags(void);

/**
 * @brief 添加断线记录
 * @param tag_id 标签ID
 * @param timestamp 时间戳
 * @param rssi 最后RSSI值
 * @return 0: 成功, -1: 失败
 */
int storage_add_disconnect_log(uint8_t tag_id, uint32_t timestamp, int8_t rssi);

/**
 * @brief 获取断线记录
 * @param logs 记录缓冲区
 * @param max_count 最大记录数
 * @return 实际记录数
 */
int storage_get_disconnect_logs(disconnect_log_t *logs, uint8_t max_count);

/**
 * @brief 清除所有断线记录
 * @return 0: 成功, -1: 失败
 */
int storage_clear_disconnect_logs(void);

/**
 * @brief 擦除所有数据
 * @return 0: 成功, -1: 失败
 */
int storage_erase_all(void);

#endif // STORAGE_H
