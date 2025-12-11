/**
 * @file tag_manager.h
 * @brief 标签管理模块接口定义
 */

#ifndef TAG_MANAGER_H
#define TAG_MANAGER_H

#include <stdint.h>
#include <stdbool.h>

#define MAX_TAGS            8
#define TAG_NAME_MAX_LEN    16

// 标签信息结构体
typedef struct {
    uint8_t         id;
    uint8_t         mac_addr[6];
    char            name[TAG_NAME_MAX_LEN];
    uint16_t        conn_handle;
    int8_t          rssi;
    uint8_t         connected;
    uint8_t         alarm_enabled;
    int8_t          rssi_threshold;
    uint32_t        last_seen_time;
    uint32_t        disconnect_time;
    uint8_t         warning_state;
    uint8_t         disconnect_alarmed;
} tag_info_t;

/**
 * @brief 初始化标签管理器
 */
void tag_manager_init(void);

/**
 * @brief 添加标签
 * @param mac_addr MAC地址
 * @param name 标签名称
 * @return 标签ID (0-7), -1: 失败
 */
int tag_manager_add(const uint8_t *mac_addr, const char *name);

/**
 * @brief 删除标签
 * @param id 标签ID
 */
void tag_manager_remove(uint8_t id);

/**
 * @brief 获取标签信息
 * @param id 标签ID
 * @return 标签信息指针, NULL: 无效ID
 */
tag_info_t* tag_manager_get(uint8_t id);

/**
 * @brief 根据MAC地址查找标签
 * @param mac_addr MAC地址
 * @return 标签信息指针, NULL: 未找到
 */
tag_info_t* tag_manager_find_by_mac(const uint8_t *mac_addr);

/**
 * @brief 根据连接句柄查找标签
 * @param conn_handle 连接句柄
 * @return 标签信息指针, NULL: 未找到
 */
tag_info_t* tag_manager_find_by_handle(uint16_t conn_handle);

/**
 * @brief 更新标签连接状态
 * @param id 标签ID
 * @param conn_handle 连接句柄
 */
void tag_manager_update_connection(uint8_t id, uint16_t conn_handle);

/**
 * @brief 更新标签断开状态
 * @param id 标签ID
 */
void tag_manager_update_disconnection(uint8_t id);

/**
 * @brief 更新标签RSSI
 * @param id 标签ID
 * @param rssi RSSI值
 */
void tag_manager_update_rssi(uint8_t id, int8_t rssi);

/**
 * @brief 获取标签数量
 * @return 标签数量
 */
uint8_t tag_manager_get_count(void);

/**
 * @brief 启用/禁用标签报警
 * @param id 标签ID
 * @param enabled 1: 启用, 0: 禁用
 */
void tag_manager_set_alarm_enabled(uint8_t id, uint8_t enabled);

/**
 * @brief 设置RSSI阈值
 * @param id 标签ID
 * @param threshold RSSI阈值
 */
void tag_manager_set_rssi_threshold(uint8_t id, int8_t threshold);

/**
 * @brief 重命名标签
 * @param id 标签ID
 * @param name 新名称
 */
void tag_manager_rename(uint8_t id, const char *name);

#endif // TAG_MANAGER_H
