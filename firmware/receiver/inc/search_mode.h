/**
 * @file search_mode.h
 * @brief 寻找模式接口定义
 */

#ifndef SEARCH_MODE_H
#define SEARCH_MODE_H

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief 初始化寻找模式
 */
void search_mode_init(void);

/**
 * @brief 开始寻找模式
 * @param tag_id 要寻找的标签ID
 * @return 0: 成功, -1: 失败
 */
int search_mode_start(uint8_t tag_id);

/**
 * @brief 停止寻找模式
 */
void search_mode_stop(void);

/**
 * @brief 获取寻找模式状态
 * @return true: 活动中, false: 停止
 */
bool search_mode_is_active(void);

/**
 * @brief 获取当前寻找的标签ID
 * @return 标签ID, 0xFF: 无
 */
uint8_t search_mode_get_target(void);

#endif // SEARCH_MODE_H

