/**
 * @file oled_ui.h
 * @brief OLED用户界面接口定义
 */

#ifndef OLED_UI_H
#define OLED_UI_H

#include <stdint.h>
#include "alarm.h"

// 菜单页面
typedef enum {
    UI_PAGE_HOME,           // 主页（标签列表）
    UI_PAGE_TAG_LIST,       // 标签列表
    UI_PAGE_SEARCH,         // 寻找模式
    UI_PAGE_PAIRING,        // 配对管理
    UI_PAGE_LOGS,           // 断线记录
    UI_PAGE_SETTINGS        // 系统设置
} ui_page_t;

/**
 * @brief 初始化OLED界面
 */
void oled_ui_init(void);

/**
 * @brief 显示欢迎界面
 */
void oled_ui_show_welcome(void);

/**
 * @brief 更新主页显示
 */
void oled_ui_update_home(void);

/**
 * @brief 显示报警信息
 * @param type 报警类型
 * @param tag_id 标签ID
 */
void oled_ui_show_alarm(alarm_type_t type, uint8_t tag_id);

/**
 * @brief 显示寻找模式界面
 * @param tag_id 标签ID
 * @param rssi RSSI值
 */
void oled_ui_show_search(uint8_t tag_id, int8_t rssi);

/**
 * @brief 切换到指定页面
 * @param page 页面类型
 */
void oled_ui_goto_page(ui_page_t page);

/**
 * @brief 处理按键输入
 * @param key 按键码
 */
void oled_ui_handle_key(uint8_t key);

/**
 * @brief 清屏
 */
void oled_ui_clear(void);

/**
 * @brief 刷新显示
 */
void oled_ui_refresh(void);

#endif // OLED_UI_H
