/**
 * @file button.h
 * @brief 按键处理接口定义
 */

#ifndef BUTTON_H
#define BUTTON_H

#include <stdint.h>

// 按键事件类型
typedef enum {
    BUTTON_EVENT_SINGLE_CLICK,
    BUTTON_EVENT_DOUBLE_CLICK,
    BUTTON_EVENT_LONG_PRESS
} button_event_t;

// 按键事件回调函数类型
typedef void (*button_event_handler_t)(button_event_t event);

/**
 * @brief 初始化按键
 */
void button_init(void);

/**
 * @brief 注册按键事件回调
 * @param handler 回调函数
 */
void button_register_handler(button_event_handler_t handler);

/**
 * @brief 单击事件处理函数（由应用实现）
 */
extern void on_single_click(void);

/**
 * @brief 双击事件处理函数（由应用实现）
 */
extern void on_double_click(void);

#endif // BUTTON_H

