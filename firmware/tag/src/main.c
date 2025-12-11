/**
 * @file main.c
 * @brief 智能物品防丢系统 - 标签主程序
 * @version 1.0
 * @date 2025-12-11
 * @author NUSOC2025 Team
 * 
 * 本程序实现智能物品防丢系统标签（从机）的核心功能，包括：
 * - BLE外设角色
 * - 广播管理
 * - 按键处理（单击/双击检测）
 * - 寻找响应（蜂鸣/振动）
 * - 低功耗管理
 */

#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include "nordic_common.h"
#include "nrf.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "app_timer.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "ble_peripheral.h"
#include "button.h"
#include "buzzer.h"
#include "power_management.h"

// 应用配置
#define APP_BLE_CONN_CFG_TAG    1
#define APP_BLE_OBSERVER_PRIO   3

// 设备名称
#define DEVICE_NAME             "AntiLoss_Tag"

// 全局变量
static bool m_alarm_enabled = true;
static bool m_find_mode = false;

/**
 * @brief 日志初始化
 */
static void log_init(void) {
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);
    
    NRF_LOG_DEFAULT_BACKENDS_INIT();
}

/**
 * @brief 定时器初始化
 */
static void timers_init(void) {
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
}

/**
 * @brief 单击事件处理
 */
void on_single_click(void) {
    m_alarm_enabled = true;
    
    // 短促蜂鸣确认
    buzzer_beep_pattern(BEEP_SHORT);
    
    NRF_LOG_INFO("Alarm enabled.");
}

/**
 * @brief 双击事件处理
 */
void on_double_click(void) {
    m_alarm_enabled = false;
    
    // 两声蜂鸣确认
    buzzer_beep_pattern(BEEP_DOUBLE);
    
    NRF_LOG_INFO("Alarm disabled.");
}

/**
 * @brief 寻找命令处理
 */
void on_find_command(bool enable) {
    m_find_mode = enable;
    
    if (enable) {
        // 持续蜂鸣
        buzzer_start_continuous();
        NRF_LOG_INFO("Find mode ON.");
    } else {
        // 停止蜂鸣
        buzzer_stop();
        NRF_LOG_INFO("Find mode OFF.");
    }
}

/**
 * @brief 获取报警启用状态
 */
bool is_alarm_enabled(void) {
    return m_alarm_enabled;
}

/**
 * @brief 空闲状态处理
 */
static void idle_state_handle(void) {
    if (NRF_LOG_PROCESS() == false) {
        nrf_pwr_mgmt_run();
    }
}

/**
 * @brief 主函数
 */
int main(void) {
    // 初始化日志
    log_init();
    
    NRF_LOG_INFO("=================================");
    NRF_LOG_INFO("Anti-Loss System - Tag");
    NRF_LOG_INFO("Version: 1.0.0");
    NRF_LOG_INFO("Date: 2025-12-11");
    NRF_LOG_INFO("=================================");
    
    // 初始化定时器
    timers_init();
    
    // 初始化电源管理
    power_management_init();
    
    // 初始化BLE协议栈
    ble_stack_init();
    
    // 初始化BLE外设
    ble_peripheral_init();
    
    // 初始化按键
    button_init();
    
    // 初始化蜂鸣器
    buzzer_init();
    
    // 开始BLE广播
    advertising_start();
    
    NRF_LOG_INFO("System initialized. Starting advertising...");
    
    // 进入主循环
    while (true) {
        idle_state_handle();
    }
}
