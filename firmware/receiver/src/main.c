/**
 * @file main.c
 * @brief 智能物品防丢系统 - 接收器主程序
 * @version 1.0
 * @date 2025-12-11
 * @author NUSOC2025 Team
 * 
 * @copyright Copyright (c) 2025 NUSOC2025 Team
 * 
 * 本程序实现智能物品防丢系统接收器（主机）的核心功能，包括：
 * - BLE中心设备角色
 * - 多标签连接管理
 * - RSSI监测与报警
 * - 寻找模式
 * - OLED用户界面
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
#include "app_scheduler.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "main.h"
#include "ble_central.h"
#include "tag_manager.h"
#include "alarm.h"
#include "search_mode.h"
#include "oled_ui.h"
#include "storage.h"

// 应用配置
#define APP_BLE_CONN_CFG_TAG        1
#define APP_BLE_OBSERVER_PRIO       3
#define SCHED_MAX_EVENT_DATA_SIZE   APP_TIMER_SCHED_EVENT_DATA_SIZE
#define SCHED_QUEUE_SIZE            10

// 全局变量
static bool m_system_initialized = false;

/**
 * @brief 断言错误处理
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t *p_file_name) {
    app_error_handler(0xDEADBEEF, line_num, p_file_name);
}

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
    
    NRF_LOG_INFO("Timers initialized.");
}

/**
 * @brief 调度器初始化
 */
static void scheduler_init(void) {
    APP_SCHED_INIT(SCHED_MAX_EVENT_DATA_SIZE, SCHED_QUEUE_SIZE);
    
    NRF_LOG_INFO("Scheduler initialized.");
}

/**
 * @brief 电源管理初始化
 */
static void power_management_init(void) {
    ret_code_t err_code;
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
    
    NRF_LOG_INFO("Power management initialized.");
}

/**
 * @brief 外设初始化
 */
static void peripherals_init(void) {
    // 初始化OLED显示屏
    oled_ui_init();
    
    // 初始化蜂鸣器和LED
    alarm_init();
    
    NRF_LOG_INFO("Peripherals initialized.");
}

/**
 * @brief 应用程序初始化
 */
static void application_init(void) {
    ret_code_t err_code;
    
    // 初始化存储
    err_code = storage_init();
    APP_ERROR_CHECK(err_code);
    
    // 加载已配对的标签
    tag_manager_init();
    storage_load_tags();
    
    // 初始化搜索模式
    search_mode_init();
    
    // 显示欢迎界面
    oled_ui_show_welcome();
    
    NRF_LOG_INFO("Application initialized. Tag count: %d", tag_manager_get_count());
}

/**
 * @brief 空闲状态处理
 */
static void idle_state_handle(void) {
    app_sched_execute();
    
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
    NRF_LOG_INFO("Anti-Loss System - Receiver");
    NRF_LOG_INFO("Version: 1.0.0");
    NRF_LOG_INFO("Date: 2025-12-11");
    NRF_LOG_INFO("=================================");
    
    // 初始化定时器
    timers_init();
    
    // 初始化调度器
    scheduler_init();
    
    // 初始化电源管理
    power_management_init();
    
    // 初始化BLE协议栈
    ble_stack_init();
    
    // 初始化BLE中心设备
    ble_central_init();
    
    // 初始化外设
    peripherals_init();
    
    // 初始化应用程序
    application_init();
    
    // 开始BLE扫描
    ble_central_scan_start();
    
    m_system_initialized = true;
    
    NRF_LOG_INFO("System initialized successfully. Entering main loop...");
    
    // 进入主循环
    while (true) {
        idle_state_handle();
    }
}

/**
 * @brief 获取系统初始化状态
 */
bool is_system_initialized(void) {
    return m_system_initialized;
}
