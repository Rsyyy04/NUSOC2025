/****************************************************************************
 * @file    bsp_can.h
 * @brief   CAN Bus Board Support Package Header for Advanced Motor Control System
 * @author  NUSOC Robotics Team
 * @date    2025/07/13
 * @version 2.3
 *
 * @details This header file provides interface definitions for comprehensive CAN
 *          communication support including motor control structures and function
 *          prototypes for M2006, GM6020, and DM4005 motor systems.
 *
 * Copyright (C) 2025 NUSOC Robotics Team.
 * Licensed under GNU General Public License v3.0
 ***************************************************************************/

#ifndef __BSP_CAN
#define __BSP_CAN

#include "can.h"

/**
 * @brief Motor information structure for feedback and control data
 * @details Comprehensive motor state data structure containing all essential
 *          parameters for closed-loop motor control and system monitoring
 */
typedef struct
{
    uint16_t can_id;         /**< CAN message identifier for motor communication */
    int16_t set_voltage;     /**< Target voltage command for motor control (mV) */
    uint16_t rotor_angle;    /**< Current rotor position from encoder (0-8191 counts) */
    int16_t rotor_speed;     /**< Current rotational velocity (RPM) */
    int16_t torque_current;  /**< Actual torque current feedback (mA) */
    uint8_t temp;            /**< Motor temperature for thermal monitoring (°C) */
} moto_info_t;

/* Function Prototypes */

/**
 * @brief Initialize CAN communication interface with optimized filtering
 * @param hcan Pointer to CAN_HandleTypeDef structure containing CAN configuration
 * @retval None
 */
void can_user_init(CAN_HandleTypeDef *hcan);

/**
 * @brief Transmit motor control voltages via CAN2 bus communication
 * @param id_range Motor group selector: 0x01(M2006 1-4), 0x02(M2006 5-8), 0x03(GM6020 1-4)
 * @param v1 Motor #1 control voltage: ±30000 range (±24V equivalent)
 * @param v2 Motor #2 control voltage: ±30000 range (±24V equivalent)
 * @param v3 Motor #3 control voltage: ±30000 range (±24V equivalent)
 * @param v4 Motor #4 control voltage: ±30000 range (±24V equivalent)
 * @retval None
 */
void set_motor_voltage(uint8_t id_range, int16_t v1, int16_t v2, int16_t v3, int16_t v4);

/**
 * @brief Send absolute position control command to high-precision motor
 * @param pos Target absolute position: 0-65535 (16-bit resolution)
 * @retval None
 */
void set_motor_position(uint16_t pos);

/**
 * @brief Request current motor position from high-precision motor
 * @param None
 * @retval None
 */
void read_motor_position(void);

/**
 * @brief Set motor current control for high-precision motor
 * @param current Target current value: ±32767 range (motor-dependent scaling)
 * @retval None
 */
void set_motor_current(int16_t current);

/**
 * @brief Set motor speed control for DM4005 high-precision motor
 * @param speed Target speed value: ±2147483647 range (32-bit signed)
 * @retval None
 */
void set_motor_speed_dm4005(int32_t speed);

/**
 * @brief Clear motor error state and reset fault conditions
 * @param None
 * @retval None
 */
void clear_motor_mistake(void);

/**
 * @brief Request current motor error status and fault information
 * @param None
 * @retval None
 */
void read_motor_mistake(void);

/**
 * @brief Set motor encoder zero position for calibration
 * @param None
 * @retval None
 */
void set_motor_zero(void);

#endif
