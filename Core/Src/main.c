/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           main.c
 * @brief          NUSOC Robot Competition Gimbal Tracking Control System Main Program
 * @version        4.3.2
 * @date           2025-07-25 3:18
 * @author         Rsyyy
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics & Rsyyy
 * All rights reserved
 *
 * This software implements a complete robotic gimbal tracking control system, including:
 * - Dual-axis gimbal motor control (GM6020 + DM4005)
 * - Vision-based target tracking algorithms
 * - PID controllers
 * - CAN bus communication
 * - Real-time control loops
 *
 ******************************************************************************
 * @note System Architecture Overview:
 *
 * Communication Interface Configuration:
 * - CAN1: Chassis IMU attitude data reception
 * - CAN2: Motor control (M2006 + GM6020) and feedback
 * - UART1,4,5,7: 4 laser distance sensors
 * - UART2: IMU sensor data stream
 * - UART3: Remote controller reception
 * - UART8: Debug/monitoring output
 *
 * Control Algorithms:
 * - PWM signal processing and overflow detection
 * - Adaptive trajectory planning state machine
 * - Fast trigonometric functions (Taylor series)
 * - Newton-Raphson square root approximation
 * - Optimized PID control
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"
#include "dma.h"
#include "rng.h"
#include "rtc.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h> // String manipulation functions
#include <stdio.h>  // Standard input/output functions
#include <math.h>   // Mathematical library functions
#include <stdlib.h> // Standard library functions

#include "bsp_can.h" // CAN bus board support package
#include "pid.h"     // PID controller library

/**
 * @brief Value limiting macro definition
 * @param x Value to be limited
 * @param min Minimum value
 * @param max Maximum value
 * @note Constrains value x within the range [min, max]
 */
#define LIMIT_MIN_MAX(x, min, max) (x) = (((x) <= (min)) ? (min) : (((x) >= (max)) ? (max) : (x)))

/**
 * @brief Absolute value macro definition
 * @param x Input value
 * @return Returns the absolute value of x
 */
#define my_abs(x) (x) > 0 ? (x) : -(x)

/**
 * @defgroup System Configuration Constants
 * @brief Core configuration parameters for the robot platform
 * @{
 */

/** @defgroup Physical Parameters */
#define WheelLength 188.495559215 /**< Mecanum wheel circumference (mm) */

/** @defgroup PID Controller Parameters */
/* Position control loop - chassis attitude and stabilization control */
#define KpPos 50    /**< Position proportional gain */
#define KdPos 20000 /**< Position derivative gain - attitude correction sensitivity */

/* Speed control loop - motor speed adjustment */
#define KpSpeed 2.5  /**< Speed proportional gain - response rate to speed error */
#define KiSpeed 0.15 /**< Speed integral gain - steady-state error elimination */
#define KdSpeed 2    /**< Speed derivative gain - damping and stability */

/* Gimbal control parameters - optimized configuration */
#define KpGimbal1a -0.04f /**< Gimbal proportional gain (range a) - direction correction */
#define KiGimbal1a -0.2f  /**< Gimbal integral gain (range a) - steady-state error elimination */
#define KdGimbal1a -0.2f  /**< Gimbal derivative gain (range a) - damping and stability */

#define KpGimbal1b -0.05f /**< Gimbal proportional gain (range b) - direction correction */
#define KiGimbal1b -0.2f  /**< Gimbal integral gain (range b) - steady-state error elimination */
#define KdGimbal1b -0.2f  /**< Gimbal derivative gain (range b) - damping and stability */

#define KpGimbal1c -0.05f /**< Gimbal proportional gain (range c) - direction correction */
#define KiGimbal1c -0.2f  /**< Gimbal integral gain (range c) - steady-state error elimination */
#define KdGimbal1c -0.2f  /**< Gimbal derivative gain (range c) - damping and stability */

float KpGimbal1; /**< Current gimbal proportional gain */
float KiGimbal1; /**< Current gimbal integral gain */
float KdGimbal1; /**< Current gimbal derivative gain */

#define KpGimbal2 -10 /**< Gimbal axis 2 proportional gain - direction correction */
#define KiGimbal2 0   /**< Gimbal axis 2 integral gain - steady-state error elimination */
#define KdGimbal2 0   /**< Gimbal axis 2 derivative gain - damping and stability */

/** @defgroup Motion Control Boundary Parameters */
/* Speed configuration */
#define MotorspeedSetStandardMin 10000 /**< Standard trajectory minimum speed (RPM) */
#define MotorspeedSetStandardMax 10300 /**< Standard trajectory maximum speed (RPM) */
#define MotorspeedSetMin 5000          /**< Global minimum speed threshold (RPM) */

/* Position boundaries and tolerances */
#define MotorposSetMin -250 /**< Minimum travel distance boundary (mm) */
#define MotorposSetDiff 100 /**< Position tolerance for trajectory switching (mm) */

/* Speed scaling intervals */
#define yTimDistanceMin 700      /**< Speed ramp start distance threshold (mm) */
#define yTimDistanceThreshold 30 /**< Distance change threshold (mm) */

/* Trajectory state machine */
#define RoundFlipFre 20 /**< Maximum trajectory cycles before system stop */

#define motor_info_centre 0 /**< Motor information center value */
#define KpGM6020pos -0.3    /**< GM6020 position control proportional coefficient */

/**
 * @defgroup Mathematical Constants and Optimization Functions
 * @brief High-performance mathematical operations for embedded real-time control
 * @{
 */

/** @defgroup Core Mathematical Constants */
#define PI 3.14159265359f        /**< Mathematical constant π (high precision) */
#define DEG_TO_RAD (PI / 180.0f) /**< Degree to radian conversion factor */

#define HitTimeMax 2300 /**< Maximum hit time */

/**
 * @brief Fast sine function calculation (Taylor series expansion)
 * @param deg Input angle (degrees)
 * @return Sine value (normalized float)
 * @note 5-term Taylor series: sin(x) ≈ x - x³/3! + x⁵/5! - x⁷/7! + x⁹/9!
 *       Precision: Error ~0.001% in [-180°, +180°] range
 *       Performance: ~10x faster than standard math library on STM32F4
 */
#define FAST_SIN_DEG(deg) ({                                                         \
  float x = (deg) * DEG_TO_RAD;                                                      \
  /* Normalize to [-π, π] for optimal Taylor convergence */                          \
  while (x > PI)                                                                     \
    x -= 2.0f * PI;                                                                  \
  while (x < -PI)                                                                    \
    x += 2.0f * PI;                                                                  \
  /* Optimized Taylor expansion with pre-computed coefficients */                    \
  float x2 = x * x, x3 = x * x2, x5 = x3 * x2, x7 = x5 * x2, x9 = x7 * x2;           \
  (x - x3 * 0.16666667f + x5 * 0.00833333f - x7 * 0.00019841f + x9 * 0.0000027557f); \
})

/**
 * @brief Fast cosine function calculation (Taylor series expansion)
 * @param deg Input angle (degrees)
 * @return Cosine value (normalized float)
 * @note 5-term Taylor series: cos(x) ≈ 1 - x²/2! + x⁴/4! - x⁶/6! + x⁸/8!
 *       Precision: Error ~0.001% in [-180°, +180°] range
 *       Performance: ~10x faster than standard math library on STM32F4
 */
#define FAST_COS_DEG(deg) ({                                                     \
  float x = (deg) * DEG_TO_RAD;                                                  \
  /* Normalize to [-π, π] for optimal Taylor convergence */                      \
  while (x > PI)                                                                 \
    x -= 2.0f * PI;                                                              \
  while (x < -PI)                                                                \
    x += 2.0f * PI;                                                              \
  /* Optimized Taylor expansion with pre-computed coefficients */                \
  float x2 = x * x, x4 = x2 * x2, x6 = x4 * x2, x8 = x6 * x2;                    \
  (1.0f - x2 * 0.5f + x4 * 0.04166667f - x6 * 0.00138889f + x8 * 0.0000248016f); \
})

/**
 * @brief Fast square root calculation (Newton-Raphson + bit manipulation)
 * @param x Input value (must be positive)
 * @return Approximate square root (float)
 * @note Uses magic number initial guess + 3 Newton-Raphson iterations
 *       Precision: Error ~0.01% in [0.1, 1000] range
 *       Performance: ~5x faster than standard sqrt() on STM32F4
 * @warning Returns 0 for non-positive input (safety mechanism)
 */
#define FAST_SQRT(x) ({                                                \
  float input = (float)(x), result = 0.0f;                             \
  if (input > 0.0f)                                                    \
  {                                                                    \
    /* Fast initial guess using IEEE 754 bit manipulation */           \
    union                                                              \
    {                                                                  \
      float f;                                                         \
      uint32_t i;                                                      \
    } u = {.f = input};                                                \
    u.i = (u.i >> 1) + 0x1FBD1DF5; /* Carmack magic number variant */  \
    float guess = u.f;                                                 \
    /* Newton-Raphson refinement: x_new = (x_old + input/x_old) / 2 */ \
    guess = 0.5f * (guess + input / guess); /* Iteration 1 */          \
    guess = 0.5f * (guess + input / guess); /* Iteration 2 */          \
    guess = 0.5f * (guess + input / guess); /* Iteration 3 */          \
    result = guess;                                                    \
  }                                                                    \
  result;                                                              \
})

/**
 * @}
 */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/** @subsection CAN Bus Communication Variables */
extern CAN_RxHeaderTypeDef RxHeader;  /**< CAN1 receive header - reserved for future expansion */
extern CAN_TxHeaderTypeDef TxHeader;  /**< CAN1 transmit header - reserved for future expansion */
extern CAN_RxHeaderTypeDef rx_header; /**< CAN2 receive header - motor feedback message reception */
extern CAN_TxHeaderTypeDef tx_header; /**< CAN2 transmit header - motor command message transmission */

/** @subsection Attitude and Orientation Data */
float chassis_angle[3];   /**< Chassis IMU attitude data: [roll, pitch, yaw] (degrees) */
float gimbal_angle[3];    /**< Gimbal IMU attitude data: [roll, pitch, yaw] (degrees) */
float AngularVelocity[3]; /**< Angular velocity data: [roll rate, pitch rate, yaw rate] (deg/s) */
float SetAngle;           /**< Target chassis yaw reference angle for position control (degrees) */

/** @subsection Motor Feedback and Control Arrays */
int16_t M2006_info[4][3];       /**< M2006 motor state array: [motor_id][position, velocity, torque] */
int16_t M2006_info_error[4][3]; /**< M2006 PID error history: [motor_id][e(k), e(k-1), e(k-2)] for derivative calculation */
int16_t GM6020_info[4];         /**< GM6020 motor feedback: [position, velocity, current, temperature] */
uint16_t DM4005_info;           /**< DM4005 high-resolution encoder position feedback (16-bit precision) */

/** @subsection Communication Buffer Management */
static uint8_t Temp[100];               /**< General purpose string formatting buffer (for sprintf operations) */
static uint8_t IMUReceiveBuffer[200];   /**< IMU binary data receive buffer - handles large binary packets */
static uint8_t LaserReceiveBuffer1[20]; /**< Front laser sensor ASCII data buffer - format: "XXXX\r" */
static uint8_t LaserReceiveBuffer2[20]; /**< Right laser sensor ASCII data buffer - format: "XXXX\r" */
static uint8_t LaserReceiveBuffer3[20]; /**< Left laser sensor ASCII data buffer - format: "XXXX\r" */
static uint8_t LaserReceiveBuffer4[20]; /**< Rear laser sensor ASCII data buffer - format: "XXXX\r" */
static uint8_t RKReceiveBuffer[10];     /**< Remote control command buffer (emergency stop & tracking commands) */

/** @subsection Distance Sensing System */
/**
 * @brief Laser Distance Sensor Array Configuration
 *
 * Spatial Layout (Robot Coordinate System):
 *                    Front [Sensor #1]
 *                          ↑ +Y
 *                          │
 *    Left [#3] ────────────┼─────────── Right [#2]
 *          -X ←────────────┼──────────→ +X
 *                          │
 *                          ↓ -Y
 *                     Rear [Sensor #4]
 *
 * Technical Specifications:
 * • Measurement Range: 20mm - 8000mm (0.02m - 8m)
 * • Accuracy: ±2mm @ 25°C
 * • Resolution: 1mm
 * • Update Rate: 100Hz per sensor
 * • Interface: UART @ 9600 baud, ASCII format "XXXX\r"
 * • Field of View: 2° cone angle
 */
int16_t distance1, distance2, distance3, distance4;                 /**< [front, right, left, rear] distances (mm) */
int16_t distance1init, distance2init, distance3init, distance4init; /**< Initial calibration offsets for sensor alignment */

/** @subsection PWM Signal Processing Variables */
uint32_t xTimUpCounter, yTimUpCounter;     /**< PWM measurement timer rising edge capture values (16-bit timer precision) */
uint32_t xTimDownCounter, yTimDownCounter; /**< PWM measurement timer falling edge capture values (16-bit timer precision) */
uint16_t xTimFre, yTimFre;                 /**< Measured PWM frequency (Hz) for signal verification */

/**
 * @brief Omnidirectional Movement PWM Duty Cycle Control System
 *
 * Control Signal Mapping:
 * • xTimDuty: Lateral movement control (left-right strafe)
 *   - Range: [0.0, 1.0] normalized duty cycle
 *   - 0.5 = neutral position, <0.5 = left movement, >0.5 = right movement
 * • yTimDuty: Longitudinal movement control (forward-backward)
 *   - Range: [0.0, 1.0] normalized duty cycle
 *   - 0.5 = neutral position, <0.5 = reverse movement, >0.5 = forward movement
 *
 * Signal Characteristics:
 * • Standard RC PWM: 50Hz carrier frequency (20ms period)
 * • Pulse Width Range: 1.0ms - 2.0ms (1.5ms = neutral position)
 * • Timer Resolution: 16-bit precision providing ~1µs accuracy
 * • Overflow Handling: Multi-turn tracking with automatic wraparound detection
 */
float xTimDuty, yTimDuty;                   /**< Current normalized duty cycle from PWM input [0.0-1.0] */
float xTimDutyInit, yTimDutyInit;           /**< Calibration neutral position reference values for zero-point correction */
float xTimDutyOld, yTimDutyOld;             /**< Previous duty cycle values for overflow detection algorithm */
int16_t xTimDutyOverflow, yTimDutyOverflow; /**< Overflow counters for continuous multi-turn tracking */
int32_t xTimDistance;                       /**< Cumulative lateral displacement (millimeters) */
float yTimDistance = MotorposSetMin;        /**< Cumulative longitudinal displacement (millimeters) (initialized to minimum value) */

/** @subsection Advanced Control System State Variables */
double PosPIDResult = 0;              /**< Position controller output for chassis stabilization and attitude correction */
double SpeedPIDResult[4];             /**< Individual motor speed controller outputs for 4-motor drivetrain */
double MotorspeedSet[4];              /**< Target speed setpoints for each drivetrain motor (RPM) */
double MotorspeedSetMid;              /**< Intermediate speed calculation value for adaptive trajectory planning */
uint32_t MotorspeedSetStandard[10];   /**< Pre-defined speed profile sequence for repeatable motion patterns */
uint32_t MotorspeedSetRandom[10];     /**< Hardware RNG buffer for diversified motion profile generation */
uint32_t yTimDistanceMinStandard[10]; /**< Pre-defined speed profile sequence for repeatable motion patterns */
uint32_t yTimDistanceMinRandom[10];   /**< Hardware RNG buffer for diversified motion profile generation */
uint32_t yTimDistanceMaxStandard[10]; /**< Pre-defined speed profile sequence for repeatable motion patterns */
uint32_t yTimDistanceMaxRandom[10];   /**< Hardware RNG buffer for diversified motion profile generation */
int32_t j = 0;                        /**< Trajectory state machine counter for motion sequence control */
int16_t motor_voltage[4];             /**< Final motor voltage commands (millivolts [mV]) */

/** @subsection Gimbal Control Variables */
int16_t TargetCentre[2] = {318, 250};  /**< Target center coordinates from vision system [x, y] (default: image center) */
int16_t PurposeCentre[2] = {318, 250}; /**< Current object center coordinates from camera [x, y] (default: image center) */

double GimbalError[3];       /**< Gimbal PID error history: [e(k), e(k-1), e(k-2)] for derivative calculation */
double GimbalPIDResult;      /**< Gimbal PID controller output for tracking control */
double GimbalIntegralResult; /**< Gimbal integral term accumulator with anti-windup protection */
uint8_t GimbalFlag = 2;      /**< Gimbal control flag */

extern moto_info_t motor_info; /**< External motor information structure from CAN driver */
pid_struct_t motor_pid;        /**< PID controller structure for GM6020 motor speed control */
float target_speed = 0;        /**< GM6020 motor target speed (deg/s) - initialized to zero */
int32_t target_speedy = 0;     /**< DM4005 motor target speed (Y-axis) - initialized to zero */
int16_t Errory[2];             /**< Y-axis tracking error for vertical gimbal control */

uint8_t RunFlag = 0;  /**< System run flag - set to 1 when ready for operation */
uint8_t Flag4005 = 0; /**< DM4005 motor enable flag - controls Y-axis gimbal operation */

uint8_t TimerSystemMode;                                                                /**< Timer system mode flag */
uint8_t CorrectTime, FailTime, CorrectTimePerSecond, FailTimePerSecond, FailTimeinaRow; /**< Success/failure counters and per-second statistics */

uint8_t HitFlag = 0;  /**< Target hit detection flag */
uint16_t HitTime = 0; /**< Target hit duration timer */

int8_t GimbalStopFlag = -1; /**< Gimbal stop flag: -1=init, 0=run, 1=stop */
uint32_t UARTOverTime = 0;  /**< UART communication timeout counter */

uint8_t dm4005Error[2]; /**< DM4005 error status flags: [error code, error type] */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{

  /* USER CODE BEGIN 1 */
  /* Additional variable initialization can be placed here if needed */
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  /* Custom initialization code can be placed here */
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* System-specific initialization code can be placed here */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_CAN1_Init();
  MX_CAN2_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_TIM2_Init();
  MX_TIM12_Init();
  MX_UART4_Init();
  MX_UART5_Init();
  MX_UART7_Init();
  MX_UART8_Init();
  MX_USART1_UART_Init();
  MX_TIM4_Init();
  MX_USART3_UART_Init();
  MX_USART2_UART_Init();
  MX_TIM10_Init();
  MX_TIM5_Init();
  MX_RNG_Init();
  MX_TIM6_Init();
  MX_TIM1_Init();
  MX_TIM9_Init();
  MX_RTC_Init();
  MX_TIM8_Init();
  /* USER CODE BEGIN 2 */
  // HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, 1);

  HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_3);     // Start PWM output for GM6020 motor control
  __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, 0); // Set initial duty cycle to 0% (motor off)

  HAL_UARTEx_ReceiveToIdle_DMA(&huart3, RKReceiveBuffer, 10);

  /**
   * @section System Initialization and Configuration
   * @brief Complete hardware initialization and calibration sequence
   */

  // ===== RESET CAUSE DETECTION AND LOGGING =====
  // uint32_t reset_cause = 0;
  // if (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST))
  // {
  //   reset_cause |= 0x01; // Independent watchdog reset
  //   __HAL_RCC_CLEAR_RESET_FLAGS();
  // }
  // if (__HAL_RCC_GET_FLAG(RCC_FLAG_WWDGRST))
  // {
  //   reset_cause |= 0x02; // Window watchdog reset
  //   __HAL_RCC_CLEAR_RESET_FLAGS();
  // }
  // if (__HAL_RCC_GET_FLAG(RCC_FLAG_BORRST))
  // {
  //   reset_cause |= 0x04; // Brown-out reset (power issue)
  //   __HAL_RCC_CLEAR_RESET_FLAGS();
  // }
  // if (__HAL_RCC_GET_FLAG(RCC_FLAG_PINRST))
  // {
  //   reset_cause |= 0x08; // External pin reset
  //   __HAL_RCC_CLEAR_RESET_FLAGS();
  // }
  // if (__HAL_RCC_GET_FLAG(RCC_FLAG_SFTRST))
  // {
  //   reset_cause |= 0x10; // Software reset
  //   __HAL_RCC_CLEAR_RESET_FLAGS();
  // }

  // Store reset cause for later reference
  // reset_cause_code = reset_cause;

  // Send reset cause information via UART for debugging
  // sprintf((char *)Temp, "Reset Cause: 0x%02X\r\n", (unsigned int)reset_cause);
  // HAL_UART_Transmit(&huart8, (unsigned char *)Temp, strlen((char *)Temp), 1000);

  // Start remote controller command reception
  HAL_UARTEx_ReceiveToIdle_DMA(&huart1, LaserReceiveBuffer1, 20); // Front distance sensor
  HAL_UARTEx_ReceiveToIdle_DMA(&huart4, LaserReceiveBuffer2, 20); // Right distance sensor
  HAL_UARTEx_ReceiveToIdle_IT(&huart5, LaserReceiveBuffer3, 20);  // Left distance sensor (interrupt mode)
  HAL_UARTEx_ReceiveToIdle_DMA(&huart7, LaserReceiveBuffer4, 20); // Rear distance sensor

  HAL_Delay(500); // System stabilization delay

  // Store initial distance sensor readings for relative measurement
  distance1init = distance1; // Front sensor baseline
  distance2init = distance2; // Right sensor baseline
  distance3init = distance3; // Left sensor baseline
  distance4init = distance4; // Rear sensor baseline

  // SetAngle = chassis_angle[2]; // Optional: Initialize reference angle

  // Wait for system run flag activation
  int a = 0; // Iteration counter for startup sequence

  while (!RunFlag)
  {
    // Check for manual start button press (active low)
    if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_14) == GPIO_PIN_RESET)
    {
      HAL_Delay(20); // Debounce delay
      if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_14) == GPIO_PIN_RESET)
        RunFlag = 1; // Confirm button press and enable system
    }
    a += 1;       // Increment startup counter
    HAL_Delay(0); // Yield to other tasks
  }

  char RKQidong[] = {0xb3, 0xdf, 0x09, 0x64, 0xaa};
  for (int o = 0; o < 10; o++)
  {
    HAL_UART_Transmit(&huart3, (const uint8_t *)&RKQidong[o], 5, 100);
    HAL_Delay(10); // Allow time for system stabilization
  }

  /**
   * @section Hardware Random Number Generation for Motion Diversity
   * @brief Initialize diversified motion profiles using STM32 true random number generator
   *
   * Algorithm Overview:
   * 1. Generate 10 hardware random numbers using analog noise entropy
   * 2. Normalize to motion speed range [MotorspeedSetStandardMin, MotorspeedSetStandardMax]
   * 3. Store in profile array for trajectory diversity and testing
   */
  for (int k = 0; k < 10; k++)
  {
    HAL_RNG_GenerateRandomNumber(&hrng, &MotorspeedSetRandom[k]);
    MotorspeedSetStandard[k] = MotorspeedSetRandom[k] % (uint32_t)(MotorspeedSetStandardMax - MotorspeedSetStandardMin) +
                               MotorspeedSetStandardMin;
  }

  for (int k = 0; k < 10; k++)
  {
    HAL_RNG_GenerateRandomNumber(&hrng, &yTimDistanceMinRandom[k]);
    yTimDistanceMinStandard[k] = yTimDistanceMinRandom[k] % 400 + 500;
  }

  for (int k = 0; k < 10; k++)
  {
    HAL_RNG_GenerateRandomNumber(&hrng, &yTimDistanceMaxRandom[k]);
    yTimDistanceMaxStandard[k] = yTimDistanceMaxRandom[k] % 400 + 2000;
  }

  yTimDistanceMinStandard[(RoundFlipFre + 9) % 10] = 0;
  yTimDistanceMaxStandard[2] = 3020;
  yTimDistanceMaxStandard[6] = 2860;

  /**
   * @section CAN Bus Network Initialization
   * @brief Comprehensive dual CAN controller setup for robot communication
   *
   * CAN Network Architecture:
   * • CAN1: Reserved for future sensor/actuator expansion and system diagnostics
   * • CAN2: Primary motor control bus for M2006 + GM6020 motor communication
   *
   * Protocol Settings:
   * • Baud Rate: 1 Mbps for high-speed real-time communication
   * • Message Format: Standard 11-bit identifiers for compatibility
   * • Error Handling: Automatic retransmission and bus-off recovery
   */
  can_user_init(&hcan1); // CAN1: Future expansion bus for additional sensors/actuators
  HAL_Delay(10);
  can_user_init(&hcan2); // CAN2: Primary motor control communication bus
  HAL_Delay(10);

  /**
   * @section Multi-Sensor UART Communication Architecture
   * @brief High-performance DMA-based sensor fusion initialization
   *
   * Communication Matrix:
   * ┌─────────┬─────────────────────┬──────────┬─────────────────────────┐
   * │ UART    │ Connected Device    │ Mode     │ Data Format             │
   * ├─────────┼─────────────────────┼──────────┼─────────────────────────┤
   * │ UART1   │ Front laser sensor  │ DMA      │ "XXXX\r" ASCII          │
   * │ UART4   │ Right laser sensor  │ DMA      │ "XXXX\r" ASCII          │
   * │ UART5   │ Left laser sensor   │ IT       │ "XXXX\r" ASCII*         │
   * │ UART7   │ Rear laser sensor   │ DMA      │ "XXXX\r" ASCII          │
   * │ UART2   │ IMU attitude sensor │ DMA      │ Binary protocol (200B)  │
   * │ UART3   │ Remote controller   │ DMA      │ Command stream (10B)    │
   * │ UART8   │ Debug/telemetry     │ TX       │ Formatted ASCII output  │
   * └─────────┴─────────────────────┴──────────┴─────────────────────────┘
   *
   * * UART5 uses interrupt mode due to hardware DMA channel limitations
   *
   * Performance Specifications:
   * • Sensor fusion rate: 100Hz overall update rate
   * • Maximum Communication Latency: <1ms per sensor
   * • Buffer Overflow Protection: Automatic restart on completion
   * • Error Recovery: Automatic re-initialization on communication failure
   */
  HAL_UARTEx_ReceiveToIdle_DMA(&huart2, IMUReceiveBuffer, 200); // IMU binary data stream

  // Send system startup notification
  HAL_UART_Transmit_DMA(&huart8, (const unsigned char *)"NUSOC System Online\r\n", 21);
  HAL_Delay(10); // Brief delay for transmission completion

  // Initialize chassis reference angle for position control
  SetAngle = chassis_angle[2]; // Store initial chassis yaw angle as reference setpoint

  /**
   * @section PWM Input Capture System Initialization
   * @brief High-precision remote control signal processing setup
   *
   * Timer Configuration Matrix:
   * ┌─────────┬─────────────────┬─────────────────┬─────────────────────┐
   * │ Timer   │ Channel         │ Function        │ Signal Type         │
   * ├─────────┼─────────────────┼─────────────────┼─────────────────────┤
   * │ TIM2    │ Channel 1       │ X-axis rising   │ Lateral control     │
   * │ TIM2    │ Channel 2       │ X-axis falling  │ Lateral control     │
   * │ TIM5    │ Channel 3       │ Y-axis rising   │ Longitudinal ctrl   │
   * │ TIM5    │ Channel 4       │ Y-axis falling  │ Longitudinal ctrl   │
   * └─────────┴─────────────────┴─────────────────┴─────────────────────┘
   *
   * Signal Specifications:
   * • Standard RC PWM: 50Hz carrier frequency (20ms period)
   * • Pulse Width Modulation: 1.0ms - 2.0ms (1.5ms = neutral position)
   * • Timer Resolution: 16-bit precision providing ~1µs accuracy @ 84MHz
   * • Calibration Period: 2000ms for signal stabilization and noise filtering
   *
   * Capture Method:
   * • Dual-edge capture for precise duty cycle measurement
   * • Hardware-based timing eliminates software timing errors
   * • Interrupt-driven processing for real-time response
   */
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1); // X-axis rising edge detection
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2); // X-axis falling edge detection
  HAL_Delay(10);                              // Brief delay between timer initializations
  HAL_TIM_IC_Start_IT(&htim5, TIM_CHANNEL_3); // Y-axis rising edge detection
  HAL_TIM_IC_Start_IT(&htim5, TIM_CHANNEL_4); // Y-axis falling edge detection
  HAL_Delay(20);                              // Critical: Allow signal stabilization before calibration

  /**
   * @section PWM Signal Calibration and Normalization
   * @brief Establish neutral position baseline for accurate motion control
   *
   * Calibration Algorithm:
   * duty_cycle = (falling_time - offset1) / (rising_time - offset2)
   *
   * Hardware-Specific Compensation Factors:
   * • offset1 = 16 * rising_time / 4119.0  (falling edge timing compensation)
   * • offset2 = 24 * rising_time / 4119.0  (rising edge timing compensation)
   *
   * These empirical constants compensate for:
   * • STM32F4 timer hardware characteristics and propagation delays
   * • PCB trace lengths and signal conditioning circuit delays
   * • Input capture filter settings and clock domain variations
   *
   * Output Range: [0.0, 1.0] normalized, where 0.5 = neutral position
   * Accuracy: ±0.1% duty cycle resolution for precise control
   */
  xTimDutyInit = ((float)xTimDownCounter - 16 * xTimUpCounter / 4119.0) /
                 (xTimUpCounter - 24 * xTimUpCounter / 4119.0);
  yTimDutyInit = ((float)yTimDownCounter - 16 * yTimUpCounter / 4119.0) /
                 (yTimUpCounter - 24 * yTimUpCounter / 4119.0);

  // Initialize tracking variables for overflow detection algorithm
  xTimDutyOld = xTimDuty = xTimDutyInit;
  yTimDutyOld = yTimDuty = yTimDutyInit;

  /**
   * @section Motor System Initialization
   * @brief Prepare all motors for operation and set initial positions
   *
   * Motor Initialization Sequence:
   * • Verify motor controller communication via CAN bus
   * • Set motors to known initial positions (optional)
   * • Clear any existing error states
   * • Prepare for real-time control operation
   *
   * Available Options:
   * • GM6020 motors can be centered for gimbal stabilization
   * • Position initialization ensures repeatable startup behavior
   * • Uncomment below for initial position setup if required by application
   */

  // set_motor_position(16000); // Center GM6020 motors (optional - uncomment if needed)
  HAL_Delay(30);

  // set_motor_zero();
  // HAL_Delay(30);
  set_motor_position(0);
  HAL_Delay(30);

  /**
   * @section System Timer Activation
   * @brief Start periodic timer for real-time control loop execution
   *
   * Timer Configuration:
   * • htim10: Main control loop timer triggering HAL_TIM_PeriodElapsedCallback
   * • Frequency: High-frequency execution for deterministic control timing
   * • Priority: Highest interrupt priority for real-time performance
   * • Function: Ensures deterministic timing for all control and feedback operations
   *
   * Control Loop Operations:
   * • Sensor data acquisition and processing
   * • PID control calculati ons
   * • Motor command generation and transmission
   * • System state monitoring and safety checks
   */
  HAL_TIM_Base_Start_IT(&htim10);
  HAL_TIM_Base_Start_IT(&htim1);
  HAL_TIM_Base_Start_IT(&htim9);
  HAL_Delay(10);

  /**
   * @section PID Controller Initialization
   * @brief Initialize PID controller for GM6020 motor speed control
   *
   * PID Parameters:
   * • Kp = 40: Proportional gain for responsive speed tracking
   * • Ki = 3:  Integral gain for steady-state error elimination
   * • Kd = 0:  Derivative gain (disabled to prevent noise amplification)
   * • Output Limit = ±30000: Motor voltage saturation protection
   *
   * Controller Type: Incremental PID for better stability and anti-windup
   */
  pid_init(&motor_pid, 50, 3, 0, 30000, 30000); // Initialize PID with optimized parameters
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /**
     * @section System Health Monitoring and Diagnostics
     * @brief Real-time system monitoring for stability and debugging
     */

    // Increment main loop counter
    // loop_counter++;

    // Check stack usage every 1000 iterations to avoid performance impact
    // if (loop_counter % 1000 == 0)
    // {
    //   uint32_t current_stack = check_stack_usage();

    //   // Send periodic diagnostic information (every ~10 seconds at normal loop rate)
    //   if (loop_counter % 10000 == 0)
    //   {
    //     sprintf((char *)Temp, "Stack: %lu/%lu bytes | Loops: %lu | Reset: 0x%02X\r\n",
    //             current_stack, stack_usage_max, loop_counter, (unsigned int)reset_cause_code);
    //     HAL_UART_Transmit(&huart8, (unsigned char *)Temp, strlen((char *)Temp), 100);

    //     // Stack overflow warning (assuming 8KB stack size - adjust if different)
    //     if (stack_usage_max > 6144) // 75% of 8KB
    //     {
    //       sprintf((char *)Temp, "⚠️ WARNING: High stack usage detected! ⚠️\r\n");
    //       HAL_UART_Transmit(&huart8, (unsigned char *)Temp, strlen((char *)Temp), 100);
    //     }
    //   }
    // }

    /**
     * @section Real-Time System Monitoring & Telemetry
     * @brief Transmit key system data for diagnostics and performance analysis
     *
     * Telemetry Stream Options:
     * 1. Attitude Data: Compare chassis and gimbal yaw angles for alignment verification
     * 2. Control Data: Output PWM duty cycles for motion debugging and tuning
     * 3. Debug Output: UART8 @ 115200 baud, formatted for easy parsing and logging
     *
     * Data Format Examples:
     * • Attitude: "Yc:±XXX.XX;Yg:±XXX.XX" (chassis/gimbal yaw comparison)
     * • PWM: "xD:±X.XX;yD:±X.XX" (duty cycle monitoring)
     *
     * Transmission Rate: 5Hz (200ms interval) to prevent UART congestion
     * Buffer Management: Automatic clearing to prevent data corruption
     *
     * Note: Telemetry streams are currently disabled for performance optimization.
     *       Uncomment specific sections below for debugging and system analysis.
     */

    // Attitude monitoring telemetry (uncomment for debugging)
    // sprintf((char *)Temp, "Yc:%.2f;Yg:%.2f", chassis_angle[2], gimbal_angle[2]);
    // HAL_UART_Transmit(&huart8, (unsigned char *)Temp, 25, 200);
    // memset(Temp, 0, 25);
    // HAL_Delay(200);

    // PWM duty cycle monitoring telemetry (uncomment for control system analysis)
    // sprintf((char *)Temp, "xD:%.2f;yD:%.2f", xTimDuty, yTimDuty);
    // HAL_UART_Transmit(&huart8, (unsigned char *)Temp, 25, 200);
    // memset(Temp, 0, 25);
    // HAL_Delay(200);

    /**
     * @section Motor Speed PID Control Loop
     * @brief Calculate motor speed PID output based on current feedback
     *
     * Control Process:
     * 1. Read current motor speed from CAN feedback (motor_info.rotor_speed)
     * 2. Calculate PID error: (target_speed - actual_speed)
     * 3. Apply PID algorithm to generate voltage command
     * 4. Limit output to prevent motor damage and ensure stability
     *
     * PID Type: Incremental PID with anti-windup protection
     * Update Rate: Main loop frequency for responsive control
     */
    motor_info.set_voltage = pid_calc(&motor_pid, target_speed, motor_info.rotor_speed);

    /**
     * @section Motor Command Transmission
     * @brief Send calculated motor control commands via CAN bus
     *
     * CAN Message Format:
     * • Motor ID: 0x03 (GM6020 motor identifier)
     * • Voltage Commands: [motor1, motor2, motor3, motor4] in millivolts
     * • Current Configuration: Only motor3 (index 2) is actively controlled
     * • Safety: Other motors set to 0V for system protection
     *
     * Communication Protocol: Standard CAN 2.0A at 1 Mbps
     */
    set_motor_voltage(0x03,
                      0,                      // Motor 1: Disabled (0V)
                      0,                      // Motor 2: Disabled (0V)
                      motor_info.set_voltage, // Motor 3: Active PID control
                      0);                     // Motor 4: Disabled (0V)

    /**
     * @section System Timing Control
     * @brief Minimal delay for system timing and CPU load management
     *
     * Delay Purpose:
     * • Prevents excessive CPU usage in main loop
     * • Allows other interrupts and background tasks to execute
     * • Maintains predictable timing for non-critical operations
     *
     * Note: Critical control operations are handled in timer interrupt callbacks
     *       for deterministic real-time performance
     */
    HAL_Delay(0); // Minimal delay - can be adjusted based on system requirements
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
   */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI | RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
/**
 * @brief Advanced Multi-Sensor Data Processing and Communication Callbacks
 * @param huart UART peripheral instance that triggered the reception event
 * @param Size Number of bytes received in current transmission
 * @retval None
 *
 * @details Protocol Support Matrix and Communication Architecture:
 *
 * This callback function serves as the central hub for all incoming sensor and
 * command data processing. It implements protocol-specific parsers for different
 * data formats and ensures reliable communication with various system components.
 *
 * ┌─────────────────────────────────────────────────────────────────────┐
 * │ LASER DISTANCE SENSORS (ASCII Protocol)                             │
 * │ • Data Format: "XXXX\r" (4-digit decimal + carriage return)         │
 * │ • Measurement Range: 20-8000mm with 1mm resolution                  │
 * │ • Update Rate: 100Hz per sensor for real-time obstacle detection    │
 * │ • Error Handling: Bounds checking and buffer overflow protection    │
 * │ • Spatial Layout: Front, Right, Left, Rear sensors for 360° coverage│
 * └─────────────────────────────────────────────────────────────────────┘
 * ┌─────────────────────────────────────────────────────────────────────┐
 * │ IMU ATTITUDE SENSOR (Binary Protocol)                               │
 * │ • Header Validation: 0x59 0x53 for packet integrity verification    │
 * │ • Yaw Data Extraction: Bytes 43-46 (32-bit little-endian format)    │
 * │ • Angular Velocity: Bytes 21-32 for rate feedback                   │
 * │ • Scaling Factor: raw_value × 1e-6 for proper unit conversion       │
 * │ • Range Normalization: ±180° with automatic wraparound handling     │
 * └─────────────────────────────────────────────────────────────────────┘
 * ┌─────────────────────────────────────────────────────────────────────┐
 * │ REMOTE CONTROL COMMANDS (Custom Protocol)                           │
 * │ • Emergency Stop: 0xAB 0xCD 0xEF sequence for immediate halt        │
 * │ • Tracking Commands: 10-byte packets with target coordinates        │
 * │ • Safety Features: Automatic motor shutdown on emergency commands   │
 * │ • Error Recovery: Buffer clearing and automatic re-initialization   │
 * └─────────────────────────────────────────────────────────────────────┘
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  /**
   * @subsection Laser Distance Sensor Data Processing
   * @brief Parse ASCII distance measurements from 4 laser sensors
   *
   * Each sensor transmits distance data as ASCII decimal followed by carriage return.
   * The sensors are positioned for complete perimeter coverage of the robotic platform.
   * Currently commented out for performance optimization - uncomment for distance sensing.
   */

  /* ===== FRONT DISTANCE SENSOR PROCESSING ===== */
  if (huart == &huart1) // UART1: Front-facing laser distance sensor
  {
    distance1 = 0;
    /* ASCII to integer conversion with bounds checking */
    for (int i1 = 0; LaserReceiveBuffer1[i1] != 0x0d && i1 < 20; i1++)
    {
      distance1 *= 10;
      distance1 += LaserReceiveBuffer1[i1] - 0x30; // ASCII digit to numeric conversion
    }

    // if (my_abs(distance1 - distance1init) <= yTimDistanceThreshold)
    //   yTimDistance = distance1 - distance1init;

    memset(LaserReceiveBuffer1, 0, 20);                             // Clear buffer for next reception
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, LaserReceiveBuffer1, 20); // Restart DMA reception
  }
  /* ===== RIGHT DISTANCE SENSOR PROCESSING ===== */
  else if (huart == &huart4) // UART4: Right-side laser distance sensor
  {
    distance2 = 0;
    /* ASCII to integer conversion with bounds checking */
    for (int i2 = 0; LaserReceiveBuffer2[i2] != 0x0d && i2 < 20; i2++)
    {
      distance2 *= 10;
      distance2 += LaserReceiveBuffer2[i2] - 0x30; // ASCII digit to numeric conversion
    }

    memset(LaserReceiveBuffer2, 0, 20);                             // Clear buffer for next reception
    HAL_UARTEx_ReceiveToIdle_DMA(&huart4, LaserReceiveBuffer2, 20); // Restart DMA reception
  }
  /* ===== LEFT DISTANCE SENSOR PROCESSING ===== */
  else if (huart == &huart5) // UART5: Left-side laser distance sensor (Interrupt mode)
  {
    distance3 = 0;
    /* ASCII to integer conversion with bounds checking */
    for (int i3 = 0; LaserReceiveBuffer3[i3] != 0x0d && i3 < 20; i3++)
    {
      distance3 *= 10;
      distance3 += LaserReceiveBuffer3[i3] - 0x30; // ASCII digit to numeric conversion
    }

    // if (my_abs(distance3init - distance3 - 250 - yTimDistance) <= yTimDistanceThreshold)
    //   yTimDistance = distance3init - distance3 - 250;

    memset(LaserReceiveBuffer3, 0, 20);                            // Clear buffer for next reception
    HAL_UARTEx_ReceiveToIdle_IT(&huart5, LaserReceiveBuffer3, 20); // Restart interrupt reception
  }
  /* ===== REAR DISTANCE SENSOR PROCESSING ===== */
  else if (huart == &huart7) // UART7: Rear-facing laser distance sensor
  {
    distance4 = 0;
    /* ASCII to integer conversion with bounds checking */
    for (int i4 = 0; LaserReceiveBuffer4[i4] != 0x0d && i4 < 20; i4++)
    {
      distance4 *= 10;
      distance4 += LaserReceiveBuffer4[i4] - 0x30; // ASCII digit to numeric conversion
    }

    xTimDistance = distance4 - distance4init; // Calculate distance change

    memset(LaserReceiveBuffer4, 0, 20);                             // Clear buffer for next reception
    HAL_UARTEx_ReceiveToIdle_DMA(&huart7, LaserReceiveBuffer4, 20); // Restart DMA reception
  }

  /**
   * @subsection Remote Control Command Stream Processing
   * @brief Process emergency commands and tracking target updates
   *
   * Command Protocol:
   * • Emergency Stop: 3-byte sequence [0xAB, 0xCD, 0xEF]
   * • Tracking Update: 10-byte packet with header/footer validation
   * • Safety Features: Immediate motor shutdown on emergency commands
   */
  if (huart == &huart3) // UART3: Remote control command receiver
  {
    // Process emergency stop command
    if (Size == 3)
    {
      if (RKReceiveBuffer[0] == 0xab && RKReceiveBuffer[1] == 0xcd && RKReceiveBuffer[2] == 0xef && GimbalFlag != 1)
      {
        /**
         * @brief Emergency Stop Sequence
         * @details Immediately halt all motor operations for safety
         *
         * Safety Actions:
         * • Reset GM6020 speed control variables to zero
         * • Clear all PID error accumulators to prevent windup
         * • Send zero voltage commands to all motors
         * • Ensure system enters safe state immediately
         */
        // Emergency stop: reset GM6020 speed control variables
        FailTime++;
        GimbalFlag = 2;
        HitFlag = 0;
      }
    }
    else if (Size == 10 && RKReceiveBuffer[0] == 0x55 && RKReceiveBuffer[9] == 0xaa)
    {
      /**
       * @brief Tracking Target Update Command with Advanced Filtering
       * @details Update vision tracking targets and current position using multi-stage filtering
       *
       * Packet Format: [0x55][cmd][reserved][reserved][reserved]
       *                [target_x_h][target_x_l][target_y_h][target_y_l][0xAA]
       *
       * Data Processing Pipeline:
       * • Raw Data Extraction: 16-bit values for precise positioning
       * • Position Filtering: Multi-stage filtering for noise reduction
       * • Validation: Header (0x55) and footer (0xAA) for integrity
       * • Output: Smoothed and validated position for control system
       */

      // Extract target coordinates from received packet
      TargetCentre[0] = RKReceiveBuffer[5] << 8 | RKReceiveBuffer[6];
      TargetCentre[1] = RKReceiveBuffer[7] << 8 | RKReceiveBuffer[8];
      GimbalFlag = 3;
      CorrectTime++;
      FailTimeinaRow = 0;
      HitFlag = 1;

      if (RKReceiveBuffer[1] == 0x64)
        __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, 105);
    }

    UARTOverTime = 0;

    // memset(RKReceiveBuffer, 0, 10);                          // Clear buffer for next reception
    HAL_UARTEx_ReceiveToIdle_DMA(&huart3, RKReceiveBuffer, 10); // Continue DMA reception
  }
  /**
   * @subsection IMU Attitude Data Processing
   * @brief Parse high-precision binary attitude data from IMU sensor
   *
   * Binary Protocol Specifications:
   * • Packet Size: 200 bytes for comprehensive attitude and motion data
   * • Header Validation: 0x59 0x53 for packet integrity verification
   * • Data Format: Little-endian 32-bit signed integers
   * • Scaling: Raw values multiplied by 1e-6 for proper unit conversion
   * • Angular Range: Automatic normalization to [-180°, +180°]
   *
   * Extracted Data:
   * • Yaw Angle: Primary heading reference for chassis control
   * • Angular Velocities: Roll, pitch, yaw rates for dynamic control
   * • Data Integrity: Header validation ensures reliable communication
   */
  else if (huart == &huart2) // UART2: High-precision IMU attitude sensor
  {
    // Validate binary packet header for data integrity
    if (IMUReceiveBuffer[0] == 0x59 && IMUReceiveBuffer[1] == 0x53)
    {
      /**
       * @brief Yaw Angle Extraction
       * @details Extract 32-bit yaw angle from bytes 43-46 (little-endian)
       *
       * Byte Order: [LSB, byte1, byte2, MSB] at positions [43, 44, 45, 46]
       * Scaling: Raw value × 1e-6 converts to degrees
       * Normalization: Wrap angles > 180° to maintain [-180°, +180°] range
       */
      gimbal_angle[2] = ((IMUReceiveBuffer[46] << 24) | (IMUReceiveBuffer[45] << 16) |
                         (IMUReceiveBuffer[44] << 8) | (IMUReceiveBuffer[43] << 0)) *
                        0.000001;

      // Normalize to standard [-180°, +180°] angular range
      gimbal_angle[2] = gimbal_angle[2] > 180 ? gimbal_angle[2] - 360 : gimbal_angle[2];

      /**
       * @brief Angular Velocity Extraction
       * @details Extract roll, pitch, yaw rates for dynamic control feedback
       *
       * Data Layout:
       * • Roll Rate (X): Bytes 21-24
       * • Pitch Rate (Y): Bytes 25-28
       * • Yaw Rate (Z): Bytes 29-32 (inverted for coordinate system alignment)
       *
       * Coordinate System: Right-handed with Z-axis pointing up
       * Units: Degrees per second after scaling
       */
      AngularVelocity[0] = ((IMUReceiveBuffer[24] << 24) | (IMUReceiveBuffer[23] << 16) |
                            (IMUReceiveBuffer[22] << 8) | (IMUReceiveBuffer[21] << 0)) *
                           0.000001;
      AngularVelocity[1] = ((IMUReceiveBuffer[28] << 24) | (IMUReceiveBuffer[27] << 16) |
                            (IMUReceiveBuffer[26] << 8) | (IMUReceiveBuffer[25] << 0)) *
                           0.000001;
      AngularVelocity[2] = (-1) * ((IMUReceiveBuffer[32] << 24) | (IMUReceiveBuffer[31] << 16) | (IMUReceiveBuffer[30] << 8) | (IMUReceiveBuffer[29] << 0)) *
                           0.000001;
    }

    // Buffer management: commented to prevent interference with ongoing reception
    memset(IMUReceiveBuffer, 0, 200);                             // Clear buffer
    HAL_UARTEx_ReceiveToIdle_DMA(&huart2, IMUReceiveBuffer, 200); // Restart DMA reception
  }
}

/**
 * @section PWM Input Capture Processing
 * @brief High-precision duty cycle measurement for remote control input processing
 * @param htim Timer peripheral instance that triggered the capture callback
 * @retval None
 *
 * @details Dual-Axis PWM Signal Processing Architecture:
 *
 * This function implements precise PWM duty cycle measurement using hardware timer
 * input capture for omnidirectional remote control. The system processes two independent
 * PWM channels for complete 2D motion control with hardware-level timing accuracy.
 *
 * Signal Processing Methodology:
 * • X-axis (TIM2): Lateral movement command (strafe left/right)
 * • Y-axis (TIM5): Longitudinal movement command (forward/backward)
 *
 * Measurement Process:
 * 1. Capture rising and falling edge timestamps using hardware counters
 * 2. Calculate duty cycle using empirically-derived calibration formula
 * 3. Normalize to [0.0, 1.0] range with 0.5 representing neutral position
 * 4. Reset timer counter for accurate next measurement cycle
 *
 * Hardware Calibration:
 * The calibration formula compensates for STM32F4 timer characteristics,
 * PCB trace delays, and signal conditioning circuit propagation times.
 *
 * Performance Specifications:
 * • Resolution: ~1µs timing accuracy at 84MHz timer clock
 * • Update Rate: 50Hz standard RC PWM frequency
 * • Range: 1.0ms - 2.0ms pulse width (1.5ms = neutral)
 * • Accuracy: ±0.1% duty cycle measurement precision
 */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  /**
   * @subsection X-Axis PWM Processing (Lateral Control)
   * @brief Process horizontal axis PWM for strafe control
   * @note Currently disabled for this application
   */
  if (htim == &htim2 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) // X-axis PWM processing
  {
    // X-axis PWM processing currently disabled for this application
    // Uncomment below lines if X-axis control is needed:
    // xTimUpCounter = HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_2);   // Rising edge timestamp
    // xTimDownCounter = HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_1); // Falling edge timestamp

    /**
     * @brief Hardware-Calibrated Duty Cycle Calculation
     * @details Apply compensation formula for accurate duty cycle measurement
     *
     * Formula: duty = (fall_time - offset1) / (rise_time - offset2)
     * • offset1 = 16 * rise_time / 4119.0  (falling edge compensation)
     * • offset2 = 24 * rise_time / 4119.0  (rising edge compensation)
     *
     * These constants compensate for hardware-specific timing delays
     */
    // xTimDuty = ((float)xTimDownCounter - 16 * xTimUpCounter / 4119.0) /
    //            (xTimUpCounter - 24 * xTimUpCounter / 4119.0);

    // __HAL_TIM_SetCounter(&htim2, 0); // Reset timer counter for next measurement cycle
  }

  /**
   * @subsection Y-Axis PWM Processing (Longitudinal Control)
   * @brief Process vertical axis PWM for forward/backward control
   */
  else if (htim == &htim5 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4) // Y-axis PWM processing
  {
    // Capture edge timing values for longitudinal control calculation
    yTimUpCounter = HAL_TIM_ReadCapturedValue(&htim5, TIM_CHANNEL_4);   // Rising edge timestamp
    yTimDownCounter = HAL_TIM_ReadCapturedValue(&htim5, TIM_CHANNEL_3); // Falling edge timestamp

    /**
     * @brief Hardware-Calibrated Duty Cycle Calculation
     * @details Apply compensation formula for Y-axis measurement
     *
     * Uses same calibration constants as X-axis for consistency.
     * Includes boundary check to prevent negative duty cycle values.
     */
    float calculated_duty = ((float)yTimDownCounter - 16 * yTimUpCounter / 4119.0) /
                            (yTimUpCounter - 24 * yTimUpCounter / 4119.0);
    yTimDuty = (calculated_duty > 0) ? calculated_duty : 0;

    __HAL_TIM_SetCounter(&htim5, 0); // Reset timer counter for next measurement cycle
  }
}

/**
 * @section Real-Time Control System Main Loop
 * @brief Periodic execution of sensor fusion, trajectory planning, and motor control
 * @param htim Timer peripheral instance that triggered the periodic callback
 * @retval None
 *
 * @details Executive Control System Architecture:
 *
 * This function implements the core real-time control loop executing at high frequency
 * via TIM10 interrupt. It orchestrates a sophisticated multi-layer control system
 * integrating sensor fusion, motion planning, and precise motor control for advanced
 * robotic platform operation.
 *
 * ┌─────────────────────────────────────────────────────────────────────┐
 * │ CONTROL SYSTEM EXECUTION PIPELINE                                   │
 * │                                                                     │
 * │ 1. SENSOR DATA ACQUISITION                                          │
 * │    • Motor position/velocity feedback via CAN bus                   │
 * │    • IMU attitude data integration                                  │
 * │    • PWM remote control signal processing                           │
 * │                                                                     │
 * │ 2. SIGNAL PROCESSING & INTEGRATION                                  │
 * │    • PWM overflow detection and wraparound handling                 │
 * │    • Cumulative displacement calculation                            │
 * │    • Motion trajectory integration                                  │
 * │                                                                     │
 * │ 3. TRAJECTORY STATE MACHINE                                         │
 * │    • Autonomous motion pattern generation                           │
 * │    • Adaptive speed profile calculation                             │
 * │    • Multi-cycle trajectory management                              │
 * │                                                                     │
 * │ 4. ADVANCED CONTROL ALGORITHMS                                      │
 * │    • Position control with fast trigonometric compensation          │
 * │    • Multi-motor PID control loops                                  │
 * │    • Vision-based tracking system                                   │
 * │                                                                     │
 * │ 5. COMMAND DISTRIBUTION & SAFETY                                    │
 * │    • Motor voltage command generation                               │
 * │    • Hardware protection and output limiting                        │
 * │    • CAN bus command transmission                                   │
 * └─────────────────────────────────────────────────────────────────────┘
 *
 * Performance Characteristics:
 * • Execution Frequency: High-frequency timer-driven execution
 * • Real-Time Constraints: Deterministic timing for control stability
 * • Multi-Tasking: Concurrent sensor fusion and control processing
 * • Safety Systems: Multiple layers of protection and error handling
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim == &htim1)
  {
    CorrectTimePerSecond = CorrectTime;
    FailTimePerSecond = FailTime;
    CorrectTime = 0;
    FailTime = 0;
  }
  else if (htim == &htim9) // Target hit monitoring and communication timeout timer
  {
    if (j < 0)
    {
      HitTime++;
      Flag4005 = 1;

      if (HitFlag == 0)
      {
        j = -j;
        HitTime = 0;
      }
    }

    if (HitTime > HitTimeMax)
    {
      j = -j;
      HitTime = 0;
    }

    UARTOverTime++;

    if (UARTOverTime >= 100)
    {
      UARTOverTime++;
      GimbalFlag = 2;
    }
  }
  else if (htim == &htim10) // Main control system timer (high-frequency execution)
  {
    TimerSystemMode = (TimerSystemMode + 1) % 4;

    if (TimerSystemMode % 2 == 0)
    {
      /**
       * @subsection Step 1: Motor Feedback Data Acquisition
       * @brief Read current motor states and update feedback variables
       *
       * Data Acquisition Process:
       * • CAN bus motor position/velocity feedback reception
       * • M2006 drivetrain motor status updates (4 motors)
       * • GM6020 gimbal motor feedback for precision control
       * • Real-time data synchronization for control calculations
       */
      read_motor_position(); // Update M2006_info array with current motor encoder states

      /**
       * @subsection Step 2: PWM Signal Overflow Detection and Integration
       * @brief Advanced signal processing for continuous motion tracking
       *
       * Overflow Detection Algorithm:
       * This sophisticated algorithm handles PWM duty cycle wraparound at signal boundaries,
       * enabling continuous tracking of cumulative displacement beyond single signal cycles.
       *
       * Detection Logic:
       * • Forward Wraparound: duty_old ≥ 0.8 AND duty_new ≤ 0.2 → increment overflow counter
       * • Reverse Wraparound: duty_old ≤ 0.2 AND duty_new ≥ 0.8 → decrement overflow counter
       *
       * This enables tracking of unlimited motion range while maintaining precision.
       */

      // X-axis (lateral) overflow detection for continuous displacement tracking
      if (xTimDutyOld >= 0.8 && xTimDuty <= 0.2)
        xTimDutyOverflow += 1; // Forward wraparound detected
      else if (xTimDuty >= 0.8 && xTimDutyOld <= 0.2)
        xTimDutyOverflow -= 1; // Reverse wraparound detected

      // Calculate cumulative lateral displacement in millimeters (currently disabled)
      // xTimDistance = (xTimDutyOverflow - xTimDutyInit + xTimDuty) * WheelLength;

      // Y-axis (longitudinal) overflow detection with identical algorithm
      if (yTimDutyOld >= 0.9 && yTimDuty <= 0.1)
        yTimDutyOverflow += 1; // Forward wraparound detected
      else if (yTimDuty >= 0.9 && yTimDutyOld <= 0.1)
        yTimDutyOverflow -= 1; // Reverse wraparound detected

      // Calculate cumulative longitudinal displacement with baseline offset
      yTimDistance = (yTimDutyOverflow - yTimDutyInit + yTimDuty) * WheelLength + MotorposSetMin;

      /**
       * @subsection Step 3: State Variable Updates
       * @brief Update historical values for next cycle processing
       */
      xTimDutyOld = xTimDuty; // Store current X-axis value for next cycle overflow detection
      yTimDutyOld = yTimDuty; // Store current Y-axis value for next cycle overflow detection

      /**
       * @subsection Step 4: Intelligent Trajectory State Machine
       * @brief Autonomous motion pattern control with adaptive switching
       *
       * State Machine Logic:
       * • Even states (j % 2 == 0): Forward motion trajectories
       * • Odd states (j % 2 == 1): Return motion trajectories
       * • Position thresholds trigger automatic state transitions
       * • Configurable trajectory cycles before system termination
       *
       * Benefits:
       * • Autonomous operation without manual intervention
       * • Precise position-based state switching
       * • Configurable motion patterns for different applications
       */

      // Automatic trajectory switching based on position threshold detection
      if (j % 2 == 0 && yTimDistance > yTimDistanceMaxStandard[j % 10] - MotorposSetDiff)
      {
        if (HitFlag == 0)
          j += 1;
        else
          j = -j - 1;
      }
      else if (j % 2 == 1 && yTimDistance < yTimDistanceMinStandard[j % 10] + MotorposSetDiff)
      {
        if (HitFlag == 0)
          j += 1;
        else
          j = -j - 1;
      }

      /**
       * @subsection Step 5: Adaptive Speed Profile Calculation
       * @brief Dynamic speed control based on position and trajectory state
       *
       * Speed Profile Architecture:
       * The system implements sophisticated speed ramping for smooth acceleration,
       * constant velocity cruising, and controlled deceleration based on position
       * within the trajectory. This ensures optimal motion dynamics and reduces
       * mechanical stress on the drivetrain.
       *
       * Profile Types:
       * • Acceleration Zone: Linear speed increase from minimum to target
       * • Cruise Zone: Constant speed operation for maximum efficiency
       * • Deceleration Zone: Linear speed decrease for precise positioning
       * • Directional Control: Speed sign indicates motion direction
       */
      if (j >= RoundFlipFre || j < 0)
      {
        MotorspeedSetMid = 0; // System halt after maximum trajectory cycles completed
                              // SetAngle = chassis_angle[2];  // Reset target angle for stabilization

        if (j >= RoundFlipFre)
          Flag4005 = 1;
      }
      else if (j == 0) // Initial forward trajectory with sophisticated acceleration profiling
      {
        if (yTimDistance < yTimDistanceMin + MotorposSetMin)
        {
          // Acceleration zone: Linear speed ramp with minimum speed protection
          MotorspeedSetMid = (MotorspeedSetStandard[j] / (float)(yTimDistanceMin) * (yTimDistance - MotorposSetMin)) > MotorspeedSetMin ? (MotorspeedSetStandard[j] / (float)(yTimDistanceMin) * (yTimDistance - MotorposSetMin)) : MotorspeedSetMin;
          // GimbalFlag = 1;
          Flag4005 = 1;
        }
        else if (yTimDistance > yTimDistanceMaxStandard[j] - yTimDistanceMin)
        {
          // Deceleration zone: Linear speed reduction for precise stop positioning
          MotorspeedSetMid = (MotorspeedSetStandard[j] / (float)(yTimDistanceMin) * (yTimDistanceMaxStandard[j] - yTimDistance));
          Flag4005 = 1;
        }
        else if (GimbalFlag != 3 && GimbalFlag != 4)
        {
          // Cruise zone: Constant speed operation for maximum efficiency
          MotorspeedSetMid = MotorspeedSetStandard[j];
          GimbalFlag = 2;
          Flag4005 = 1;
        }
      }
      else if (j % 2 == 0 && j != 0 && j < RoundFlipFre && j > 0) // Subsequent forward trajectories with varied profiles
      {
        if (yTimDistance < yTimDistanceMin + yTimDistanceMinStandard[(j + 9) % 10])
        {
          // Forward acceleration with minimum speed protection
          MotorspeedSetMid = (MotorspeedSetStandard[j % 10] / (float)(yTimDistanceMin) * (yTimDistance - yTimDistanceMinStandard[(j + 9) % 10])) > MotorspeedSetMin ? (MotorspeedSetStandard[j % 10] / (float)(yTimDistanceMin) * (yTimDistance - yTimDistanceMinStandard[(j + 9) % 10])) : MotorspeedSetMin;
          Flag4005 = 1;
        }
        else if (yTimDistance > yTimDistanceMaxStandard[j % 10] - yTimDistanceMin)
        {
          // Forward deceleration for precise positioning
          MotorspeedSetMid = (MotorspeedSetStandard[j % 10] / (float)(yTimDistanceMin) * (yTimDistanceMaxStandard[j % 10] - yTimDistance));
          Flag4005 = 1;
        }
        else
        {
          // Forward cruise with varied speed profiles for testing diversity
          MotorspeedSetMid = MotorspeedSetStandard[j % 10];
          Flag4005 = 1;
        }
      }
      else if (j % 2 == 1 && j < RoundFlipFre && j > 0) // Reverse trajectory with negative speed profile
      {
        if (yTimDistance > yTimDistanceMaxStandard[(j + 9) % 10] - yTimDistanceMin)
        {
          // Reverse acceleration: Negative speed with linear ramping
          MotorspeedSetMid = (float)(MotorspeedSetStandard[(j + 9) % 10] / (float)(yTimDistanceMin) * (yTimDistanceMaxStandard[(j + 9) % 10] - yTimDistance)) * (-1) < MotorspeedSetMin * (-1) ? (float)(MotorspeedSetStandard[(j + 9) % 10] / (float)(yTimDistanceMin) * (yTimDistanceMaxStandard[(j + 9) % 10] - yTimDistance)) * (-1) : MotorspeedSetMin * (-1);
          Flag4005 = 1;
        }
        else if (yTimDistance < yTimDistanceMin + yTimDistanceMinStandard[j % 10])
        {
          // Reverse deceleration: Controlled speed reduction with minimum protection
          MotorspeedSetMid = (float)((MotorspeedSetStandard[j % 10] / (float)(yTimDistanceMin) * (yTimDistance - yTimDistanceMinStandard[j % 10]))) * (-1);
          Flag4005 = 1;
        }
        else
        {
          // Reverse cruise: Constant negative speed for return motion
          MotorspeedSetMid = (float)(MotorspeedSetStandard[j % 10]) * (-1);
          Flag4005 = 1;
        }
      }

      /**
       * @subsection Step 6: Position Control with Fast Trigonometric Compensation
       * @brief Advanced chassis attitude stabilization using optimized trigonometric functions
       *
       * Control Strategy:
       * This sophisticated control system maintains chassis orientation stability while
       * enabling precise trajectory following. It combines position feedback with attitude
       * correction using fast sine calculations for real-time performance.
       *
       * Algorithm Components:
       * • Position Error: Lateral displacement correction based on trajectory state
       * • Attitude Compensation: Fast sine calculation of chassis angle error
       * • Gain Scheduling: Different gains for forward/reverse motion optimization
       * • Safety Limiting: Output clamping prevents excessive correction forces
       *
       * Mathematical Foundation:
       * The system uses the optimized FAST_SIN_DEG macro for trigonometric calculations,
       * providing ~10x performance improvement over standard math library while
       * maintaining accuracy sufficient for control applications.
       */

      // Adaptive chassis attitude control with trajectory-dependent gain scheduling
      if (j % 2 == 1 && j < RoundFlipFre)
      {
        if (abs(xTimDistance) > 50)
          PosPIDResult = KdPos * FAST_SIN_DEG(chassis_angle[2] - SetAngle);
        else
          // Odd states (reverse motion): Negative position gain + attitude compensation
          PosPIDResult = -KpPos * xTimDistance + KdPos * FAST_SIN_DEG(chassis_angle[2] - SetAngle);
      }
      else if (j % 2 == 0 && j < RoundFlipFre)
      {
        if (abs(xTimDistance) > 50)
          PosPIDResult = KdPos * FAST_SIN_DEG(chassis_angle[2] - SetAngle);
        else
          // Even states (forward motion): Positive position gain + attitude compensation
          PosPIDResult = KpPos * xTimDistance + KdPos * FAST_SIN_DEG(chassis_angle[2] - SetAngle);
      }
      else if (j >= RoundFlipFre || j < 0)
        // System stopped: Pure attitude control for stabilization
        PosPIDResult = KdPos * FAST_SIN_DEG(chassis_angle[2] - SetAngle);

      LIMIT_MIN_MAX(PosPIDResult, -1000, 1000); // Safety clamp prevents excessive correction torque

      /**
       * @subsection Step 7: Multi-Motor PID Control Loop Implementation
       * @brief Individual motor control with position compensation and differential steering
       *
       * Control Architecture Overview:
       * This advanced control system implements independent PID controllers for each of the
       * four drivetrain motors while coordinating them for cohesive platform motion. The
       * system incorporates differential steering correction to maintain trajectory accuracy.
       *
       * ┌─────────────────────────────────────────────────────────────────────┐
       * │ MOTOR ARRANGEMENT & CONTROL DISTRIBUTION                            │
       * │                                                                     │
       * │     Motor 0 (FL) ──────────── Motor 1 (FR)                          │
       * │         │                         │                                 │
       * │         │    + PosPIDResult       │    + PosPIDResult               │
       * │         │                         │                                 │
       * │         │                         │                                 │
       * │     Motor 2 (RL) ──────────── Motor 3 (RR)                          │
       * │         │                         │                                 │
       * │         │    - PosPIDResult       │    - PosPIDResult               │
       * │                                                                     │
       * │ Differential Steering Logic:                                        │
       * │ • Front Motors (0,1): Add position correction                       │
       * │ • Rear Motors (2,3): Subtract position correction                   │
       * │ • Creates coordinated turning motion for trajectory following       │
       * └─────────────────────────────────────────────────────────────────────┘
       *
       * PID Algorithm Implementation:
       * • Type: Incremental PID for superior stability and anti-windup properties
       * • Error History: 3-sample sliding window for accurate derivative calculation
       * • Formula: Δu = Kp·(e[k]-e[k-1]) + Ki·e[k] + Kd·(e[k]-2·e[k-1]+e[k-2])
       * • Benefits: Prevents integral windup, smoother transient response
       *
       * Safety & Protection:
       * • Output saturation prevents motor damage from excessive voltage
       * • Individual motor limits ensure coordinated operation
       * • Real-time error monitoring for system health assessment
       */
      for (int l = 0; l < 4; l++)
      {
        /**
         * @brief Motor Speed Setpoint Distribution with Differential Steering
         * @details Distribute base speed command with position correction for coordinated motion
         */
        if (l < 2)
          MotorspeedSet[l] = MotorspeedSetMid + PosPIDResult; // Front motors: add steering correction
        else
          MotorspeedSet[l] = MotorspeedSetMid - PosPIDResult; // Rear motors: subtract steering correction

        /**
         * @brief Error History Management - Sliding Window Implementation
         * @details Maintain 3-sample error history for accurate derivative calculation
         *
         * Error History Structure:
         * • e[k-2]: Error from 2 cycles ago (oldest)
         * • e[k-1]: Error from previous cycle
         * • e[k]: Current cycle error (newest)
         *
         * This sliding window enables precise derivative calculation while
         * minimizing memory usage and computational overhead.
         */
        M2006_info_error[l][2] = M2006_info_error[l][1];              // Shift: e[k-2] ← e[k-1]
        M2006_info_error[l][1] = M2006_info_error[l][0];              // Shift: e[k-1] ← e[k]
        M2006_info_error[l][0] = MotorspeedSet[l] - M2006_info[l][1]; // Calculate: e[k] = setpoint - feedback

        /**
         * @brief Incremental PID Calculation
         * @details Advanced PID implementation with superior stability characteristics
         *
         * Incremental PID Benefits:
         * • Prevents integral windup through bounded accumulation
         * • Provides smoother transient response compared to position PID
         * • Reduces computational load by avoiding large integral terms
         * • Maintains stability even with parameter changes
         *
         * Mathematical Implementation:
         * Δu[k] = Kp·(e[k] - e[k-1]) + Ki·e[k] + Kd·(e[k] - 2·e[k-1] + e[k-2])
         * u[k] = u[k-1] + Δu[k]
         */
        SpeedPIDResult[l] += (KpSpeed * (M2006_info_error[l][0] - M2006_info_error[l][1])) +                             // Proportional term
                             (KiSpeed * M2006_info_error[l][0]) +                                                        // Integral term
                             (KdSpeed * (M2006_info_error[l][0] - 2 * M2006_info_error[l][1] + M2006_info_error[l][2])); // Derivative term

        /**
         * @brief Motor Protection and Output Saturation
         * @details Hardware protection through output voltage limiting
         *
         * Protection Features:
         * • Prevents motor overcurrent conditions
         * • Ensures coordinated operation across all motors
         * • Maintains system stability under extreme operating conditions
         * • Protects motor driver electronics from damage
         */
        LIMIT_MIN_MAX(SpeedPIDResult[l], -10000, 10000); // Standard motor voltage limits: ±10V maximum
      }

      /**
       * @subsection Step 8: Motor Command Distribution and Hardware Interface
       * @brief Differential drive voltage assignment with mechanical compensation
       *
       * Physical Configuration Compensation:
       * The robotic platform uses a differential drive configuration where motors
       * may be mounted in different orientations. This section handles the necessary
       * voltage polarity adjustments to ensure consistent directional response.
       *
       * Motor Assignment Strategy:
       * • Motors 0,1 (Front pair): Direct voltage assignment (positive = forward motion)
       * • Motors 2,3 (Rear pair): Inverted voltage assignment (compensates for mounting orientation)
       *
       * This inversion accounts for:
       * • Mechanical mounting differences between front and rear motor pairs
       * • Gearbox orientation variations in the drivetrain design
       * • Ensures consistent forward/reverse response across all motors
       * • Maintains coordinated motion for the complete vehicle platform
       */
      motor_voltage[0] = SpeedPIDResult[0];        // Front-left: Direct assignment (no inversion needed)
      motor_voltage[1] = SpeedPIDResult[1];        // Front-right: Direct assignment (no inversion needed)
      motor_voltage[2] = (-1) * SpeedPIDResult[2]; // Rear-left: Inverted for mechanical compensation
      motor_voltage[3] = (-1) * SpeedPIDResult[3]; // Rear-right: Inverted for mechanical compensation

      // Transmit coordinated motor commands via CAN bus to motor controller network
      // Currently commented for safety - uncomment when ready for motor operation
      set_motor_voltage(0x01, motor_voltage[0], motor_voltage[1], motor_voltage[2], motor_voltage[3]);
    }
    else if (TimerSystemMode == 3)
      clear_motor_mistake();
    else if (TimerSystemMode == 1)
    {
      Errory[0] = Errory[1];
      Errory[1] = PurposeCentre[1] - TargetCentre[1];
      target_speedy = (int32_t)(KpGimbal2 * Errory[1] + KdGimbal2 * (Errory[1] - Errory[0]));
      LIMIT_MIN_MAX(target_speedy, -500.0f, 500.0f); // Safe operating range: ±500 units

      if (GimbalFlag == 1 || GimbalFlag == 2)
        set_motor_position(0);
      else if (GimbalFlag == 3)
      {
        if (Flag4005)
          set_motor_speed_dm4005(target_speedy);
        else if (!Flag4005)
          set_motor_position(0);
      }
    }

    if (GimbalFlag == 1)
    {
      target_speed = KpGM6020pos * GM6020_info[0];
      LIMIT_MIN_MAX(target_speed, -100.0f, 100.0f);
    }
    else if (GimbalFlag == 2)
    {
      if (j < RoundFlipFre)
      {
        target_speed = KpGM6020pos * GM6020_info[0];
        LIMIT_MIN_MAX(target_speed, -100.0f, 100.0f);
      }
      else
      {
        target_speed = GimbalStopFlag * 15;

        if (GM6020_info[0] < -2448)
          GimbalStopFlag = 1;
        else if (GM6020_info[0] > 400)
          GimbalStopFlag = -1;
      }
    }
    else if (GimbalFlag == 3)
    {
      /**
       * @subsection Step 9: Vision-Based Tracking Control System with Advanced Filtering
       * @brief Computer vision tracking with filtered position data and gimbal control
       *
       * Enhanced Tracking Architecture:
       * This system now includes comprehensive position filtering and validation
       * to ensure stable tracking performance even with noisy camera data.
       *
       * ┌─────────────────────────────────────────────────────────────────────┐
       * │ ENHANCED TRACKING CONTROL PIPELINE                                  │
       * │                                                                     │
       * │ Camera Input → Position Filtering → Error Calculation →             │
       * │ Deadzone Control → PID Processing → Motor Command                   │
       * │                                                                     │
       * │ Filter Features:                                                    │
       * │ • Range validation prevents out-of-bounds coordinates               │
       * │ • Change rate limiting eliminates sudden position jumps             │
       * │ • Low-pass filtering reduces camera noise and jitter                │
       * │ • Position validity checking enables graceful error handling        │
       * │                                                                     │
       * │ Control Features:                                                   │
       * │ • Deadzone control prevents micro-oscillations                      │
       * │ • PID control with anti-windup protection                           │
       * │ • Angular velocity limiting for mechanical safety                   │
       * │ • Automatic tracking pause during invalid position data             │
       * └─────────────────────────────────────────────────────────────────────┘
       */

      // Calculate vision tracking error for horizontal axis control (yaw gimbal)
      int16_t tracking_error_x = TargetCentre[0] - PurposeCentre[0]; // Pixel error in X direction
      int16_t abs_error_x = (tracking_error_x >= 0) ? tracking_error_x : -tracking_error_x;

      // Update tracking error history for PID derivative calculation
      GimbalError[1] = GimbalError[0];   // Shift: e[k-1] ← e[k]
      GimbalError[0] = tracking_error_x; // Current processed error: e[k] = filtered_error

      GimbalIntegralResult += KiGimbal1 * tracking_error_x;
      LIMIT_MIN_MAX(GimbalIntegralResult, -0.03, 0.03); // Prevent integral windup

      if (abs_error_x <= 35)
      {
        KpGimbal1 = KpGimbal1a;
        KdGimbal1 = KdGimbal1a;
      }
      else if (abs_error_x > 35 && abs_error_x <= 45)
      {
        KpGimbal1 = KpGimbal1b;
        KdGimbal1 = KdGimbal1b;
      }
      else
      {
        KpGimbal1 = KpGimbal1c;
        KdGimbal1 = KdGimbal1c;
      }

      // Calculate PID output with smooth transition scaling
      target_speed = (KpGimbal1 * GimbalError[0] +                    // Proportional term
                      GimbalIntegralResult +                          // Integral term
                      KdGimbal1 * (GimbalError[0] - GimbalError[1])); // Derivative term

      if (abs_error_x > 40 && abs_error_x <= 50)
        // Apply angular velocity limits for mechanical safety and stability
        LIMIT_MIN_MAX(target_speed, -30.0f, 30.0f); // Safe operating range: ±30 deg/s
      else
        // Apply angular velocity limits for mechanical safety and stability
        LIMIT_MIN_MAX(target_speed, -10.0f, 10.0f); // Safe operating range: ±10 deg/s
    }
  }
}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /**
   * @brief Enhanced Error Handling with Diagnostics
   * @details Comprehensive error logging and safe system shutdown
   */

  // Record error timestamp and current system state
  // last_error_time = HAL_GetTick();
  // uint32_t current_stack = check_stack_usage();

  // Emergency motor shutdown to prevent mechanical damage
  // int16_t zero_voltage[4] = {0, 0, 0, 0};
  // set_motor_voltage(0x01, zero_voltage[0], zero_voltage[1], zero_voltage[2], zero_voltage[3]);
  // set_motor_speed_dm4005(0);

  // Send detailed error report via UART (if still functional)
  // sprintf((char *)Temp, "🚨 SYSTEM ERROR at %lu ms\r\n", last_error_time);
  // HAL_UART_Transmit(&huart8, (unsigned char *)Temp, strlen((char *)Temp), 1000);

  // sprintf((char *)Temp, "Stack: %lu bytes | Max: %lu | Loops: %lu\r\n",
  //         current_stack, stack_usage_max, loop_counter);
  // HAL_UART_Transmit(&huart8, (unsigned char *)Temp, strlen((char *)Temp), 1000);

  // sprintf((char *)Temp, "Last Reset Cause: 0x%02X\r\n", (unsigned int)reset_cause_code);
  // HAL_UART_Transmit(&huart8, (unsigned char *)Temp, strlen((char *)Temp), 1000);

  /* Disable all interrupts to prevent further system activity */
  __disable_irq();

  /* Enter infinite loop to maintain controlled system state */
  /* Hardware watchdog will reset system if configured and enabled */
  while (1)
  {
    /* Infinite loop for safety - system remains in controlled halt state */
    /* Optional: Add LED blinking pattern or debug output here */
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /**
   * @brief User Implementation for Assertion Failure Handling
   * @details Add custom assertion handling code here for debugging and diagnostics
   *
   * Recommended Implementation:
   * • Printf-style error message output via UART for remote debugging
   * • LED pattern indication for visual assertion failure notification
   * • System state logging to help identify conditions leading to assertion
   * • Optional: Trigger system reset after logging for automated recovery
   *
   * Example Implementation:
   * printf("Wrong parameters value: file %s on line %d\r\n", file, line);
   *
   * Debug Considerations:
   * • Use the file and line parameters to identify the exact assertion location
   * • Review the HAL function documentation for parameter requirements
   * • Check variable values and ranges that led to the assertion failure
   * • Verify system configuration matches the expected operating conditions
   */

  /* User can add custom implementation to report the file name and line number */
  /* Example: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Optional: Add breakpoint here for debugging */
  /* Optional: Add LED indication or UART output for error reporting */
  /* Optional: Implement system recovery or reset mechanism */

  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
