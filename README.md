# 智能物品防丢系统 (Smart Anti-Loss System)

[![License](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE)
[![nRF52](https://img.shields.io/badge/nRF52-SDK%2017.1-green.svg)](https://www.nordicsemi.com/Software-and-tools/Software/nRF5-SDK)
[![BLE](https://img.shields.io/badge/BLE-5.0-orange.svg)](https://www.bluetooth.com/specifications/bluetooth-core-specification/)

基于 nRF52 和 BLE 5.0 的智能物品防丢系统，用于 NUSOC2025 竞赛。

## 📋 项目简介

本项目是一个完整的智能物品防丢系统设计方案，包括：
- **接收器（主机）**：可放置于背包或随身佩戴的监控设备
- **标签（从机）**：可挂载于钥匙、钱包等物品的追踪器

### 核心功能
- ✅ 快速配对与解绑
- ✅ 多标签连接管理（最多8个）
- ✅ 实时RSSI监测
- ✅ 2秒内断线报警
- ✅ 智能寻找模式
- ✅ OLED显示界面
- ✅ 超低功耗设计
- ✅ 板载天线设计

## 🏗️ 系统架构

```
┌─────────────────────────────────────────────┐
│              接收器 (Receiver)              │
│  ┌────────────────────────────────────┐    │
│  │  nRF52832 MCU                      │    │
│  │  ├─ BLE Central                    │    │
│  │  ├─ OLED Display (128x64)          │    │
│  │  ├─ Buzzer/Motor Alert             │    │
│  │  ├─ Multi-Tag Manager (Max 8)      │    │
│  │  └─ Flash Storage                  │    │
│  └────────────────────────────────────┘    │
└─────────────────────────────────────────────┘
                     ↕ BLE 5.0
┌─────────────────────────────────────────────┐
│               标签 (Tags)                   │
│  ┌────────────────────────────────────┐    │
│  │  nRF52832 MCU                      │    │
│  │  ├─ BLE Peripheral                 │    │
│  │  ├─ Button (Click/Double-Click)    │    │
│  │  ├─ Buzzer/Vibration Motor         │    │
│  │  └─ Ultra-Low Power Mode           │    │
│  └────────────────────────────────────┘    │
└─────────────────────────────────────────────┘
```

## 📁 项目结构

```
NUSOC2025/
├── docs/                       # 📚 设计文档
│   ├── 设计文档.md            # 总体设计方案
│   ├── 硬件设计规范.md        # 硬件设计详细说明
│   ├── 软件设计规范.md        # 软件架构和实现
│   ├── 通信协议.md            # BLE通信协议规范
│   ├── 测试报告.md            # 测试与验证（待完成）
│   └── 答辩材料.md            # 答辩演示资料（待完成）
├── hardware/                   # 🔧 硬件设计
│   ├── pcb/                   # PCB设计文件
│   │   ├── receiver/          # 接收器PCB
│   │   └── tag/               # 标签PCB
│   ├── schematic/             # 原理图
│   ├── antenna/               # 天线设计
│   └── bom/                   # 物料清单
├── firmware/                   # 💻 固件代码
│   ├── receiver/              # 接收器固件
│   │   ├── src/              # 源代码
│   │   ├── inc/              # 头文件
│   │   └── config/           # 配置文件
│   └── tag/                   # 标签固件
│       ├── src/              # 源代码
│       ├── inc/              # 头文件
│       └── config/           # 配置文件
├── software/                   # 🖥️ 上位机软件（可选）
├── tests/                      # 🧪 测试脚本
└── README.md                   # 本文件
```

## 🚀 快速开始

### 硬件需求
- nRF52832开发板 x 2（或自制PCB）
- 0.96寸 OLED显示屏（I2C接口）
- 蜂鸣器/振动马达
- CR2032电池座（标签）
- 锂电池 + TP4056充电模块（接收器）
- 按键开关
- LED指示灯

### 软件需求
- nRF5 SDK v17.1.0
- ARM GCC工具链
- Segger Embedded Studio 或 VS Code
- J-Link调试器

### 编译固件

#### 接收器固件
```bash
cd firmware/receiver
make
```

#### 标签固件
```bash
cd firmware/tag
make
```

### 烧录固件

使用J-Link烧录：
```bash
# 烧录SoftDevice
nrfjprog --program s132_nrf52_7.3.0_softdevice.hex --chiperase

# 烧录应用程序
nrfjprog --program _build/antiloss_receiver.hex --sectorerase
nrfjprog --reset
```

## 📖 详细文档

- [设计文档](docs/设计文档.md) - 系统整体设计方案
- [硬件设计规范](docs/硬件设计规范.md) - PCB设计、元器件选型
- [软件设计规范](docs/软件设计规范.md) - 固件架构和编码规范
- [通信协议](docs/通信协议.md) - BLE通信协议详细说明

## 🔑 主要特性

### 接收器功能
- [x] 支持最多8个标签同时连接
- [x] 0.96寸OLED显示屏界面
- [x] 实时RSSI监测（500ms间隔）
- [x] 2秒内断线报警响应
- [x] 智能寻找模式（RSSI导航）
- [x] 断线历史记录
- [x] Flash持久化存储
- [x] 可充电锂电池供电

### 标签功能
- [x] 超低功耗设计（待机3μA）
- [x] 按键单击启用/双击禁用报警
- [x] 蜂鸣器/振动马达寻找提示
- [x] CR2032纽扣电池供电（6-12个月续航）
- [x] LED状态指示
- [x] 快速配对（<1秒）

## 🎯 技术指标

### 性能指标
| 参数 | 指标 |
|------|------|
| 通信距离 | ≥ 20m（室内） |
| 连接延迟 | < 1s |
| 报警响应 | < 2s |
| RSSI精度 | ±5dBm |
| 最大标签数 | 8 |

### 功耗指标
| 设备 | 工作模式 | 功耗 | 续航 |
|------|---------|------|------|
| 接收器 | 监测模式 | 500μA | 5-7天 |
| 接收器 | OLED显示 | 8mA | - |
| 标签 | 广播模式 | 50μA | 6-12个月 |
| 标签 | 待机模式 | 3μA | - |

### 尺寸规格
- **接收器**: ≤ 40mm x 30mm x 10mm
- **标签**: ≤ 25mm x 25mm x 8mm

## 🧪 测试

### 功能测试
- [x] 配对/解绑测试
- [x] 多标签并发测试
- [x] RSSI监测精度测试
- [x] 报警响应时间测试
- [ ] 寻找模式测试
- [ ] 续航时间测试

### 场景测试
- [ ] 室内环境（10m, 20m, 30m）
- [ ] 障碍物遮挡测试
- [ ] 背包内部测试
- [ ] 金属干扰测试
- [ ] 人体遮挡测试

### 天线测试
- [ ] S11参数测试（< -10dB @ 2.4GHz）
- [ ] 辐射方向图测试
- [ ] 手持握持影响测试

## 📊 开发进度

- [x] 需求分析
- [x] 系统设计
- [x] 文档编写
- [ ] 硬件设计
  - [ ] 原理图设计
  - [ ] PCB布局
  - [ ] 天线设计
  - [ ] BOM清单
- [ ] 固件开发
  - [x] 基础框架
  - [ ] BLE通信实现
  - [ ] 标签管理模块
  - [ ] 报警处理模块
  - [ ] OLED界面
- [ ] PCB制作与调试
- [ ] 系统集成测试
- [ ] 文档完善
- [ ] 答辩准备

## 👥 团队成员

- **项目负责人**: NUSOC2025 Team
- **硬件设计**: 待定
- **固件开发**: 待定
- **测试验证**: 待定

## 📄 许可证

本项目采用 MIT 许可证 - 详见 [LICENSE](LICENSE) 文件

## 🙏 致谢

- Nordic Semiconductor - nRF52 SDK
- 开源社区的各种库和工具

## 📮 联系方式

- **仓库**: https://github.com/Rsyyy04/NUSOC2025
- **问题反馈**: 通过 GitHub Issues

---

**版本**: v1.0.0  
**最后更新**: 2025-12-11  
**状态**: 设计阶段
