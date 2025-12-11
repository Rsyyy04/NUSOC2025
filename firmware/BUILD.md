# 固件编译说明

## 环境准备

### 1. 安装工具链

#### ARM GCC工具链
```bash
# Ubuntu/Debian
sudo apt-get install gcc-arm-none-eabi

# macOS
brew install gcc-arm-embedded

# Windows
# 下载并安装 ARM GCC: https://developer.arm.com/tools-and-software/open-source-software/developer-tools/gnu-toolchain/gnu-rm
```

#### nRF Command Line Tools
```bash
# 下载并安装: https://www.nordicsemi.com/Products/Development-tools/nrf-command-line-tools
# 包含 nrfjprog 工具用于烧录
```

### 2. 下载nRF5 SDK

```bash
# 下载 nRF5 SDK v17.1.0
wget https://www.nordicsemi.com/-/media/Software-and-other-downloads/SDKs/nRF5/Binaries/nRF5_SDK_17.1.0_ddde560.zip

# 解压到合适的位置
unzip nRF5_SDK_17.1.0_ddde560.zip
```

### 3. 配置SDK路径

有两种方式配置SDK路径：

#### 方式1: 环境变量（推荐）
```bash
# 在 ~/.bashrc 或 ~/.zshrc 中添加
export NRF5_SDK_ROOT=/path/to/nRF5_SDK_17.1.0_ddde560

# 重新加载配置
source ~/.bashrc
```

#### 方式2: 修改Makefile
编辑 `firmware/receiver/Makefile` 和 `firmware/tag/Makefile`，修改：
```makefile
SDK_ROOT ?= /path/to/your/nRF5_SDK_17.1.0_ddde560
```

## 编译固件

### 接收器固件

```bash
cd firmware/receiver

# 编译
make

# 编译输出位于 _build/ 目录
ls _build/
# antiloss_receiver.hex
# antiloss_receiver.elf
# antiloss_receiver.map
```

### 标签固件

```bash
cd firmware/tag

# 编译
make

# 编译输出位于 _build/ 目录
ls _build/
# antiloss_tag.hex
# antiloss_tag.elf
# antiloss_tag.map
```

## 烧录固件

### 1. 烧录SoftDevice（仅首次需要）

```bash
# 接收器
cd firmware/receiver
make flash_softdevice

# 标签
cd firmware/tag
make flash_softdevice
```

### 2. 烧录应用程序

```bash
# 接收器
cd firmware/receiver
make flash

# 标签
cd firmware/tag
make flash
```

### 3. 一次性烧录SoftDevice和应用程序

```bash
# 先烧录SoftDevice
make flash_softdevice

# 再烧录应用程序
make flash
```

## Makefile命令

| 命令 | 说明 |
|------|------|
| `make` | 编译固件 |
| `make clean` | 清除编译文件 |
| `make flash` | 烧录应用程序 |
| `make flash_softdevice` | 烧录SoftDevice |
| `make erase` | 擦除芯片 |

## 调试

### 使用J-Link调试

#### Segger Embedded Studio
1. 打开项目文件
2. 配置调试器为J-Link
3. 按F5开始调试

#### VS Code + Cortex-Debug
1. 安装Cortex-Debug插件
2. 配置 `.vscode/launch.json`:

```json
{
    "version": "0.2.0",
    "configurations": [
        {
            "name": "Debug (J-Link)",
            "type": "cortex-debug",
            "request": "launch",
            "servertype": "jlink",
            "cwd": "${workspaceRoot}",
            "executable": "./_build/antiloss_receiver.elf",
            "device": "nRF52832_xxAA",
            "interface": "swd"
        }
    ]
}
```

### 串口日志

使用RTT Viewer查看日志：

```bash
# 启动JLinkRTTViewer
JLinkRTTViewerExe

# 或使用命令行
JLinkRTTClient
```

## 常见问题

### Q1: 编译时提示找不到SDK

**解决方案**: 
- 检查SDK_ROOT环境变量是否设置
- 或修改Makefile中的SDK_ROOT路径

### Q2: 烧录时提示设备未找到

**解决方案**:
- 检查J-Link是否正确连接
- 检查电源是否开启
- 运行 `nrfjprog --list` 查看设备

### Q3: 编译时提示链接脚本不存在

**解决方案**:
链接脚本需要从SDK复制或创建：
```bash
cp $NRF5_SDK_ROOT/examples/ble_peripheral/ble_app_template/pca10040/s132/armgcc/ble_app_template_gcc_nrf52.ld firmware/receiver/linker_script.ld
```

### Q4: 烧录后设备无响应

**解决方案**:
- 检查SoftDevice是否正确烧录
- 使用 `make erase` 擦除后重新烧录
- 检查硬件连接

## 目标硬件

- **开发板**: nRF52832 DK / 自制PCB
- **MCU**: nRF52832-QFAA
- **调试接口**: SWD (SWDIO, SWCLK)
- **电源**: 3.3V

## 性能优化

### 编译优化级别

在Makefile中可调整优化级别：

```makefile
# 接收器（平衡性能和代码大小）
OPT = -O3 -g3

# 标签（优化功耗，最小代码）
OPT = -Os -g3
```

### 代码大小

典型编译后大小：
- 接收器: ~60KB Flash, ~8KB RAM
- 标签: ~40KB Flash, ~4KB RAM

---

**版本**: v1.0  
**最后更新**: 2025-12-11
