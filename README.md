# Wireless ECU Console

Wireless ECU Console 是一套可复现的车辆状态采集与转发演示系统。nRF52840 控制台生成 RPM、温度和故障状态，通过 BLE Nordic UART Service 发送到 Windows 电脑；Python 中继程序经 USB-TTL 把固定长度数据帧送入 STM32 Node A，再由 Node A 通过 CAN 转发给 Node B 显示状态。

## 系统链路

```text
nRF52840 控制台
  └─ BLE NUS
      └─ Windows Python 中继
          └─ USB-TTL / USART2
              └─ STM32F103ZE Node A
                  └─ CAN 500 kbit/s, ID 0x201
                      └─ STM32F103ZE Node B / 状态 LED
```

## 主要功能

- EC11 编码器调节 RPM，按键调节温度并注入故障。
- ST7789 + LVGL 显示 RPM、温度、故障与状态。
- WS2812 灯带显示 NORMAL、HIGH_RPM、OVER_TEMP、CRITICAL 效果。
- 同一套 15 字节协议用于 BLE NUS、USB CDC 和电脑到 Node A 的串口链路。
- Node A 校验帧头、长度、CRC 与序号，再转换为 8 字节 CAN 帧。
- Node B 根据状态点亮对应 LED，并在 CAN 超时 1.5 秒后红黄交替闪烁。

## 仓库结构

| 路径 | 内容 |
|---|---|
| `boards/atguigu/demo01_blinky/` | nRF52840 自定义开发板设备树 |
| `src/` | nRF52840 Zephyr 应用、协议和 BLE/USB 传输 |
| `firmware/STM32_Wireless_Node_A/` | Node A Keil 工程 |
| `firmware/STM32_Wireless_Node_B/` | Node B Keil 工程 |
| `firmware/STM32_Wireless_CAN_Gateway/tools/` | Windows BLE→串口中继程序 |
| `tools/` | 调试工具及 PowerShell 启动脚本 |
| `docs/M8_TEST_REPORT.md` | 硬件在环验收记录 |

## 车辆状态规则

| 条件 | 状态 |
|---|---|
| RPM < 5000 且温度 < 100°C 且无故障 | `NORMAL` |
| RPM >= 5000，未同时超温且无故障 | `HIGH_RPM` |
| 温度 >= 100°C，未同时高转速且无故障 | `OVER_TEMP` |
| 故障激活，或高转速与超温同时发生 | `CRITICAL` |

## 15 字节无线/串口协议 v1

所有多字节字段均为 Little Endian。CRC 使用 CRC-16/CCITT-FALSE，覆盖字节 2–12。

| 偏移 | 长度 | 字段 |
|---:|---:|---|
| 0 | 1 | SOF0 = `0x55` |
| 1 | 1 | SOF1 = `0xAA` |
| 2 | 1 | Version = `0x01` |
| 3 | 1 | Type = `0x10` |
| 4 | 2 | Sequence |
| 6 | 1 | Payload Length = `6` |
| 7 | 2 | RPM |
| 9 | 2 | Temperature，`int16_t`，单位 °C |
| 11 | 1 | State：0–3 |
| 12 | 1 | Flags：bit0 = fault active |
| 13 | 2 | CRC16 |

详细定义见 `src/vehicle_protocol_v1.md`。

## CAN 数据帧

- 标准帧 ID：`0x201`
- 波特率：`500 kbit/s`
- DLC：`8`

| CAN 数据字节 | 内容 |
|---|---|
| 0–1 | RPM |
| 2–3 | Temperature |
| 4 | State |
| 5 | Fault |
| 6–7 | Sequence |

## 1. 编译 nRF52840 控制台

环境：nRF Connect SDK / Zephyr，已验证的工具链版本为 nRF Connect SDK Toolchain v3.2.3。

在仓库根目录执行：

```powershell
west build --build-dir build_1 . --pristine --board demo01_blinky/nrf52840 --sysbuild -- -DBOARD_ROOT=.
```

编译成功后，通过 nRF Connect for VS Code 的 `Flash` 操作烧录。

## 2. 编译 STM32 Node A 与 Node B

使用 Keil MDK-ARM V5，打开以下工程并分别执行 `Rebuild`、`Download`：

```text
firmware/STM32_Wireless_Node_A/STM32_Template_F103ZE.uvprojx
firmware/STM32_Wireless_Node_B/STM32_Template_F103ZE.uvprojx
```

目标芯片为 STM32F103ZET6，系统时钟 72 MHz。

## 3. 硬件连接

### USB-TTL 到 Node A

| USB-TTL | Node A |
|---|---|
| TXD | PA3 / USART2_RX |
| RXD | PA2 / USART2_TX，可选 |
| GND | GND |

USB-TTL 必须使用 3.3 V TTL 电平。Node A 可单独供电，避免把不确定的 5 V 接到 MCU 引脚。

### Node A 到 Node B CAN

| Node A | Node B |
|---|---|
| CANH | CANH |
| CANL | CANL |
| GND | GND |

两端 CAN1 均使用 PB8/RX、PB9/TX，经开发板 CAN 收发器连接。总线两端应有 120 Ω 终端电阻。

## 4. 运行电脑中继

先打开电脑蓝牙并给 nRF52840 控制台上电，然后在 PowerShell 中执行：

```powershell
python -m pip install -r requirements.txt
python -m serial.tools.list_ports -v
.\tools\run_vehicle_bridge.ps1 -Port COM3 -ScanTimeout 30
```

将 `COM3` 换成 USB-TTL 实际端口。也可以直接运行：

```powershell
python -u .\firmware\STM32_Wireless_CAN_Gateway\tools\vehicle_ble_to_stm32_bridge.py --port COM3 --scan-timeout 30
```

成功时终端会持续显示：

```text
BLE RX SEQ:... RPM:... TEMP:... FAULT:... STATE:... CRC:OK -> UART TX COM3 15 bytes
```

## Node B 指示灯

开发板 LED 为低电平点亮。

| 状态 | Node B 指示 |
|---|---|
| `NORMAL` | 黄色 LED-1 / PA0 |
| `HIGH_RPM` | 蓝色 LED-2 / PA1 |
| `OVER_TEMP` | 绿色 LED-3 / PA8 |
| `CRITICAL` 或 fault=1 | 红色 LED-0 / PG11 |
| CAN 超过 1.5 秒无有效帧 | 红色与黄色交替闪烁 |

## 已完成验证

- Node A、Node B 均在 Keil ARMCC 5.06 update 7 下零错误编译并烧录。
- 10 分钟连续链路测试：`good=1225`、`bad=0`、`lost=0`。
- BLE 断开、Node A 复位、Node B 复位均能触发预期超时或恢复行为。
- NORMAL、HIGH_RPM、OVER_TEMP、CRITICAL 四种状态均完成实板验证。

详见 `docs/M8_TEST_REPORT.md`。

## 安全说明

本项目用于教学和台架验证，不应直接用于真实车辆的安全关键控制。接线或改变供电前请先断电，并确认 TTL 电平、CAN 收发器供电和共地关系。
