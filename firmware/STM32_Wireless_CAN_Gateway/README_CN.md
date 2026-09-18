# Wireless ECU Console - STM32 Node A / Node B

适用硬件：STM32F103ZET6 核心板、标准外设库、Keil MDK ARMCC 5、两块 SN65HVD230。

## 1. 完整链路

```text
nRF52840 控制台
  -> BLE NUS
电脑中继脚本
  -> USB-TTL / USART2
STM32 Node A 网关
  -> CAN 0x201
STM32 Node B 接收 ECU
  -> USART1 日志 + 板载 LED
```

Node A 不重新生成车辆数据。它接收并校验 nRF52840 发出的 15 字节原始协议帧，然后把有效字段压缩为一帧 8 字节 CAN 报文。Node B 解码 CAN 报文、检查序号、显示状态并监测 1.5 秒通信超时。

## 2. 经原理图确认的引脚

| 功能 | MCU 引脚 | 60 Pin 接口 | 用途 |
|---|---:|---:|---|
| USART1 TX | PA9 | H1-26 | Node A / B 调试日志 |
| USART1 RX | PA10 | H1-28 | 调试串口可选接收 |
| USART2 TX | PA2 | H1-46 | Node A 返回通道，当前可不接 |
| USART2 RX | PA3 | H1-48 | Node A 接收电脑中继数据 |
| CAN1 RX | PB8 | H1-34 | 连接 SN65HVD230 的 CRX/RXD |
| CAN1 TX | PB9 | H1-36 | 连接 SN65HVD230 的 CTX/TXD |
| LED-1 | PA0 | H1-25 | Node A 收帧翻转；Node B NORMAL |
| LED-2 | PA1 | H1-45 | Node B HIGH_RPM |
| LED-3 | PA8 | H1-15 | Node B OVER_TEMP |
| LED-0 | PG11 | H1-54 | Node B CRITICAL / 超时闪烁 |

四颗 LED 均为低电平点亮。

## 3. 打开现成的 Keil 工程

仓库已经包含完整标准外设库、启动文件和工程配置，不需要再复制模板或手动添加源码。

```text
../STM32_Wireless_Node_A/STM32_Template_F103ZE.uvprojx
../STM32_Wireless_Node_B/STM32_Template_F103ZE.uvprojx
```

分别在 Keil 中打开两个工程，依次执行 `Rebuild` 和 `Download`。工程已配置：

```text
Target: STM32F103ZE
Defines: USE_STDPERIPH_DRIVER, STM32F10X_HD
Includes: ./User, ./CMSIS, ./FWLIB/inc
Startup: startup_stm32f10x_hd.s
```

## 4. USB-TTL 接线

电脑中继使用 Node A 的 USART2：

| USB-TTL | Node A |
|---|---|
| TXD | PA3 / H1-48 / USART2_RX |
| RXD | PA2 / H1-46 / USART2_TX，可选 |
| GND | 任意 GND |

USB-TTL 必须使用 3.3 V TTL 电平。开发板已有供电时不要连接 USB-TTL 的 VCC。

调试日志使用 USART1，需要第二个 USB-TTL：

| 调试 USB-TTL | Node A 或 Node B |
|---|---|
| RXD | PA9 / H1-26 / USART1_TX |
| TXD | PA10 / H1-28 / USART1_RX，可选 |
| GND | 任意 GND |

只有一个 USB-TTL 时，先把它用于 Node A 的 USART2 数据输入；可以根据 PA0 每收到一帧翻转一次来判断 Node A 是否收帧，并直接在 Node B 的 LED 上观察端到端结果。

## 5. CAN 收发器接线

两块板的连接方式相同：

| STM32F103ZE | SN65HVD230 |
|---|---|
| PB9 / CAN_TX | CTX、TXD 或 D |
| PB8 / CAN_RX | CRX、RXD 或 R |
| 3V3 | VCC |
| GND | GND |

总线侧：

```text
Node A CANH ---- CANH Node B
Node A CANL ---- CANL Node B
Node A GND  ---- GND  Node B
```

总线两端各启用一个 120 欧终端电阻。断电测量 CANH 与 CANL 应约为 60 欧。

## 6. CAN 数据格式

标准帧 ID：`0x201`，DLC：8，速率：500 kbit/s。

| 字节 | 内容 | 格式 |
|---|---|---|
| 0-1 | RPM | uint16，小端 |
| 2-3 | 温度 | int16，小端 |
| 4 | 状态 | 0 NORMAL；1 HIGH_RPM；2 OVER_TEMP；3 CRITICAL |
| 5 | 故障 | 0 或 1 |
| 6-7 | 序号 | uint16，小端 |

例如 RPM=900、TEMP=40、STATE=NORMAL、FAULT=0、SEQ=7486：

```text
84 03 28 00 00 00 3E 1D
```

## 7. 电脑中继程序

安装依赖：

```powershell
python -m pip install -r requirements.txt
```

查看 USB-TTL 端口号：

```powershell
python -m serial.tools.list_ports -v
```

在仓库根目录运行。假设端口为 COM3：

```powershell
python -u .\firmware\STM32_Wireless_CAN_Gateway\tools\vehicle_ble_to_stm32_bridge.py --port COM3 --scan-timeout 30
```

预期输出：

```text
UART ready: COM3 @ 115200 8N1
Scanning for 'Wireless ECU Console' ...
Connected and subscribed; forwarding valid frames. Ctrl+C stops.
BLE RX SEQ: 7486 RPM: 900 TEMP: 40 C FAULT:0 STATE:NORMAL CRC:OK -> UART TX COM3 15 bytes
```

## 8. 分阶段验证

### M6：BLE 到 Node A 串口

1. 暂时可不接 CAN。
2. 若不希望未连接 CAN 时出现 `NO_MAILBOX`，把 `../STM32_Wireless_Node_A/User/board_config.h` 中 `NODE_A_CAN_FORWARDING` 改为 `0U` 后重新编译。
3. 烧录 Node A，运行电脑中继。
4. PA0 应随有效帧翻转；若接了第二个 USB-TTL，Node A 日志应显示：

```text
UART FRAME OK SEQ=... RPM=... TEMP=... STATE=NORMAL FAULT=0 CAN=DISABLED
```

停止中继超过 1.5 秒：

```text
WIRELESS TIMEOUT
```

恢复中继：

```text
WIRELESS RECOVERED
```

### M7：Node A 到 Node B CAN

1. 将 `NODE_A_CAN_FORWARDING` 设为 `1U`。
2. 接好两块 CAN 收发器和终端电阻。
3. 分别烧录 Node A、Node B。
4. 运行电脑中继。
5. Node B 日志应显示：

```text
CAN RX OK ID=0x201 SEQ=... RPM=... TEMP=... STATE=NORMAL FAULT=0
```

旋钮、温度键和故障键变化后，Node B 数据及状态 LED 应同步变化。停止中继超过 1.5 秒后，Node B 的红色 PG11 与黄色 PA0 交替闪烁并输出 `CAN TIMEOUT`；恢复后输出 `CAN RECOVERED`。

## 9. 首次编译时重点检查

- Target Device 必须是 STM32F103ZE。
- Startup 文件使用 `startup_stm32f10x_hd.s`。
- `system_stm32f10x.c` 应配置 72 MHz；CAN 参数按 APB1=36 MHz 计算。
- Node A 和 Node B 是两个独立工程，不要把两套 `main.c` 加入同一个 Target。
- 如果 Keil 报某个函数重复定义，先删除 Group 中旧的同名 `main.c` 或 `stm32f10x_it.c`。
