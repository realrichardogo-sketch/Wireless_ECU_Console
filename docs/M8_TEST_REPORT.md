# M8 硬件在环验收报告

测试日期：2026-09-18

## 测试链路

```text
nRF52840 -> BLE NUS -> Windows bridge -> USB-TTL -> Node A -> CAN -> Node B
```

## 验收环境

| 项目 | 配置 |
|---|---|
| nRF52840 | Wireless ECU Console，BLE 名称相同 |
| Node A / Node B | STM32F103ZET6 大开发板 |
| STM32 编译器 | ARMCC 5.06 update 7 (build 960) |
| PC 串口 | CH340，COM3，115200 8N1 |
| CAN | 标准帧 0x201，500 kbit/s |

## 结果

| 测试项 | 观察结果 | 结论 |
|---|---|---|
| 正常链路 | BLE 帧 CRC 正确并转发至 COM3，Node B 黄色灯亮 | 通过 |
| 高转速 | RPM >= 5000，Node B 蓝色灯亮 | 通过 |
| 超温 | 温度 >= 100°C，Node B 绿色灯亮 | 通过 |
| 故障/严重状态 | fault=1 或 CRITICAL，Node B 红色灯亮 | 通过 |
| 10 分钟稳定性 | good=1225，bad=0，lost=0；SEQ 12638–13862 | 通过 |
| BLE 断开 | 中继停止；Node B 进入红黄交替超时指示 | 通过 |
| BLE 恢复 | 中继重新连接后 Node B 恢复当前状态灯 | 通过 |
| Node A 复位 | 按住复位后 Node B 超时；释放后恢复 CAN 状态 | 通过 |
| Node B 复位 | 复位后重新接收 CAN，并恢复当前状态 | 通过 |

## 代表性终端记录

```text
UART ready: COM3 @ 115200 8N1
Scanning for 'Wireless ECU Console' ...
Connected and subscribed; forwarding valid frames. Ctrl+C stops.
BLE RX SEQ:13862 RPM:4100 TEMP: 65 C FAULT:0 STATE:NORMAL CRC:OK -> UART TX COM3 15 bytes
```

```text
File=logs/M8_soak_20260918_202749.txt
good=1225 bad=0 lost=0 first=12638 last=13862
```

## 验收结论

完整无线、串口和 CAN 链路满足当前教学台架的功能及恢复性要求。所有安全关键应用仍需额外完成电气保护、EMC、故障注入覆盖率和长期压力测试。
