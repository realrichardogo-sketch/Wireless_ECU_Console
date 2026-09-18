# Wireless ECU Console Vehicle Protocol v1

## 1. 目标

同一车辆数据帧用于 USB CDC、BLE NUS 和后续电脑到 STM32 的串口链路。接收端可以利用帧头重新同步，利用序号检测丢帧，利用 CRC16 检测数据损坏。

## 2. 字节序与校验

- 多字节整数：Little Endian。
- CRC：CRC-16/CCITT-FALSE。
- 多项式：`0x1021`。
- 初始值：`0xFFFF`。
- RefIn/RefOut：false。
- XorOut：`0x0000`。
- CRC覆盖范围：字节2到字节12，不包含帧头与CRC字段。

## 3. VehicleStatus固定帧

总长度：15字节。

| 偏移 | 长度 | 字段 | 说明 |
|---:|---:|---|---|
| 0 | 1 | SOF0 | 固定 `0x55` |
| 1 | 1 | SOF1 | 固定 `0xAA` |
| 2 | 1 | Version | 当前为 `0x01` |
| 3 | 1 | Message Type | `0x10` = VehicleStatus |
| 4 | 2 | Sequence | 每发送一帧递增 |
| 6 | 1 | Payload Length | 固定为6 |
| 7 | 2 | RPM | 0–8000 |
| 9 | 2 | Temperature | `int16_t`，单位°C |
| 11 | 1 | State | 0=NORMAL，1=HIGH_RPM，2=OVER_TEMP，3=CRITICAL |
| 12 | 1 | Flags | bit0=故障激活，其余保留 |
| 13 | 2 | CRC16 | Little Endian |

## 4. 发送策略

- 上电完成后立即发送一帧。
- RPM、温度、故障或状态改变后立即发送。
- 没有变化时每500 ms发送心跳帧。
- Sequence在每次发送后递增，`65535`之后自然回绕到`0`。

## 5. 接收端处理顺序

1. 搜索连续字节 `0x55 0xAA`。
2. 检查版本、消息类型和Payload Length。
3. 收齐15字节。
4. 重新计算CRC并比较。
5. 解析车辆数据。
6. 比较Sequence，统计重复帧或丢帧。

## 6. 后续扩展

后续消息通过不同 Message Type 扩展，例如：

- `0x11`：主机控制命令。
- `0x12`：通信确认。
- `0x13`：诊断与故障注入。

现有 `VehicleStatus` 帧格式保持不变，保证电脑中继与 STM32 网关兼容。
