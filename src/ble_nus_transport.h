#ifndef BLE_NUS_TRANSPORT_H_
#define BLE_NUS_TRANSPORT_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 初始化Bluetooth协议栈、NUS服务并开始可连接广播。 */
int ble_nus_transport_init(void);

/* 已连接且主机已经订阅NUS TX通知时返回true。 */
bool ble_nus_transport_is_ready(void);

/*
 * 通过NUS TX特征发送一帧原始二进制数据。
 * 返回0：通知已提交。
 * 返回-ENOTCONN：尚未连接或主机尚未订阅通知。
 */
int ble_nus_transport_send(const uint8_t *data, size_t length);

#ifdef __cplusplus
}
#endif

#endif /* BLE_NUS_TRANSPORT_H_ */
