#ifndef USB_CDC_TRANSPORT_H_
#define USB_CDC_TRANSPORT_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 初始化USB Device Next协议栈和CDC ACM接口。
 * 设备会在Windows中枚举成一个虚拟COM端口。
 */
int usb_cdc_transport_init(void);

/* 主机已经打开COM端口并置位DTR时返回true。 */
bool usb_cdc_transport_is_ready(void);

/*
 * 发送一帧原始二进制数据。
 * 返回0：发送完成。
 * 返回-ENOTCONN：COM端口尚未打开或DTR尚未置位。
 */
int usb_cdc_transport_send(const uint8_t *data, size_t length);

#ifdef __cplusplus
}
#endif

#endif /* USB_CDC_TRANSPORT_H_ */
