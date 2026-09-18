#include "usb_cdc_transport.h"

#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>
#include <zephyr/usb/usbd.h>

#include <errno.h>

#define USB_DEVICE_VID          0x1915
#define USB_DEVICE_PID          0xEC01
#define USB_DEVICE_MANUFACTURER "Wireless ECU"
#define USB_DEVICE_PRODUCT      "ECU Console CDC"

BUILD_ASSERT(DT_NODE_HAS_STATUS(DT_NODELABEL(usbd), okay),
	     "USB device controller is missing or disabled");
BUILD_ASSERT(DT_NODE_HAS_STATUS(DT_NODELABEL(cdc_acm_uart0), okay),
	     "cdc_acm_uart0 is missing or disabled");

USBD_DEVICE_DEFINE(ecu_usbd,
		   DEVICE_DT_GET(DT_NODELABEL(usbd)),
		   USB_DEVICE_VID,
		   USB_DEVICE_PID);

USBD_DESC_LANG_DEFINE(ecu_usb_lang);
USBD_DESC_MANUFACTURER_DEFINE(ecu_usb_manufacturer,
			      USB_DEVICE_MANUFACTURER);
USBD_DESC_PRODUCT_DEFINE(ecu_usb_product,
			 USB_DEVICE_PRODUCT);

USBD_DESC_CONFIG_DEFINE(ecu_usb_fs_cfg_desc, "FS Configuration");
USBD_CONFIGURATION_DEFINE(ecu_usb_fs_config,
			  0,
			  250,
			  &ecu_usb_fs_cfg_desc);

static const struct device *const cdc_dev =
	DEVICE_DT_GET(DT_NODELABEL(cdc_acm_uart0));

static const char *const class_blocklist[] = {
	NULL,
};

static volatile bool usb_stack_enabled;
static volatile bool usb_configured;
static volatile bool usb_dtr_asserted;
static volatile bool usb_suspended;

static int usb_stack_enable(void)
{
	int ret;

	if (usb_stack_enabled) {
		return 0;
	}

	ret = usbd_enable(&ecu_usbd);
	if (ret == 0) {
		usb_stack_enabled = true;
		printk("USB CDC stack enabled\n");
	}

	return ret;
}

static void usb_message_callback(
	struct usbd_context *const usbd_context,
	const struct usbd_msg *const message)
{
	ARG_UNUSED(usbd_context);

	if (message->type == USBD_MSG_CDC_ACM_CONTROL_LINE_STATE) {
		uint32_t dtr = 0U;
		int ret;

		if (message->dev != cdc_dev) {
			return;
		}

		ret = uart_line_ctrl_get(
			cdc_dev,
			UART_LINE_CTRL_DTR,
			&dtr);
		if (ret < 0) {
			printk("ERROR: USB CDC DTR read failed: %d\n", ret);
			return;
		}

		usb_dtr_asserted = dtr != 0U;
		printk("USB CDC host: %s\n",
		       usb_dtr_asserted ? "CONNECTED" : "DISCONNECTED");
		return;
	}

	switch (message->type) {
	case USBD_MSG_VBUS_READY:
		if (!usb_stack_enabled) {
			int ret = usb_stack_enable();

			if (ret < 0) {
				printk("ERROR: USB CDC enable failed: %d\n", ret);
			}
		}
		break;

	case USBD_MSG_VBUS_REMOVED:
		usb_dtr_asserted = false;
		usb_configured = false;
		usb_suspended = false;

		if (usb_stack_enabled) {
			int ret = usbd_disable(&ecu_usbd);

			if (ret < 0) {
				printk("ERROR: USB CDC disable failed: %d\n", ret);
			} else {
				usb_stack_enabled = false;
			}
		}
		break;

	case USBD_MSG_CONFIGURATION:
		usb_configured = message->status != 0U;
		if (!usb_configured) {
			usb_dtr_asserted = false;
		}
		printk("USB CDC configuration: %s\n",
		       usb_configured ? "ACTIVE" : "INACTIVE");
		break;

	case USBD_MSG_SUSPEND:
		usb_suspended = true;
		break;

	case USBD_MSG_RESUME:
		usb_suspended = false;
		break;

	default:
		break;
	}
}

static int usb_descriptors_init(void)
{
	int ret;

	ret = usbd_add_descriptor(&ecu_usbd, &ecu_usb_lang);
	if (ret < 0) {
		return ret;
	}

	ret = usbd_add_descriptor(&ecu_usbd, &ecu_usb_manufacturer);
	if (ret < 0) {
		return ret;
	}

	ret = usbd_add_descriptor(&ecu_usbd, &ecu_usb_product);
	if (ret < 0) {
		return ret;
	}

	ret = usbd_add_configuration(
		&ecu_usbd,
		USBD_SPEED_FS,
		&ecu_usb_fs_config);
	if (ret < 0) {
		return ret;
	}

	ret = usbd_register_all_classes(
		&ecu_usbd,
		USBD_SPEED_FS,
		1,
		class_blocklist);
	if (ret < 0) {
		return ret;
	}

	return usbd_device_set_code_triple(
		&ecu_usbd,
		USBD_SPEED_FS,
		0,
		0,
		0);
}

int usb_cdc_transport_init(void)
{
	int ret;

	if (!device_is_ready(cdc_dev)) {
		printk("ERROR: USB CDC device is not ready\n");
		return -ENODEV;
	}

	usb_stack_enabled = false;
	usb_configured = false;
	usb_dtr_asserted = false;
	usb_suspended = false;

	ret = usbd_msg_register_cb(&ecu_usbd, usb_message_callback);
	if (ret < 0) {
		printk("ERROR: USB callback registration failed: %d\n", ret);
		return ret;
	}

	ret = usb_descriptors_init();
	if (ret < 0) {
		printk("ERROR: USB descriptor setup failed: %d\n", ret);
		return ret;
	}

	ret = usbd_init(&ecu_usbd);
	if (ret < 0) {
		printk("ERROR: USB stack initialization failed: %d\n", ret);
		return ret;
	}

	/*
	 * nRF52840支持VBUS检测时，VBUS_READY回调负责启用协议栈。
	 * 对不支持VBUS检测的控制器，在这里直接启用。
	 */
	if (!usbd_can_detect_vbus(&ecu_usbd)) {
		ret = usb_stack_enable();
		if (ret < 0) {
			printk("ERROR: USB stack enable failed: %d\n", ret);
			return ret;
		}
	}

	printk("USB CDC initialized; waiting for COM port DTR\n");
	return 0;
}

bool usb_cdc_transport_is_ready(void)
{
	return usb_stack_enabled &&
	       usb_configured &&
	       usb_dtr_asserted &&
	       !usb_suspended;
}

int usb_cdc_transport_send(const uint8_t *data, size_t length)
{
	if ((data == NULL) && (length != 0U)) {
		return -EINVAL;
	}

	if (!usb_cdc_transport_is_ready()) {
		return -ENOTCONN;
	}

	for (size_t i = 0; i < length; i++) {
		uart_poll_out(cdc_dev, data[i]);
	}

	return 0;
}
