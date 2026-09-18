#include "ble_nus_transport.h"

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/services/nus.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>

#include <errno.h>
#include <limits.h>

static struct bt_conn *active_conn;
static bool notifications_enabled;
static struct k_mutex transport_lock;
static struct k_work advertising_work;

static const struct bt_data advertising_data[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS,
		      BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR),
	BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_NUS_SRV_VAL),
};

static const struct bt_data scan_response_data[] = {
	BT_DATA(BT_DATA_NAME_COMPLETE,
		CONFIG_BT_DEVICE_NAME,
		sizeof(CONFIG_BT_DEVICE_NAME) - 1U),
};

static int advertising_start(void)
{
	int ret = bt_le_adv_start(BT_LE_ADV_CONN_FAST_1,
				  advertising_data,
				  ARRAY_SIZE(advertising_data),
				  scan_response_data,
				  ARRAY_SIZE(scan_response_data));

	if ((ret == 0) || (ret == -EALREADY)) {
		if (ret == 0) {
			printk("BLE NUS advertising: %s\n",
			       CONFIG_BT_DEVICE_NAME);
		}
		return 0;
	}

	printk("ERROR: BLE advertising failed: %d\n", ret);
	return ret;
}

static void advertising_work_handler(struct k_work *work)
{
	ARG_UNUSED(work);
	(void)advertising_start();
}

static void connected(struct bt_conn *conn, uint8_t error)
{
	if (error != 0U) {
		printk("ERROR: BLE connection failed: %u\n", error);
		(void)k_work_submit(&advertising_work);
		return;
	}

	k_mutex_lock(&transport_lock, K_FOREVER);
	if (active_conn == NULL) {
		active_conn = bt_conn_ref(conn);
	}
	notifications_enabled = false;
	k_mutex_unlock(&transport_lock);

	printk("BLE NUS connected; waiting for TX subscription\n");
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	k_mutex_lock(&transport_lock, K_FOREVER);
	if (active_conn == conn) {
		bt_conn_unref(active_conn);
		active_conn = NULL;
	}
	notifications_enabled = false;
	k_mutex_unlock(&transport_lock);

	printk("BLE NUS disconnected: 0x%02X\n", reason);
	(void)k_work_submit(&advertising_work);
}

BT_CONN_CB_DEFINE(ble_connection_callbacks) = {
	.connected = connected,
	.disconnected = disconnected,
};

static void notification_changed(bool enabled, void *context)
{
	ARG_UNUSED(context);

	k_mutex_lock(&transport_lock, K_FOREVER);
	notifications_enabled = enabled && (active_conn != NULL);
	k_mutex_unlock(&transport_lock);

	printk("BLE NUS notifications: %s\n",
	       enabled ? "ENABLED" : "DISABLED");
}

static void data_received(struct bt_conn *conn,
			  const void *data,
			  uint16_t length,
			  void *context)
{
	ARG_UNUSED(conn);
	ARG_UNUSED(data);
	ARG_UNUSED(context);

	/* M5.3只验证状态上行；下行控制在后续阶段实现。 */
	printk("BLE NUS RX ignored: %u bytes\n", (unsigned int)length);
}

static struct bt_nus_cb nus_callbacks = {
	.notif_enabled = notification_changed,
	.received = data_received,
};

int ble_nus_transport_init(void)
{
	int ret;

	active_conn = NULL;
	notifications_enabled = false;
	k_mutex_init(&transport_lock);
	k_work_init(&advertising_work, advertising_work_handler);

	ret = bt_nus_cb_register(&nus_callbacks, NULL);
	if (ret < 0) {
		printk("ERROR: BLE NUS callback registration failed: %d\n", ret);
		return ret;
	}

	ret = bt_enable(NULL);
	if (ret < 0) {
		printk("ERROR: Bluetooth initialization failed: %d\n", ret);
		return ret;
	}

	ret = advertising_start();
	if (ret < 0) {
		return ret;
	}

	printk("BLE NUS initialized\n");
	return 0;
}

bool ble_nus_transport_is_ready(void)
{
	bool ready;

	k_mutex_lock(&transport_lock, K_FOREVER);
	ready = (active_conn != NULL) && notifications_enabled;
	k_mutex_unlock(&transport_lock);

	return ready;
}

int ble_nus_transport_send(const uint8_t *data, size_t length)
{
	struct bt_conn *connection;
	int ret;

	if ((data == NULL) && (length != 0U)) {
		return -EINVAL;
	}

	if (length > UINT16_MAX) {
		return -EMSGSIZE;
	}

	k_mutex_lock(&transport_lock, K_FOREVER);
	if ((active_conn == NULL) || !notifications_enabled) {
		k_mutex_unlock(&transport_lock);
		return -ENOTCONN;
	}

	connection = bt_conn_ref(active_conn);
	k_mutex_unlock(&transport_lock);

	ret = bt_nus_send(connection, data, (uint16_t)length);
	bt_conn_unref(connection);

	return ret;
}
