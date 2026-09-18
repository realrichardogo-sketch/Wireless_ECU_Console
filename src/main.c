#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/display.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/led.h>
#include <zephyr/drivers/led_strip.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/sys/printk.h>

#include <lvgl.h>

#include <errno.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "vehicle_protocol.h"
#include "usb_cdc_transport.h"
#include "ble_nus_transport.h"

#define LED0_NODE DT_ALIAS(led0)
#define QDEC_NODE DT_ALIAS(qdec0)
#define GPIO0_NODE DT_NODELABEL(gpio0)
#define DISPLAY_NODE DT_CHOSEN(zephyr_display)
#define BACKLIGHT_NODE DT_ALIAS(backlight)
#define LED_STRIP_NODE DT_CHOSEN(zephyr_led_strip)
#define LED_STRIP_NUM_PIXELS DT_PROP(LED_STRIP_NODE, chain_length)

#define HIGH_RPM_THRESHOLD      5000U
#define OVER_TEMP_THRESHOLD     100

#define RPM_STEP                100
#define RPM_MIN                 0
#define RPM_MAX                 8000

#define TEMP_STEP               5
#define TEMP_MIN                (-40)
#define TEMP_MAX                150

#define ENCODER_STEPS_PER_REV   80LL
#define PULSES_PER_DETENT       4LL

#define DETENT_ANGLE_UDEG \
	(360000000LL / (ENCODER_STEPS_PER_REV / PULSES_PER_DETENT))

/* 键盘矩阵：当前控制功能都位于COL3。 */
#define MATRIX_COL3_PIN          30U
#define ENCODER_BUTTON_ROW_PIN   15U
#define TEMP_MINUS_ROW_PIN        7U
#define TEMP_PLUS_ROW_PIN         4U

#define KEY_DEBOUNCE_MS          30
#define MAIN_LOOP_PERIOD_MS      10
#define HEARTBEAT_PERIOD_MS      500
#define PROTOCOL_HEARTBEAT_MS    500

/* M4.2灯效参数：亮度较低，兼顾观感和供电余量。 */
#define STRIP_NORMAL_BRIGHTNESS   28U
#define STRIP_WARNING_BRIGHTNESS  32U
#define STRIP_CRITICAL_BRIGHTNESS 40U
#define STRIP_BREATH_MIN           4U
#define STRIP_BREATH_MAX          36U
#define STRIP_BREATH_STEP          1U
#define STRIP_BREATH_TICK_MS       40
#define STRIP_CRITICAL_BLINK_MS   250

BUILD_ASSERT(DT_NODE_HAS_STATUS(QDEC_NODE, okay),
	     "QDEC node is missing or disabled");
BUILD_ASSERT(DT_NODE_HAS_STATUS(GPIO0_NODE, okay),
	     "GPIO0 node is missing or disabled");
BUILD_ASSERT(DT_HAS_CHOSEN(zephyr_display),
	     "Missing zephyr,display chosen node");
BUILD_ASSERT(DT_NODE_HAS_STATUS(BACKLIGHT_NODE, okay),
	     "Backlight node is missing or disabled");
BUILD_ASSERT(DT_NODE_HAS_STATUS(LED_STRIP_NODE, okay),
	     "Missing zephyr,led-strip chosen node");
BUILD_ASSERT(DT_NODE_HAS_PROP(LED_STRIP_NODE, supply_gpios),
	     "Missing supply-gpios on LED strip node");
BUILD_ASSERT(LED_STRIP_NUM_PIXELS == 17U,
	     "This application expects 17 WS2812 pixels");

static const struct gpio_dt_spec led =
	GPIO_DT_SPEC_GET(LED0_NODE, gpios);

static const struct device *const qdec =
	DEVICE_DT_GET(QDEC_NODE);

static const struct device *const gpio0 =
	DEVICE_DT_GET(GPIO0_NODE);

static const struct device *const display_dev =
	DEVICE_DT_GET(DISPLAY_NODE);

static const struct device *const backlight_dev =
	DEVICE_DT_GET(DT_PARENT(BACKLIGHT_NODE));

static const uint32_t backlight_index =
	DT_NODE_CHILD_IDX(BACKLIGHT_NODE);

static const struct device *const led_strip_dev =
	DEVICE_DT_GET(LED_STRIP_NODE);

static const struct gpio_dt_spec led_strip_en =
	GPIO_DT_SPEC_GET(LED_STRIP_NODE, supply_gpios);

static struct led_rgb led_strip_pixels[LED_STRIP_NUM_PIXELS];

enum vehicle_state {
	VEHICLE_NORMAL,
	VEHICLE_HIGH_RPM,
	VEHICLE_OVER_TEMP,
	VEHICLE_CRITICAL
};

struct led_effect_state {
	enum vehicle_state vehicle_state;
	int64_t last_tick_ms;
	uint8_t breath_brightness;
	int8_t breath_direction;
	bool critical_on;
	bool initialized;
};

static struct led_effect_state led_effect;

struct vehicle_data {
	uint16_t rpm;
	int16_t temperature_c;
	bool fault_active;
	enum vehicle_state state;
};

enum console_key_id {
	CONSOLE_KEY_ENCODER,
	CONSOLE_KEY_TEMP_MINUS,
	CONSOLE_KEY_TEMP_PLUS,
	CONSOLE_KEY_COUNT
};

struct console_key {
	const char *name;
	gpio_pin_t row_pin;
	bool raw_pressed;
	bool stable_pressed;
	int64_t raw_changed_ms;
};

struct dashboard_ui {
	lv_obj_t *rpm_value;
	lv_obj_t *temp_value;
	lv_obj_t *state_panel;
	lv_obj_t *state_value;
	lv_obj_t *fault_value;
};

static struct console_key console_keys[CONSOLE_KEY_COUNT] = {
	[CONSOLE_KEY_ENCODER] = {
		.name = "ENCODER_BUTTON",
		.row_pin = ENCODER_BUTTON_ROW_PIN,
	},
	[CONSOLE_KEY_TEMP_MINUS] = {
		.name = "TEMP_MINUS",
		.row_pin = TEMP_MINUS_ROW_PIN,
	},
	[CONSOLE_KEY_TEMP_PLUS] = {
		.name = "TEMP_PLUS",
		.row_pin = TEMP_PLUS_ROW_PIN,
	},
};

static struct dashboard_ui dashboard;

static enum vehicle_state evaluate_vehicle_state(
	const struct vehicle_data *vehicle)
{
	bool high_rpm = vehicle->rpm >= HIGH_RPM_THRESHOLD;
	bool over_temp =
		vehicle->temperature_c >= OVER_TEMP_THRESHOLD;

	if (vehicle->fault_active || (high_rpm && over_temp)) {
		return VEHICLE_CRITICAL;
	}

	if (high_rpm) {
		return VEHICLE_HIGH_RPM;
	}

	if (over_temp) {
		return VEHICLE_OVER_TEMP;
	}

	return VEHICLE_NORMAL;
}

static const char *vehicle_state_to_string(enum vehicle_state state)
{
	switch (state) {
	case VEHICLE_NORMAL:
		return "NORMAL";
	case VEHICLE_HIGH_RPM:
		return "HIGH RPM";
	case VEHICLE_OVER_TEMP:
		return "OVER TEMP";
	case VEHICLE_CRITICAL:
		return "CRITICAL";
	default:
		return "UNKNOWN";
	}
}

static void print_vehicle_status(
	const char *source,
	const struct vehicle_data *vehicle)
{
	printk("%s | RPM: %u | TEMP: %d C | FAULT: %d | STATE: %s\n",
	       source,
	       (unsigned int)vehicle->rpm,
	       (int)vehicle->temperature_c,
	       vehicle->fault_active ? 1 : 0,
	       vehicle_state_to_string(vehicle->state));
}

static int64_t sensor_value_to_udeg(
	const struct sensor_value *value)
{
	return ((int64_t)value->val1 * 1000000LL) + value->val2;
}

static uint16_t clamp_rpm(int32_t rpm)
{
	if (rpm < RPM_MIN) {
		return RPM_MIN;
	}

	if (rpm > RPM_MAX) {
		return RPM_MAX;
	}

	return (uint16_t)rpm;
}

static int16_t clamp_temperature(int32_t temperature)
{
	if (temperature < TEMP_MIN) {
		return TEMP_MIN;
	}

	if (temperature > TEMP_MAX) {
		return TEMP_MAX;
	}

	return (int16_t)temperature;
}

static int console_keys_init(void)
{
	int ret;

	if (!device_is_ready(gpio0)) {
		printk("ERROR: GPIO0 device is not ready\n");
		return -ENODEV;
	}

	ret = gpio_pin_configure(
		gpio0,
		MATRIX_COL3_PIN,
		GPIO_OUTPUT_HIGH);

	if (ret < 0) {
		printk("ERROR: COL3 configuration failed: %d\n", ret);
		return ret;
	}

	for (size_t i = 0; i < CONSOLE_KEY_COUNT; i++) {
		ret = gpio_pin_configure(
			gpio0,
			console_keys[i].row_pin,
			GPIO_INPUT | GPIO_PULL_DOWN);

		if (ret < 0) {
			printk("ERROR: %s configuration failed: %d\n",
			       console_keys[i].name,
			       ret);
			return ret;
		}
	}

	printk("MATRIX ready: ENCODER_BUTTON / TEMP_MINUS / TEMP_PLUS\n");
	return 0;
}

static bool console_key_pressed_event(
	struct console_key *key,
	int64_t now_ms)
{
	int value = gpio_pin_get(gpio0, key->row_pin);
	bool pressed;

	if (value < 0) {
		return false;
	}

	pressed = value != 0;

	if (pressed != key->raw_pressed) {
		key->raw_pressed = pressed;
		key->raw_changed_ms = now_ms;
	}

	if ((pressed != key->stable_pressed) &&
	    ((now_ms - key->raw_changed_ms) >= KEY_DEBOUNCE_MS)) {
		key->stable_pressed = pressed;
		return pressed;
	}

	return false;
}

static int led_strip_show_rgb(uint8_t red, uint8_t green, uint8_t blue)
{
	for (size_t i = 0; i < LED_STRIP_NUM_PIXELS; i++) {
		led_strip_pixels[i].r = red;
		led_strip_pixels[i].g = green;
		led_strip_pixels[i].b = blue;
	}

	return led_strip_update_rgb(
		led_strip_dev,
		led_strip_pixels,
		LED_STRIP_NUM_PIXELS);
}

static int led_strip_render_state(enum vehicle_state state)
{
	switch (state) {
	case VEHICLE_NORMAL:
		return led_strip_show_rgb(
			0,
			STRIP_NORMAL_BRIGHTNESS,
			0);

	case VEHICLE_HIGH_RPM:
		return led_strip_show_rgb(
			STRIP_WARNING_BRIGHTNESS,
			STRIP_WARNING_BRIGHTNESS,
			0);

	case VEHICLE_OVER_TEMP:
		/* 橙色：红色为主，绿色约为三分之一。 */
		return led_strip_show_rgb(
			led_effect.breath_brightness,
			led_effect.breath_brightness / 3U,
			0);

	case VEHICLE_CRITICAL:
		if (led_effect.critical_on) {
			return led_strip_show_rgb(
				STRIP_CRITICAL_BRIGHTNESS,
				0,
				0);
		}

		return led_strip_show_rgb(0, 0, 0);

	default:
		return -EINVAL;
	}
}

static int led_strip_set_vehicle_state(
	enum vehicle_state state,
	int64_t now_ms)
{
	int ret;

	led_effect.vehicle_state = state;
	led_effect.last_tick_ms = now_ms;
	led_effect.breath_brightness = STRIP_BREATH_MIN;
	led_effect.breath_direction = 1;
	led_effect.critical_on = true;
	led_effect.initialized = true;

	ret = led_strip_render_state(state);
	if (ret == 0) {
		printk("LED EFFECT: %s\n", vehicle_state_to_string(state));
	}

	return ret;
}

static int led_strip_effect_tick(int64_t now_ms)
{
	int ret = 0;

	if (!led_effect.initialized) {
		return -EACCES;
	}

	switch (led_effect.vehicle_state) {
	case VEHICLE_NORMAL:
	case VEHICLE_HIGH_RPM:
		/* 常亮状态无需周期刷新。 */
		break;

	case VEHICLE_OVER_TEMP:
		if ((now_ms - led_effect.last_tick_ms) >=
		    STRIP_BREATH_TICK_MS) {
			int16_t next_brightness =
				(int16_t)led_effect.breath_brightness +
				((int16_t)led_effect.breath_direction *
				 (int16_t)STRIP_BREATH_STEP);

			if (next_brightness >=
			    (int16_t)STRIP_BREATH_MAX) {
				next_brightness = STRIP_BREATH_MAX;
				led_effect.breath_direction = -1;
			} else if (next_brightness <=
				   (int16_t)STRIP_BREATH_MIN) {
				next_brightness = STRIP_BREATH_MIN;
				led_effect.breath_direction = 1;
			}

			led_effect.breath_brightness =
				(uint8_t)next_brightness;
			ret = led_strip_render_state(VEHICLE_OVER_TEMP);
			led_effect.last_tick_ms = now_ms;
		}
		break;

	case VEHICLE_CRITICAL:
		if ((now_ms - led_effect.last_tick_ms) >=
		    STRIP_CRITICAL_BLINK_MS) {
			led_effect.critical_on = !led_effect.critical_on;
			ret = led_strip_render_state(VEHICLE_CRITICAL);
			led_effect.last_tick_ms = now_ms;
		}
		break;

	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static int led_strip_effect_init(enum vehicle_state initial_state)
{
	int ret;

	if (!device_is_ready(led_strip_dev)) {
		printk("ERROR: LED strip device is not ready\n");
		return -ENODEV;
	}

	if (!gpio_is_ready_dt(&led_strip_en)) {
		printk("ERROR: LED strip power GPIO is not ready\n");
		return -ENODEV;
	}

	ret = gpio_pin_configure_dt(&led_strip_en, GPIO_OUTPUT_INACTIVE);
	if (ret < 0) {
		printk("ERROR: LED strip power GPIO configuration failed: %d\n",
		       ret);
		return ret;
	}

	memset(led_strip_pixels, 0, sizeof(led_strip_pixels));

	ret = gpio_pin_set_dt(&led_strip_en, 1);
	if (ret < 0) {
		printk("ERROR: LED strip power enable failed: %d\n", ret);
		return ret;
	}

	/* 给灯带电源留出稳定时间。 */
	k_sleep(K_MSEC(5));

	memset(&led_effect, 0, sizeof(led_effect));
	ret = led_strip_set_vehicle_state(
		initial_state,
		k_uptime_get());
	if (ret < 0) {
		printk("ERROR: Initial LED effect update failed: %d\n", ret);
		return ret;
	}

	printk("LED STRIP ready: %u pixels\n",
	       (unsigned int)LED_STRIP_NUM_PIXELS);
	return 0;
}

static void dashboard_style_panel(lv_obj_t *panel)
{
	lv_obj_set_style_bg_color(
		panel, lv_color_hex(0x172033), LV_PART_MAIN);
	lv_obj_set_style_bg_opa(panel, LV_OPA_COVER, LV_PART_MAIN);
	lv_obj_set_style_border_color(
		panel, lv_color_hex(0x34415C), LV_PART_MAIN);
	lv_obj_set_style_border_width(panel, 1, LV_PART_MAIN);
	lv_obj_set_style_radius(panel, 8, LV_PART_MAIN);
	lv_obj_set_style_pad_all(panel, 0, LV_PART_MAIN);
	lv_obj_remove_flag(panel, LV_OBJ_FLAG_SCROLLABLE);
}

static lv_obj_t *dashboard_create_caption(
	lv_obj_t *parent,
	const char *text)
{
	lv_obj_t *label = lv_label_create(parent);

	lv_label_set_text(label, text);
	lv_obj_set_style_text_color(
		label, lv_color_hex(0x8FA2BF), LV_PART_MAIN);
	lv_obj_set_style_text_font(
		label, &lv_font_montserrat_14, LV_PART_MAIN);
	lv_obj_set_pos(label, 10, 7);

	return label;
}

static void dashboard_update(const struct vehicle_data *vehicle)
{
	char rpm_text[24];
	char temp_text[24];
	char fault_text[24];
	lv_color_t state_color;
	lv_color_t state_text_color;

	snprintf(rpm_text, sizeof(rpm_text), "%u RPM",
		 (unsigned int)vehicle->rpm);
	snprintf(temp_text, sizeof(temp_text), "%d C",
		 (int)vehicle->temperature_c);
	snprintf(fault_text, sizeof(fault_text), "FAULT: %s",
		 vehicle->fault_active ? "ON" : "OFF");

	lv_label_set_text(dashboard.rpm_value, rpm_text);
	lv_label_set_text(dashboard.temp_value, temp_text);
	lv_label_set_text(
		dashboard.state_value,
		vehicle_state_to_string(vehicle->state));
	lv_label_set_text(dashboard.fault_value, fault_text);

	switch (vehicle->state) {
	case VEHICLE_NORMAL:
		state_color = lv_color_hex(0x159A55);
		state_text_color = lv_color_hex(0xFFFFFF);
		break;
	case VEHICLE_HIGH_RPM:
		state_color = lv_color_hex(0xF2C230);
		state_text_color = lv_color_hex(0x111827);
		break;
	case VEHICLE_OVER_TEMP:
		state_color = lv_color_hex(0xF07A28);
		state_text_color = lv_color_hex(0x111827);
		break;
	case VEHICLE_CRITICAL:
	default:
		state_color = lv_color_hex(0xD9363E);
		state_text_color = lv_color_hex(0xFFFFFF);
		break;
	}

	lv_obj_set_style_bg_color(
		dashboard.state_panel, state_color, LV_PART_MAIN);
	lv_obj_set_style_text_color(
		dashboard.state_value, state_text_color, LV_PART_MAIN);
	lv_obj_set_style_text_color(
		dashboard.fault_value, state_text_color, LV_PART_MAIN);
}

static int dashboard_init(const struct vehicle_data *vehicle)
{
	struct display_capabilities capabilities;
	lv_obj_t *screen;
	lv_obj_t *title;
	lv_obj_t *live;
	lv_obj_t *rpm_panel;
	lv_obj_t *temp_panel;
	int ret;

	if (!device_is_ready(display_dev)) {
		printk("ERROR: Display device is not ready\n");
		return -ENODEV;
	}

	if (!device_is_ready(backlight_dev)) {
		printk("ERROR: Backlight device is not ready\n");
		return -ENODEV;
	}

	display_get_capabilities(display_dev, &capabilities);
	printk("DISPLAY ready: %u x %u | pixel format: 0x%x\n",
	       capabilities.x_resolution,
	       capabilities.y_resolution,
	       capabilities.current_pixel_format);

	ret = led_set_brightness(backlight_dev, backlight_index, 0);
	if (ret < 0) {
		printk("ERROR: Backlight off failed: %d\n", ret);
		return ret;
	}

	screen = lv_screen_active();
	lv_obj_remove_flag(screen, LV_OBJ_FLAG_SCROLLABLE);
	lv_obj_set_style_bg_color(
		screen, lv_color_hex(0x090E18), LV_PART_MAIN);
	lv_obj_set_style_bg_opa(screen, LV_OPA_COVER, LV_PART_MAIN);
	lv_obj_set_style_pad_all(screen, 0, LV_PART_MAIN);

	title = lv_label_create(screen);
	lv_label_set_text(title, "WIRELESS ECU");
	lv_obj_set_style_text_color(
		title, lv_color_hex(0xF2F6FF), LV_PART_MAIN);
	lv_obj_set_style_text_font(
		title, &lv_font_montserrat_20, LV_PART_MAIN);
	lv_obj_set_pos(title, 8, 4);

	live = lv_label_create(screen);
	lv_label_set_text(live, "LIVE");
	lv_obj_set_style_text_color(
		live, lv_color_hex(0x4ADE80), LV_PART_MAIN);
	lv_obj_set_style_text_font(
		live, &lv_font_montserrat_14, LV_PART_MAIN);
	lv_obj_align(live, LV_ALIGN_TOP_RIGHT, -9, 8);

	rpm_panel = lv_obj_create(screen);
	lv_obj_set_size(rpm_panel, 150, 78);
	lv_obj_set_pos(rpm_panel, 6, 34);
	dashboard_style_panel(rpm_panel);
	dashboard_create_caption(rpm_panel, "ENGINE SPEED");

	dashboard.rpm_value = lv_label_create(rpm_panel);
	lv_obj_set_style_text_color(
		dashboard.rpm_value,
		lv_color_hex(0xF2F6FF),
		LV_PART_MAIN);
	lv_obj_set_style_text_font(
		dashboard.rpm_value,
		&lv_font_montserrat_28,
		LV_PART_MAIN);
	lv_obj_align(dashboard.rpm_value, LV_ALIGN_BOTTOM_MID, 0, -10);

	temp_panel = lv_obj_create(screen);
	lv_obj_set_size(temp_panel, 152, 78);
	lv_obj_set_pos(temp_panel, 162, 34);
	dashboard_style_panel(temp_panel);
	dashboard_create_caption(temp_panel, "TEMPERATURE");

	dashboard.temp_value = lv_label_create(temp_panel);
	lv_obj_set_style_text_color(
		dashboard.temp_value,
		lv_color_hex(0xF2F6FF),
		LV_PART_MAIN);
	lv_obj_set_style_text_font(
		dashboard.temp_value,
		&lv_font_montserrat_28,
		LV_PART_MAIN);
	lv_obj_align(dashboard.temp_value, LV_ALIGN_BOTTOM_MID, 0, -10);

	dashboard.state_panel = lv_obj_create(screen);
	lv_obj_set_size(dashboard.state_panel, 308, 48);
	lv_obj_set_pos(dashboard.state_panel, 6, 118);
	lv_obj_set_style_border_width(
		dashboard.state_panel, 0, LV_PART_MAIN);
	lv_obj_set_style_radius(
		dashboard.state_panel, 8, LV_PART_MAIN);
	lv_obj_set_style_pad_all(
		dashboard.state_panel, 0, LV_PART_MAIN);
	lv_obj_remove_flag(
		dashboard.state_panel, LV_OBJ_FLAG_SCROLLABLE);

	dashboard.state_value = lv_label_create(dashboard.state_panel);
	lv_obj_set_style_text_font(
		dashboard.state_value,
		&lv_font_montserrat_20,
		LV_PART_MAIN);
	lv_obj_align(dashboard.state_value, LV_ALIGN_LEFT_MID, 12, 0);

	dashboard.fault_value = lv_label_create(dashboard.state_panel);
	lv_obj_set_style_text_font(
		dashboard.fault_value,
		&lv_font_montserrat_14,
		LV_PART_MAIN);
	lv_obj_align(dashboard.fault_value, LV_ALIGN_RIGHT_MID, -12, 0);

	dashboard_update(vehicle);
	lv_timer_handler();

	ret = display_blanking_off(display_dev);
	if (ret < 0) {
		printk("ERROR: Display blanking off failed: %d\n", ret);
		return ret;
	}

	ret = led_set_brightness(backlight_dev, backlight_index, 100);
	if (ret < 0) {
		printk("ERROR: Backlight on failed: %d\n", ret);
		return ret;
	}

	printk("DASHBOARD ready\n");
	return 0;
}

static bool handle_console_key(
	enum console_key_id key_id,
	struct vehicle_data *vehicle)
{
	switch (key_id) {
	case CONSOLE_KEY_ENCODER:
		vehicle->fault_active = !vehicle->fault_active;
		break;
	case CONSOLE_KEY_TEMP_MINUS:
		vehicle->temperature_c =
			clamp_temperature(
				(int32_t)vehicle->temperature_c - TEMP_STEP);
		break;
	case CONSOLE_KEY_TEMP_PLUS:
		vehicle->temperature_c =
			clamp_temperature(
				(int32_t)vehicle->temperature_c + TEMP_STEP);
		break;
	default:
		return false;
	}

	vehicle->state = evaluate_vehicle_state(vehicle);
	print_vehicle_status(console_keys[key_id].name, vehicle);
	return true;
}

static bool protocol_send(
	const char *reason,
	const struct vehicle_data *vehicle,
	uint16_t *sequence)
{
	struct vehicle_status_message message;
	uint8_t frame[VEHICLE_STATUS_FRAME_SIZE];
	size_t frame_length;
	uint16_t sent_sequence;
	int cdc_result;
	int ble_result;

	if ((reason == NULL) || (vehicle == NULL) || (sequence == NULL)) {
		return false;
	}

	sent_sequence = *sequence;
	message.sequence = sent_sequence;
	message.rpm = vehicle->rpm;
	message.temperature_c = vehicle->temperature_c;
	message.state = (uint8_t)vehicle->state;
	message.fault_active = vehicle->fault_active;

	frame_length = vehicle_protocol_encode_status(
		frame,
		sizeof(frame),
		&message);
	if (frame_length == 0U) {
		printk("ERROR: Vehicle protocol encoding failed\n");
		return false;
	}

	cdc_result = usb_cdc_transport_send(frame, frame_length);
	ble_result = ble_nus_transport_send(frame, frame_length);

	printk("PROTO TX %-9s SEQ:%u USB:",
	       reason,
	       (unsigned int)sent_sequence);

	if (cdc_result == 0) {
		printk("SENT ");
	} else if (cdc_result == -ENOTCONN) {
		printk("WAIT ");
	} else {
		printk("ERROR(%d) ", cdc_result);
	}

	printk("BLE:");
	if (ble_result == 0) {
		printk("SENT FRAME:");
	} else if (ble_result == -ENOTCONN) {
		printk("WAIT FRAME:");
	} else {
		printk("ERROR(%d) FRAME:", ble_result);
	}

	for (size_t i = 0; i < frame_length; i++) {
		printk(" %02X", frame[i]);
	}

	printk("\n");
	(*sequence)++;
	return true;
}

int main(void)
{
	struct vehicle_data vehicle = {
		.rpm = 800,
		.temperature_c = 35,
		.fault_active = false,
		.state = VEHICLE_NORMAL
	};
	int64_t angle_remainder_udeg = 0;
	int64_t last_heartbeat_ms;
	int64_t last_protocol_tx_ms;
	uint16_t protocol_sequence = 0U;
	int ret;

	if (!gpio_is_ready_dt(&led)) {
		printk("ERROR: LED device is not ready\n");
		return 0;
	}

	ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_INACTIVE);
	if (ret < 0) {
		printk("ERROR: LED configuration failed: %d\n", ret);
		return 0;
	}

	if (!device_is_ready(qdec)) {
		printk("ERROR: QDEC device is not ready\n");
		return 0;
	}

	ret = console_keys_init();
	if (ret < 0) {
		return 0;
	}

	vehicle.state = evaluate_vehicle_state(&vehicle);

	ret = dashboard_init(&vehicle);
	if (ret < 0) {
		return 0;
	}

	ret = led_strip_effect_init(vehicle.state);
	if (ret < 0) {
		return 0;
	}

	ret = usb_cdc_transport_init();
	if (ret < 0) {
		return 0;
	}

	ret = ble_nus_transport_init();
	if (ret < 0) {
		return 0;
	}

	last_heartbeat_ms = k_uptime_get();

	printk("\nWireless ECU Console started\n");
	printk("QDEC ready: P0.10 / P1.06\n");
	print_vehicle_status("START", &vehicle);
	(void)protocol_send(
		"START",
		&vehicle,
		&protocol_sequence);
	last_protocol_tx_ms = k_uptime_get();

	while (1) {
		struct sensor_value rotation;
		int64_t now_ms;
		bool vehicle_changed = false;

		ret = sensor_sample_fetch(qdec);
		if (ret == 0) {
			ret = sensor_channel_get(
				qdec,
				SENSOR_CHAN_ROTATION,
				&rotation);
		}

		if (ret == 0) {
			int64_t total_udeg =
				angle_remainder_udeg +
				sensor_value_to_udeg(&rotation);
			int32_t detents =
				(int32_t)(total_udeg / DETENT_ANGLE_UDEG);

			angle_remainder_udeg =
				total_udeg -
				((int64_t)detents * DETENT_ANGLE_UDEG);

			if (detents != 0) {
				int32_t new_rpm =
					(int32_t)vehicle.rpm +
					(detents * RPM_STEP);

				vehicle.rpm = clamp_rpm(new_rpm);
				vehicle.state =
					evaluate_vehicle_state(&vehicle);
				vehicle_changed = true;

				printk("ENCODER: %d | RPM: %u | "
				       "TEMP: %d C | FAULT: %d | STATE: %s\n",
				       detents,
				       (unsigned int)vehicle.rpm,
				       (int)vehicle.temperature_c,
				       vehicle.fault_active ? 1 : 0,
				       vehicle_state_to_string(vehicle.state));
			}
		}

		now_ms = k_uptime_get();

		for (size_t i = 0; i < CONSOLE_KEY_COUNT; i++) {
			if (console_key_pressed_event(
				    &console_keys[i], now_ms)) {
				vehicle_changed |= handle_console_key(
					(enum console_key_id)i,
					&vehicle);
			}
		}

		if (vehicle_changed) {
			dashboard_update(&vehicle);

			if (vehicle.state != led_effect.vehicle_state) {
				ret = led_strip_set_vehicle_state(
					vehicle.state,
					now_ms);
				if (ret < 0) {
					printk("ERROR: LED effect state update failed: %d\n",
					       ret);
				}
			}

			if (protocol_send(
				    "CHANGE",
				    &vehicle,
				    &protocol_sequence)) {
				last_protocol_tx_ms = now_ms;
			}
		}

		ret = led_strip_effect_tick(now_ms);
		if (ret < 0) {
			printk("ERROR: LED effect tick failed: %d\n", ret);
		}

		if ((now_ms - last_protocol_tx_ms) >=
		    PROTOCOL_HEARTBEAT_MS) {
			if (protocol_send(
				    "HEARTBEAT",
				    &vehicle,
				    &protocol_sequence)) {
				last_protocol_tx_ms = now_ms;
			}
		}

		lv_timer_handler();

		if ((now_ms - last_heartbeat_ms) >=
		    HEARTBEAT_PERIOD_MS) {
			gpio_pin_toggle_dt(&led);
			last_heartbeat_ms = now_ms;
		}

		k_sleep(K_MSEC(MAIN_LOOP_PERIOD_MS));
	}

	return 0;
}
