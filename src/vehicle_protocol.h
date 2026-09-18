#ifndef VEHICLE_PROTOCOL_H_
#define VEHICLE_PROTOCOL_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define VEHICLE_PROTOCOL_SOF0              0x55U
#define VEHICLE_PROTOCOL_SOF1              0xAAU
#define VEHICLE_PROTOCOL_VERSION           0x01U

#define VEHICLE_MSG_STATUS                 0x10U
#define VEHICLE_STATUS_PAYLOAD_SIZE        6U
#define VEHICLE_STATUS_FRAME_SIZE         15U

#define VEHICLE_STATUS_FLAG_FAULT_ACTIVE   0x01U

struct vehicle_status_message {
	uint16_t sequence;
	uint16_t rpm;
	int16_t temperature_c;
	uint8_t state;
	bool fault_active;
};

uint16_t vehicle_protocol_crc16_ccitt_false(
	const uint8_t *data,
	size_t length);

size_t vehicle_protocol_encode_status(
	uint8_t *frame,
	size_t frame_capacity,
	const struct vehicle_status_message *message);

#ifdef __cplusplus
}
#endif

#endif /* VEHICLE_PROTOCOL_H_ */
