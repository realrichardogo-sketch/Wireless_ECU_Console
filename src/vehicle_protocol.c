#include "vehicle_protocol.h"

enum vehicle_status_frame_offset {
	FRAME_OFFSET_SOF0 = 0,
	FRAME_OFFSET_SOF1 = 1,
	FRAME_OFFSET_VERSION = 2,
	FRAME_OFFSET_TYPE = 3,
	FRAME_OFFSET_SEQUENCE = 4,
	FRAME_OFFSET_PAYLOAD_LENGTH = 6,
	FRAME_OFFSET_RPM = 7,
	FRAME_OFFSET_TEMPERATURE = 9,
	FRAME_OFFSET_STATE = 11,
	FRAME_OFFSET_FLAGS = 12,
	FRAME_OFFSET_CRC = 13,
};

static void put_u16_le(uint8_t *destination, uint16_t value)
{
	destination[0] = (uint8_t)(value & 0xFFU);
	destination[1] = (uint8_t)((value >> 8) & 0xFFU);
}

uint16_t vehicle_protocol_crc16_ccitt_false(
	const uint8_t *data,
	size_t length)
{
	uint16_t crc = 0xFFFFU;

	if (data == NULL) {
		return 0U;
	}

	for (size_t i = 0; i < length; i++) {
		crc ^= (uint16_t)data[i] << 8;

		for (uint8_t bit = 0; bit < 8U; bit++) {
			if ((crc & 0x8000U) != 0U) {
				crc = (uint16_t)((crc << 1) ^ 0x1021U);
			} else {
				crc <<= 1;
			}
		}
	}

	return crc;
}

size_t vehicle_protocol_encode_status(
	uint8_t *frame,
	size_t frame_capacity,
	const struct vehicle_status_message *message)
{
	uint16_t crc;
	uint16_t temperature_bits;

	if ((frame == NULL) || (message == NULL) ||
	    (frame_capacity < VEHICLE_STATUS_FRAME_SIZE)) {
		return 0U;
	}

	frame[FRAME_OFFSET_SOF0] = VEHICLE_PROTOCOL_SOF0;
	frame[FRAME_OFFSET_SOF1] = VEHICLE_PROTOCOL_SOF1;
	frame[FRAME_OFFSET_VERSION] = VEHICLE_PROTOCOL_VERSION;
	frame[FRAME_OFFSET_TYPE] = VEHICLE_MSG_STATUS;
	put_u16_le(&frame[FRAME_OFFSET_SEQUENCE], message->sequence);
	frame[FRAME_OFFSET_PAYLOAD_LENGTH] = VEHICLE_STATUS_PAYLOAD_SIZE;
	put_u16_le(&frame[FRAME_OFFSET_RPM], message->rpm);

	temperature_bits = (uint16_t)message->temperature_c;
	put_u16_le(&frame[FRAME_OFFSET_TEMPERATURE], temperature_bits);

	frame[FRAME_OFFSET_STATE] = message->state;
	frame[FRAME_OFFSET_FLAGS] =
		message->fault_active ?
			VEHICLE_STATUS_FLAG_FAULT_ACTIVE : 0U;

	/* CRC覆盖Version到Payload末尾，不覆盖SOF和CRC自身。 */
	crc = vehicle_protocol_crc16_ccitt_false(
		&frame[FRAME_OFFSET_VERSION],
		FRAME_OFFSET_CRC - FRAME_OFFSET_VERSION);
	put_u16_le(&frame[FRAME_OFFSET_CRC], crc);

	return VEHICLE_STATUS_FRAME_SIZE;
}
