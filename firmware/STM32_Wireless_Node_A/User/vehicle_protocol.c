#include "vehicle_protocol.h"

#define FRAME_HEADER_0       0U
#define FRAME_HEADER_1       1U
#define FRAME_VERSION        2U
#define FRAME_TYPE           3U
#define FRAME_SEQ_LO         4U
#define FRAME_SEQ_HI         5U
#define FRAME_LENGTH         6U
#define FRAME_RPM_LO         7U
#define FRAME_RPM_HI         8U
#define FRAME_TEMP_LO        9U
#define FRAME_TEMP_HI        10U
#define FRAME_STATE          11U
#define FRAME_FLAGS          12U
#define FRAME_FLAG_FAULT_ACTIVE  0x01U
#define FRAME_CRC_LO         13U
#define FRAME_CRC_HI         14U

uint16_t vehicle_crc16_ccitt_false(const uint8_t *data, uint16_t length)
{
    uint16_t crc = 0xFFFFU;
    uint16_t i;
    uint8_t bit;

    for (i = 0U; i < length; ++i) {
        crc ^= (uint16_t)data[i] << 8;
        for (bit = 0U; bit < 8U; ++bit) {
            if ((crc & 0x8000U) != 0U) {
                crc = (uint16_t)((crc << 1) ^ 0x1021U);
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

void vehicle_parser_init(VehicleParser *parser)
{
    parser->index = 0U;
}

static VehicleParseResult vehicle_decode(const uint8_t *frame,
                                         VehicleData *data)
{
    uint16_t received_crc;
    uint16_t calculated_crc;

    if ((frame[FRAME_HEADER_0] != 0x55U) ||
        (frame[FRAME_HEADER_1] != 0xAAU) ||
        (frame[FRAME_VERSION] != VEHICLE_PROTOCOL_VERSION) ||
        (frame[FRAME_TYPE] != VEHICLE_MESSAGE_STATE) ||
        (frame[FRAME_LENGTH] != VEHICLE_PAYLOAD_SIZE) ||
        (frame[FRAME_STATE] > (uint8_t)VEHICLE_CRITICAL)) {
        return VEHICLE_PARSE_FORMAT_ERROR;
    }

    received_crc = (uint16_t)frame[FRAME_CRC_LO] |
                   ((uint16_t)frame[FRAME_CRC_HI] << 8);

    /* The nRF52840 protocol calculates CRC over bytes 2..12. */
    calculated_crc = vehicle_crc16_ccitt_false(&frame[FRAME_VERSION], 11U);
    if (received_crc != calculated_crc) {
        return VEHICLE_PARSE_CRC_ERROR;
    }

    data->sequence = (uint16_t)frame[FRAME_SEQ_LO] |
                     ((uint16_t)frame[FRAME_SEQ_HI] << 8);
    data->rpm = (uint16_t)frame[FRAME_RPM_LO] |
                ((uint16_t)frame[FRAME_RPM_HI] << 8);
    data->temperature_c = (int16_t)((uint16_t)frame[FRAME_TEMP_LO] |
                          ((uint16_t)frame[FRAME_TEMP_HI] << 8));
    data->state = frame[FRAME_STATE];
    data->fault =
        ((frame[FRAME_FLAGS] & FRAME_FLAG_FAULT_ACTIVE) != 0U) ? 1U : 0U;
    return VEHICLE_PARSE_OK;
}

VehicleParseResult vehicle_parser_feed(VehicleParser *parser,
                                       uint8_t byte,
                                       VehicleData *data)
{
    VehicleParseResult result;

    if (parser->index == 0U) {
        if (byte == 0x55U) {
            parser->bytes[0] = byte;
            parser->index = 1U;
        }
        return VEHICLE_PARSE_NONE;
    }

    if (parser->index == 1U) {
        if (byte == 0xAAU) {
            parser->bytes[1] = byte;
            parser->index = 2U;
        } else if (byte == 0x55U) {
            parser->bytes[0] = byte;
            parser->index = 1U;
        } else {
            parser->index = 0U;
        }
        return VEHICLE_PARSE_NONE;
    }

    parser->bytes[parser->index] = byte;
    ++parser->index;

    if (parser->index < VEHICLE_FRAME_SIZE) {
        return VEHICLE_PARSE_NONE;
    }

    result = vehicle_decode(parser->bytes, data);
    parser->index = 0U;
    return result;
}

const char *vehicle_state_name(uint8_t state)
{
    switch (state) {
    case VEHICLE_NORMAL:
        return "NORMAL";
    case VEHICLE_HIGH_RPM:
        return "HIGH_RPM";
    case VEHICLE_OVER_TEMP:
        return "OVER_TEMP";
    case VEHICLE_CRITICAL:
        return "CRITICAL";
    default:
        return "INVALID";
    }
}
