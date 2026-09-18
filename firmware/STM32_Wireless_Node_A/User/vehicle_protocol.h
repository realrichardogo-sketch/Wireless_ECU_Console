#ifndef VEHICLE_PROTOCOL_H
#define VEHICLE_PROTOCOL_H

#include <stdint.h>

#define VEHICLE_FRAME_SIZE          15U
#define VEHICLE_PROTOCOL_VERSION    0x01U
#define VEHICLE_MESSAGE_STATE       0x10U
#define VEHICLE_PAYLOAD_SIZE        0x06U

typedef enum {
    VEHICLE_NORMAL = 0,
    VEHICLE_HIGH_RPM = 1,
    VEHICLE_OVER_TEMP = 2,
    VEHICLE_CRITICAL = 3
} VehicleState;

typedef struct {
    uint16_t sequence;
    uint16_t rpm;
    int16_t temperature_c;
    uint8_t fault;
    uint8_t state;
} VehicleData;

typedef enum {
    VEHICLE_PARSE_NONE = 0,
    VEHICLE_PARSE_OK,
    VEHICLE_PARSE_FORMAT_ERROR,
    VEHICLE_PARSE_CRC_ERROR
} VehicleParseResult;

typedef struct {
    uint8_t bytes[VEHICLE_FRAME_SIZE];
    uint8_t index;
} VehicleParser;

void vehicle_parser_init(VehicleParser *parser);
VehicleParseResult vehicle_parser_feed(VehicleParser *parser,
                                       uint8_t byte,
                                       VehicleData *data);
uint16_t vehicle_crc16_ccitt_false(const uint8_t *data, uint16_t length);
const char *vehicle_state_name(uint8_t state);

#endif
