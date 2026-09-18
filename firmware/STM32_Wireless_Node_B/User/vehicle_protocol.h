#ifndef VEHICLE_PROTOCOL_H
#define VEHICLE_PROTOCOL_H

#include <stdint.h>

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

const char *vehicle_state_name(uint8_t state);

#endif
