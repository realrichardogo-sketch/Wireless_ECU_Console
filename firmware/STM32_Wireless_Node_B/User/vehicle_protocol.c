#include "vehicle_protocol.h"

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
