#ifndef VEHICLE_CAN_H
#define VEHICLE_CAN_H

#include "vehicle_protocol.h"
#include <stdint.h>

void vehicle_can_init(void);
uint8_t vehicle_can_send(const VehicleData *data);

#endif
