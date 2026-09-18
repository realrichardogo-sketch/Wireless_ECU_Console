#include "vehicle_can.h"

#include "board_config.h"
#include "stm32f10x.h"

void vehicle_can_init(void)
{
    GPIO_InitTypeDef gpio;
    CAN_InitTypeDef can;
    CAN_FilterInitTypeDef filter;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO,
                           ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);

    GPIO_PinRemapConfig(GPIO_Remap1_CAN1, ENABLE);

    gpio.GPIO_Pin = GPIO_Pin_9;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOB, &gpio);

    gpio.GPIO_Pin = GPIO_Pin_8;
    gpio.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOB, &gpio);

    CAN_DeInit(CAN1);
    CAN_StructInit(&can);
    can.CAN_TTCM = DISABLE;
    can.CAN_ABOM = ENABLE;
    can.CAN_AWUM = DISABLE;
    can.CAN_NART = DISABLE;
    can.CAN_RFLM = DISABLE;
    can.CAN_TXFP = DISABLE;
    can.CAN_Mode = CAN_Mode_Normal;
    can.CAN_SJW = CAN_SJW_1tq;
    can.CAN_BS1 = CAN_BS1_12tq;
    can.CAN_BS2 = CAN_BS2_5tq;
    can.CAN_Prescaler = 4U;
    CAN_Init(CAN1, &can);

    filter.CAN_FilterNumber = 0U;
    filter.CAN_FilterMode = CAN_FilterMode_IdMask;
    filter.CAN_FilterScale = CAN_FilterScale_32bit;
    filter.CAN_FilterIdHigh = (uint16_t)(VEHICLE_CAN_ID << 5);
    filter.CAN_FilterIdLow = 0U;
    filter.CAN_FilterMaskIdHigh = 0xFFE8U;
    filter.CAN_FilterMaskIdLow = 0U;
    filter.CAN_FilterFIFOAssignment = CAN_FIFO0;
    filter.CAN_FilterActivation = ENABLE;
    CAN_FilterInit(&filter);
}

uint8_t vehicle_can_decode(const uint8_t data[8], VehicleData *vehicle)
{
    uint16_t temperature_bits;

    vehicle->rpm = (uint16_t)data[0] | ((uint16_t)data[1] << 8);
    temperature_bits = (uint16_t)data[2] | ((uint16_t)data[3] << 8);
    vehicle->temperature_c = (int16_t)temperature_bits;
    vehicle->state = data[4];
    vehicle->fault = data[5];
    vehicle->sequence = (uint16_t)data[6] | ((uint16_t)data[7] << 8);

    if ((vehicle->state > (uint8_t)VEHICLE_CRITICAL) ||
        (vehicle->fault > 1U)) {
        return 0U;
    }
    return 1U;
}
