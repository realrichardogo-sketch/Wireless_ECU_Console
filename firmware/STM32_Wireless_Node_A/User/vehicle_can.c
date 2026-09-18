#include "vehicle_can.h"

#include "board_config.h"
#include "stm32f10x.h"

void vehicle_can_init(void)
{
    GPIO_InitTypeDef gpio;
    CAN_InitTypeDef can;

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
}

uint8_t vehicle_can_send(const VehicleData *data)
{
    CanTxMsg message;
    uint8_t mailbox;
    uint16_t temperature_bits;

    temperature_bits = (uint16_t)data->temperature_c;
    message.StdId = VEHICLE_CAN_ID;
    message.ExtId = 0U;
    message.IDE = CAN_Id_Standard;
    message.RTR = CAN_RTR_Data;
    message.DLC = 8U;
    message.Data[0] = (uint8_t)(data->rpm & 0xFFU);
    message.Data[1] = (uint8_t)(data->rpm >> 8);
    message.Data[2] = (uint8_t)(temperature_bits & 0xFFU);
    message.Data[3] = (uint8_t)(temperature_bits >> 8);
    message.Data[4] = data->state;
    message.Data[5] = data->fault;
    message.Data[6] = (uint8_t)(data->sequence & 0xFFU);
    message.Data[7] = (uint8_t)(data->sequence >> 8);

    mailbox = CAN_Transmit(CAN1, &message);
    return (mailbox != CAN_TxStatus_NoMailBox) ? 1U : 0U;
}
