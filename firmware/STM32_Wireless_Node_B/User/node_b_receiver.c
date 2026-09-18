#include "node_b_receiver.h"

#include "board_config.h"
#include "debug_uart.h"
#include "stm32f10x.h"
#include "vehicle_can.h"
#include "vehicle_protocol.h"

typedef struct {
    uint8_t data[8];
    uint8_t pending;
} CanRxSlot;

static volatile uint32_t system_ms;
static volatile CanRxSlot rx_slot;
static volatile uint32_t rx_overflow;
static volatile uint32_t wrong_can_frames;

static uint32_t last_valid_ms;
static uint16_t last_sequence;
static uint8_t have_sequence;
static uint8_t can_timed_out;
static uint32_t valid_frames;
static uint32_t lost_frames;

static void leds_init(void)
{
    GPIO_InitTypeDef gpio;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOG,
                           ENABLE);

    gpio.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_8;
    gpio.GPIO_Speed = GPIO_Speed_2MHz;
    gpio.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOA, &gpio);

    gpio.GPIO_Pin = GPIO_Pin_11;
    GPIO_Init(GPIOG, &gpio);

    GPIO_SetBits(GPIOA, GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_8);
    GPIO_SetBits(GPIOG, GPIO_Pin_11);
}

static void leds_all_off(void)
{
    GPIO_SetBits(GPIOA, GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_8);
    GPIO_SetBits(GPIOG, GPIO_Pin_11);
}

static void leds_show_state(uint8_t state, uint8_t fault)
{
    leds_all_off();

    if ((fault != 0U) || (state == (uint8_t)VEHICLE_CRITICAL)) {
        GPIO_ResetBits(GPIOG, GPIO_Pin_11); /* LED-0 red */
    } else if (state == (uint8_t)VEHICLE_NORMAL) {
        GPIO_ResetBits(GPIOA, GPIO_Pin_0);  /* LED-1 yellow */
    } else if (state == (uint8_t)VEHICLE_HIGH_RPM) {
        GPIO_ResetBits(GPIOA, GPIO_Pin_1);  /* LED-2 blue */
    } else if (state == (uint8_t)VEHICLE_OVER_TEMP) {
        GPIO_ResetBits(GPIOA, GPIO_Pin_8);  /* LED-3 green */
    }
}

static void leds_show_timeout(void)
{
    leds_all_off();

    /* Alternate red/yellow every 512 ms without a blocking delay. */
    if ((system_ms & 0x200U) == 0U) {
        GPIO_ResetBits(GPIOG, GPIO_Pin_11);
    } else {
        GPIO_ResetBits(GPIOA, GPIO_Pin_0);
    }
}

static void can_rx_interrupt_init(void)
{
    NVIC_InitTypeDef nvic;

    CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);
    nvic.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 1U;
    nvic.NVIC_IRQChannelSubPriority = 0U;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);
}

static uint8_t take_can_frame(uint8_t data[8])
{
    uint8_t i;

    __disable_irq();
    if (rx_slot.pending == 0U) {
        __enable_irq();
        return 0U;
    }

    for (i = 0U; i < 8U; ++i) {
        data[i] = rx_slot.data[i];
    }
    rx_slot.pending = 0U;
    __enable_irq();
    return 1U;
}

static void update_sequence_statistics(uint16_t sequence)
{
    uint16_t delta;

    if (have_sequence != 0U) {
        delta = (uint16_t)(sequence - last_sequence);
        if ((delta > 1U) && (delta < 0x8000U)) {
            lost_frames += (uint32_t)(delta - 1U);
        }
    }

    last_sequence = sequence;
    have_sequence = 1U;
}

static void handle_vehicle_data(const VehicleData *vehicle)
{
    ++valid_frames;
    update_sequence_statistics(vehicle->sequence);
    last_valid_ms = system_ms;

    if (can_timed_out != 0U) {
        can_timed_out = 0U;
        debug_printf("CAN RECOVERED\r\n");
    }

    leds_show_state(vehicle->state, vehicle->fault);
    debug_printf("CAN RX OK ID=0x201 SEQ=%u RPM=%u TEMP=%d STATE=%s FAULT=%u\r\n",
                 vehicle->sequence,
                 vehicle->rpm,
                 vehicle->temperature_c,
                 vehicle_state_name(vehicle->state),
                 vehicle->fault);
}

void node_b_init(void)
{
    leds_init();
    debug_uart_init();
    vehicle_can_init();
    can_rx_interrupt_init();

    last_valid_ms = system_ms;
    debug_printf("\r\nSTM32 Node B receiver started\r\n");
    debug_printf("DEBUG USART1: PA9 TX / PA10 RX, 115200 8N1\r\n");
    debug_printf("CAN1: PB8 RX / PB9 TX, ID=0x201, 500 kbit/s\r\n");
}

void node_b_process(void)
{
    uint8_t data[8];
    VehicleData vehicle;

    while (take_can_frame(data) != 0U) {
        if (vehicle_can_decode(data, &vehicle) != 0U) {
            handle_vehicle_data(&vehicle);
        } else {
            debug_printf("CAN PAYLOAD ERROR\r\n");
        }
    }

    if ((can_timed_out == 0U) &&
        ((uint32_t)(system_ms - last_valid_ms) > CAN_TIMEOUT_MS)) {
        can_timed_out = 1U;
        debug_printf("CAN TIMEOUT valid=%lu lost=%lu rx_overflow=%lu wrong=%lu\r\n",
                     valid_frames,
                     lost_frames,
                     rx_overflow,
                     wrong_can_frames);
    }

    if (can_timed_out != 0U) {
        leds_show_timeout();
    }
}

void node_b_systick_isr(void)
{
    ++system_ms;
}

void node_b_can_rx0_isr(void)
{
    CanRxMsg message;
    uint8_t i;

    while (CAN_MessagePending(CAN1, CAN_FIFO0) != 0U) {
        CAN_Receive(CAN1, CAN_FIFO0, &message);

        if ((message.IDE == CAN_Id_Standard) &&
            (message.RTR == CAN_RTR_Data) &&
            (message.StdId == VEHICLE_CAN_ID) &&
            (message.DLC == 8U)) {
            if (rx_slot.pending != 0U) {
                ++rx_overflow;
            }
            for (i = 0U; i < 8U; ++i) {
                rx_slot.data[i] = message.Data[i];
            }
            rx_slot.pending = 1U;
        } else {
            ++wrong_can_frames;
        }
    }
}
