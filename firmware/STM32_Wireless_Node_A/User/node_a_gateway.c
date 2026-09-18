#include "node_a_gateway.h"

#include "board_config.h"
#include "debug_uart.h"
#include "stm32f10x.h"
#include "vehicle_can.h"
#include "vehicle_protocol.h"

#define UART_RX_BUFFER_SIZE  128U
#define UART_RX_BUFFER_MASK  (UART_RX_BUFFER_SIZE - 1U)

static volatile uint32_t system_ms;
static volatile uint8_t uart_rx_buffer[UART_RX_BUFFER_SIZE];
static volatile uint16_t uart_rx_head;
static volatile uint16_t uart_rx_tail;
static volatile uint32_t uart_rx_overflow;

static VehicleParser parser;
static uint32_t last_valid_ms;
static uint16_t last_sequence;
static uint8_t have_sequence;
static uint8_t wireless_timed_out;
static uint32_t valid_frames;
static uint32_t crc_errors;
static uint32_t format_errors;
static uint32_t lost_frames;
static uint32_t can_tx_failures;

static void status_led_init(void)
{
    GPIO_InitTypeDef gpio;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    gpio.GPIO_Pin = GPIO_Pin_0;
    gpio.GPIO_Speed = GPIO_Speed_2MHz;
    gpio.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOA, &gpio);
    GPIO_SetBits(GPIOA, GPIO_Pin_0); /* Active-low LED off. */
}

static void status_led_toggle(void)
{
    GPIOA->ODR ^= GPIO_Pin_0;
}

static void bridge_uart_init(void)
{
    GPIO_InitTypeDef gpio;
    USART_InitTypeDef usart;
    NVIC_InitTypeDef nvic;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

    gpio.GPIO_Pin = GPIO_Pin_2;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &gpio);

    gpio.GPIO_Pin = GPIO_Pin_3;
    gpio.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &gpio);

    USART_StructInit(&usart);
    usart.USART_BaudRate = BRIDGE_USART_BAUDRATE;
    usart.USART_WordLength = USART_WordLength_8b;
    usart.USART_StopBits = USART_StopBits_1;
    usart.USART_Parity = USART_Parity_No;
    usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    usart.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART2, &usart);

    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);

    nvic.NVIC_IRQChannel = USART2_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 1U;
    nvic.NVIC_IRQChannelSubPriority = 0U;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);

    USART_Cmd(USART2, ENABLE);
}

static uint8_t uart_rx_pop(uint8_t *byte)
{
    uint16_t tail;

    tail = uart_rx_tail;
    if (tail == uart_rx_head) {
        return 0U;
    }

    *byte = uart_rx_buffer[tail];
    uart_rx_tail = (uint16_t)((tail + 1U) & UART_RX_BUFFER_MASK);
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

static void process_valid_frame(const VehicleData *data)
{
    uint8_t can_ok = 0U;

    ++valid_frames;
    update_sequence_statistics(data->sequence);
    last_valid_ms = system_ms;

    if (wireless_timed_out != 0U) {
        wireless_timed_out = 0U;
        debug_printf("WIRELESS RECOVERED\r\n");
    }

#if NODE_A_CAN_FORWARDING
    can_ok = vehicle_can_send(data);
    if (can_ok == 0U) {
        ++can_tx_failures;
    }
#endif

    status_led_toggle();
    debug_printf("UART FRAME OK SEQ=%u RPM=%u TEMP=%d STATE=%s FAULT=%u CAN=%s\r\n",
                 data->sequence,
                 data->rpm,
                 data->temperature_c,
                 vehicle_state_name(data->state),
                 data->fault,
#if NODE_A_CAN_FORWARDING
                 (can_ok != 0U) ? "QUEUED" : "NO_MAILBOX");
#else
                 "DISABLED");
#endif
}

void node_a_init(void)
{
    vehicle_parser_init(&parser);
    status_led_init();
    debug_uart_init();
    bridge_uart_init();
#if NODE_A_CAN_FORWARDING
    vehicle_can_init();
#endif

    last_valid_ms = system_ms;
    debug_printf("\r\nSTM32 Node A gateway started\r\n");
    debug_printf("BRIDGE USART2: PA3 RX / PA2 TX, 115200 8N1\r\n");
    debug_printf("DEBUG  USART1: PA9 TX / PA10 RX, 115200 8N1\r\n");
    debug_printf("CAN1: PB8 RX / PB9 TX, ID=0x201, 500 kbit/s\r\n");
}

void node_a_process(void)
{
    uint8_t byte;
    VehicleData data;
    VehicleParseResult result;

    while (uart_rx_pop(&byte) != 0U) {
        result = vehicle_parser_feed(&parser, byte, &data);
        if (result == VEHICLE_PARSE_OK) {
            process_valid_frame(&data);
        } else if (result == VEHICLE_PARSE_CRC_ERROR) {
            ++crc_errors;
            debug_printf("UART FRAME CRC ERROR count=%lu\r\n", crc_errors);
        } else if (result == VEHICLE_PARSE_FORMAT_ERROR) {
            ++format_errors;
            debug_printf("UART FRAME FORMAT ERROR count=%lu\r\n", format_errors);
        }
    }

    if ((wireless_timed_out == 0U) &&
        ((uint32_t)(system_ms - last_valid_ms) > WIRELESS_TIMEOUT_MS)) {
        wireless_timed_out = 1U;
        debug_printf("WIRELESS TIMEOUT valid=%lu lost=%lu rx_overflow=%lu can_fail=%lu\r\n",
                     valid_frames, lost_frames, uart_rx_overflow, can_tx_failures);
    }
}

void node_a_systick_isr(void)
{
    ++system_ms;
}

void node_a_usart2_isr(void)
{
    uint8_t byte;
    uint16_t next_head;

    if (USART_GetITStatus(USART2, USART_IT_RXNE) != RESET) {
        byte = (uint8_t)USART_ReceiveData(USART2);
        next_head = (uint16_t)((uart_rx_head + 1U) & UART_RX_BUFFER_MASK);
        if (next_head != uart_rx_tail) {
            uart_rx_buffer[uart_rx_head] = byte;
            uart_rx_head = next_head;
        } else {
            ++uart_rx_overflow;
        }
    }
}

