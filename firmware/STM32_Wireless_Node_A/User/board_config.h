#ifndef BOARD_CONFIG_H
#define BOARD_CONFIG_H

/* Core board: STM32F103ZET6, HSE = 8 MHz, SYSCLK = 72 MHz. */

/* PC bridge input: USART2 on PA2/PA3, 3.3 V TTL, 115200 8N1. */
#define BRIDGE_USART_BAUDRATE       115200U

/* Debug output: USART1 TX on PA9, 115200 8N1. */
#define DEBUG_USART_BAUDRATE        115200U

/* CAN1 is remapped to PB8 (RX) and PB9 (TX), 500 kbit/s. */
#define VEHICLE_CAN_ID              0x201U
#define VEHICLE_CAN_BITRATE         500000U

/* Set to 0 for the M6 UART-only bench test; use 1 for the complete M7 link. */
#define NODE_A_CAN_FORWARDING 1U

#define WIRELESS_TIMEOUT_MS         1500U
#define VEHICLE_HEARTBEAT_MS        500U

#endif
