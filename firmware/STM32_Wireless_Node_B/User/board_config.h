#ifndef BOARD_CONFIG_H
#define BOARD_CONFIG_H

/* Core board: STM32F103ZET6, HSE = 8 MHz, SYSCLK = 72 MHz. */
#define DEBUG_USART_BAUDRATE        115200U

/* CAN1 is remapped to PB8 (RX) and PB9 (TX), 500 kbit/s. */
#define VEHICLE_CAN_ID              0x201U
#define VEHICLE_CAN_BITRATE         500000U

#define CAN_TIMEOUT_MS              1500U

#endif
