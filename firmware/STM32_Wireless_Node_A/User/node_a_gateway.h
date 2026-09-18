#ifndef NODE_A_GATEWAY_H
#define NODE_A_GATEWAY_H

#include <stdint.h>

void node_a_init(void);
void node_a_process(void);
void node_a_systick_isr(void);
void node_a_usart2_isr(void);

#endif
