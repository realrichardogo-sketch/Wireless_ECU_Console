#ifndef NODE_B_RECEIVER_H
#define NODE_B_RECEIVER_H

void node_b_init(void);
void node_b_process(void);
void node_b_systick_isr(void);
void node_b_can_rx0_isr(void);

#endif
