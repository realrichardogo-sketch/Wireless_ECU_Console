#include "stm32f10x_it.h"
#include "node_a_gateway.h"

void NMI_Handler(void) {}
void HardFault_Handler(void) { while (1) {} }
void MemManage_Handler(void) { while (1) {} }
void BusFault_Handler(void) { while (1) {} }
void UsageFault_Handler(void) { while (1) {} }
void SVC_Handler(void) {}
void DebugMon_Handler(void) {}
void PendSV_Handler(void) {}

void SysTick_Handler(void)
{
    node_a_systick_isr();
}

void USART2_IRQHandler(void)
{
    node_a_usart2_isr();
}
