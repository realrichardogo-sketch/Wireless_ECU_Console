#include "stm32f10x.h"
#include "node_a_gateway.h"

int main(void)
{
    SystemCoreClockUpdate();
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

    if (SysTick_Config(SystemCoreClock / 1000U) != 0U) {
        while (1) {
        }
    }

    node_a_init();

    while (1) {
        node_a_process();
    }
}

