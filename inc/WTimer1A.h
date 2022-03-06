#include <stdint.h>

void WideTimer1A_Init(void(*task)(void), uint32_t period, uint32_t priority);

void WideTimer1_Stop(void);
