/*
*Module to implement edge triggered interrupts on PF1 and PF2
*
*/
#include <stdint.h>

void GPIOPortF_Int_Setup(uint32_t priority);
void GPIOPortF_Handler(void);
extern void(*SW1Task)(void);
extern void(*SW2Task)(void); //lab3

