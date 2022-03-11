/*----------------------------------------------------------------------------
  PF1 Interrupt Handler
 *----------------------------------------------------------------------------*/
#include <stdint.h>
#include "../inc/LaunchPad.h"
#include "../inc/tm4c123gh6pm.h"
#include "../inc/CortexM.h"
#include "../inc/PLL.h"
#include <stdbool.h>

void(*SW1Task)(void);
void(*SW2Task)(void);
//Sets up PF4 and PF0 as edge triggered interrupts. TODO LAB 3 CHANGE PRIORITY BASED ON INPUT
void GPIOPortF_Int_Setup(uint32_t priority)
{
  static bool setup = false;
  
  if(setup == true)
  {
    //if port f already setup then just update priority if higher
    uint32_t old_pri = NVIC_PRI7_R&0x00FF0000;
    if((priority << 21) > old_pri)
    {
      NVIC_PRI7_R = (NVIC_PRI7_R&0xFF00FFFF)||(priority<<21);
    }
    return;
  }
  
  SYSCTL_RCGCGPIO_R |= 0x00000020; // (a) activate clock for port F
  
  GPIO_PORTF_LOCK_R = 0x4C4F434B;   // 2) unlock GPIO Port F
  GPIO_PORTF_CR_R = 0x11;           // allow changes to PF4 and PF0
 
  GPIO_PORTF_DIR_R &= ~0x11;    // (c) make PF4,0 inputs (built-in button)
  GPIO_PORTF_AFSEL_R &= ~0x11;  //     disable alt funct on PF4,0
  GPIO_PORTF_DEN_R |= 0x11;     //     enable digital I/O on PF4, PF0   
  GPIO_PORTF_PCTL_R &= ~0x000FFFFF; // configure PF4 as GPIO
  GPIO_PORTF_AMSEL_R = 0;       //     disable analog functionality on PF
  GPIO_PORTF_PUR_R |= 0x11;     //     enable weak pull-up on PF4, PF0
  GPIO_PORTF_IS_R &= ~0x11;     // (d) PF4, PF0 is edge-sensitive
  GPIO_PORTF_IBE_R &= ~0x11;    //     PF4, PF0 is not both edges
  GPIO_PORTF_IEV_R &= ~0x11;    //     PF4, PF0 falling edge event
  GPIO_PORTF_ICR_R = 0x11;      // (e) clear flag4 and flag0
  GPIO_PORTF_IM_R |= 0x11;      // (f) arm interrupt on PF4 *** No IME bit as mentioned in Book ***
  NVIC_PRI7_R = (NVIC_PRI7_R&0xFF00FFFF)||(priority<<21); // (g) priority 5
  NVIC_EN0_R = 0x40000000;      // (h) enable interrupt 30 in NVIC
}



void GPIOPortF_Handler(void){
  
  if(GPIO_PORTF_RIS_R&0x10) // PF4 triggered
  {
    GPIO_PORTF_ICR_R = 0x10; //ack flag
    if(SW1Task != 0)
    {
      SW1Task();
    }
  }
  
  if(GPIO_PORTF_RIS_R&0x01) //PF0 triggered (lab3)
  {
    GPIO_PORTF_ICR_R = 0x01; //ack flag
    if(SW2Task != 0)
    {
      SW2Task();
    }
  }
  
 
}
