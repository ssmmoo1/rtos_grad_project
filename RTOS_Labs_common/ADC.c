// *************ADC.c**************
// EE445M/EE380L.6 Labs 1, 2, Lab 3, and Lab 4 
// mid-level ADC functions
// you are allowed to call functions in the low level ADCSWTrigger driver
// 
// Runs on LM4F120/TM4C123
// Jonathan W. Valvano Jan 5, 2020, valvano@mail.utexas.edu
#include <stdint.h>
#include "../inc/tm4c123gh6pm.h"
#include "../inc/ADCSWTrigger.h"

#define PIN(num) (1 << num)

// pin number for each channel
static const uint32_t channelsToPins[] = {PIN(3),
                                          PIN(2),
                                          PIN(1),
                                          PIN(0),
                                          PIN(3),
                                          PIN(2),
                                          PIN(1),
                                          PIN(0),
                                          PIN(5),
                                          PIN(4),
                                          PIN(4),
                                          PIN(5)};

// channelNum (0 to 11) specifies which pin is sampled with sequencer 3
// software start
// return with error 1, if channelNum>11, 
// otherwise initialize ADC and return 0 (success)
int ADC_Init(uint32_t channelNum){
// put your Lab 1 code here
  
  // input error checking
  if (channelNum > 11) {
    return 1;
  }
  
  // get bitmask for pin number we are using
  uint32_t pinMask = channelsToPins[channelNum];
  
  SYSCTL_RCGCADC_R |= 0x0001;   // activate ADC0 
  
  if (channelNum < 4 || (channelNum >= 8 && channelNum < 10)) {
    // port E
                                    // 1) activate clock for Port E
    SYSCTL_RCGCGPIO_R |= 0x10;
    while((SYSCTL_PRGPIO_R&0x10) != 0x10){};
    GPIO_PORTE_DIR_R &= ~pinMask;      // 2) make Port input
    GPIO_PORTE_AFSEL_R |= pinMask;     // 3) enable alternate function on Port
    GPIO_PORTE_DEN_R &= ~pinMask;      // 4) disable digital I/O on Port
    GPIO_PORTE_AMSEL_R |= pinMask;     // 5) enable analog functionality on Port
  } else {
    // port D
                                      // 1) activate clock for Port D
    SYSCTL_RCGCGPIO_R |= 0x08;
    while((SYSCTL_PRGPIO_R&0x08) != 0x08){};
    GPIO_PORTD_DIR_R &= ~pinMask;      // 2) make Port input
    GPIO_PORTD_AFSEL_R |= pinMask;     // 3) enable alternate function on Port
    GPIO_PORTD_DEN_R &= ~pinMask;      // 4) disable digital I/O on Port
    GPIO_PORTD_AMSEL_R |= pinMask;     // 5) enable analog functionality on Port
  }

  // wait for ADC0 peripheral to be ready   
  while((SYSCTL_PRADC_R&0x0001) != 0x0001){};    // good code, but not yet implemented in simulator


  ADC0_PC_R &= ~0xF;              // 7) clear max sample rate field
  ADC0_PC_R |= 0x1;               //    maximum speed is 125K samples/sec
  ADC0_SSPRI_R = 0x0123;          // 8) Sequencer 3 is highest priority
  ADC0_ACTSS_R &= ~0x0008;        // 9) disable sample sequencer 3
  ADC0_EMUX_R &= ~0xF000;         // 10) seq3 is software trigger
  ADC0_SSMUX3_R &= ~0x000F;       // 11) clear SS3 field
  ADC0_SSMUX3_R += 9;             //    set channel
  ADC0_SSCTL3_R = 0x0006;         // 12) no TS0 D0, yes IE0 END0
  ADC0_IM_R &= ~0x0008;           // 13) disable SS3 interrupts
  ADC0_ACTSS_R |= 0x0008;         // 14) enable sample sequencer 3
  
  return 0;
}
// software start sequencer 3 and return 12 bit ADC result
uint32_t ADC_In(void){
// put your Lab 1 code here
  uint32_t result;
  ADC0_PSSI_R = 0x0008;            // 1) initiate SS3
  while((ADC0_RIS_R&0x08)==0){};   // 2) wait for conversion done
    // if you have an A0-A3 revision number, you need to add an 8 usec wait here
  result = ADC0_SSFIFO3_R&0xFFF;   // 3) read result
  ADC0_ISC_R = 0x0008;             // 4) acknowledge completion
  return result;
}
