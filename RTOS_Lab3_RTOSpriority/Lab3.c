// Lab3.c - Grad project demo

//Task Set Scheduler/Visualizer
//Use with a logic analyzer to view how the tasks are scheduled
//Supports up to 3 tasks (+ and idle task) with configurable periods and run time
//Visualize with a logic analyzer, 

//PD0 - Idle Task Will rapdily toggle only when idle task is running 
//PD1 - Task 1 Will rapidly toggle only when task 1 is running
//PD2 - Task 2 Will rapidly toggle only when task 2 is running
//PD3 - Task 3 Will rapidly toggle only when Task 3 is running
//PD6 - Task 1 Scheduled/Deadline will output a quick pulse when task 1 is scheduled again
//PB0 - Task 2 Scheduled/Deadline will output a quick pulse when task 2 is scheduled again
//PB1 - Task 3 Scheduled/Deadline will output a quick pulse when task 3 is scheduled again

//Once the task set is complete it will output stats to the LCD


#define task_set_3
#define USE_EDF true


#ifdef task_set_1 
//90% utilization succeeds with RMS
//Used for the Task scheduler
#define TASK_SCHED_RES 1 //time resolution of when we can spawn periodic tasks in MS. Don't want it too frequently 

//Task Time is in units of milliseconds (how long the tasks will take to run)
//Must be a multiple of TASK_SCHED_RES
#define TASK1_TIME 50
#define TASK2_TIME 40
#define TASK3_TIME 80

//Periods for spawning tasks in milliseconds
#define TASK1_PERIOD 100
#define TASK2_PERIOD 200
#define TASK3_PERIOD 400
#endif


#ifdef task_set_2
//Fails with RMS
//Used for the Task scheduler
#define TASK_SCHED_RES 1 //time resolution of when we can spawn periodic tasks in MS. Don't want it too frequently 

//Task Time is in units of milliseconds (how long the tasks will take to run)
//Must be a multiple of TASK_SCHED_RES
#define TASK1_TIME 3
#define TASK2_TIME 25
#define TASK3_TIME 25

//Periods for spawning tasks in milliseconds
#define TASK1_PERIOD 10
#define TASK2_PERIOD 60
#define TASK3_PERIOD 100
#endif


#ifdef task_set_3
//Used for the Task scheduler
#define TASK_SCHED_RES 1 //time resolution of when we can spawn periodic tasks in MS. Don't want it too frequently 

//Task Time is in units of milliseconds (how long the tasks will take to run)
//Must be a multiple of TASK_SCHED_RES
#define TASK1_TIME 10
#define TASK2_TIME 300
#define TASK3_TIME 300

//Periods for spawning tasks in milliseconds
#define TASK1_PERIOD 10
#define TASK2_PERIOD 60
#define TASK3_PERIOD 100
#endif



#include <stdint.h>
#include "../inc/tm4c123gh6pm.h"
#include "../inc/CortexM.h"
#include "../inc/LaunchPad.h"
#include "../inc/PLL.h"
#include "../inc/LPF.h"
#include "../RTOS_Labs_common/UART0int.h"
#include "../RTOS_Labs_common/ADC.h"
#include "../inc/ADCT0ATrigger.h"
#include "../inc/IRDistance.h"
#include "../RTOS_Labs_common/OS.h"
#include "../RTOS_Labs_common/Interpreter.h"
#include "../RTOS_Labs_common/ST7735.h"

//---------------------Task Set Visualization-----------------------
#define PD0  (*((volatile uint32_t *)0x40007004)) //idle task on
#define PD1  (*((volatile uint32_t *)0x40007008)) //task 1 on
#define PD2  (*((volatile uint32_t *)0x40007010)) //task 2 on
#define PD3  (*((volatile uint32_t *)0x40007020)) //task 3 on
#define PD6  (*((volatile uint32_t *)0x40007100)) //task 1 scheudled
#define PB0  (*((volatile uint32_t *)0x40005004)) //task 2 scheduled
#define PB1  (*((volatile uint32_t *)0x40005008)) //task 3 scheduled
  
void PortD_Init(void){ 
  SYSCTL_RCGCGPIO_R |= 0x08;       // activate port D
  while((SYSCTL_RCGCGPIO_R&0x08)==0){};      
  GPIO_PORTD_DIR_R |= 0xCF;        // make PD3-0, 6,7 output heartbeats
  GPIO_PORTD_AFSEL_R &= ~0xCF;     // disable alt funct on PD3-0 6,7
  GPIO_PORTD_DEN_R |= 0xCF;        // enable digital I/O on PD3-0 ,6,7
  GPIO_PORTD_PCTL_R = ~0xFF00FFFF;
  GPIO_PORTD_AMSEL_R &= ~0xCF;;    // disable analog functionality on PD
}

void PortB_Init(void)
{
  SYSCTL_RCGCGPIO_R |= 0x02;       // activate port B
  while((SYSCTL_RCGCGPIO_R&0x02)==0){};      
  GPIO_PORTB_DIR_R |= 0x0F;        // make PB3-0, output heartbeats
  GPIO_PORTB_AFSEL_R &= ~0x0F;     // disable alt funct on PB3-0 
  GPIO_PORTB_DEN_R |= 0x0F;        // enable digital I/O on PB3-0 
  GPIO_PORTB_PCTL_R = ~0x0000FFFF;
  GPIO_PORTB_AMSEL_R &= ~0x0F;;    // disable analog functionality on PB
    
  PB0 = 0;
  PB1 = 0;
}

uint32_t find_lcm(uint32_t a, uint32_t b)
{
  uint32_t max = (a > b) ? a : b;
  while(1)
  {
    if(max % a == 0 && max % b == 0)
    {
      return max;
    }
    max++;
  }
}
//******** Idle Task *************** 
// foreground thread, runs when nothing else does
// never blocks, never sleeps, never dies
// measures CPU idle time, i.e. CPU utilization
// inputs:  none
// outputs: none
//#define SUBS_PER_MS_G 3636 //times to sub from volatile global to get to 1ms  1.000208ms
#define SUBS_PER_MS_G 2222 //times to sub and toggle to get to 1ms with global var 
volatile uint32_t IdleCounter_g = 0xFFFFFFFF; //counter to measure CPU utilization

void Idle(void){
  while(IdleCounter_g > 0)
  {
    PD0 ^= 0x01;
    IdleCounter_g--;
  }
  OS_Kill();
}


//----------------- Dummy Tasks Sets --------------
//Configure in the defines at the top
//#define SUBS_PER_MS_L 6665 //times to sub from voltalile local to get to 1ms
#define SUBS_PER_MS_L 3325 //time to sub and toggle to get to 1 ms of work

bool schedulable = true;

bool task_1_lock = false;
void dummy_task_1(void)
{
  if(task_1_lock == true)
  {
    schedulable = false;
  }
  else
  {
    task_1_lock = true;
  }
  volatile uint32_t i = SUBS_PER_MS_L * TASK1_TIME; //volatile is important, without it timing is not consistent
  while(i > 0) 
  {
    PD1 ^= 0x02;
    i--;
  }
  task_1_lock = false;
  OS_Kill();
}

bool task_2_lock = false;
void dummy_task_2(void)
{
  if(task_2_lock == true)
  {
    schedulable = false;
  }
  else
  {
    task_2_lock = true;
  }
 volatile uint32_t i = SUBS_PER_MS_L * TASK2_TIME; //volatile is important, without it timing is not consistent
  while(i > 0) 
  {
    PD2 ^= 0x04;
    i--;
  }
  task_2_lock = false;
  OS_Kill();
}

bool task_3_lock = false;
void dummy_task_3(void)
{
  if(task_3_lock == true)
  {
    schedulable = false;
  }
  else
  {
    task_3_lock = true;
  }
 volatile uint32_t i = SUBS_PER_MS_L * TASK3_TIME; //volatile is important, without it timing is not consistent
  while(i > 0) 
  {
    PD3 ^= 0x08;
    i--;
  }
  task_3_lock = false;
  OS_Kill();
}

void system_stats(void);
//-----------------Peiodic thread scheduler------------------
//Schedules the defined task set
//Must run as a periodic background thread and it will spawn the tasks according to their defined period
void periodic_thread_creator()
{
  static uint32_t times_called = 0;
  static bool scheduler_complete = false;
  static uint32_t lcm = TASK3_PERIOD;
  //Calculate how long to run
  if(times_called == 0)
  {
    //Need to run for LCM of defined periods
    lcm = find_lcm(TASK1_PERIOD, TASK2_PERIOD);
    lcm = find_lcm(lcm, TASK3_PERIOD);
    IdleCounter_g = 0xFFFFFFFF; //reset idle counter
  }
 
  if(scheduler_complete) return;
  
  
  
  
  if(times_called >= lcm/TASK_SCHED_RES)
  {
    scheduler_complete = true;
    OS_AddThread_D(&system_stats, 128, 0, 1000000000);
    PD6 = 0x40;
    PD6 = 0x00;
    PB0 = 0x01;
    PB0 = 0x00;
    PB1 = 0x02;
    PB1 = 0x00;
    return;
  }
  
  //Spawn periodic tasks
  if(times_called % (TASK1_PERIOD / TASK_SCHED_RES) == 0)
  {
    PD6 = 0x40;
    OS_AddThread_D(&dummy_task_1, 128, 1, TASK1_PERIOD); //deadline param must match the period in ms
    PD6 = 0x00;
    
  }
  if(times_called % (TASK2_PERIOD / TASK_SCHED_RES) == 0)
  {
    PB0 = 0x01;
    OS_AddThread_D(&dummy_task_2, 128, 2, TASK2_PERIOD);
    PB0 = 0x00;
  }
  if(times_called % (TASK3_PERIOD / TASK_SCHED_RES) == 0)
  {
    PB1 = 0x02;
    OS_AddThread_D(&dummy_task_3, 128, 3, TASK3_PERIOD); 
    PB1 = 0x00;
  }
  
  times_called++;
  
}



//Compute system stats about our EDF scheduler
void system_stats(void)
{
  uint32_t lcm;
  lcm = find_lcm(TASK1_PERIOD, TASK2_PERIOD);
  lcm = find_lcm(lcm, TASK3_PERIOD);
  uint32_t idle_util = ((0xFFFFFFFF - IdleCounter_g) / SUBS_PER_MS_G) * 1000 / lcm; 
  uint32_t task1_util = TASK1_TIME * 1000 / TASK1_PERIOD;
  uint32_t task2_util = TASK2_TIME * 1000 / TASK2_PERIOD;
  uint32_t task3_util = TASK3_TIME * 1000 / TASK3_PERIOD;
  uint32_t OS_util = 1000 - idle_util - task1_util - task2_util - task3_util;
  
  while(1)
  {
    
    ST7735_Message(0,0, "Task Set Stats %", 0, 0);
    ST7735_Message_Dec(0,1, "Task1 Util:", task1_util, true); 
    ST7735_Message_Dec(0,2, "Task2 Util:", task2_util, true); 
    ST7735_Message_Dec(0,3, "Task3 Util:", task3_util, true); 
    ST7735_Message_Dec(0,4, "TaskSet Util:", task3_util+task2_util+task1_util, true);
    ST7735_Message_Dec(0,5, "Idle Util:", idle_util, true); 
    ST7735_Message_Dec(0,6, "OS Util:", OS_util, true); 
    
    ST7735_Message(1, 0, "Run Time MS:", lcm, true);
    
    if(schedulable)
    {
      ST7735_Message(1, 1, "Scheduled Success!", 0, false);
    }
    else
    {
      ST7735_Message(1, 1, "Scheduling Failed!", 0, false);
    }
    
    if(USE_EDF)
    {
      ST7735_Message(1, 3, "Using EDF", 0, false);
    }
    else
    {
      ST7735_Message(1, 3, "Using RMS", 0, false);
    }
    
    
  }
 
}



void pi_dummy_task_1(void)
{
  volatile uint32_t i = SUBS_PER_MS_L * TASK1_TIME; //volatile is important, without it timing is not consistent
  while(i > 0) 
  {
    PD1 ^= 0x02;
    i--;
  }
}
void pi_dummy_task_2(void)
{
  volatile uint32_t i = SUBS_PER_MS_L * TASK2_TIME; //volatile is important, without it timing is not consistent
  while(i > 0) 
  {
    PD2 ^= 0x04;
    i--;
  }
}
void pi_dummy_task_3(void)
{
  volatile uint32_t i = SUBS_PER_MS_L * TASK3_TIME; //volatile is important, without it timing is not consistent
  while(i > 0) 
  {
    PD3 ^= 0x08;
    i--;
  }
}

/***************************
 * Priority Inversion tasks
***************************/

LockType lock;

void task_high(void) {
  OS_InitLock(&lock);
  
  while (1) {
    OS_Lock(&lock);
    PF1 ^= 2;
    pi_dummy_task_1();
    OS_Unlock(&lock);
    OS_Sleep(100);
  }
}

void task_mid(void) {
  while(1) {
    pi_dummy_task_2();
    OS_Sleep(50);
  }
}

void task_low(void) {
  while (1) {
    OS_Lock(&lock);
    pi_dummy_task_3();
    OS_Unlock(&lock);
  }
}

void priorityInversion_lock(void) {
  OS_Init(false);
  PortD_Init();
  LaunchPad_Init();
  OS_AddThread(task_high, DEFAULT_STACK_SIZE, 1);
  OS_AddThread(task_mid, DEFAULT_STACK_SIZE, 2);
  OS_AddThread(task_low, DEFAULT_STACK_SIZE, 3);
  OS_AddThread(&Idle, DEFAULT_STACK_SIZE, 5);
  
  OS_Launch(TIME_1MS);
  while(1);
}





//Priority inversion with semaphores
Sema4Type sema;

void task_high_sema(void) {
  OS_InitSemaphore(&sema, 1);
  
  while (1) {
    OS_bWait(&sema);
    PF1 ^= 2;
    pi_dummy_task_1();
    OS_bSignal(&sema);
    OS_Sleep(100);
  }
}

void task_mid_sema(void) {
  while(1) {
    pi_dummy_task_2();
    OS_Sleep(50);
  }
}

void task_low_sema(void) {
  while (1) {
    OS_bWait(&sema);
    pi_dummy_task_3();
    OS_bSignal(&sema);
  }
}

void priorityInversion_sema(void) {
  OS_Init(false);
  PortD_Init();
  LaunchPad_Init();
  OS_AddThread(task_high_sema, DEFAULT_STACK_SIZE, 1);
  OS_AddThread(task_mid_sema, DEFAULT_STACK_SIZE, 2);
  OS_AddThread(task_low_sema, DEFAULT_STACK_SIZE, 3);
  OS_AddThread(&Idle, DEFAULT_STACK_SIZE, 5);
  
  OS_Launch(TIME_1MS);
  while(1);
}


int realmain(void){ // realmain
  OS_Init(USE_EDF);        // initialize, disable interrupts
  PortD_Init();     // debugging profile
  PortB_Init();

  OS_AddPeriodicThread(&periodic_thread_creator,TIME_1MS*TASK_SCHED_RES,1); //task to spawn our EDF task set periodically
  OS_AddThread_D(&Idle, 128, 5, 2000000000);
 
  OS_Launch(TIME_1MS); // doesn't return, interrupts enabled in here
  while(1);
  return 0;            // this never executes
}


//*******************Trampoline for selecting main to execute**********
int main(void) {       // main 
  priorityInversion_lock();
}
