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

/*
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
*/


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
    OS_AddThread(&system_stats, 128, 0);
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
    
    
  }
 
}


int realmain(void){ // realmain
  OS_Init(true);        // initialize, disable interrupts
  PortD_Init();     // debugging profile
  PortB_Init();

  OS_AddPeriodicThread(&periodic_thread_creator,TIME_1MS*TASK_SCHED_RES,1); //task to spawn our EDF task set periodically
  OS_AddThread(&Idle, 128, 5);
 
  OS_Launch(TIME_1MS); // doesn't return, interrupts enabled in here
  while(1);
  return 0;            // this never executes
}

//+++++++++++++++++++++++++DEBUGGING CODE++++++++++++++++++++++++
// ONCE YOUR RTOS WORKS YOU CAN COMMENT OUT THE REMAINING CODE
// 
//*******************Initial TEST**********
// This is the simplest configuration, test this first, (Lab 2 part 1)
// run this with 
// no UART interrupts
// no SYSTICK interrupts
// no timer interrupts
// no switch interrupts
// no ADC serial port or LCD output
// no calls to semaphores
uint32_t Count1;   // number of times thread1 loops
uint32_t Count2;   // number of times thread2 loops
uint32_t Count3;   // number of times thread3 loops
uint32_t Count4;   // number of times thread4 loops
uint32_t Count5;   // number of times thread5 loops
void Thread1(void){
  Count1 = 0;          
  for(;;){
    PD0 ^= 0x01;       // heartbeat
    Count1++;
    OS_Suspend();      // cooperative multitasking
  }
}
void Thread2(void){
  Count2 = 0;          
  for(;;){
    PD1 ^= 0x02;       // heartbeat
    Count2++;
    OS_Suspend();      // cooperative multitasking
  }
}
void Thread3(void){
  Count3 = 0;          
  for(;;){
    PD2 ^= 0x04;       // heartbeat
    Count3++;
    OS_Suspend();      // cooperative multitasking
  }
}

int Testmain1(void){  // Testmain1
  OS_Init(false);          // initialize, disable interrupts
  PortD_Init();       // profile user threads
  NumCreated = 0 ;
  NumCreated += OS_AddThread(&Thread1,128,0); 
  NumCreated += OS_AddThread(&Thread2,128,0); 
  NumCreated += OS_AddThread(&Thread3,128,0); 
  // Count1 Count2 Count3 should be equal or off by one at all times
  OS_Launch(TIME_2MS); // doesn't return, interrupts enabled in here
  return 0;            // this never executes
}

//*******************Second TEST**********
// Once the initalize test runs, test this (Lab 2 part 1)
// no UART interrupts
// SYSTICK interrupts, with or without period established by OS_Launch
// no timer interrupts
// no switch interrupts
// no ADC serial port or LCD output
// no calls to semaphores
void Thread1b(void){
  Count1 = 0;          
  for(;;){
    PD0 ^= 0x01;       // heartbeat
    Count1++;
  }
}
void Thread2b(void){
  Count2 = 0;          
  for(;;){
    PD1 ^= 0x02;       // heartbeat
    Count2++;
  }
}
void Thread3b(void){
  Count3 = 0;          
  for(;;){
    PD2 ^= 0x04;       // heartbeat
    Count3++;
  }
}

int Testmain2(void){  // Testmain2
  OS_Init(false);          // initialize, disable interrupts
  PortD_Init();       // profile user threads
  NumCreated = 0 ;
  NumCreated += OS_AddThread(&Thread1b,128,0); 
  NumCreated += OS_AddThread(&Thread2b,128,0); 
  NumCreated += OS_AddThread(&Thread3b,128,0); 
  // Count1 Count2 Count3 should be equal on average
  // counts are larger than Testmain1
 
  OS_Launch(TIME_2MS); // doesn't return, interrupts enabled in here
  return 0;            // this never executes
}

//*******************Third TEST**********
// Once the second test runs, test this (Lab 2 part 2)
// no UART interrupts
// SYSTICK interrupts, with or without period established by OS_Launch
// no timer interrupts
// no switch interrupts
// no ADC serial port or LCD output
// no calls to semaphores
// tests AddThread, Sleep and Kill
void Thread1c(void){ int i;
  Count1 = 0;          
  for(i=0;i<=42;i++){
    PD0 ^= 0x01;       // heartbeat
    Count1++;
  }
  OS_Kill();
  Count1 = 0;
}
void Thread2c(void){
  Count2 = 0;          
  for(;;){
    PD1 ^= 0x02;       // heartbeat
    Count2++;
    NumCreated += OS_AddThread(&Thread1c,128,0); 
    OS_Sleep(5);
  }
}
void Thread3c(void){
  Count3 = 0;          
  for(;;){
    PD2 ^= 0x04;       // heartbeat
    Count3++;
  }
}

int Testmain3(void){  // Testmain3
  OS_Init(false);          // initialize, disable interrupts
  PortD_Init();       // profile user threads
  NumCreated = 0 ;
  NumCreated += OS_AddThread(&Thread2c,128,0); 
  NumCreated += OS_AddThread(&Thread3c,128,0); 
  // Count3 should be larger than Count2, Count1 should be 42
 
  OS_Launch(TIME_2MS); // doesn't return, interrupts enabled in here
  return 0;            // this never executes
}

//*******************Fourth TEST**********
// Once the third test runs, test this (Lab 2 part 2 and Lab 3)
// no UART1 interrupts
// SYSTICK interrupts, with or without period established by OS_Launch
// Timer interrupts, with or without period established by OS_AddPeriodicThread
// PortF GPIO interrupts, active low
// no ADC serial port or LCD output
// tests priorities and blocking semaphores, tests Sleep and Kill
Sema4Type Readyd;        // set in background
int Lost;
void BackgroundThread1d(void){   // called at 1000 Hz
  Count1++;
  OS_Signal(&Readyd);
}
void Thread5d(void){
  for(;;){
    OS_Wait(&Readyd);
    Count5++;   // Count2 + Count5 should equal Count1 
    Lost = Count1-Count5-Count2;
  }
}
void Thread2d(void){
  OS_InitSemaphore(&Readyd,0);
  Count1 = 0;    // number of times signal is called      
  Count2 = 0;    
  Count5 = 0;    // Count2 + Count5 should equal Count1  
  NumCreated += OS_AddThread(&Thread5d,128,1); 
  OS_AddPeriodicThread(&BackgroundThread1d,TIME_1MS,0); 
  for(;;){
    OS_Wait(&Readyd);
    Count2++;   // Count2 + Count5 should equal Count1
    Lost = Count1-Count5-Count2;
  }
}
void Thread3d(void){
  Count3 = 0;          
  for(;;){
    Count3++;
  }
}
void Thread4d(void){ int i;
  for(i=0;i<64;i++){
    Count4++;
    OS_Sleep(10);
  }
  OS_Kill();
  Count4 = 0;
}
void BackgroundThread5d(void){   // called when Select button pushed
  NumCreated += OS_AddThread(&Thread4d,128,1); 
}
      
int Testmain4(void){   // Testmain4
  Count4 = 0;          
  OS_Init(false);           // initialize, disable interrupts
  // Count2 + Count5 should equal Count1
  // With priorities, Count5 should be zero 
  // Count4 increases by 64 every time select is pressed
  NumCreated = 0 ;
  OS_AddSW1Task(&BackgroundThread5d,2);
  NumCreated += OS_AddThread(&Thread2d,128,0); // Lab 3 highest priority 
  NumCreated += OS_AddThread(&Thread3d,128,1); 
  NumCreated += OS_AddThread(&Thread4d,128,1); 
  OS_Launch(TIME_2MS); // doesn't return, interrupts enabled in here
  return 0;            // this never executes
}

//*******************Fith TEST**********
// Once the fourth test runs, run this example (Lab 2 part 2 and Lab 3)
// no UART interrupts
// SYSTICK interrupts, with or without period established by OS_Launch
// Timer interrupts, with or without period established by OS_AddPeriodicThread
// Select switch interrupts, active low
// no ADC serial port or LCD output
// tests the blocking semaphores, tests Sleep and Kill
// uses priorities to test proper blocking of sempahore waits
Sema4Type Readye;        // set in background
void BackgroundThread1e(void){   // called at 1000 Hz
static int i=0;
  i++;
  if(i==50){
    i = 0;         //every 50 ms
    Count1++;
    OS_bSignal(&Readye);
  }
}
void Thread2e(void){
  OS_InitSemaphore(&Readye,0);
  Count1 = 0;          
  Count2 = 0;          
  for(;;){
    OS_bWait(&Readye);
    Count2++;     // Count2 should be equal to Count1
  }
}
void Thread3e(void){
  Count3 = 0;          
  for(;;){
    Count3++;     // Count3 should be large
  }
}
void Thread4e(void){ int i;
  for(i=0;i<640;i++){
    Count4++;     // Count4 should increase on button press
    OS_Sleep(1);
  }
  OS_Kill();
}
void BackgroundThread5e(void){   // called when Select button pushed
  NumCreated += OS_AddThread(&Thread4e,128,1); 
}

int Testmain5(void){   // Testmain5
  Count4 = 0;          
  OS_Init(false);           // initialize, disable interrupts
  // Count1 should exactly equal Count2
  // Count3 should be very large
  // Count4 increases by 640 every time select is pressed
  NumCreated = 0 ;
  OS_AddPeriodicThread(&BackgroundThread1e,TIME_1MS*5,0); 
  OS_AddSW1Task(&BackgroundThread5e,2);
  NumCreated += OS_AddThread(&Thread2e,128,0); // Lab 3 set to highest priority
  NumCreated += OS_AddThread(&Thread3e,128,1); 
  NumCreated += OS_AddThread(&Thread4e,128,1); 
  OS_Launch(TIME_2MS); // doesn't return, interrupts enabled in here
  return 0;            // this never executes
}

//******************* Lab 3 Procedure 2**********
// Modify this so it runs with your RTOS (i.e., fix the time units to match your OS)
// run this with 
// UART0, 115200 baud rate, used to output results 
// SYSTICK interrupts, period established by OS_Launch
// first timer interrupts, period established by first call to OS_AddPeriodicThread
// second timer interrupts, period established by second call to OS_AddPeriodicThread
// SW1 no interrupts
// SW2 no interrupts
uint32_t CountA;   // number of times Task A called
uint32_t CountB;   // number of times Task B called
uint32_t Count1;   // number of times thread1 loops

// simple time delay, simulates user program doing real work
// Input: amount of work in 100ns units (free free to change units)
void PseudoWork(uint32_t work){
uint32_t startTime;
  startTime = OS_Time();    // time in 100ns units
  while(OS_TimeDifference(startTime,OS_Time()) <= work){} 
}
void Thread6(void){  // foreground thread
  Count1 = 0;          
  for(;;){
    Count1++; 
    PD0 ^= 0x01;        // debugging toggle bit 0  
  }
}
//extern void Jitter(int32_t, uint32_t const, uint32_t []); // prints jitter information (write this)
void Thread7(void){  // foreground thread
  UART_OutString("\n\rEE345M/EE380L, Lab 3 Procedure 2\n\r");
  OS_Sleep(5000);   // 10 seconds        
  Jitter(MaxJitter_1, JITTERSIZE, JitterHistogram_1);  // print jitter information
  Jitter(MaxJitter_2, JITTERSIZE, JitterHistogram_2);  // print jitter of second thread
  UART_OutString("\n\r\n\r");
  OS_Kill();
}
#define workA 500       // {5,50,500 us} work in Task A
#define counts1us 10    // number of OS_Time counts per 1us
void TaskA(void){       // called every {1000, 2990us} in background
  PD1 = 0x02;      // debugging profile  
  OS_Jitter_1(TIME_1MS);
  CountA++;
  PseudoWork(workA*counts1us); //  do work (100ns time resolution)
  PD1 = 0x00;      // debugging profile  
}
#define workB 250       // 250 us work in Task B
void TaskB(void){       // called every pB in background
  PD2 = 0x04;      // debugging profile  
  OS_Jitter_2(TIME_1MS * 2);
  CountB++;
  PseudoWork(workB*counts1us); //  do work (100ns time resolution)
  PD2 = 0x00;      // debugging profile  
}

int Testmain6(void){       // Testmain6 Lab 3
  PortD_Init();
  OS_Init(false);           // initialize, disable interrupts
  NumCreated = 0 ;
  NumCreated += OS_AddThread(&Thread7,128,1); 
  NumCreated += OS_AddThread(&Thread6,128,2); 
  OS_AddPeriodicThread(&TaskA,TIME_1MS,0);           // 1 ms, higher priority
  OS_AddPeriodicThread(&TaskB,2*TIME_1MS,1);         // 2 ms, lower priority
 
  OS_Launch(TIME_2MS); // 2ms, doesn't return, interrupts enabled in here
  return 0;             // this never executes
}

//******************* Lab 3 Procedure 4**********
// Modify this so it runs with your RTOS used to test blocking semaphores
// run this with 
// UART0, 115200 baud rate,  used to output results 
// SYSTICK interrupts, period established by OS_Launch
// first timer interrupts, period established by first call to OS_AddPeriodicThread
// second timer interrupts, period established by second call to OS_AddPeriodicThread
// SW1 no interrupts, 
// SW2 no interrupts
Sema4Type s;            // test of this counting semaphore
uint32_t SignalCount1;   // number of times s is signaled
uint32_t SignalCount2;   // number of times s is signaled
uint32_t SignalCount3;   // number of times s is signaled
uint32_t WaitCount1;     // number of times s is successfully waited on
uint32_t WaitCount2;     // number of times s is successfully waited on
uint32_t WaitCount3;     // number of times s is successfully waited on
#define MAXCOUNT 200
void OutputThread(void){  // foreground thread
  UART_OutString("\n\rEE445M/EE380L, Lab 3 Procedure 4\n\r");
  while(SignalCount1+SignalCount2+SignalCount3<100*MAXCOUNT){
    OS_Sleep(1000);   // 1 second
    UART_OutString(".");
  }       
  UART_OutString(" done\n\r");
  UART_OutString("Signalled="); UART_OutUDec(SignalCount1+SignalCount2+SignalCount3);
  UART_OutString(", Waited="); UART_OutUDec(WaitCount1+WaitCount2+WaitCount3);
  UART_OutString("\n\r");
  OS_Kill();
}
void Wait1(void){  // foreground thread
  for(;;){
    OS_Wait(&s);    // three threads waiting
    WaitCount1++; 
  }
}
void Wait2(void){  // foreground thread
  for(;;){
    OS_Wait(&s);    // three threads waiting
    WaitCount2++; 
  }
}
void Wait3(void){   // foreground thread
  for(;;){
    OS_Wait(&s);    // three threads waiting
    WaitCount3++; 
  }
}
void Signal1(void){      // called every 799us in background
  OS_Jitter_1((799*TIME_1MS)/1000);
  if(SignalCount1<MAXCOUNT){
    OS_Signal(&s);
    SignalCount1++;
  }
}
// edit this so it changes the periodic rate
void Signal2(void){       // called every 1111us in background
  OS_Jitter_2((1111*TIME_1MS)/1000);
  if(SignalCount2<MAXCOUNT){
    OS_Signal(&s);
    SignalCount2++;
  }
}
void Signal3(void){       // foreground
  while(SignalCount3<98*MAXCOUNT){
    OS_Signal(&s);
    SignalCount3++;
  }
  OS_Kill();
}

int32_t add(const int32_t n, const int32_t m){
static int32_t result;
  result = m+n;
  return result;
}
int Testmain7(void){      // Testmain7  Lab 3
  volatile uint32_t delay;
  OS_Init(false);           // initialize, disable interrupts
  delay = add(3,4);
  PortD_Init();
  SignalCount1 = 0;   // number of times s is signaled
  SignalCount2 = 0;   // number of times s is signaled
  SignalCount3 = 0;   // number of times s is signaled
  WaitCount1 = 0;     // number of times s is successfully waited on
  WaitCount2 = 0;     // number of times s is successfully waited on
  WaitCount3 = 0;    // number of times s is successfully waited on
  OS_InitSemaphore(&s,0);   // this is the test semaphore
  OS_AddPeriodicThread(&Signal1,(799*TIME_1MS)/1000,0);   // 0.799 ms, higher priority
  OS_AddPeriodicThread(&Signal2,(1111*TIME_1MS)/1000,1);  // 1.111 ms, lower priority
  NumCreated = 0 ;
  NumCreated += OS_AddThread(&OutputThread,128,2);   // results output thread
  NumCreated += OS_AddThread(&Signal3,128,2);   // signalling thread
  NumCreated += OS_AddThread(&Wait1,128,2);   // waiting thread
  NumCreated += OS_AddThread(&Wait2,128,2);   // waiting thread
  NumCreated += OS_AddThread(&Wait3,128,2);   // waiting thread
  NumCreated += OS_AddThread(&Thread6,128,5);      // idle thread to keep from crashing
 
  OS_Launch(TIME_1MS);  // 1ms, doesn't return, interrupts enabled in here
  return 0;             // this never executes
}

//*******************Measurement of context switch time**********
// Run this to measure the time it takes to perform a task switch
// UART0 not needed 
// SYSTICK interrupts, period established by OS_Launch
// first timer not needed
// second timer not needed
// SW1 not needed, 
// SW2 not needed
// logic analyzer on PF1 for systick interrupt (in your OS)
//                on PD0 to measure context switch time
void ThreadCS(void){       // only thread running
  while(1){
    PD0 ^= 0x01;      // debugging profile  
  }
}
int TestmainCS(void){       // TestmainCS
  PortD_Init();
  OS_Init(false);           // initialize, disable interrupts
  NumCreated = 0 ;
  NumCreated += OS_AddThread(&ThreadCS,128,0); 
  OS_Launch(TIME_1MS); // 100us, doesn't return, interrupts enabled in here
  return 0;             // this never executes
}

//context switch takes 3.708 us 





//*******************Trampoline for selecting main to execute**********
int main(void) {       // main 
  realmain();
}
