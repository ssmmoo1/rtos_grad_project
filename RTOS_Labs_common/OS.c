// *************os.c**************
// EE445M/EE380L.6 Labs 1, 2, 3, and 4 
// High-level OS functions
// Students will implement these functions as part of Lab
// Runs on LM4F120/TM4C123
// Jonathan W. Valvano 
// Jan 12, 2020, valvano@mail.utexas.edu


#include <stdint.h>
#include <stdio.h>
#include "../inc/tm4c123gh6pm.h"
#include "../inc/CortexM.h"
#include "../inc/PLL.h"
#include "../inc/LaunchPad.h"
#include "../inc/Timer4A.h"
#include "../inc/WTimer0A.h"
#include "../RTOS_Labs_common/OS.h"
#include "../RTOS_Labs_common/ST7735.h"
#include "../inc/ADCT0ATrigger.h"
#include "../RTOS_Labs_common/UART0int.h"
#include "../RTOS_Labs_common/eFile.h"
#include "../inc/WTimer1A.h"
#include "../RTOS_Labs_common/PortFInt.h"
#include "TaskList.h"

#define LOW_PRIORITY 5

void OS_UpdateSleep(void);
void OS_IncrementMsTime(void);

//Functions from OSasm.s
void StartOS(void);
void ContextSwitch(void);


// system time variable
static uint32_t systemTime;
static uint32_t msTimeOffset = 0; //used to offset time to 0 when MS time is cleared. 
static uint32_t time_slice_ms = 1; //number of MS for a time slice. Set in OS_Launch. Systick always at 1 ms

// pool of TCBs to draw from
#define MAX_THREADS 24
static TCBType TCBPool[MAX_THREADS];

// TCB ready list
//TCBType *tcbReadyList = NULL;


TCBType *tcbReadyList[LOW_PRIORITY+1];



// run pointer
TCBType *currentTCB = NULL;

// TCB sleep list
TCBType *sleepList = NULL;

// Performance Measurements 
uint32_t const JitterSize=JITTERSIZE;
uint32_t JitterHistogram_1[JITTERSIZE]={0,};
uint32_t JitterHistogram_2[JITTERSIZE]={0,};
uint32_t MaxJitter_1 = 0;
uint32_t MaxJitter_2 = 0;

//Global Fifo setup
//uses 2 semaphore implementation. ASSUMES PUTS WILL NOT BE INTERRUPTED not sure if this is safe but I think it is.
#define OS_FIFO_SIZE_MAX 128
uint32_t volatile *OS_Fifo_Put_Pt;
uint32_t volatile *OS_Fifo_Get_Pt;
uint32_t static OS_Fifo_Arr[OS_FIFO_SIZE_MAX];
uint32_t static OS_Fifo_Size_v; //user defined fifo size must be smaller than OS_FIFO_SIZE_MAX
Sema4Type OS_Fifo_Size_Sema;
Sema4Type OS_Fifo_mutex;

//Global Mailbox setup
//uses 2 semaphores. ASSUMES IT IS ONLY USED BY FOREGROUND THREADS
Sema4Type OS_Mail_Send_Sema;
Sema4Type OS_Mail_Ack_Sema;
uint32_t OS_Mail_data;


/*------------------------------------------------------------------------------
  Idle Task
  Task that does nothing
  used so the scheduler always has something to run
 *------------------------------------------------------------------------------*/
static void OS_TaskIdle(void) {
	while (1) {
		OS_Suspend();
	}
}


/*------------------------------------------------------------------------------
  Systick Interrupt Handler
  SysTick interrupt happens every 10 ms
  used for preemptive thread switch
 *------------------------------------------------------------------------------*/
void SysTick_Handler(void) {
	static int systick_counter = 0; //used to track when a context switch should occur
  
  systick_counter+=1;
  
	OS_IncrementMsTime();
	
  if(systick_counter % time_slice_ms == 0)
  {
    ContextSwitch();
  }
  
	OS_UpdateSleep();

} // end SysTick_Handler

unsigned long OS_LockScheduler(void){
  // lab 4 might need this for disk formating
  return 0;// replace with solution
}
void OS_UnLockScheduler(unsigned long previous){
  // lab 4 might need this for disk formating
}


/*
*Scheduler gets called inside PendSV and moves the run pointer to the next appropriate thread
*Currently round robin with sleep support. Will loop forever if all threads are asleep
*Assumes interrupts are disabled when this function is called
*/
void OS_Scheduler(void)
{
	//find next ready thread
	//TaskList_Iterate(&tcbReadyList);
  //currentTCB = tcbReadyList;
  
  int max_pri = LOW_PRIORITY;
  for(int i = 0; i < LOW_PRIORITY + 1; i++)
  {
      if(tcbReadyList[i] != NULL)
      {
        max_pri = i;
        break;
      }
  }
  TaskList_Iterate(&(tcbReadyList[max_pri]));
  currentTCB = tcbReadyList[max_pri];
  
}

void SysTick_Init(unsigned long period){
  NVIC_ST_CTRL_R = 0;                   // disable SysTick during setup
  NVIC_ST_RELOAD_R = (period-1);            // set reload value
  NVIC_ST_CURRENT_R = 0;                // any write to current clears it
	NVIC_SYS_PRI3_R = (NVIC_SYS_PRI3_R&0x00FFFFFF)|0xC0000000; // priority 6 (just above PendSv)
                                        // enable SysTick with core clock and interrupts
  NVIC_ST_CTRL_R = NVIC_ST_CTRL_ENABLE | NVIC_ST_CTRL_CLK_SRC | NVIC_ST_CTRL_INTEN;
}

/*
*Place this function call into a periodic task and pass in the expected period
*Each time this function is called it will calculate the time difference with the last time it was called
*The jitter is calculated the difference from the expected period and put into the jitter array
*/

void OS_Jitter_1(uint32_t expected_period){ 
  static uint32_t LastTime = 0;  // time at previous ADC sample
	//only execute on first call to setup LastTime
	if(LastTime == 0)
	{
		LastTime = OS_Time();
		return;
	}
	
  uint32_t thisTime;              // time at current function call
  long jitter;                    // time between measured and expected, in us

	thisTime = OS_Time();       // current time, 12.5 ns
   
	uint32_t diff = OS_TimeDifference(LastTime,thisTime);
	if(diff > expected_period){
		jitter = (diff-expected_period);  // in 0.1 usec
    //jitter = (diff-expected_period+4)/8;  // in 0.1 usec
	}else{
		jitter = (expected_period-diff);  // in 0.1 usec
	}
	if(jitter > MaxJitter_1){
		MaxJitter_1 = jitter; // in usec
	}       // jitter should be 0
	if(jitter >= JitterSize){
		jitter = JitterSize-1;
	}
	JitterHistogram_1[jitter]++; 
	LastTime = thisTime;
}


/*
*Same as OS_Jitter_1. Populates a new histogram and max jitter value
*/
void OS_Jitter_2(uint32_t expected_period){ 
  static uint32_t LastTime = 0;  // time at previous ADC sample
	//only execute on first call to setup LastTime
	if(LastTime == 0)
	{
		LastTime = OS_Time();
		return;
	}
	
  uint32_t thisTime;              
  long jitter;                    // time between measured and expected, in us

	thisTime = OS_Time();       // current time, 12.5 ns
   
	uint32_t diff = OS_TimeDifference(LastTime,thisTime);
	if(diff > expected_period){
		//jitter = (diff-expected_period+4)/8;  // in 0.1 usec
    jitter = (diff-expected_period);
	}else{
		//jitter = (expected_period-diff+4)/8;  // in 0.1 usec
    jitter = (expected_period-diff);
	}
	if(jitter > MaxJitter_2){
		MaxJitter_2 = jitter; // in usec
	}       // jitter should be 0
	if(jitter >= JitterSize){
		jitter = JitterSize-1;
	}
	JitterHistogram_2[jitter]++; 
	LastTime = thisTime;
}


/**
 * @details  Initialize operating system, disable interrupts until OS_Launch.
 * Initialize OS controlled I/O: serial, ADC, systick, LaunchPad I/O and timers.
 * Interrupts not yet enabled.
 * @param  none
 * @return none
 * @brief  Initialize OS
 */
void OS_Init(void){
  // put Lab 2 (and beyond) solution here
	PLL_Init(Bus80MHz);
	ST7735_InitR(INITR_REDTAB); // LCD initialization
	UART_Init(); //UART Init
	DisableInterrupts();
	
	// make PendSV lowest priority (7)
	NVIC_SYS_PRI3_R |= 0x00E00000;
	
	// mark all TCBs as invalid on startup
	for (uint32_t i = 0; i < MAX_THREADS; ++i) {
		TCBPool[i].valid = false;
	}
	
	//OS_AddThread(OS_TaskIdle, DEFAULT_STACK_SIZE, LOW_PRIORITY);
}; 

// ******** OS_InitSemaphore ************
// initialize semaphore 
// input:  pointer to a semaphore
// output: none
void OS_InitSemaphore(Sema4Type *semaPt, int32_t value){
  // put Lab 2 (and beyond) solution here
	semaPt->Value = value;
	semaPt->waiters = NULL;
}; 

// ******** OS_Wait ************
// decrement semaphore 
// Lab2 spinlock
// Lab3 block if less than zero
// input:  pointer to a counting semaphore
// output: none
void OS_Wait(Sema4Type *semaPt){
  // put Lab 2 (and beyond) solution here
	DisableInterrupts();
	semaPt->Value--;
	if (semaPt->Value < 0) {
		//first remove from ready list
    TaskList_PopFront(&(tcbReadyList[currentTCB->priority]));
    
    // block this task and add it to the semaphore's list of waiters
		TaskList_PushBack(&(semaPt->waiters), currentTCB);
		currentTCB->blocked = (void *)semaPt;
		
    EnableInterrupts();
		OS_Suspend();
	}
	EnableInterrupts();
} 

// ******** OS_Signal ************
// increment semaphore 
// Lab2 spinlock
// Lab3 wakeup blocked thread if appropriate 
// input:  pointer to a counting semaphore
// output: none
void OS_Signal(Sema4Type *semaPt){
  // put Lab 2 (and beyond) solution here
	long status = StartCritical();
	if (semaPt->Value < 0) {
		// unblock one waiting thread
		TCBType *unblock = TaskList_PopFront(&(semaPt->waiters));
		unblock->blocked = NULL;
		TaskList_PushBack(&(tcbReadyList[unblock->priority]), unblock);
	}
	semaPt->Value++;
  EndCritical(status);
	OS_Suspend();
	
}; 

// ******** OS_bWait ************
// Lab2 spinlock, set to 0
// Lab3 block if less than zero
// input:  pointer to a binary semaphore
// output: none
void OS_bWait(Sema4Type *semaPt){
  // put Lab 2 (and beyond) solution here
	DisableInterrupts();
  semaPt->Value--;
	if (semaPt->Value < 0) {
    TaskList_PopFront(&(tcbReadyList[currentTCB->priority]));
		// block this task and add it to the semaphore's list of waiters
		TaskList_PushBack(&(semaPt->waiters), currentTCB);
		currentTCB->blocked = (void *)semaPt;
		
    EnableInterrupts();
    OS_Suspend();
   
	}
	EnableInterrupts();
}; 

// ******** OS_bSignal ************
// Lab2 spinlock, set to 1
// Lab3 wakeup blocked thread if appropriate 
// input:  pointer to a binary semaphore
// output: none
void OS_bSignal(Sema4Type *semaPt){
  // put Lab 2 (and beyond) solution here
	long status = StartCritical();
	if (semaPt->Value < 0) {
		// unblock one waiting thread
		TCBType *unblock = TaskList_PopFront(&(semaPt->waiters));
		unblock->blocked = NULL;
		TaskList_PushBack(&(tcbReadyList[unblock->priority]), unblock);
	}
  semaPt->Value = (semaPt->Value >= 0) ? 1 : semaPt->Value+1; //increment if negative but do not go past 1
	EndCritical(status);
  OS_Suspend();
	
}; 


long* OS_InitStack(long *sp, void(*task)(void))
{
	//*(sp) = (long)task;
	//first push the registers interrupts handle R0-R3, R12, LR, PC, PSR
	*(sp) = 0x01000000; //PSR needs thumb bit
	*(--sp) = (long)task; //r14 PC
	*(--sp) = 0x13131313; //r13 LR
	*(--sp) = 0x12121212; //r12
	*(--sp) = 0x03030303; //r3
	*(--sp) = 0x02020202; //r2
	*(--sp) = 0x01010101; //r1
	*(--sp) = 0x00000000; //r0
	
	//push registers not handled by ISR r11-r4
	*(--sp) = (long) 0x11111111; //r11
	*(--sp) = (long) 0x10101010L; //r10
	*(--sp) = (long) 0x09090909L; //r9
	*(--sp) = (long) 0x08080808L; //r8
	*(--sp) = (long) 0x07070707L; //r7
	*(--sp) = (long) 0x06060606L; //r6
	*(--sp) = (long) 0x05050505L; //r5
	*(--sp) = (long) 0x04040404L; //r4
	
	return sp;
}


//******** OS_AddThread *************** 
// add a foregound thread to the scheduler
// Inputs: pointer to a void/void foreground task
//         number of bytes allocated for its stack
//         priority, 0 is highest, 5 is the lowest
// Outputs: 1 if successful, 0 if this thread can not be added
// stack size must be divisable by 8 (aligned to double word boundary)
// In Lab 2, you can ignore both the stackSize and priority fields
// In Lab 3, you can ignore the stackSize fields
int OS_AddThread(void(*task)(void), 
   uint32_t stackSize, uint32_t priority){
  // put Lab 2 (and beyond) solution here
	static uint32_t idCounter = 0;
	long sr;

	// allocate a TCB
	TCBType *tcb = NULL;
	for (uint32_t i = 0; i < MAX_THREADS; ++i) {
		if (TCBPool[i].valid == false) {
			tcb = &(TCBPool[i]);
			break;
		}
	}
	// error if no more TCBs available
	if (tcb == NULL) {
		return 0;
	}
	
	// initialize TCB
	// set up stack pointer
	tcb->sp = (char *)tcb + sizeof(TCBType) - sizeof(uint32_t);
	//push fake register values onto stack
	tcb->sp = OS_InitStack(tcb->sp, task);
	
	
	// increment id counter and set thread id (critical section)
	sr = StartCritical();
	tcb->id = idCounter++;
	EndCritical(sr);
	// do not start thread sleeping or blocked
	tcb->sleepCounter = 0;
	tcb->blocked = NULL;
	// set priority and mark thread as valid
	tcb->priority = priority;
	tcb->valid = true;
	
	// add TCB to ready list (critical section)
	TaskList_PushBack(&(tcbReadyList[priority]), tcb);
     
	// success
  return 1;
};

//******** OS_AddProcess *************** 
// add a process with foregound thread to the scheduler
// Inputs: pointer to a void/void entry point
//         pointer to process text (code) segment
//         pointer to process data segment
//         number of bytes allocated for its stack
//         priority (0 is highest)
// Outputs: 1 if successful, 0 if this process can not be added
// This function will be needed for Lab 5
// In Labs 2-4, this function can be ignored
int OS_AddProcess(void(*entry)(void), void *text, void *data, 
  unsigned long stackSize, unsigned long priority){
  // put Lab 5 solution here

     
  return 0; // replace this line with Lab 5 solution
}


//******** OS_Id *************** 
// returns the thread ID for the currently running thread
// Inputs: none
// Outputs: Thread ID, number greater than zero 
uint32_t OS_Id(void){
  // put Lab 2 (and beyond) solution here
  
  return 0; // replace this line with solution
};


//******** OS_AddPeriodicThread *************** 
// add a background periodic task
// typically this function receives the highest priority
// Inputs: pointer to a void/void background function
//         period given in system time units (12.5ns)
//         priority 0 is the highest, 5 is the lowest
// Outputs: 1 if successful, 0 if this thread can not be added
// You are free to select the time resolution for this function
// It is assumed that the user task will run to completion and return
// This task can not spin, block, loop, sleep, or kill
// This task can call OS_Signal  OS_bSignal   OS_AddThread
// This task does not have a Thread ID
// In lab 1, this command will be called 1 time
// In lab 2, this command will be called 0 or 1 times
// In lab 2, the priority field can be ignored
// In lab 3, this command will be called 0 1 or 2 times
// In lab 3, there will be up to four background threads, and this priority field 
//           determines the relative priority of these four threads
int OS_AddPeriodicThread(void(*task)(void), uint32_t period, uint32_t priority)
{
	static int pthread_count = 0;
	
	if(pthread_count == 0)
	{
		WideTimer0A_Init(task, period, priority);
		pthread_count+=1;
	}
	else if(pthread_count == 1)
	{
		WideTimer1A_Init(task, period, priority);
		pthread_count+=1;
	}
	else
	{
		//Hit max number of supported periodic threads
		return 0;
	}
     
  return 1; // replace this line with solution
};




//******** OS_AddSW1Task *************** 
// add a background task to run whenever the SW1 (PF4) button is pushed
// Inputs: pointer to a void/void background function
//         priority 0 is the highest, 5 is the lowest
// Outputs: 1 if successful, 0 if this thread can not be added
// It is assumed that the user task will run to completion and return
// This task can not spin, block, loop, sleep, or kill
// This task can call OS_Signal  OS_bSignal   OS_AddThread
// This task does not have a Thread ID
// In labs 2 and 3, this command will be called 0 or 1 times
// In lab 2, the priority field can be ignored
// In lab 3, there will be up to four background threads, and this priority field 
//           determines the relative priority of these four threads
int OS_AddSW1Task(void(*task)(void), uint32_t priority){
  // put Lab 2 (and beyond) solution here
	static bool used = 0;
	if(used == true)
	{
		return 0;
	}
	
	long sr = StartCritical();
	
	// initialize port F with edge triggered ints
	GPIOPortF_Int_Setup(priority);

	SW1Task = task;
	
	EndCritical(sr);
	used = true;
  return 1; // replace this line with solution
};

//******** OS_AddSW2Task *************** 
// add a background task to run whenever the SW2 (PF0) button is pushed
// Inputs: pointer to a void/void background function
//         priority 0 is highest, 5 is lowest
// Outputs: 1 if successful, 0 if this thread can not be added
// It is assumed user task will run to completion and return
// This task can not spin block loop sleep or kill
// This task can call issue OS_Signal, it can call OS_AddThread
// This task does not have a Thread ID
// In lab 2, this function can be ignored
// In lab 3, this command will be called will be called 0 or 1 times
// In lab 3, there will be up to four background threads, and this priority field 
//           determines the relative priority of these four threads
int OS_AddSW2Task(void(*task)(void), uint32_t priority){
  // put Lab 2 (and beyond) solution here
  static bool used = false;
	
	if(used == true)
	{
		return 0;
	}
	
	long sr = StartCritical();
	// initialize port F with edge triggered ints
	GPIOPortF_Int_Setup(priority);
	SW2Task = task;
	EndCritical(sr);
	used = true;
	return 1;
};


// ******** OS_Sleep ************
// place this thread into a dormant state
// input:  number of msec to sleep
// output: none
// You are free to select the time resolution for this function
// OS_Sleep(0) implements cooperative multitasking
void OS_Sleep(uint32_t sleepTime){
  // put Lab 2 (and beyond) solution here
	DisableInterrupts();
  currentTCB->sleepCounter = sleepTime;
	
	// add to sleep list and remove from ready list
	TaskList_PopFront(&(tcbReadyList[currentTCB->priority]));
	TaskList_PushBack(&sleepList, currentTCB);
	
  //tcbReadyList = tcbReadyList->prev; //move back one otherwisee a thread will be skipped on the next context switch
  
	OS_Suspend();
	EnableInterrupts();
};  

// ******** OS_Kill ************
// kill the currently running thread, release its TCB and stack
// input:  none
// output: none
void OS_Kill(void){
  // put Lab 2 (and beyond) solution here
	DisableInterrupts();
	
	currentTCB->valid = false;
	//remove head from linked list
	TaskList_PopFront(&(tcbReadyList[currentTCB->priority]));

	OS_Suspend();
	EnableInterrupts();
	//should immedietly context switch to next thread
	
    
}; 

// ******** OS_Suspend ************
// suspend execution of currently running thread
// scheduler will choose another thread to execute
// Can be used to implement cooperative multitasking 
// Same function as OS_Sleep(0)
// input:  none
// output: none
void OS_Suspend(void){
  // put Lab 2 (and beyond) solution here
 
	ContextSwitch();

};
  
// ******** OS_Fifo_Init ************
// Initialize the Fifo to be empty
// Inputs: size
// Outputs: none 
// In Lab 2, you can ignore the size field
// In Lab 3, you should implement the user-defined fifo size
// In Lab 3, you can put whatever restrictions you want on size
//    e.g., 4 to 64 elements
//    e.g., must be a power of 2,4,8,16,32,64,128
// Synchronizes on OS_Fifo_Sema;
// 



void OS_Fifo_Init(uint32_t size){
  // put Lab 2 (and beyond) solution here
	size = (size < OS_FIFO_SIZE_MAX) ? size: OS_FIFO_SIZE_MAX;//input size checking 
	OS_Fifo_Size_v = size; //user defined size
	OS_InitSemaphore(&OS_Fifo_mutex, 1);
	OS_InitSemaphore(&OS_Fifo_Size_Sema, 0);
	OS_Fifo_Get_Pt = OS_Fifo_Put_Pt = &OS_Fifo_Arr[0]; //starts at index 0 address
	
	
  
};

// ******** OS_Fifo_Put ************
// Enter one data sample into the Fifo
// Called from the background, so no waiting 
// Inputs:  data
// Outputs: true if data is properly saved,
//          false if data not saved, because it was full
// Since this is called by interrupt handlers 
//  this function can not disable or enable interrupts
int OS_Fifo_Put(uint32_t data){
 
	if(OS_Fifo_Size_Sema.Value == OS_Fifo_Size_v)
	{
		return 0;
	}
	
	*(OS_Fifo_Put_Pt) = data;
	OS_Fifo_Put_Pt++;
	if(OS_Fifo_Put_Pt == &OS_Fifo_Arr[OS_Fifo_Size_v])
	{
		OS_Fifo_Put_Pt = &OS_Fifo_Arr[0]; //wrap around
	}
	
	OS_Signal(&OS_Fifo_Size_Sema);
	return 1;
	
	};  

// ******** OS_Fifo_Get ************
// Remove one data sample from the Fifo
// Called in foreground, will spin/block if empty
// Inputs:  none
// Outputs: data 
uint32_t OS_Fifo_Get(void){
  // put Lab 2 (and beyond) solution here
	uint32_t ret_data;
	OS_Wait(&OS_Fifo_Size_Sema); //wait for available item
	OS_Wait(&OS_Fifo_mutex); //lock fifo once available
	ret_data = *(OS_Fifo_Get_Pt);
	OS_Fifo_Get_Pt++;
	if(OS_Fifo_Get_Pt == &OS_Fifo_Arr[OS_Fifo_Size_v])
	{
		OS_Fifo_Get_Pt = &OS_Fifo_Arr[0]; //loop around
	}
	OS_Signal(&OS_Fifo_mutex);
  return ret_data; // replace this line with solution
};

// ******** OS_Fifo_Size ************
// Check the status of the Fifo
// Inputs: none
// Outputs: returns the number of elements in the Fifo
//          greater than zero if a call to OS_Fifo_Get will return right away
//          zero or less than zero if the Fifo is empty 
//          zero or less than zero if a call to OS_Fifo_Get will spin or block
int32_t OS_Fifo_Size(void){
  // put Lab 2 (and beyond) solution here
   
  return OS_Fifo_Size_Sema.Value; // replace this line with solution
};


// ******** OS_MailBox_Init ************
// Initialize communication channelOS_Fifo_Size
// Inputs:  none
// Outputs: none
void OS_MailBox_Init(void){
  //set up 2 semaphores
	
  OS_InitSemaphore(&OS_Mail_Send_Sema, 0);
	OS_InitSemaphore(&OS_Mail_Ack_Sema, 1);
};

// ******** OS_MailBox_Send ************
// enter mail into the MailBox
// Inputs:  data to be sent
// Outputs: none
// This function will be called from a foreground thread
// It will spin/block if the MailBox contains data not yet received 
void OS_MailBox_Send(uint32_t data){
  // put Lab 2 (and beyond) solution here
  // put solution here
	OS_Wait(&OS_Mail_Ack_Sema);
	OS_Mail_data = data;
	OS_Signal(&OS_Mail_Send_Sema);
};

// ******** OS_MailBox_Recv ************
// remove mail from the MailBox
// Inputs:  none
// Outputs: data received
// This function will be called from a foreground thread
// It will spin/block if the MailBox is empty 
uint32_t OS_MailBox_Recv(void){
  // put Lab 2 (and beyond) solution here
  uint32_t ret_data;
	OS_Wait(&OS_Mail_Send_Sema);
	ret_data = OS_Mail_data;
	OS_Signal(&OS_Mail_Ack_Sema);
	return ret_data;
};

// ******** OS_Time ************
// return the system time 
// Inputs:  none
// Outputs: time in 12.5ns units, 0 to 4294967295
// The time resolution should be less than or equal to 1us, and the precision 32 bits
// It is ok to change the resolution and precision of this function as long as 
//   this function and OS_TimeDifference have the same resolution and precision 
uint32_t OS_Time(void){
  // put Lab 2 (and beyond) solution here
	long sr = StartCritical();
	uint32_t retval = systemTime * (NVIC_ST_RELOAD_R + 1) + (NVIC_ST_RELOAD_R - NVIC_ST_CURRENT_R);
	EndCritical(sr);
  return retval;
};

// ******** OS_TimeDifference ************
// Calculates difference between two times
// Inputs:  two times measured with OS_Time
// Outputs: time difference in 12.5ns units 
// The time resolution should be less than or equal to 1us, and the precision at least 12 bits
// It is ok to change the resolution and precision of this function as long as 
//   this function and OS_Time have the same resolution and precision 
uint32_t OS_TimeDifference(uint32_t start, uint32_t stop){
  // put Lab 2 (and beyond) solution here
  return (stop - start); // return delta scaled by bus frequency to 0.1us
};


/*
*Should be called periodically to decrement the sleep values in the TCBs
*
*/
void OS_UpdateSleep(void)
{
  
  if(sleepList == NULL)
  {
    return;
  }
  
	TCBType *start_tcb = sleepList;
	
	do
	{
		if (sleepList->sleepCounter == 0) {
			TCBType *wakeTcb = TaskList_PopFront(&sleepList);
			TaskList_PushBack(&(tcbReadyList[wakeTcb->priority]), wakeTcb);
		}
    //handle case if PopFront causes an empty sleep list
		if(sleepList == NULL)
    {
      break; 
		}
    else //if more elements then decrement and continue
    {
      sleepList->sleepCounter--;
      TaskList_Iterate(&sleepList);
    }
	}
	while(start_tcb != sleepList);
	
}


// private function for incrementing system time
static void OS_IncrementMsTime(void) {
	systemTime++;
}

// ******** OS_ClearMsTime ************
// sets the system time to zero (solve for Lab 1), and start a periodic interrupt
// Inputs:  none
// Outputs: none
// You are free to change how this works
void OS_ClearMsTime(void){
  // put Lab 1 solution here
	msTimeOffset = systemTime;
};

// ******** OS_MsTime ************
// reads the current time in msec (solve for Lab 1)
// Inputs:  none
// Outputs: time in ms units
// You are free to select the time resolution for this function
// For Labs 2 and beyond, it is ok to make the resolution to match the first call to OS_AddPeriodicThread
uint32_t OS_MsTime(void){
  // put Lab 1 solution here
  return systemTime - msTimeOffset;
};


//******** OS_Launch *************** 
// start the scheduler, enable interrupts
// Inputs: number of 12.5ns clock cycles for each time slice
//         you may select the units of this parameter
// Outputs: none (does not return)
// In Lab 2, you can ignore the theTimeSlice field
// In Lab 3, you should implement the user-defined TimeSlice field
// It is ok to limit the range of theTimeSlice to match the 24-bit SysTick
void OS_Launch(uint32_t theTimeSlice){
  // put Lab 2 (and beyond) solution here
  if(tcbReadyList[LOW_PRIORITY] == NULL)
	{
		return;
	}
	
	else
	{
		// initialize Systick for preemptive scheduling
		SysTick_Init(80000); // 1 ms reload value. Should always be 1ms for correct timing 
    time_slice_ms = theTimeSlice / 80000; //set time slice in whole milliseconds. 
		OS_ClearMsTime();
    currentTCB = tcbReadyList[LOW_PRIORITY];
		StartOS();
	}
    
};

//************** I/O Redirection *************** 
// redirect terminal I/O to UART or file (Lab 4)

int StreamToDevice=0;                // 0=UART, 1=stream to file (Lab 4)

int fputc (int ch, FILE *f) { 
  if(StreamToDevice==1){  // Lab 4
    if(eFile_Write(ch)){          // close file on error
       OS_EndRedirectToFile(); // cannot write to file
       return 1;                  // failure
    }
    return 0; // success writing
  }
  
  // default UART output
  UART_OutChar(ch);
  return ch; 
}

int fgetc (FILE *f){
  char ch = UART_InChar();  // receive from keyboard
  UART_OutChar(ch);         // echo
  return ch;
}

int OS_RedirectToFile(const char *name){  // Lab 4
  eFile_Create(name);              // ignore error if file already exists
  if(eFile_WOpen(name)) return 1;  // cannot open file
  StreamToDevice = 1;
  return 0;
}

int OS_EndRedirectToFile(void){  // Lab 4
  StreamToDevice = 0;
  if(eFile_WClose()) return 1;    // cannot close file
  return 0;
}

int OS_RedirectToUART(void){
  StreamToDevice = 0;
  return 0;
}

int OS_RedirectToST7735(void){
  
  return 1;
}

