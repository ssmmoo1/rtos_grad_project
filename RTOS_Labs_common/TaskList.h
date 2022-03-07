/**
 * Library for handling linked lists of TCBs
 */

#ifndef TASKLIST_H
#define TASKLIST_H

#include "OS.h"

// set the head to the next tcb in the list
void TaskList_Iterate(TCBType **list);

// add the tcb to the end of the list
void TaskList_PushBack(TCBType **list, TCBType *tcb);

// remove the specified element from the list
// without modifying the head pointer
//void TaskList_Remove(TCBType *list, TCBType *elem);

// remove and return the head of the list
TCBType *TaskList_PopFront(TCBType **list);
 
#endif
