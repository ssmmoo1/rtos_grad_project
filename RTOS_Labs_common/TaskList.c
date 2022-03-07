/**
 * Library for handling linked lists of TCBs
 */

#include <stdio.h>
#include "TaskList.h"
#include "OS.h"
#include "../inc/CortexM.h"

// set the head to the next tcb in the list
void TaskList_Iterate(TCBType **list) {
	long sr = StartCritical();
	while ((uint32_t)(*list)->next > 0xF0000000 || (uint32_t)(*list)->next < 0x20000000);	// debug assertion
	*list = (*list)->next;
	while (((uint32_t)(*list) > 0xF0000000) || (uint32_t)(*list) < 0x20000000);	// debug assertion
	EndCritical(sr);
}

// add the tcb to the end of the list
void TaskList_PushBack(TCBType **list, TCBType *tcb) {
	long sr = StartCritical();
	if (*list == NULL) {
		// add at front if list is empty
		*list = tcb;
		while (((uint32_t)(*list) > 0xF0000000) || (uint32_t)(*list) < 0x20000000);	// debug assertion
		(*list)->next = tcb;
		(*list)->prev = tcb;
	} else {
		// add at end otherwise
		TCBType *oldEnd = (*list)->prev;
		oldEnd->next = tcb;
		tcb->prev = oldEnd;
		(*list)->prev = tcb;
		tcb->next = (*list);
	}
	EndCritical(sr);
}
	

// remove the specified element from the list
// without modifying the head pointer
/*
void TaskList_Remove(TCBType *list, TCBType *elem) {
	long sr = StartCritical();
	TCBType *pos = list;
	do {
		pos = pos->next;
	} while ((pos != elem) && (pos != list));
	if (pos == elem) {
		list->next->prev = list->prev;
		list->prev->next = list->next;
	}
	EndCritical(sr);
}
*/

// remove and return the head of the list
TCBType *TaskList_PopFront(TCBType **list) {
	long sr = StartCritical();
	TCBType *head = *list;
	
	head->prev->next = head->next;
	head->next->prev = head->prev;
	
	while (*list == NULL);		// debug
	
	if (*list != (*list)->next) {
		*list = (*list)->next;
		while (((uint32_t)(*list) > 0xF0000000) || (uint32_t)(*list) < 0x20000000);	// debug assertion
	} else {
		*list = NULL;
	}
	EndCritical(sr);
	return head;
}
	
