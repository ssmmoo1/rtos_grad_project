// *************Interpreter.c**************
// Students implement this as part of EE445M/EE380L.12 Lab 1,2,3,4 
// High-level OS user interface
// 
// Runs on LM4F120/TM4C123
// Jonathan W. Valvano 1/18/20, valvano@mail.utexas.edu
#include <stdint.h>
#include <string.h> 
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>		// for atoi
#include "../RTOS_Labs_common/OS.h"
#include "../RTOS_Labs_common/ST7735.h"
#include "../inc/ADCT0ATrigger.h"
#include "../inc/ADCSWTrigger.h"
#include "../RTOS_Labs_common/UART0int.h"
#include "../RTOS_Labs_common/eDisk.h"
#include "../RTOS_Labs_common/eFile.h"
#include "ADC.h"
#include "../inc/Launchpad.h"

#define PD1  (*((volatile uint32_t *)0x40007008))

static const uint8_t MAX_TOKENS = 6;
static const uint8_t MAX_COMMAND_LENGTH = 30;
extern uint32_t NumCreated;
int32_t MaxJitter;

// Print jitter histogram
void Jitter(int32_t MaxJitter, uint32_t const JitterSize, uint32_t JitterHistogram[]){
  // write this for Lab 3 (the latest)
	
}

// helper function for checking if two strings are equal
static inline bool strEquals(const char *a, const char *b) {
	return strcmp(a, b) == 0;
}

// inString holds command to be parsed and ran
static void handleCommand(char *inString) {
	static char commandTokens[MAX_TOKENS + 1][MAX_COMMAND_LENGTH];
	
	char *tok = inString;
	char *savePtr;
	uint8_t numTokens = 0;
	
	
	//Store 1 argument that is quoted. Used for lcd message
	char msg_tok[30] = {0}; //stores a token entered with quotes
	uint8_t msg_tok_en = 0;
	char* quote_msg_start = strchr(inString, '"') + 1;
	char* quote_msg_end = strrchr(inString, '"');
	if(quote_msg_start != NULL && quote_msg_end != NULL)
	{
		msg_tok_en = 1;
		memcpy(msg_tok, quote_msg_start, (quote_msg_end - quote_msg_start));
	}
	
	
//	static char outString[MAX_COMMAND_LENGTH];
//	 UART_OutString("Entering handleCommand()...\n\r");
	
	
	// tokenize the input command
	while ((tok != NULL) && (numTokens < MAX_TOKENS)) {
		tok = strtok_r(inString, " ", &savePtr);
		inString = NULL;
		strcpy(commandTokens[numTokens], tok);
		if (tok != NULL) {
			++numTokens;
		}
	}
	
//	sprintf(outString, "numTokens:%d\n\r", numTokens);
//	UART_OutString(outString);
	
	// return if no command
	if (numTokens == 0) {
		return;
	}
	
	if (numTokens > MAX_TOKENS) {
		UART_OutString("Error! Too many arguments");
		return;
	}
	
	// check which command to run
	// TODO: add LCD commands
	if (strEquals(commandTokens[0], "time")) {
		// OS time command
		// "time" to see the system time
		// "time -c" or "time --clear" to clear system time
		// "time -?" or "time --help" for help
		if (numTokens == 1) {
			uint32_t sysTime = OS_MsTime();
			UART_OutUDec(sysTime);
		} else if (strEquals(commandTokens[1], "-c") || strEquals(commandTokens[1], "--clear")) {
			OS_ClearMsTime();
		} else if (strEquals(commandTokens[1], "-?") || strEquals(commandTokens[1], "--help")) {
			UART_OutString("Usage: time [OPTION]\n\r");
			UART_OutString("       Get the current system time\n\r\n\r");
			UART_OutString("       Optional arguments\n\r\n\r");
			UART_OutString("        -c, --clear   set system time to zero and start system time periodic interrupt\n\r");
			UART_OutString("        -?, --help    display this help message\n\r");
		} else {
			UART_OutString("time: unknown arguement: ");
			UART_OutString(commandTokens[1]);
			UART_OutString("\n\rTry time --help for more information\n\r");
		}
	} else if (strEquals(commandTokens[0], "adc")) {
		// ACD command
		// "adc" to get most recent ADC reading
		// "adc <channel>" to initialize ADC on a specific channel
		if (numTokens == 1) {
			uint32_t adcVal = ADC_In();
			UART_OutUDec(adcVal);
		} else {
			uint32_t channel = (uint32_t) atoi(commandTokens[1]);
			if (channel <= 11) {
				ADC_Init(channel);
			} else {
				UART_OutString("adc: Error unknown argument: ");
				UART_OutString(commandTokens[1]);
			}
		}
	} else if (strEquals(commandTokens[0], "led")) {
		PF2 ^= 0x04;
	}
	// Add more commands here if needed
	
	//HELP CMD
	else if(strEquals(commandTokens[0], "help"))
	{
		
		UART_OutString("Available Commands \n\r adc \n\r time \n\r lcd \n\r led \n\r threads \n\r jitter \n\r To see more information for a command the use the flag -? \n\r Example usage: lcd -?");
		
	}
	
	else if(strEquals(commandTokens[0], "lcd"))
	{
		if(strEquals(commandTokens[1], "-c") || strEquals(commandTokens[1], "--clear")) //clear the display
		{
			ST7735_FillRect(0, 0, 150, 150, ST7735_Color565(0,0,0)); // black out the screen
			UART_OutString("LCD Cleared");
		}
		else if(strEquals(commandTokens[1], "-?") || strEquals(commandTokens[1], "--help"))
		{
			UART_OutString("Usage: lcd -m [DEVICE] [LINE] [MESSAGE]\n\r");
			UART_OutString("Usage: lcd -c \n\r");
			UART_OutString("Print to or clear LCD\n\r\n\r");
			UART_OutString("			 Required Arguments\n\r\n\r");
			UART_OutString("       DEVICE [0,1] corresponds to top or bottom half\n\r");
			UART_OutString("       LINE [0-7] determines which line number to write to\n\r");
			UART_OutString("       MESSAGE, string to be printed to lcd\n\r");
			UART_OutString("       Example Usage: lcd -m 0 3 hello \n\r\n\r");
			UART_OutString("       Optional arguments\n\r\n\r");
			UART_OutString("        -c, --clear   clears the display. Use with no other arguments\n\r");
			UART_OutString("        -?, --help    display this help message\n\r");
		}
		else if(strEquals(commandTokens[1], "-m") || strEquals(commandTokens[1], "--message")) //print message to lcd
		{
			uint8_t device_num = atoi(commandTokens[2]);
			uint8_t line_num = atoi(commandTokens[3]);
			char* out_msg = (msg_tok_en) ? msg_tok : commandTokens[4];

			ST7735_Message(device_num, line_num, out_msg, 0, 0);
		}
		
		else
		{
			UART_OutString("lcd: Error unknown argument: ");
			UART_OutString(commandTokens[1]);
		}
		
		
	}
	
	else if(strEquals(commandTokens[0], "jitter"))
	{
		UART_OutString("jitter: ");
		UART_OutUDec(MaxJitter);
	}
	
	else if(strEquals(commandTokens[0], "threads"))
	{
		UART_OutString("Number of threads: ");
		UART_OutUDec(NumCreated);
	}
	
	
}

// *********** Command line interpreter (shell) ************
void Interpreter(void){ 
  // write this  
	UART_Init();
	static char out_str[MAX_COMMAND_LENGTH];
	static char in_str[MAX_COMMAND_LENGTH];
	sprintf(out_str, "\n\rShell Starting %d\n\r", 1);
	UART_OutString(out_str);
	
	while(1)
	{
		sprintf(out_str, "Enter CMD: ");
		UART_OutString(out_str);
		UART_InString(in_str, MAX_COMMAND_LENGTH);
		UART_OutString("\n\r");
		
		UART_OutString(in_str);
		UART_OutString("\n\r");
		
		// handle command
		handleCommand(in_str);
		UART_OutString("\n\r");
		
	}
	
}


