/*
 * Story1.c
 *
 * Created: 4/15/2020 9:19:37 AM
 *  Author: Khaled
 */


/********************************************************************************
 * 								  Included Files								*
 ********************************************************************************/

/* Header file of the story */
#include "Story1.h"

#include "../../../SERVICE/FREE_RTOS/FreeRTOS.h"
#include "../../../SERVICE/FREE_RTOS/task.h"
#include "../../../SERVICE/FREE_RTOS/semphr.h"
#include "../../../SERVICE/FREE_RTOS/queue.h"

/* Used hardware driver headers */
#include "../../../ECUAL/PUSH_BUTTON/pushButton.h"
#include "../../../ECUAL/LCD/LCD.h"
#include "../../../ECUAL/KEYPAD/keypad.h"
#include "../../../MCAL/UART/UART.h"
#include <string.h>


/********************************************************************************
 * 								Preprocessor Macros								*
 ********************************************************************************/

#define PB1_PRESSED_COUNT_INITIAL_VALUE       0
#define PB2_PRESSED_COUNT_INITIAL_VALUE       0
#define PB_PRESSED                            4
#define PB_RELEASED                           0


/********************************************************************************
 * 							  Static API's Declaration							*
 ********************************************************************************/

/* The initialization task */
static void Sprint2_Story1_InitTask (void *pvParameters);

/* The push button task, which decides if the button is pressed or released */
static void Sprint2_Story1_Task_D_MONITORING_PUSHBUTTON (void *pvParameters);

/* The KEYPAD task, which Indicates if a key is pressed or not */
static void Sprint2_Story1_Task_B_MONITORING_KEYPAD (void *pvParameters);

/* The LCD task, which outputs the suitable string */
static void Sprint2_Story1_Task_A_OUTPUTTING_LCD (void *pvParameters);

/* The UART task, which indicates if a message is received through Uart or not */
static void Sprint2_Story1_Task_C_MONITORING_UART (void *pvParameters);

/********************************************************************************
 * 							  Global Static Variables							*
 ********************************************************************************/

/* initiate init task handle*/
static TaskHandle_t Sprint2_Story1_InitTaskHandle;

/* PB1 task handle */
static TaskHandle_t Sprint2_Story1_PBTaskHandle;

/* A handle used for checking if the mutex is used or not */
static TaskHandle_t Sprint3_Story1_MutexTakenByPB;

/* The mutex used in the story */
static SemaphoreHandle_t Story1_MutexHandle_PB;

/* The string received through UART and is the output on the LCD */
sint8_t receivedChar[16] = " ";

/* A semaphore related to the UART-LCD process */
static SemaphoreHandle_t Story1_BinarySemph_UART;

/* Ranges of LCD_Source: */
#define LCD_NONE			0
#define LCD_FROM_UART		1
#define LCD_FROM_KEYPAD		2

/* A global static variable to determine which module is currently using the LCD  */
static uint8_t LCD_Source = LCD_NONE;

/* Initial condition of the static global variable key */
#define KEYPAD_NOT_PRESSED	255
static uint8_t gu8_Key = KEYPAD_NOT_PRESSED;


/********************************************************************************
 * 								 API's Definitions								*
 ********************************************************************************/

void UART_APP(void)
{
	/* A counter to loop the string which will be received through the UART */
	static uint8_t counter = 0;

	/* If it's the beginning of a new string.. */
	if(counter == 0)
	{
		/* Clear the previous one to overwrite it */
		strcpy(receivedChar, "                ");
	}

	/* Store the newly received character */
	receivedChar[counter] = UART_receive();

	/* If the last received character is an Enter button.. */
	if(receivedChar[counter] == '\r')
	{
		/* Then overwrite it with a space to avoid printing it.. */
		receivedChar[counter] = ' ';
		/* ,zero the counter.. */
		counter = 0;
		/* and finally give the semaphore */
		xSemaphoreGive(Story1_BinarySemph_UART);
	}
	/* Otherwise */
	else
	{
		/* increment the counter.. */
		counter++;
	}
}

void Sprint2_Story1 (void)
{
	/* Task creations */
	xTaskCreate (Sprint2_Story1_InitTask, "S2", 100, NULL, 5,&Sprint2_Story1_InitTaskHandle);
	xTaskCreate (Sprint2_Story1_Task_A_OUTPUTTING_LCD, "A", 500, NULL, 1,NULL);
	xTaskCreate (Sprint2_Story1_Task_B_MONITORING_KEYPAD, "B", 100, NULL, 3,NULL);
	xTaskCreate (Sprint2_Story1_Task_C_MONITORING_UART, "C", 100, NULL, 4,&Sprint2_Story1_PBTaskHandle);
	xTaskCreate (Sprint2_Story1_Task_D_MONITORING_PUSHBUTTON, "D", 100, NULL, 2,&Sprint2_Story1_PBTaskHandle);

	/*Start Scheduler*/
	vTaskStartScheduler();
}

void Sprint2_Story1_InitTask (void *pvParameters)
{
	UART_ConfigType UART_InitCfg;

	UART_InitCfg.charSize = UART_EIGHT_BITS;
	UART_InitCfg.communicationMode = UART_RECEIVER_MODE;
	UART_InitCfg.interruptMode = UART_INTERRUPT_ENABLED;
	UART_InitCfg.mode = UART_ASYNCHRONOUS_MODE;
	UART_InitCfg.parityMode = UART_PARITY_DISABLED;
	UART_InitCfg.stopBits = UART_ONE_STOP_BIT;

	while(1)
	{
		/* LCD module initialize */
		LCD_init();

		/* UART module initialize */
		UART_RXC_setCallBack(&UART_APP);
		UART_init(&UART_InitCfg);

		/* push bottom module initialize */
		pushButtonInit(BTN_0);

		/* suspend the task after finishing the initialization */
		vTaskSuspend( Sprint2_Story1_InitTaskHandle );
	}
}

void Sprint2_Story1_Task_A_OUTPUTTING_LCD (void *pvParameters)
{
	/* reference tick for periodicity function */
	TickType_t xLastWakeTime;

	static uint8_t keyColumn = 0;

	/* periodicity value */
	const TickType_t xFrequency = 200;

	TaskHandle_t Sprint3_Story1_MutexTaken;

	// Initialize the xLastWakeTime variable with the current time.
	xLastWakeTime = xTaskGetTickCount();

	while(1)
	{
		vTaskDelayUntil(&xLastWakeTime, xFrequency);

		/* Storing output of the function into the mutex checker to decide whether the mutex is taken or not */
		Sprint3_Story1_MutexTaken = xSemaphoreGetMutexHolder( Story1_MutexHandle_PB );

		/* In case the UART is the user of the LCD.. */
		if(LCD_Source == LCD_FROM_UART)
		{
			/* Display the string received through the UART on the second row of the LCD */
			LCD_displayStringRowColumn(1, 0, receivedChar);
			/* And take the semaphore of the UART */
			xSemaphoreTake(Story1_BinarySemph_UART, (TickType_t)100);
		}
		/* In case the Keypad is the user of the LCD.. */
		else if(LCD_Source == LCD_FROM_KEYPAD)
		{
			/* Display the key received through the Keypad on the first row of the LCD */
			LCD_displayIntegerRowColumn(0, keyColumn++, gu8_Key);
			LCD_Source = LCD_NONE;
		}

		/* If the pushbutton was pressed.. */
		if(Sprint3_Story1_MutexTaken == Sprint2_Story1_PBTaskHandle)
		{
			/* Clear the LCD */
			LCD_clear();
			keyColumn = COLUMN0;
		}
	}
}

void Sprint2_Story1_Task_B_MONITORING_KEYPAD (void *pvParameters)
{
	/* reference tick for periodicity function */
	TickType_t xLastWakeTime;

	/* periodicity value */
	const TickType_t xFrequency = 200;

	// Initialize the xLastWakeTime variable with the current time.
	xLastWakeTime = xTaskGetTickCount();

	while(1)
	{
		vTaskDelayUntil(&xLastWakeTime, xFrequency);

		/* Finding out whether a key from the keypad is pressed or not */
		KeyPad_getPressedKey(&gu8_Key);

		/* If a key is pressed.. */
		if(gu8_Key != KEYPAD_NOT_PRESSED)
		{
			/* then forward the case to the LCD task */
			LCD_Source = LCD_FROM_KEYPAD;
		}
	}
}

void Sprint2_Story1_Task_C_MONITORING_UART (void *pvParameters)
{
	/* reference tick for periodicity function */
	TickType_t xLastWakeTime;

	/* periodicity value */
	const TickType_t xFrequency = 200;

	// Initialize the xLastWakeTime variable with the current time.
	xLastWakeTime = xTaskGetTickCount();

	/* Binary Semaphore created */
	Story1_BinarySemph_UART = xSemaphoreCreateBinary();

	while(1)
	{
		vTaskDelayUntil(&xLastWakeTime, xFrequency);

		/* If the semaphore is not taken, take it.. */
		if(xSemaphoreTake(Story1_BinarySemph_UART, (TickType_t)100) == pdTRUE)
		{
			/* and forward the case to the LCD task */
			LCD_Source = LCD_FROM_UART;
		}else
		{
			LCD_Source = LCD_NONE;
		}
	}
}

void Sprint2_Story1_Task_D_MONITORING_PUSHBUTTON (void *pvParameters)
{
	uint8_t au8_PBPressedCount = PB1_PRESSED_COUNT_INITIAL_VALUE;
	uint8_t au8_PBStatus = RELEASED;

	/* reference tick for periodicity function */
	TickType_t xLastWakeTime;

	/* periodicity value */
	const TickType_t xFrequency = 15;

	// Initialize the xLastWakeTime variable with the current time.
	xLastWakeTime = xTaskGetTickCount();

	/* Mutex created */
	Story1_MutexHandle_PB = xSemaphoreCreateMutex();

	while(1)
	{
		vTaskDelayUntil(&xLastWakeTime, xFrequency);

		/* Now the variable au8_PB1_Status Decides whether the button1 is pressed or not */
		pushButtonGetStatus(BTN_1,&au8_PBStatus);

		/* Storing output of the function into the mutexCheck to decide whether the mutex is taken or not */
		Sprint3_Story1_MutexTakenByPB = xSemaphoreGetMutexHolder( Story1_MutexHandle_PB );

		/* In case the pushbutton is pressed.. */
		if ((PRESSED == au8_PBStatus) && (PB_PRESSED > au8_PBPressedCount))
		{
			/* Increment this variable to figure out if the pushbutton is pressed for real,
			   or it's a debouncing problem */
			au8_PBPressedCount++;
		}
		/* In case the pushbutton is released for 60 ms.. */
		else if ((RELEASED == au8_PBStatus) && (PB_RELEASED < au8_PBPressedCount))
		{
			/* Decrement the variable */
			au8_PBPressedCount--;
		}

		/* In case the pushbutton was firgured out to be pressed.. */
		if ((au8_PBPressedCount == PB_PRESSED) && (Sprint3_Story1_MutexTakenByPB == NULL))
		{
			/* Take the mutex */
			xSemaphoreTake(Story1_MutexHandle_PB, (TickType_t)15);
		}
		/* Otherwise.. */
		else if (au8_PBPressedCount == PB_RELEASED)
		{
			/* Give the mutex */
			xSemaphoreGive(Story1_MutexHandle_PB);
		}
	}
}
