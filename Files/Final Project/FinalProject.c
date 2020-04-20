/*
 * FinalProject.c
 *
 * Created: 4/18/2020 12:53:18 PM
 *  Author: K.Aladawy
 */ 
#include "FinalProject.h"


/*** LOCAL MACROS ***/
#define TRANSMIT_MAX_SIZE               6       
#define DATA_COUNTER_INITIAL_VALUE      0
#define TRANSMIT_COUNTER_INITIAL_VALUE  0
#define TRANSMIT_TASK_FREQ 100
#define KEYPAD_DATA_INITIAIL_VALUE 255 
#define SEND_FLAG_INITIAL_VALUE  255
#define RECEIVE_DATA_SIZE_INITIAL_VALUE  0
#define RECEIVE_COUNTER_INITIAL_VALUE 0 

/*** LOCAL FUNCTION PROTOTYPES ***/
static void UART_RecData (uint8_t Data);
static void ReceiveTask (void *pvParameters);
static void TransmitTask (void *pvParameters);
static void SystemInitTask (void *pvParameters);


/*** GLOBAL VARIABLES ***/
static TaskHandle_t TransmitTaskHAndle;
static TaskHandle_t ReceiveTaskHAndle;
static TaskHandle_t SystemInitTaskHAndle;
static SemaphoreHandle_t ReceiveSemaphoreHandle; 
static uint8_t garru8_ReceiveArr[TRANSMIT_MAX_SIZE];
static uint8_t gu8_ReceiveDataSize = RECEIVE_DATA_SIZE_INITIAL_VALUE ;
static uint8_t gu8_ReceiveCounter =  RECEIVE_COUNTER_INITIAL_VALUE;

/*** API IMPELEMNTATION ***/
void FinalProject (void)
{
    xTaskCreate (SystemInitTask, "R",100, NULL, 4,&SystemInitTaskHAndle); /*initiate system task*/

    xTaskCreate (TransmitTask, "T", 300, NULL, 2,&TransmitTaskHAndle); /*transmit task*/

    xTaskCreate (ReceiveTask, "R", 200, NULL, 3,&ReceiveTaskHAndle); /*receive task*/
    

    /*Start Scheduler*/
    vTaskStartScheduler();
}


void SystemInitTask (void *pvParameters)
{
    
    
    while (1)
    {
        Led_Init(LED_0); /*initialize LED0*/
        
        Led_Init(LED_1); /*initialize LED1*/
        
        while(NOT_INITIALIZED == gu8_LCD_InitFlag)
        LCD_init(); /*initialize LCD module*/
        
        UART_INIT(Uart_Parity_no,Uart_Stop_Bit_One,Uart_Data_8_Bit,Uart_Baud_Rate_9600,Uart_Async_Normal_Speed,Uart_Multi_Processor_Communication_Disable,
                  Uart_Interrupt,UART_RecData); /*initialize UART module*/
                  
        pushButtonInit(BTN_1); /*initialize push button1*/
        
        KeyPad_Init(); /*initiate keypad module*/

        vTaskSuspend( SystemInitTaskHAndle );
        
    }
    
}


void TransmitTask (void *pvParameters)
{
    /*local variables*/
    TickType_t xLastWakeTime;
    TickType_t xLEdLastWakeTime;
    const TickType_t xFrequency = TRANSMIT_TASK_FREQ ;
    uint8_t au8_KeyPadData = KEYPAD_DATA_INITIAIL_VALUE,
            au8_PBStatus    = Released,
            au8_TransmitCounter = TRANSMIT_COUNTER_INITIAL_VALUE,
            au8_SendFlag = SEND_FLAG_INITIAL_VALUE ,
            au8_DataCounter = DATA_COUNTER_INITIAL_VALUE,
            arru8_TransmitArr[TRANSMIT_MAX_SIZE];
    
    static SemaphoreHandle_t SendDataSemaphoreHandle; 
    
    SendDataSemaphoreHandle = xSemaphoreCreateBinary(); /*Create binary semaphore*/

    // Initialise the xLastWakeTime variable with the current time.
    xLastWakeTime = xTaskGetTickCount(); /*get tick count refrence*/
    
    while(1)
    {
        // Wait for the next cycle.
        vTaskDelayUntil( &xLastWakeTime, xFrequency );
        
        keyPad_GetPressedkey(&au8_KeyPadData); /*get keypad pressed number*/
        
        if ( 255 != au8_KeyPadData) /*check if the keypad pressed*/
        {
            if ( au8_DataCounter < TRANSMIT_MAX_SIZE)
            {
                arru8_TransmitArr[au8_DataCounter] = au8_KeyPadData; /*store the number in transmit data array*/
                
                while(NOT_INITIALIZED == gu8_LCD_DisplayStringRowColumnFlag)
                {
                    LCD_DisplayStringRowColumn(&au8_KeyPadData,1,au8_DataCounter);/*display the pressed number on the lcd*/
                }
                gu8_LCD_DisplayStringRowColumnFlag = NOT_INITIALIZED; /*reinit the flag to use the function again*/
                
                au8_DataCounter++;
                
                xSemaphoreGive( SendDataSemaphoreHandle );/*give semaphore to use the data*/
            }
            
        }
        
        
        pushButtonGetStatus(BTN_1,&au8_PBStatus); /*get the push button status*/
        
        /*check if the push button is pressed and the thier is data ready from feypad*/
        if ((Pressed == au8_PBStatus) && (pdTRUE == xSemaphoreTake( SendDataSemaphoreHandle, ( TickType_t ) 1 )))
        {
            Led_On(LED_1);/*turn on the led*/
            
            xLEdLastWakeTime = xTaskGetTickCount(); /*get the tick counter to use as a refrence in delay*/
            
            UART_TRANSMIT_Char(au8_DataCounter +'0', NULL); /*send data size*/
            
            while (au8_DataCounter > au8_TransmitCounter)
            {
                UART_TRANSMIT_Char(arru8_TransmitArr[au8_TransmitCounter], NULL); /*send the data*/
                au8_TransmitCounter++;
                
            }
            au8_DataCounter = DATA_COUNTER_INITIAL_VALUE; /* reinit the data counter to reused it from it's initial value*/
            au8_TransmitCounter = TRANSMIT_COUNTER_INITIAL_VALUE; /* reinit the transmit counter to reused it from it's initial value*/
            
            while(NOT_INITIALIZED == gu8_LCD_DisplayStringRowColumnFlag)
            {
                LCD_DisplayStringRowColumn((uint8_t*)"         ",TRANSMIT_MAX_SIZE,0); /*clear the first row in lcd*/
            }
            
            gu8_LCD_DisplayStringRowColumnFlag = NOT_INITIALIZED;
            
            // Wait for the next cycle.
            vTaskDelayUntil( &xLEdLastWakeTime, 200 );/*wait for transimt led delay*/
            
            Led_Off(LED_1); /*turn off the led*/
             
        }
        
        
    }
}


void ReceiveTask (void *pvParameters)
{
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = 100;
    
    ReceiveSemaphoreHandle = xSemaphoreCreateBinary(); /*creat binary semaphore */
    
    while(1)
    {
        
        if (pdTRUE == xSemaphoreTake( ReceiveSemaphoreHandle, ( TickType_t ) 50 ))
        {
            // Initialise the xLastWakeTime variable with the current time.
            xLastWakeTime = xTaskGetTickCount();
            
            Led_On(LED_0); /*turn the led on*/
            
            while(NOT_INITIALIZED == gu8_LCD_DisplayStringRowColumnFlag)
            {
                 LCD_DisplayStringRowColumn(garru8_ReceiveArr,gu8_ReceiveDataSize,16);/*display recived data*/
            }
            gu8_LCD_DisplayStringRowColumnFlag = NOT_INITIALIZED;
            
            gu8_ReceiveDataSize = RECEIVE_DATA_SIZE_INITIAL_VALUE;/*reinit data size to it's initial value to reused it*/
            gu8_ReceiveCounter = RECEIVE_COUNTER_INITIAL_VALUE;   /*reinit counter to it's initial value to reused it*/
            
            // Wait for the next cycle.
            vTaskDelayUntil( &xLastWakeTime, 500 );
            
            Led_Off(LED_0);
            
            // Wait for the next cycle.
            vTaskDelayUntil( &xLastWakeTime, 2000 );   
            
            while(NOT_INITIALIZED == gu8_LCD_DisplayStringRowColumnFlag)
            {
                LCD_DisplayStringRowColumn((uint8_t*)"         ",TRANSMIT_MAX_SIZE,16);/*clear lcd second row */
            }
            gu8_LCD_DisplayStringRowColumnFlag = NOT_INITIALIZED;
                          
        }        
    }
}

/*uart recive call back function*/
void UART_RecData (uint8_t Data)
{
    if (gu8_ReceiveDataSize == 0)
    {
        gu8_ReceiveDataSize = Data - '0';/*recive the data size and store it*/
    }
    else if (gu8_ReceiveDataSize > gu8_ReceiveCounter)
    {
        garru8_ReceiveArr[gu8_ReceiveCounter] = Data;/*recive the data and store it*/
        
        gu8_ReceiveCounter++;
    }
    
    if (gu8_ReceiveDataSize == gu8_ReceiveCounter)
    {
        xSemaphoreGiveFromISR( ReceiveSemaphoreHandle,pdTRUE  ); /*give semaphore the use the data*/
    }
}