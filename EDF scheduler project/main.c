/*
 * FreeRTOS Kernel V10.2.0
 * Copyright (C) 2019 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * http://www.FreeRTOS.org
 * http://aws.amazon.com/freertos
 *
 * 1 tab == 4 spaces!
 */

/* 
	NOTE : Tasks run in system mode and the scheduler runs in Supervisor mode.
	The processor MUST be in supervisor mode when vTaskStartScheduler is 
	called.  The demo applications included in the FreeRTOS.org download switch
	to supervisor mode prior to main being called.  If you are not using one of
	these demo application projects then ensure Supervisor mode is used.
*/


/*
 * Creates all the demo application tasks, then starts the scheduler.  The WEB
 * documentation provides more details of the demo application tasks.
 * 
 * Main.c also creates a task called "Check".  This only executes every three 
 * seconds but has the highest priority so is guaranteed to get processor time.  
 * Its main function is to check that all the other tasks are still operational.
 * Each task (other than the "flash" tasks) maintains a unique count that is 
 * incremented each time the task successfully completes its function.  Should 
 * any error occur within such a task the count is permanently halted.  The 
 * check task inspects the count of each task to ensure it has changed since
 * the last time the check task executed.  If all the count variables have 
 * changed all the tasks are still executing error free, and the check task
 * toggles the onboard LED.  Should any task contain an error at any time 
 * the LED toggle rate will change from 3 seconds to 500ms.
 *
 */

/* Standard includes. */
#include <stdlib.h>
#include <stdio.h>

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "lpc21xx.h"
#include "queue.h"

/* Peripheral includes. */
#include "serial.h"
#include "GPIO.h"

/* Constants to setup I/O and processor. */
#define BUS_CLK_FULL						( (unsigned char)0x01 )
/* Constants for the ComTest demo application tasks. */
#define TEST_BAUD_RATE						( (unsigned long)115200 )
/*	Defintions used in the main */
#define NULL_PTR 							(void *)0
#define LOGIC_HIGH			 				(1)
#define LOGIC_LOW			 				(0)

/*	Period Definitions	*/
#define	BUTTON1_MONITOR_PERIOD				(50)
#define	BUTTON2_MONITOR_PERIOD				(50)
#define PERIODIC_TX_PERIOD					(100)
#define UART_RX_RECEIVER          			(20)
#define Load1_Simulation_PERIOD      		(10)
#define LOAD2_SIMULATION_PERIOD      		(100)

/*	Task Handlers	*/
TaskHandle_t Button1_Handler = NULL;
TaskHandle_t Button2_Handler = NULL;
TaskHandle_t Periodic_TxHandler = NULL;
TaskHandle_t Uart_RxHandler = NULL;
TaskHandle_t Load1Handler = NULL;
TaskHandle_t Load2Handler = NULL;

/* Global variables	*/
int task1_in_time = 0, task1_out_time = 0, task1_total_time;
int task2_in_time = 0, task2_out_time = 0, task2_total_time;
int task3_in_time = 0, task3_out_time = 0, task3_total_time;
int task4_in_time = 0, task4_out_time = 0, task4_total_time;
int task5_in_time = 0, task5_out_time = 0, task5_total_time;
int task6_in_time = 0, task6_out_time = 0, task6_total_time;
int system_time = 0;
int cpu_load = 0;

QueueHandle_t MainQueue = NULL;
/* Tick Hook function implementation */
void vApplicationTickHook()
{
	GPIO_write(PORT_0, PIN1, PIN_IS_HIGH);
	GPIO_write(PORT_0, PIN1, PIN_IS_LOW);
}
/*Implement Tick and Idle Hooks */
void vApplicationIdleHook(void){}
static void prvSetupHardware(void);
/* Task to be created. */
void Button1_monitor(void * pvParameters)
{
	int xLastWakeTime = xTaskGetTickCount();
	pinState_t prevState = PIN_IS_LOW , current_state;
	char* button1Mess = NULL_PTR;
	vTaskSetApplicationTaskTag(NULL, (void *) 1);
	while(1)
	{
		current_state = GPIO_read(PORT_1, PIN0);
		if (prevState != current_state)
		{
			if (current_state == PIN_IS_LOW)
			{
				button1Mess = "Falling Edge FB1\n";
				xQueueSend(MainQueue, (void *) &button1Mess, (TickType_t) 0 );
			}
			else
			{
				button1Mess = "Rising Edge FB1\n";
				xQueueSend(MainQueue, (void *) &button1Mess, (TickType_t) 0 );
			}
			prevState = current_state;
		}
		vTaskDelayUntil(&xLastWakeTime, BUTTON1_MONITOR_PERIOD);
	}
}
void Button2_monitor(void * pvParameters)
{
	int xLastWakeTime = xTaskGetTickCount();
	pinState_t prevState = PIN_IS_LOW , current_state;
	char* button2Mess = NULL_PTR;
	vTaskSetApplicationTaskTag(NULL, (void *) 2);
	while(1)
	{
		current_state = GPIO_read(PORT_1, PIN1);
		if (prevState != current_state )
		{
			if (current_state == PIN_IS_LOW)
			{
				button2Mess = "Falling edge FB2\n";
				xQueueSend(MainQueue, (void *) &button2Mess, (TickType_t) 0 );
			}
			else
			{
				button2Mess = "Rising edge FB2\n";
				xQueueSend(MainQueue, (void *) &button2Mess, (TickType_t) 0 );
			}
			prevState = current_state;
		}
		vTaskDelayUntil(&xLastWakeTime, BUTTON2_MONITOR_PERIOD);
	}
}
void Periodic_Transmitter(void * pvParameters)
{
	char* periodicMess = NULL_PTR;
	int xLastWakeTime = xTaskGetTickCount();
	vTaskSetApplicationTaskTag(NULL,(void *) 3);
	while(1)
	{
		if ( ( uxQueueSpacesAvailable( MainQueue ) ) > 0 && (MainQueue != NULL) )
		{
			periodicMess = "Periodic message\n";
			xQueueSend(MainQueue, (void *) &periodicMess, (TickType_t) 0 );
		}
		vTaskDelayUntil(&xLastWakeTime, PERIODIC_TX_PERIOD);
	}

}
void Uart_Receiver(void * pvParameters)
{
	char* receivedMess = NULL_PTR;
	int xLastWakeTime = xTaskGetTickCount();
	vTaskSetApplicationTaskTag(NULL, (void *) 4);
	while(1)
	{
		if( xQueueReceive( MainQueue, &(receivedMess), (TickType_t) 10 ) == pdPASS )
		{
			vSerialPutString(receivedMess, 20);
		}
		vTaskDelayUntil(&xLastWakeTime, UART_RX_RECEIVER);
	}
}
void Load1_Simulation(void * pvParameters)
{
	int i= 0;
	int xLastWakeTime = xTaskGetTickCount();
	vTaskSetApplicationTaskTag(NULL, (void *)5);
    while(1)
    {
		for (i = 0; i < 37313; i++){}
		vTaskDelayUntil(&xLastWakeTime, Load1_Simulation_PERIOD);
    } 


}
void Load2_Simulation(void * pvParameters)
{
	int i = 0;
	int xLastWakeTime = xTaskGetTickCount();
	vTaskSetApplicationTaskTag(NULL, (void *)6);
    while(1)
    {
		for ( i = 0; i < 89552; i++){}
		vTaskDelayUntil(&xLastWakeTime, LOAD2_SIMULATION_PERIOD);
    } 


}



int main(void)
{
	/* Setup the hardware to use keil sim. board. */
	prvSetupHardware();
	/* Create Tasks. */
	xTaskPeriodicCreate(
						Button1_monitor,
						"Button1_monitor",
						100, 
						( void * ) 0,   
						1,
						&Button1_Handler,
						BUTTON1_MONITOR_PERIOD
	); 
	xTaskPeriodicCreate(
						Button2_monitor,
						"Button2_monitor",
						100,
						( void * ) 0,
						1,
						&Button2_Handler,
						BUTTON2_MONITOR_PERIOD
	);									
	xTaskPeriodicCreate(
						Uart_Receiver,
						"Load1_Simulation",
						100,
						( void * ) 0, 
						1,
						&Uart_RxHandler,
						UART_RX_RECEIVER
	); 
	xTaskPeriodicCreate(
						Periodic_Transmitter,
						"Periodic_Transmitter",  
						100,  
						( void * ) 0, 
						1,
						&Periodic_TxHandler,
						PERIODIC_TX_PERIOD
	);	
	 xTaskPeriodicCreate(
						Load1_Simulation,
						"Load1_Simulation",
						100,
						( void * ) 0,
						1,
						&Load1Handler,
						Load1_Simulation_PERIOD
	);								
	xTaskPeriodicCreate(
						Load2_Simulation,
						"Load2_Simulation",
						100,
						( void * ) 0, 
						1,
						&Load2Handler,
						LOAD2_SIMULATION_PERIOD
	); 								
	MainQueue = xQueueCreate( 3, sizeof(unsigned char[15]) );
	vTaskStartScheduler();
	while(1){}
}

void timer1Reset(void)
{
	T1TCR |= 0x2;
	T1TCR &= ~0x2;
}
static void configTimer1(void)
{
	T1PR = 1000;
	T1TCR |= 0x1;
}
static void prvSetupHardware( void )
{
	/* Configure UART */
	xSerialPortInitMinimal(TEST_BAUD_RATE);
	/* Configure GPIO */
	GPIO_init();
	/* Config trace timer 1 and read T1TC to get current tick */
	configTimer1();

	VPBDIV = BUS_CLK_FULL;
}


