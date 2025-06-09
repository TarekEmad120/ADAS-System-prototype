/******************************************************************************
 *
 * Module: SYSTICK
 *
 * File Name: SYSTICK.h
 *
 * Description: Header file for the ARM Cortex M4 SYSTICK driver
 *
 * Author: Tarek Emad
 *
 *******************************************************************************/

#ifndef SYSTICK_H_
#define SYSTICK_H_

/************************************ Systick Timer Driver ************************************/
#include "tm4c123gh6pm_registers.h"
#include "std_types.h"


/*******************************************************************************
 *                           Functions prototypes                              *
 *******************************************************************************/

/*********************************************************************
 * Service Name: SysTick_Init
 * Sync/Async: Synchronous
 * Reentrancy: reentrant
 * Parameters (in): a_TimeInMilliSeconds - Time in MilliSeconds
 * Parameters (inout): None
 * Parameters (out): None
 * Return value: None
 * Description: Initialize the SysTick timer with the specified time in milliseconds
 * using interrupts. This function is used to setup the timer to generate periodic
 * interrupts every specified time in milliseconds.
 * ********************************************************************/

void SysTick_Init(uint16 a_TimeInMilliSeconds);

/*********************************************************************
 * Service Name: SysTick_StartBusyWait
 * Sync/Async: Synchronous
 * Reentrancy: reentrant
 * Parameters (in): a_TimeInMilliSeconds - Time in MilliSeconds
 * Parameters (inout): None
 * Parameters (out): None
 * Return value: None
 * Description: Initialize the SysTick timer with the specified time in milliseconds
 * using polling or busy-wait technique. The function should exit when the time is
 * elapsed and stops the timer at the end.
 * ********************************************************************/

 void SysTick_StartBusyWait(uint16 a_TimeInMilliSeconds);

/*********************************************************************
 * Service Name: SysTick_Handler
 * Sync/Async: Synchronous
 * Reentrancy: reentrant
 * Parameters (in): None
 * Parameters (inout): None
 * Parameters (out): None
 * Return value: None
 * Description: Handler for SysTick interrupt use to call the call-back function.
 * ********************************************************************/

 void SysTick_Handler(void);

/*********************************************************************
 * Service Name: SysTick_SetCallBack
 * Sync/Async: Synchronous
 * Reentrancy: reentrant
 * Parameters (in): Ptr2Func - Pointer to function
 * Parameters (inout): None
 * Parameters (out): None
 * Return value: None
 * Description: Function to setup the SysTick Timer call back to be executed in the SysTick Handler.
 * ********************************************************************/

void SysTick_SetCallBack(volatile void (*Ptr2Func) (void));

/*********************************************************************
 * Service Name: SysTick_Stop
 * Sync/Async: Synchronous
 * Reentrancy: reentrant
 * Parameters (in): None
 * Parameters (inout): None
 * Parameters (out): None
 * Return value: None
 * Description: Function to stop (pause) the SysTick Timer.
 * ********************************************************************/

 void SysTick_Stop(void);

/*********************************************************************
 * Service Name: SysTick_Start
 * Sync/Async: Synchronous
 * Reentrancy: reentrant
 * Parameters (in): None
 * Parameters (inout): None
 * Parameters (out): None
 * Return value: None
 * Description: Function to resume the SysTick Timer.
 * ********************************************************************/

 void SysTick_Start(void);

/*********************************************************************
 * Service Name: SysTick_DeInit
 * Sync/Async: Synchronous
 * Reentrancy: reentrant
 * Parameters (in): None
 * Parameters (inout): None
 * Parameters (out): None
 * Return value: None
 * Description: Function to Deinitialize the SysTick Timer.
 * ********************************************************************/

 void SysTick_DeInit(void);





#endif /* SYSTICK_H_ */
