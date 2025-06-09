/*
 * SYSTICK.c
 *
 *  Created on: Mar 18, 2025
 *      Author: Tarek Emad
 */

#include "SYSTICK.h"
#include "tm4c123gh6pm_registers.h"


#define SYSTICK_PRIORITY_MASK       0x1FFFFFFF
#define SYSTICK_INTERRUPT_PRIORITY  3
#define SYSTICK_PRIORITY_BITS_POS   29
#define SYSTICK_RELOAD_VALUE        16000

static volatile void (*gPtr2Func)(void) = NULL_PTR;

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

void SysTick_Init(uint16 a_TimeInMilliSeconds)
{
    SYSTICK_CTRL_REG = 0; /* Disable the SysTick Timer by Clear the ENABLE Bit (Bit0) */
    SYSTICK_RELOAD_REG = (SYSTICK_RELOAD_VALUE * a_TimeInMilliSeconds) - 1; /* Set the Reload value */
    SYSTICK_CURRENT_REG = 0; /* Clear the Current Register */
    /* Configure the SysTick Control Register
     * Enable the SysTick Timer (ENABLE = 1)
     * Enable SysTick Interrupt (INTEN = 1)
     * Choose the clock source to be System Clock (CLK_SRC = 1)
     */
    SYSTICK_CTRL_REG   |= 0x07;
}


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

void SysTick_StartBusyWait(uint16 a_TimeInMilliSeconds)
{
    SYSTICK_CTRL_REG = 0; /* Disable the SysTick Timer by Clear the ENABLE Bit (Bit0) */
    SYSTICK_RELOAD_REG = (SYSTICK_RELOAD_VALUE * a_TimeInMilliSeconds) - 1; /* Set the Reload value */
    SYSTICK_CURRENT_REG = 0; /* Clear the Current Register */
    /* Configure the SysTick Control Register
     * Enable the SysTick Timer (ENABLE = 1)
     * Choose the clock source to be System Clock (CLK_SRC = 1)
     */
    SYSTICK_CTRL_REG   |= 0x05;
    while((SYSTICK_CTRL_REG & 0x10000) == 0); /* Wait until the COUNTFLAG is set */
    SYSTICK_CTRL_REG = 0; /* Disable the SysTick Timer by Clear the ENABLE Bit (Bit0) */
    SYSTICK_CURRENT_REG = 0; /* Clear the Current Register */
}

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

void SysTick_Handler (void)
{
    if(gPtr2Func != NULL_PTR)
    {
        (*gPtr2Func)(); /* Call the Call Back function */
    }
}

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

void SysTick_SetCallBack(volatile void (*Ptr2Func) (void))
{
    gPtr2Func = Ptr2Func;
}

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
void SysTick_Stop(void)
{
    SYSTICK_CTRL_REG  &= ~(1); /* Disable the SysTick Timer by Clear the ENABLE Bit (Bit0) */
}

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
void SysTick_Start(void)
{
    SYSTICK_CTRL_REG   |= 0x01; /* Enable the SysTick Timer (ENABLE = 1) */
}

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
void SysTick_DeInit(void)
{
    SYSTICK_CTRL_REG = 0; /* Disable the SysTick Timer by Clear the ENABLE Bit (Bit0) */
    SYSTICK_RELOAD_REG = 0; /* Clear the Reload Register */
    SYSTICK_CURRENT_REG = 0; /* Clear the Current Register */
    gPtr2Func = NULL_PTR;
}
