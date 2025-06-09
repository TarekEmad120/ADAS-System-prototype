/*
 * NVIC.c
 *
 *  Created on: Mar 18, 2025
 *      Author: Tarek Emad
 */

/*  this driver is used for tiva c launchpad to enable and disable interrupts and set the priority of the interrupts */
#include "NVIC.h"
#include "tm4c123gh6pm_registers.h"
#include "std_types.h"


#define BUS_FAULT_ENABLE_BIT_POS           17


volatile uint32* NVIC_PRI_REGS[] = {
    &NVIC_PRI0_REG, &NVIC_PRI1_REG, &NVIC_PRI2_REG, &NVIC_PRI3_REG,
    &NVIC_PRI4_REG, &NVIC_PRI5_REG, &NVIC_PRI6_REG, &NVIC_PRI7_REG,
    &NVIC_PRI8_REG, &NVIC_PRI9_REG, &NVIC_PRI10_REG, &NVIC_PRI11_REG,
    &NVIC_PRI12_REG, &NVIC_PRI13_REG, &NVIC_PRI14_REG, &NVIC_PRI15_REG,
    &NVIC_PRI16_REG, &NVIC_PRI17_REG, &NVIC_PRI18_REG, &NVIC_PRI19_REG,
    &NVIC_PRI20_REG, &NVIC_PRI21_REG, &NVIC_PRI22_REG, &NVIC_PRI23_REG,
    &NVIC_PRI24_REG, &NVIC_PRI25_REG, &NVIC_PRI26_REG, &NVIC_PRI27_REG,
    &NVIC_PRI28_REG, &NVIC_PRI29_REG, &NVIC_PRI30_REG, &NVIC_PRI31_REG,
    &NVIC_PRI32_REG, &NVIC_PRI33_REG, &NVIC_PRI34_REG
};

/*********************************************************************
* Service Name: NVIC_EnableIRQ
* Sync/Async: Synchronous
* Reentrancy: reentrant
* Parameters (in): IRQ_Num - Number of the IRQ from the target vector table
* Parameters (inout): None
* Parameters (out): None
* Return value: None
* Description: Function to enable Interrupt request for specific IRQ
**********************************************************************/


void NVIC_EnableIRQ(NVIC_IRQType IRQNum)
{
    uint8 reg_offset = IRQNum / 32;
    uint8 bit_offset = IRQNum % 32;
    if (reg_offset == 0)
    {
        NVIC_EN0_REG |= (1 << bit_offset);
    }
    else if (reg_offset == 1)
    {
        NVIC_EN1_REG |= (1 << bit_offset);
    }
    else if (reg_offset == 2)
    {
        NVIC_EN2_REG |= (1 << bit_offset);
    }
    else if (reg_offset == 3)
    {
        NVIC_EN3_REG |= (1 << bit_offset);
    }
}

/*********************************************************************
 * Service Name: NVIC_DisableIRQ
 * Sync/Async: Synchronous
 * Reentrancy: reentrant
 * Parameters (in): IRQ_Num - Number of the IRQ from the target vector table
 * Parameters (inout): None
 * Parameters (out): None
 * Return value: None
 * Description: Function to disable Interrupt request for specific IRQ
 * **********************************************************************/

void NVIC_DisableIRQ(NVIC_IRQType IRQNum)
{
    uint8 reg_offset = IRQNum / 32;
    uint8 bit_offset = IRQNum % 32;
    if (reg_offset == 0)
    {
        NVIC_DIS0_REG |= (1 << bit_offset);
    }
    else if (reg_offset == 1)
    {
        NVIC_DIS1_REG |= (1 << bit_offset);
    }
    else if (reg_offset == 2)
    {
        NVIC_DIS2_REG |= (1 << bit_offset);
    }
    else if (reg_offset == 3)
    {
        NVIC_DIS3_REG |= (1 << bit_offset);
    }
}


/*********************************************************************
 * Service Name: NVIC_SetPriorityIRQ
 * Sync/Async: Synchronous
 * Reentrancy: reentrant
 * Parameters (in): IRQ_Num - Number of the IRQ from the target vector table
 *                 IRQ_Priority - Priority of the IRQ
 * Parameters (inout): None
 * Parameters (out): None
 * Return value: None
 * Description: Function to set the priority of specific IRQ
 * **********************************************************************/

void NVIC_SetPriorityIRQ(NVIC_IRQType IRQ_Num, NVIC_IRQPriorityType IRQ_Priority){
    uint8 reg_offset = IRQ_Num / 4;
    uint8 bit_offset = (IRQ_Num % 4) * 8 +5;
    *NVIC_PRI_REGS[reg_offset] = (*NVIC_PRI_REGS[reg_offset] & ~(0x7 << bit_offset)) | (IRQ_Priority << bit_offset);
}

/*********************************************************************
 * Service Name: NVIC_EnableException
 * Sync/Async: Synchronous
 * Reentrancy: reentrant
 * Parameters (in): Exception_Num - Number of the Exception
 * Parameters (inout): None
 * Parameters (out): None
 * Return value: None
 * Description: Function to enable specific ARM system or fault exceptions
 * **********************************************************************/

void NVIC_EnableException(NVIC_ExceptionType Exception_Num)
{

    if (Exception_Num == EXCEPTION_MEM_FAULT_TYPE)
    {
        NVIC_SYSTEM_SYSHNDCTRL |= (1<<MEM_MANAGE_FAULT_ENABLE_BIT_POS);
    }
    else if (Exception_Num == EXCEPTION_BUS_FAULT_TYPE)
    {
        NVIC_SYSTEM_SYSHNDCTRL |= (1<<BUS_FAULT_ENABLE_BIT_POS);
    }
    else if (Exception_Num == EXCEPTION_USAGE_FAULT_TYPE)
    {
        NVIC_SYSTEM_SYSHNDCTRL |= (1<<USAGE_FAULT_ENABLE_BIT_POS);
    }
    else
    {
        Enable_Faults();
    }
}

/*********************************************************************
 * Service Name: NVIC_DisableException
 * Sync/Async: Synchronous
 * Reentrancy: reentrant
 * Parameters (in): Exception_Num - Number of the Exception
 * Parameters (inout): None
 * Parameters (out): None
 * Return value: None
 * Description: Function to disable specific ARM system or fault exceptions
 * **********************************************************************/

 void NVIC_DisableException(NVIC_ExceptionType Exception_Num)
 {
        if (Exception_Num == EXCEPTION_MEM_FAULT_TYPE)
        {
            NVIC_SYSTEM_SYSHNDCTRL &= ~(1<<MEM_MANAGE_FAULT_ENABLE_BIT_POS);
        }
        else if (Exception_Num == EXCEPTION_BUS_FAULT_TYPE)
        {
            NVIC_SYSTEM_SYSHNDCTRL &= ~(1<<BUS_FAULT_ENABLE_BIT_POS);
        }
        else if (Exception_Num == EXCEPTION_USAGE_FAULT_TYPE)
        {
            NVIC_SYSTEM_SYSHNDCTRL &= ~(1<<USAGE_FAULT_ENABLE_BIT_POS);
        }
        else
        {
            Disable_Faults();
        }
 }

/*********************************************************************
 * Service Name: NVIC_SetPriorityException
 * Sync/Async: Synchronous
 * Reentrancy: reentrant
 * Parameters (in): Exception_Num - Number of the Exception
 *                Exception_Priority - Priority of the Exception
 * Parameters (inout): None
 * Parameters (out): None
 * Return value: None
 * Description: Function to set the priority of specific Exception
 * **********************************************************************/

void NVIC_SetPriorityException(NVIC_ExceptionType Exception_Num, NVIC_ExceptionPriorityType Exception_Priority)
{

    if (Exception_Num == EXCEPTION_MEM_FAULT_TYPE)
    {
        NVIC_SYSTEM_PRI1_REG = (NVIC_SYSTEM_PRI1_REG & MEM_FAULT_PRIORITY_MASK ) | (Exception_Priority << MEM_FAULT_PRIORITY_BITS_POS);
    }
    else if (Exception_Num == EXCEPTION_BUS_FAULT_TYPE)
    {
        NVIC_SYSTEM_PRI1_REG = (NVIC_SYSTEM_PRI1_REG & BUS_FAULT_PRIORITY_MASK) | (Exception_Priority << BUS_FAULT_PRIORITY_BITS_POS);
    }
    else if (Exception_Num == EXCEPTION_USAGE_FAULT_TYPE)
    {
        NVIC_SYSTEM_PRI1_REG = (NVIC_SYSTEM_PRI1_REG & USAGE_FAULT_PRIORITY_MASK) | (Exception_Priority << USAGE_FAULT_PRIORITY_BITS_POS);
    }
    else if (Exception_Num == EXCEPTION_SVC_TYPE)
    {
        NVIC_SYSTEM_PRI2_REG = (NVIC_SYSTEM_PRI2_REG & SVC_PRIORITY_MASK) | (Exception_Priority << SVC_PRIORITY_BITS_POS);
    }
    else if (Exception_Num == EXCEPTION_DEBUG_MONITOR_TYPE)
    {
        NVIC_SYSTEM_PRI3_REG =(NVIC_SYSTEM_PRI3_REG &DEBUG_MONITOR_PRIORITY_MASK) | (Exception_Priority<< DEBUG_MONITOR_PRIORITY_BITS_POS);
    }
    else if (Exception_Num == EXCEPTION_PEND_SV_TYPE)
    {
        NVIC_SYSTEM_PRI3_REG =  (NVIC_SYSTEM_PRI3_REG & PENDSV_PRIORITY_MASK) | (Exception_Priority << PENDSV_PRIORITY_BITS_POS);
    }
    else if (Exception_Num == EXCEPTION_SYSTICK_TYPE)
    {
        NVIC_SYSTEM_PRI3_REG =  (NVIC_SYSTEM_PRI3_REG & SYSTICK_PRIORITY_MASK) | (Exception_Priority << SYSTICK_PRIORITY_BITS_POS);
    }
}
