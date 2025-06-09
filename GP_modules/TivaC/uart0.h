 /******************************************************************************
 *
 * Module: UART0
 *
 * File Name: uart0.h
 *
 * Description: Header file for the TM4C123GH6PM UART0 driver
 *
 * Author: Tarek Emad
 *
 *******************************************************************************/

#ifndef UART0_H_
#define UART0_H_

#include "std_types.h"

/*******************************************************************************
 *                             PreProcessor Macros                             *
 *******************************************************************************/

#define UART_DATA_5BITS          0x0
#define UART_DATA_6BITS          0x1
#define UART_DATA_7BITS          0x2
#define UART_DATA_8BITS          0x3
#define UART_LCRH_WLEN_BITS_POS  5
#define UART_CTL_UARTEN_MASK     0x00000001
#define UART_CTL_TXE_MASK        0x00000100
#define UART_CTL_RXE_MASK        0x00000200
#define UART_FR_TXFE_MASK        0x00000080
#define UART_FR_RXFE_MASK        0x00000010

/*******************************************************************************
 *                            Functions Prototypes                             *
 *******************************************************************************/

 /*
 * Service Name: UART0_Init
 * Sync/Async: Synchronous
 * Reentrancy: Reentrant
 * Parameters (in): None
 * Parameters (inout): None
 * Parameters (out): None
 * Return value: None
 * Description: to initialize the UART0 module
 */
extern void UART0_Init(void);

/*
 * Service Name: UART0_Data_Recieved
 * Sync/Async: Synchronous
 * Reentrancy: Reentrant
 * Parameters (in): None
 * Parameters (inout): None
 * Parameters (out): None
 * Return value: boolean - TRUE if data is received, FALSE otherwise
 * Description: to check if data is received on the UART0 module
 */
extern bool UART0_Data_Recieved(void);

/*
 * Service Name: UART0_SendByte
 * Sync/Async: Synchronous
 * Reentrancy: Reentrant
 * Parameters (in): data - the byte to be sent
 * Parameters (inout): None
 * Parameters (out): None
 * Return value: None
 * Description: to send a byte through UART0
 */
extern void UART0_SendByte(uint8 data);

/*
 * Service Name: UART0_ReceiveByte
 * Sync/Async: Synchronous
 * Reentrancy: Reentrant
 * Parameters (in): None
 * Parameters (inout): None
 * Parameters (out): None
 * Return value: uint8 - the received byte
 * Description: to receive a byte from UART0
 */
extern uint8 UART0_ReceiveByte(void);

/*
 * Service Name: UART0_SendNumber
 * Sync/Async: Synchronous
 * Reentrancy: Reentrant
 * Parameters (in): num - the number to be sent
 * Parameters (inout): None
 * Parameters (out): None
 * Return value: None
 * Description: to send a number through UART0
 */
extern void UART0_SendNumber(uint32 num);

/*
 * Service Name: UART0_SendData
 * Sync/Async: Synchronous
 * Reentrancy: Reentrant
 * Parameters (in): pData - pointer to the data to be sent
 *                  uSize - size of the data to be sent
 * Parameters (inout): None
 * Parameters (out): None
 * Return value: None
 * Description: to send a block of data through UART0
 */
extern void UART0_SendData(const uint8 *pData, uint32 uSize);

/*
 * Service Name: UART0_ReceiveData
 * Sync/Async: Synchronous
 * Reentrancy: Reentrant
 * Parameters (in): pData - pointer to the buffer to store the received data
 *                  uSize - size of the data to be received
 * Parameters (inout): None
 * Parameters (out): None
 * Return value: None
 * Description: to receive a block of data through UART0
 */
extern void UART0_ReceiveData(uint8 *pData, uint32 uSize);

/*
 * Service Name: UART0_SendString
 * Sync/Async: Synchronous
 * Reentrancy: Reentrant
 * Parameters (in): pData - pointer to the string to be sent
 * Parameters (inout): None
 * Parameters (out): None
 * Return value: None
 * Description: to send a string through UART0
 */
extern void UART0_SendString(const uint8 *pData);

/*
 * Service Name: UART0_ReceiveString
 * Sync/Async: Synchronous
 * Reentrancy: Reentrant
 * Parameters (in): pData - pointer to the buffer to store the received string
 * Parameters (inout): None
 * Parameters (out): None
 * Return value: None
 * Description: to receive a string through UART0 until '#' character
 */
extern void UART0_ReceiveString(uint8 *pData);

#endif
