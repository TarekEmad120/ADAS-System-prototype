 /******************************************************************************
 *
 * Module: UART0
 *
 * File Name: uart0.c
 *
 * Description: Source file for the TM4C123GH6PM UART0 driver
 *
 * Author: Tarek Emad
 *
 *******************************************************************************/

#include "uart0.h"
#include "tm4c123gh6pm_registers.h"

/*******************************************************************************
 *                         Private Functions Definitions                       *
 *******************************************************************************/

 /*
 * Service Name: GPIO_SetupUART0Pins
 * Sync/Async: Synchronous
 * Reentrancy: Reentrant
 * Parameters (in): None
 * Parameters (inout): None
 * Parameters (out): None
 * Return value: None
 * Description: to initialize the GPIO pins for UART0
 */ 
static void GPIO_SetupUART0Pins(void)
{
    SYSCTL_RCGCGPIO_REG  |= 0x01;         /* Enable clock for GPIO PORTA */
    while(!(SYSCTL_PRGPIO_REG & 0x01));   /* Wait until GPIO PORTA clock is activated and it is ready for access*/
   
    GPIO_PORTA_AMSEL_REG &= 0xFC;         /* Disable Analog on PA0 & PA1 */
    GPIO_PORTA_DIR_REG   &= 0xFE;         /* Configure PA0 as input pin */
    GPIO_PORTA_DIR_REG   |= 0x02;         /* Configure PA1 as output pin */
    GPIO_PORTA_AFSEL_REG |= 0x03;         /* Enable alternative function on PA0 & PA1 */
    /* Set PMCx bits for PA0 & PA1 with value 1 to use PA0 as UART0 RX pin and PA1 as UART0 Tx pin */
    GPIO_PORTA_PCTL_REG  = (GPIO_PORTA_PCTL_REG & 0xFFFFFF00) | 0x00000011;
    GPIO_PORTA_DEN_REG   |= 0x03;         /* Enable Digital I/O on PA0 & PA1 */
}

/*******************************************************************************
 *                         Public Functions Definitions                        *
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
void UART0_Init(void)
{
    /* Setup UART0 pins PA0 --> U0RX & PA1 --> U0TX */
    GPIO_SetupUART0Pins();
    
    SYSCTL_RCGCUART_REG |= 0x01;          /* Enable clock for UART0 */
    while(!(SYSCTL_PRUART_REG & 0x01));   /* Wait until UART0 clock is activated and it is ready for access*/
    
    UART0_CTL_REG = 0;                    /* Disable UART0 at the beginning */

    UART0_CC_REG  = 0;                    /* Use System Clock*/
    
    /* To Configure UART0 with Baud Rate 115200*/
    UART0_IBRD_REG = 8;
    UART0_FBRD_REG = 44;
    
    /* UART Line Control Register Settings
     * BRK = 0 Normal Use
     * PEN = 0 Disable Parity
     * EPS = 0 No affect as the parity is disabled
     * STP2 = 0 1-stop bit at end of the frame
     * FEN = 0 FIFOs are disabled
     * WLEN = 0x3 8-bits data frame
     * SPS = 0 no stick parity
     */
    UART0_LCRH_REG = (UART_DATA_8BITS << UART_LCRH_WLEN_BITS_POS);
    
    /* UART Control Register Settings
     * RXE = 1 Enable UART Receive
     * TXE = 1 Enable UART Transmit
     * HSE = 0 The UART is clocked using the system clock divided by 16
     * UARTEN = 1 Enable UART
     */
    UART0_CTL_REG = UART_CTL_UARTEN_MASK | UART_CTL_TXE_MASK | UART_CTL_RXE_MASK;
}

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
bool UART0_Data_Recieved(void)
{
    /* Check if there is any data received in the receive FIFO
     * RXFE = 0 There are data in the receive FIFO
     * RXFF = 1 There are no data in the receive FIFO
    */
    return ((UART0_FR_REG & UART_FR_RXFE_MASK) == 0);
}


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
void UART0_SendByte(uint8 data)
{
    while(!(UART0_FR_REG & UART_FR_TXFE_MASK)); /* Wait until the transmit FIFO is empty */
    UART0_DR_REG = data; /* Send the byte */
}

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
uint8 UART0_ReceiveByte(void)
{
    while(UART0_FR_REG & UART_FR_RXFE_MASK); /* Wait until the receive FIFO is not empty */
    return UART0_DR_REG; /* Read the byte */
}



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
void UART0_SendNumber(uint32 num) {
    char buffer[12]; // Buffer to hold the number as a string
    uint8 i = 0;

    if (num == 0) {
        UART0_SendByte('0');
        return;
    }

    // Convert the number to a string (reverse order)
    while (num > 0) {
        buffer[i++] = (num % 10) + '0';
        num /= 10;
    }

    // Send the string in the correct order
    while (i > 0) {
        UART0_SendByte(buffer[--i]);
    }
}


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
void UART0_SendString(const uint8 *pData)
{
    uint32 uCounter =0;
	/* Transmit the whole string */
    while(pData[uCounter] != '\0')  /* until the end of the string  AT LETTER '\0' */
    {
        UART0_SendByte(pData[uCounter]); /* Send the byte */
        uCounter++; /* increment the counter to the next byte */
    }
}


/*
 * Service Name: UART0_ReceiveString
 * Sync/Async: Synchronous
 * Reentrancy: Reentrant
 * Parameters (in): pData - pointer to the buffer to store the received string
 * Parameters (inout): None
 * Parameters (out): None
 * Return value: None
 * Description: to receive a string through UART0 until '#' character is received
 */
void UART0_ReceiveString(uint8 *pData)
{
    uint32 uCounter =0;

    pData[uCounter] = UART0_ReceiveByte(); /* Receive the first byte */
	
	/* Receive the whole string until the '#' */
    while(pData[uCounter] != '#')
    {
		uCounter++;
        pData[uCounter] = UART0_ReceiveByte();
    }
	/* After receiving the whole string plus the '#', replace the '#' with '\0' */
    pData[uCounter] = '\0';
}


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
void UART0_SendData(const uint8 *pData, uint32 uSize)
{
    /* Transmit the number of bytes requested on the UART port */
    while(uSize--)
    {
        
        UART0_SendByte(*pData); /* Send the byte */
        pData++; /* increment pointer to the next byte */
    }
}


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
void UART0_ReceiveData(uint8 *pData, uint32 uSize)
{
    /* Receive the number of bytes requested on the UART port */
    while(uSize--)
    {
        *pData = UART0_ReceiveByte(); /* receive a byte from the UART */
         pData++; /* increment pointer to receive the next byte in the next location */
    }
}
