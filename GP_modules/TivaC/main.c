#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "std_types.h"
#include "Auto_Parking.h"
#include "Data_PreProcessing.h"
#include "COM_Protocol.h"
#include "uart0.h"
#include "tm4c123gh6pm_registers.h"



void Leds_Init(void)
{
    SYSCTL_RCGCGPIO_REG |= 0x20; 
    while (!(SYSCTL_PRGPIO_REG & 0x20));                              
    GPIO_PORTF_AMSEL_REG &= 0xF1;     
    GPIO_PORTF_PCTL_REG &= 0xFFFF000F;
    GPIO_PORTF_DIR_REG |= 0x0E;    
    GPIO_PORTF_AFSEL_REG &= 0xF1;     
    GPIO_PORTF_DEN_REG |= 0x0E;      
    GPIO_PORTF_DATA_REG &= 0xF1;      
}

void LED_Auto_Parking(void)
{
    ParkingState status = AutoParking_GetState();
    switch (status)
    {
    case PARKING_STATE_IDLE:
        // Red LED
        GPIO_PORTF_DATA_REG = (GPIO_PORTF_DATA_REG & 0xF1) | 0x02;
        break;
    case PARKING_STATE_FORWARD:
        // Blue LED
        GPIO_PORTF_DATA_REG = (GPIO_PORTF_DATA_REG & 0xF1) | 0x04;
        break;
    case PARKING_STATE_BACKWARD:
        // Green LED
        GPIO_PORTF_DATA_REG = (GPIO_PORTF_DATA_REG & 0xF1) | 0x08;
        break;
    case PARKING_STATE_SEARCHING:
        // Yellow LED
        GPIO_PORTF_DATA_REG = (GPIO_PORTF_DATA_REG & 0xF1) | 0x0A;
        break;
    case PARKING_STATE_ALLIGN:
        // Purple LED
        GPIO_PORTF_DATA_REG = (GPIO_PORTF_DATA_REG & 0xF1) | 0x06;
        break;
    case PARKING_STATE_FINISHED:
        // White LED
        GPIO_PORTF_DATA_REG = (GPIO_PORTF_DATA_REG & 0xF1) | 0x0E;
        break;

    default:
        break;
    }
}

void print_hex_byte(uint8 byte)
{
    char hex[5];
    sprintf(hex, "0x%02X ", byte);
    UART0_SendString(hex);
}

int main(void)
{
    uint8 data;
    uint16 sensor_readings[SENSORS_COUNT];
    UART0_Init();
    Leds_Init();
    Data_PreProcessing_Init();
    AutoParking_Init();

    // Send a test message at startup
    UART0_SendString("TivaC initialized and ready\r\n");

    // Flash all LEDs to indicate startup
    GPIO_PORTF_DATA_REG = 0x0E; // All LEDs on

    // Set initial LED state
    LED_Auto_Parking();

    while (1)
    {
        if (UART0_Data_Recieved())
        {
            data = UART0_ReceiveByte();

            // Debug the received byte
            UART0_SendString("Got byte: ");
            UART0_SendByte(data); // Show the actual character
            UART0_SendString("\r\n");

            // Special handling for P command (0x50)
            // Special handling for P command (0x50)
            // Special handling for P command (0x50)
            if (data == 0x50 || data == 'P')
            {
                AutoParking_Activate();
                UART0_SendString("Auto-parking activated!\r\n");
                LED_Auto_Parking(); // Update LED state
            }
            // Just look for 'S' and read the entire string without checking individual characters
            else if (data == 'S' )
            {
                // Read the rest of the string after S
                char data_buffer[100];
                data_buffer[0] = 'S';                 // Store the 'S' we already read
                UART0_ReceiveString(data_buffer + 1); // Read starting from index 1 (after S)

                UART0_SendString("Full string: ");
                UART0_SendString(data_buffer);
                UART0_SendString("\r\n");

                // Check if it starts with "SD:"
                if (data_buffer[1] == 'D' && data_buffer[2] == ':')
                {
                    UART0_SendString("Valid SD: prefix detected\r\n");

                    // Process just the data portion (after "SD:")
                    if (Process_String_Data(data_buffer + 3)) // Skip the "SD:" prefix
                    {
                        UART0_SendString("Data processed successfully\r\n");

                        // Get sensor readings
                        Data_PreProcessing_Get_UltraSonic_Readings(sensor_readings);

                        // Only process auto-parking if active
                        if (AutoParking_IsActive())
                        {
                            UART0_SendString("Running auto-parking algorithm\r\n");
                            AutoParking_Process(sensor_readings,
                                                Data_PreProcessing_Get_Vehicle_Speed());
                            LED_Auto_Parking();

                        
                        }
                    }
                }
                else
                {
                    UART0_SendString("Invalid data format - expected SD: prefix\r\n");
                    // Debug what was received
                    UART0_SendString("Received: S");
                    UART0_SendByte(data_buffer[1]);
                    UART0_SendByte(data_buffer[2]);
                    UART0_SendString("...\r\n");
                }
            }
        }
    }
}
