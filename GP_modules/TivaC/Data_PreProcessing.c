 /******************************************************************************
 *
 * Module: Data PreProcessing
 *
 * File Name: Data_PreProcessing.c
 *
 * Description: Source file for the Data PreProcessing System when receiving data from the PC
 *
 * Author: Tarek Emad
 *
 *******************************************************************************/

#include "Data_PreProcessing.h"
#include "tm4c123gh6pm_registers.h"
#include "COM_Protocol.h"
#include "uart0.h"

//#define MAX_BUFFER_SIZE  200

/*******************************************************************************
 *                             Global Variables                                *
 *******************************************************************************/

static uint16 recieved_data_buffer[PACKET_SIZE];
static uint16 sensor_readings[SENSORS_COUNT];
static uint8 data_index = 0;
static bool packet_start = FALSE;
static bool new_packet = FALSE;
static uint16 vehicle_speed = 0;  
static uint16 throuttle = 0;
static uint16 steering = 0;
static uint16 brake = 0;
static char string_buffer[200];


/*******************************************************************************
 *                             Global Functions                                *
 *******************************************************************************/

/*
 * Service Name: AutoParking_Init
 * Sync/Async: Synchronous
 * Reentrancy: Reentrant
 * Parameters (in): None
 * Parameters (inout): None
 * Parameters (out): None
 * Return value: None
 * Description: to initialize the Auto Parking System
 */
void Data_PreProcessing_Init(void){
    data_index = 0;
    packet_start = FALSE;
    new_packet = FALSE;
    vehicle_speed = 0;
    throuttle = 0;
    steering = 0;
    brake = 0;
    uint8 i=0;
    for ( i = 0; i < SENSORS_COUNT; i++){
        sensor_readings[i] = 0;
    }
}

/*
 * Service Name: Data_PreProcessing_Process
 * Sync/Async: Synchronous
 * Reentrancy: Reentrant
 * Parameters (in): received_byte - the byte received from the UART
 * Parameters (inout): None
 * Parameters (out): None
 * Return value: boolean - TRUE if a new packet is received, FALSE otherwise
 * Description: to process the received data and extract sensor readings and vehicle data
 */
bool Data_PreProcessing_Process(uint16 received_byte) {
    
    if (received_byte == 'S' && UART0_Data_Recieved() && UART0_ReceiveByte() == 'D' && 
        UART0_Data_Recieved() && UART0_ReceiveByte() == ':') {
        UART0_ReceiveString(string_buffer);
        return Process_String_Data(string_buffer);
    }
    
    return FALSE;
}

/*
 * Service Name: Data_PreProcessing_Get_UltraSonic_Readings
 * Sync/Async: Synchronous
 * Reentrancy: Reentrant
 * Parameters (in): output_sensor_readings - pointer to the array to store the sensor readings
 * Parameters (inout): None
 * Parameters (out): None
 * Return value: None
 * Description: to get the ultrasonic sensor readings
 */
void Data_PreProcessing_Get_UltraSonic_Readings(uint16 *output_sensor_readings){
    uint8 i ;
    for (i = 0; i < SENSORS_COUNT; i++){
        output_sensor_readings[i] = sensor_readings[i];
    }  
}


/*
 * Service Name: Data_PreProcessing_Get_Vehicle_Speed
 * Sync/Async: Synchronous
 * Reentrancy: Reentrant
 * Parameters (in): None
 * Parameters (inout): None
 * Parameters (out): None
 * Return value: uint16 - vehicle speed
 * Description: to get the vehicle speed
 */
uint16 Data_PreProcessing_Get_Vehicle_Speed(void){
    return vehicle_speed;
}

/*
 * Service Name: Data_PreProcessing_Get_Throuttle
 * Sync/Async: Synchronous
 * Reentrancy: Reentrant
 * Parameters (in): None
 * Parameters (inout): None
 * Parameters (out): None
 * Return value: uint16 - throttle value
 * Description: to get the throttle value
 */
uint16 Data_PreProcessing_Get_Throuttle(void){
    return throuttle;
}

/*
 * Service Name: Data_PreProcessing_Get_Steering
 * Sync/Async: Synchronous
 * Reentrancy: Reentrant
 * Parameters (in): None
 * Parameters (inout): None
 * Parameters (out): None
 * Return value: uint16 - steering value
 * Description: to get the steering value
 */
uint16 Data_PreProcessing_Get_Steering(void){
    return steering;
}

/*
 * Service Name: Data_PreProcessing_Get_Brake
 * Sync/Async: Synchronous
 * Reentrancy: Reentrant
 * Parameters (in): None
 * Parameters (inout): None
 * Parameters (out): None
 * Return value: uint16 - brake value
 * Description: to get the brake value
 */
uint16 Data_PreProcessing_Get_Brake(void){
    return brake;
}

/*
 * Service Name: Process_Sensor_Data
 * Sync/Async: Synchronous
 * Reentrancy: Reentrant
 * Parameters (in): data - pointer to the received data buffer
 *                  length - length of the received data
 * Parameters (inout): None
 * Parameters (out): None
 * Return value: None
 * Description: to process the sensor data received from the PC
 */
void Process_Sensor_Data(uint8* data, uint8 length) {
    // Parse sensor data - simple, no checksum
    if (length >= 24) {  // 12 sensors × 2 bytes each
        uint8 i;
        for (i = 0; i < SENSORS_COUNT; i++) {
            sensor_readings[i] = (data[i*2+1] << 8) | data[i*2];
        }
        
        // Parse vehicle data if available
        if (length >= 28) {
            vehicle_speed = (data[25] << 8) | data[24];
            throuttle = data[26];
            brake = data[27];
            
            if (length >= 29) {
                steering = data[28];
            }
        }
    
        char debug[50];
        sprintf(debug, "Right sensors: %d, %d\r\n", 
                sensor_readings[10], sensor_readings[11]);
        UART0_SendString(debug);

        AutoParking_Process(sensor_readings, vehicle_speed);
    }
}



/*
 * Service Name: Process_String_Data
 * Sync/Async: Synchronous
 * Reentrancy: Reentrant
 * Parameters (in): str - pointer to the string data
 * Parameters (inout): None
 * Parameters (out): None
 * Return value: boolean - TRUE if processing was successful, FALSE otherwise
 * Description: to process the string data received from the PC
 */
bool Process_String_Data(char* str) {
    UART0_SendString("Processing data: ");
    UART0_SendString(str);
    UART0_SendString("\r\n");

    // Parse vehicle data
    UART0_SendString("Parsing vehicle data...\r\n");
    char* ptr = str;
    char* end;

    // Parse vehicle speed
    vehicle_speed = strtol(ptr, &end, 10);
    if (ptr == end) {
        UART0_SendString("Error: Invalid vehicle speed\r\n");
        return FALSE; 
    }
    UART0_SendString("Vehicle speed: ");
    UART0_SendNumber(vehicle_speed);
    UART0_SendString("\r\n");
    ptr = end + 1;

    // Parse throttle
    throuttle = strtol(ptr, &end, 10);
    if (ptr == end) {
        UART0_SendString("Error: Invalid throttle\r\n");
        return FALSE;
    }
    UART0_SendString("Throttle: ");
    UART0_SendNumber(throuttle);
    UART0_SendString("\r\n");
    ptr = end + 1;

    // Parse brake
    brake = strtol(ptr, &end, 10);
    if (ptr == end) {
        UART0_SendString("Error: Invalid brake\r\n");
        return FALSE;
    }
    UART0_SendString("Brake: ");
    UART0_SendNumber(brake);
    UART0_SendString("\r\n");
    ptr = end + 1;

    // Parse steering
    steering = strtol(ptr, &end, 10);
    if (ptr == end) {
        UART0_SendString("Error: Invalid steering\r\n");
        return FALSE;
    }
    UART0_SendString("Steering: ");
    UART0_SendNumber(steering);
    UART0_SendString("\r\n");
    ptr = end + 1;

    // Parse sensor values
    uint8 i;
    for (i = 0; i < SENSORS_COUNT; i++) {
        sensor_readings[i] = strtol(ptr, &end, 10);
        if (sensor_readings[i] == 999) {
            sensor_readings[i] = 600;
        }
        if (ptr == end) {
            UART0_SendString("Error: Not enough sensor values\r\n");
            return FALSE;
        }
        ptr = end + 1;
    }


    UART0_SendString("Speed: ");
    UART0_SendNumber(vehicle_speed);
    UART0_SendString(", Right sensors: ");
    UART0_SendNumber(sensor_readings[10]);
    UART0_SendString(", ");
    UART0_SendNumber(sensor_readings[11]);
    UART0_SendString("\r\n");

    if (AutoParking_IsActive()) {
        UART0_SendString("Auto-parking active, processing data\r\n");
        AutoParking_Process(sensor_readings, vehicle_speed);
    }

    return TRUE;
}