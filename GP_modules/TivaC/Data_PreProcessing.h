 /******************************************************************************
 *
 * Module: Data PreProcessing
 *
 * File Name: Data_PreProcessing.h
 *
 * Description: Header file for the Data PreProcessing System when receiving data from the PC
 *
 * Author: Tarek Emad
 *
 *******************************************************************************/

#ifndef DATA_PREPROCESSING_H_
#define DATA_PREPROCESSING_H_

#include "std_types.h"
#include "tm4c123gh6pm_registers.h"
#include "uart0.h"
#include "COM_Protocol.h"
#include <stdlib.h>
#include <string.h>


/*******************************************************************************
 *                             Functions Prototypes                            *
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
void Data_PreProcessing_Init(void);

/*
 * Service Name: Data_PreProcessing_Process
 * Sync/Async: Synchronous
 * Reentrancy: Reentrant
 * Parameters (in): received_byte - the byte received from the UART
 * Parameters (inout): None
 * Parameters (out): None
 * Return value: boolean 
 * Description: to process the received data and extract sensor readings and vehicle data
 */
bool Data_PreProcessing_Process(uint16 recieved_data);

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
void Data_PreProcessing_Get_UltraSonic_Readings(uint16 *sensor_readings);

/*
 * Service Name: Data_PreProcessing_Get_Vehicle_Speed
 * Sync/Async: Synchronous
 * Reentrancy: Reentrant
 * Parameters (in): None
 * Parameters (inout): None
 * Parameters (out): None
 * Return value: vehicle speed
 * Description: to get the vehicle speed
 */
uint16 Data_PreProcessing_Get_Vehicle_Speed(void);

/*
 * Service Name: Data_PreProcessing_Get_Throuttle
 * Sync/Async: Synchronous
 * Reentrancy: Reentrant
 * Parameters (in): None
 * Parameters (inout): None
 * Parameters (out): None
 * Return value: throttle value
 * Description: to get the throttle value
 */
uint16 Data_PreProcessing_Get_Throuttle(void);

/*
 * Service Name: Data_PreProcessing_Get_Steering
 * Sync/Async: Synchronous
 * Reentrancy: Reentrant
 * Parameters (in): None
 * Parameters (inout): None
 * Parameters (out): None
 * Return value: steering value
 * Description: to get the steering value
 */
uint16 Data_PreProcessing_Get_Steering(void);

/*
 * Service Name: Data_PreProcessing_Get_Brake
 * Sync/Async: Synchronous
 * Reentrancy: Reentrant
 * Parameters (in): None
 * Parameters (inout): None
 * Parameters (out): None
 * Return value: brake value
 * Description: to get the brake value
 */
uint16 Data_PreProcessing_Get_Brake(void);

/*
 * Service Name: Process_Sensor_Data
 * Sync/Async: Synchronous
 * Reentrancy: Reentrant
 * Parameters (in): pointer to the sensor data array, length of the sensor data array
 * Parameters (inout): None
 * Parameters (out): None
 * Return value: None
 * Description: to process the sensor data received from the PC
 */
void Process_Sensor_Data(uint8* data, uint8 length);

/*
 * Service Name: Process_String_Data
 * Sync/Async: Synchronous
 * Reentrancy: Reentrant
 * Parameters (in): pointer to the string data received from the PC
 * Parameters (inout): None
 * Parameters (out): None
 * Return value: boolean
 * Description: to process the string data received from the PC
 */
bool Process_String_Data(char* str);

#endif /* DATA_PREPROCESSING_H_ */
