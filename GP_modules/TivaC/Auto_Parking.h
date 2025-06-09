 /******************************************************************************
 *
 * Module: Auto Parking
 *
 * File Name: Auto_Parking.h
 *
 * Description: Header file for the Auto Parking System
 *
 * Author: Tarek Emad
 *
 *******************************************************************************/

#ifndef AUTO_PARKING_H_
#define AUTO_PARKING_H_

#include "std_types.h"
#include "tm4c123gh6pm_registers.h"
#include "uart0.h"
#include "COM_Protocol.h"


/*******************************************************************************
 *                             PreProcessor Macros                             *
 *******************************************************************************/

#define MIN_PARKING_SIZE_SPOT 300
#define SAFE_DISTANCE 100
#define DANGER_DISTANCE 10
#define ALLIGNMENT_THRESHOLD 10
#define ALLIGNMENT_DISTANCE 60



typedef enum
{
    PARKING_STATE_IDLE,
    PARKING_STATE_FORWARD,
    PARKING_STATE_BACKWARD,
    PARKING_STATE_RIGHT,
    PARKING_STATE_LEFT,
    PARKING_STATE_ALLIGN,
    PARKING_STATE_STOP,
    PARKING_STATE_SEARCHING,
    PARKING_STATE_FINISHED
}ParkingState;


/*******************************************************************************
 *                            Functions Prototypes                             *
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
void AutoParking_Init(void);

/*
 * Service Name: AutoParking_Activate
 * Sync/Async: Synchronous
 * Reentrancy: Reentrant
 * Parameters (in): None
 * Parameters (inout): None
 * Parameters (out): None
 * Return value: None
 * Description: to activate the Auto Parking System
 */
void AutoParking_Activate(void);

/*
 * Service Name: AutoParking_Deactivate
 * Sync/Async: Synchronous
 * Reentrancy: Reentrant
 * Parameters (in): None
 * Parameters (inout): None
 * Parameters (out): None
 * Return value: None
 * Description: to deactivate the Auto Parking System
 */
void AutoParking_Deactivate(void);

/*
 * Service Name: AutoParking_IsActive
 * Sync/Async: Synchronous
 * Reentrancy: Reentrant
 * Parameters (in): None
 * Parameters (inout): None
 * Parameters (out): None
 * Return value: boolean 
 * Description: to check if the Auto Parking System is active
 */
bool AutoParking_IsActive(void);

/*
 * Service Name: AutoParking_Process
 * Sync/Async: Synchronous
 * Reentrancy: Reentrant
 * Parameters (in): sensor_readings - array of sensor readings
 *                  vehicle_speed - current speed of the vehicle
 * Parameters (inout): None
 * Parameters (out): None
 * Return value: None
 * Description: to process the Auto Parking System logic based on sensor readings and vehicle speed
 */
void AutoParking_Process(uint16 *sensor_readings, uint16 vehicle_speed);

/*
 * Service Name: AutoParking_GetState
 * Sync/Async: Synchronous
 * Reentrancy: Reentrant
 * Parameters (in): None
 * Parameters (inout): None
 * Parameters (out): None
 * Return value: ParkingState 
 * Description: to get the current state of the Auto Parking System
 */
ParkingState AutoParking_GetState(void);

/*
 * Service Name: AutoParking_SendData
 * Sync/Async: Synchronous
 * Reentrancy: Reentrant
 * Parameters (in): control - control command to be sent
 *                  value - value associated with the control command
 * Parameters (inout): None
 * Parameters (out): None
 * Return value: None
 * Description: to send control data to the Auto Parking System
 */
void AutoParking_SendData(uint16 control, uint16 value);


#endif /* AUTO_PARKING_H_ */
