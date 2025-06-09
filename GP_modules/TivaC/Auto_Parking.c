 /******************************************************************************
 *
 * Module: Auto Parking
 *
 * File Name: Auto_Parking.c
 *
 * Description: Source file for the Auto Parking System
 *
 * Author: Tarek Emad
 *
 *******************************************************************************/


#include "Auto_Parking.h"
#include "tm4c123gh6pm_registers.h"
#include "uart0.h"
#include "COM_Protocol.h"


/*******************************************************************************
 *                             Global Functions                                *
 *******************************************************************************/

 static ParkingState ParkingStatus = PARKING_STATE_IDLE;
 static boolean ParkingActive = FALSE;
 static boolean AutoparkState = FALSE;
 static boolean AutoparkState2 = FALSE;
 static uint8 AlignmentPhase = 0;

/*******************************************************************************
 *                         Public Functions Definitions                        *
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
void AutoParking_Init(void)
{
    ParkingStatus = PARKING_STATE_IDLE;
    ParkingActive = FALSE;
    AlignmentPhase = 0;
    AutoparkState = FALSE;
    AutoparkState2 = FALSE;
}

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
void AutoParking_Activate(void)
{
    ParkingStatus = PARKING_STATE_IDLE;
    ParkingActive = TRUE;
    AutoparkState = FALSE;
    AutoparkState2 = FALSE;
    AlignmentPhase = 0;
    UART0_SendString("Auto-parking activated!\r\n");
}

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
void AutoParking_Deactivate(void)
{
    ParkingStatus = PARKING_STATE_IDLE;
    ParkingActive = FALSE;
    AutoparkState = FALSE;
    AutoparkState2 = FALSE;
    AlignmentPhase = 0;
    AutoParking_SendData(CONTROL_STOP, 0);
    UART0_SendString("Auto-parking deactivated\r\n");
}

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
bool AutoParking_IsActive(void)
{
    return ParkingActive;
}

/*
 * Service Name: AutoParking_Process
 * Sync/Async: Synchronous
 * Reentrancy: Reentrant
 * Parameters (in): sensor_readings - array of sensor readings
 * Parameters (inout): None
 * Parameters (out): None
 * Return value: None
 * Description: to process the auto-parking logic based on sensor readings
 */
void AutoParking_Process(uint16 *sensor_readings, uint16 vehicle_speed)
{
    if (!ParkingActive)
    {
        return;
    }

    /*                  Front Sensors Reading                     */
    uint16 frontDistance1 = sensor_readings[0];
    uint16 frontDistance2 = sensor_readings[1];
    uint16 frontFarLeftDistance = sensor_readings[2];
    uint16 frontFarRightDistance = sensor_readings[3];

    /*                  Rear Sensors Reading                      */
    uint16 rearDistance1 = sensor_readings[4];
    uint16 rearDistance2 = sensor_readings[5];
    uint16 rearFarLeftDistance = sensor_readings[6];
    uint16 rearFarRightDistance = sensor_readings[7];

    /*                  Left Sensors Reading                      */
    uint16 leftFrontDistance = sensor_readings[8];
    uint16 leftRearDistance = sensor_readings[9];

    /*                  Right Sensors Reading                     */
    uint16 rightFrontDistance = sensor_readings[10];
    uint16 rightRearDistance = sensor_readings[11];

    /*                        Debug                               */
    UART0_SendString("Front: ");
    UART0_SendNumber(frontDistance1);
    UART0_SendString(", ");
    UART0_SendNumber(frontDistance2);
    UART0_SendString(" | Front Far L/R: ");
    UART0_SendNumber(frontFarLeftDistance);
    UART0_SendString(", ");
    UART0_SendNumber(frontFarRightDistance);
    UART0_SendString("\r\n");
    UART0_SendString("Rear: ");
    UART0_SendNumber(rearDistance1);
    UART0_SendString(", ");
    UART0_SendNumber(rearDistance2);
    UART0_SendString(" | Rear Far L/R: ");
    UART0_SendNumber(rearFarLeftDistance);
    UART0_SendString(", ");
    UART0_SendNumber(rearFarRightDistance);
    UART0_SendString("\r\n");
    UART0_SendString("Side: Left F/R: ");
    UART0_SendNumber(leftFrontDistance);
    UART0_SendString(", ");
    UART0_SendNumber(leftRearDistance);
    UART0_SendString(" | Right F/R: ");
    UART0_SendNumber(rightFrontDistance);
    UART0_SendString(", ");
    UART0_SendNumber(rightRearDistance);
    UART0_SendString("\r\n");



    switch(ParkingStatus){
        case PARKING_STATE_IDLE:
            ParkingStatus = PARKING_STATE_SEARCHING;
            UART0_SendString("State: SEARCHING\r\n");
            AutoParking_SendData(CONTROL_FORWARD, 20);
            AutoparkState = FALSE;
            AutoparkState2 = FALSE;
            break;

        case PARKING_STATE_SEARCHING:
                if (frontFarRightDistance < MIN_PARKING_SIZE_SPOT && rearFarRightDistance < MIN_PARKING_SIZE_SPOT && 
                    !AutoparkState)
                {
                    UART0_SendString("Potential parking spot found\r\n");
                    AutoParking_SendData(CONTROL_FORWARD, 20);
                    AutoparkState = TRUE;
                }
                else if (frontFarRightDistance < MIN_PARKING_SIZE_SPOT && 
                        rightFrontDistance > 120 && 
                        rightRearDistance > 120 && 
                        AutoparkState)
                {
                    UART0_SendString("Parking spot opening detected\r\n");
                    AutoParking_SendData(CONTROL_FORWARD, 20);
                    AutoparkState2 = TRUE;
                }
                else if (rightFrontDistance < 250 && 
                        rightRearDistance < 250 && 
                        AutoparkState2)
                {
                    UART0_SendString("Parking spot has been found\r\n");
                    ParkingStatus = PARKING_STATE_ALLIGN;
                    AutoParking_SendData(CONTROL_STOP, 0);
                    AutoparkState2 = FALSE;
                    AutoparkState = FALSE;
                    /* Initialize alignment phase */
                    AlignmentPhase = 0;
                }
                else
                {
                    AutoParking_SendData(CONTROL_FORWARD, 10);
                    UART0_SendString("State: SEARCHING\r\n");
                    UART0_SendString("AutoparkState: ");
                    UART0_SendNumber(AutoparkState);
                    UART0_SendString(", AutoparkState2: ");
                    UART0_SendNumber(AutoparkState2);
                    UART0_SendString("\r\n");
                }
                break;

        
        case PARKING_STATE_ALLIGN:
            switch (AlignmentPhase)
            {
                case 0:
                    /* Initial alignment - move forward until we're past the spot */
                    if (rightFrontDistance < MIN_PARKING_SIZE_SPOT && 
                        rightRearDistance < MIN_PARKING_SIZE_SPOT && 
                        rearFarRightDistance > MIN_PARKING_SIZE_SPOT)
                    {
                        UART0_SendString("Initial alignment complete - preparing for reverse maneuver\r\n");
                        AutoParking_SendData(CONTROL_STOP, 0);
                        AlignmentPhase = 1;
                    }
                    else
                    {
                        AutoParking_SendData(CONTROL_FORWARD, 10);
                        UART0_SendString("State: ALLIGNING - Moving forward past spot\r\n");
                    }
                    break;

                case 1:
                    /* PHASE 1: Reverse right at an angle until the car behind is visible in left mirror */
                    {
                        /* Calculate alignment metrics */
                        uint16 right_distance_delta = rightFrontDistance - rightRearDistance;
                        
                        /* Safety checks */
                        uint16 safety_margin = 40;  /* cm */
                        boolean is_obstacle_detected = (
                            rearDistance1 < safety_margin || 
                            rearDistance2 < safety_margin ||
                            (sensor_readings[0] < 15 || sensor_readings[1] < 15 || 
                            sensor_readings[2] < 15 || sensor_readings[3] < 15 || 
                            sensor_readings[4] < 15 || sensor_readings[5] < 15 || 
                            sensor_readings[6] < 15 || sensor_readings[7] < 15 || 
                            sensor_readings[8] < 15 || sensor_readings[9] < 15 || 
                            sensor_readings[10] < 15 || sensor_readings[11] < 15)
                        );
                        
                        /* Can we see the car behind us in left mirror? */
                        boolean car_behind_visible = rearFarLeftDistance < 420;  /* Detecting car/object behind us */
                        
                        if (is_obstacle_detected)
                        {
                            UART0_SendString("Safety stop - obstacle detected behind vehicle\r\n");
                            AutoParking_SendData(CONTROL_STOP, 0);
                            AlignmentPhase = 2;
                        }
                        else if (car_behind_visible)
                        {
                            UART0_SendString("Reverse-right phase complete - Car behind is visible in left mirror\r\n");
                            UART0_SendString("Rear Far left sensor reading: ");
                            UART0_SendNumber(rearFarLeftDistance);
                            UART0_SendString("cm\r\n");
                            AutoParking_SendData(CONTROL_STOP, 0);
                            AlignmentPhase = 2;
                        }
                        else
                        {
                            AutoParking_SendData(CONTROL_REVERSE_RIGHT, 40);
                            UART0_SendString("STATE: REVERSE RIGHT - Backing at angle until car behind is visible\r\n");
                            UART0_SendString("Rear Far left sensor reading: ");
                            UART0_SendNumber(rearFarLeftDistance);
                            UART0_SendString("cm (waiting for <420cm)\r\n");
                        }
                    }
                    break;

                case 2:
                    /* PHASE 2: Reverse straight */
                    {
                        uint16 safety_margin = 30; 
                        boolean is_obstacle_detected = (
                            rearDistance1 < safety_margin || 
                            rearDistance2 < safety_margin
                        );
                        
                        boolean right_position_reached = rearFarLeftDistance < 260;
                        
                        if (is_obstacle_detected)
                        {
                            UART0_SendString("Safety stop - obstacle detected behind vehicle\r\n");
                            AutoParking_SendData(CONTROL_STOP, 0);
                            AlignmentPhase = 3;
                        }
                        else if (right_position_reached)
                        {
                            UART0_SendString("Straight reversing complete - Right position reached\r\n");
                            UART0_SendString("Rear Far Left distance: ");
                            UART0_SendNumber(rearFarLeftDistance);
                            UART0_SendString("cm\r\n");
                            AutoParking_SendData(CONTROL_STOP, 0);
                            AlignmentPhase = 3;
                        }
                        else
                        {
                            AutoParking_SendData(CONTROL_BACKWARD, 30);
                            UART0_SendString("STATE: REVERSE STRAIGHT - Until right position\r\n");
                            UART0_SendString("Rear Far left distance: ");
                            UART0_SendNumber(rearFarLeftDistance);
                            UART0_SendString("cm (waiting for <260cm)\r\n");
                        }
                    }
                    break;

                case 3:
                    /* PHASE 3: Turn wheels fully left and reverse into final position */
                    {
                        uint16 safety_margin = 30; 
                        boolean is_obstacle_detected = (
                            rearDistance1 < safety_margin || 
                            rearDistance2 < safety_margin
                        );
                        
                        /* Calculate how parallel we are to the curb/parking spot */
                        uint16 right_distance_delta = abs(rightFrontDistance - rightRearDistance);
                        
                        /* Calculate average distances to determine centering */
                        uint16 right_avg = (rightFrontDistance + rightRearDistance) / 2;
                        boolean is_parallel = right_distance_delta < 30;  /* Right sensors should read similar values */
                        boolean is_positioned = right_avg > 150 && right_avg < 300; 
                        
                        if (is_obstacle_detected)
                        {
                            UART0_SendString("Safety stop - obstacle detected while parking\r\n");
                            AutoParking_SendData(CONTROL_STOP, 0);
                            AlignmentPhase = 4;  /* Final adjustment phase */
                        }
                        else if (is_parallel && is_positioned)
                        {
                            UART0_SendString("Final position reached - Parallel with curb\r\n");
                            UART0_SendString("Right side difference: ");
                            UART0_SendNumber(right_distance_delta);
                            UART0_SendString("cm, avg: ");
                            UART0_SendNumber(right_avg);
                            UART0_SendString("cm\r\n");
                            AutoParking_SendData(CONTROL_STOP, 0);
                            AlignmentPhase = 4;  /* Final adjustment phase */
                        }
                        else
                        {
                            /* Full left turn while reversing with moderate speed */
                            AutoParking_SendData(CONTROL_REVERSE_LEFT, 35);
                            UART0_SendString("STATE: REVERSE LEFT - Final positioning\r\n");
                            UART0_SendString("Right side difference: ");
                            UART0_SendNumber(right_distance_delta);
                            UART0_SendString("cm, avg: ");
                            UART0_SendNumber(right_avg);
                            UART0_SendString("cm\r\n");
                        }
                    }
                    break;

                case 4:
                    /* PHASE 4: Final alignment adjustments */
                    {
                        /* Simple alignment using front and rear center sensors only */
                        uint16 front_left = frontDistance1; 
                        uint16 front_right = frontDistance2;  
                        uint16 rear_left = rearDistance1;   
                        uint16 rear_right = rearDistance2; 
                        
                        /* Calculate front/rear alignment metrics */
                        uint16 front_diff = abs(front_left - front_right);
                        uint16 rear_diff = abs(rear_left - rear_right);
                        
                        /* Calculate safe distances */
                        uint16 min_safe = 25;  
                        boolean rear_too_close = (rear_left < min_safe && rear_left > 0) || 
                                                 (rear_right < min_safe && rear_right > 0);
                        boolean front_too_close = (front_left < min_safe && front_left > 0) || 
                                                  (front_right < min_safe && front_right > 0);
                        
                        UART0_SendString("Front sensors: ");
                        UART0_SendNumber(front_left);
                        UART0_SendString("cm, ");
                        UART0_SendNumber(front_right);
                        UART0_SendString("cm (diff: ");
                        UART0_SendNumber(front_diff);
                        UART0_SendString("cm)\r\n");
                        
                        UART0_SendString("Rear sensors: ");
                        UART0_SendNumber(rear_left);
                        UART0_SendString("cm, ");
                        UART0_SendNumber(rear_right);
                        UART0_SendString("cm (diff: ");
                        UART0_SendNumber(rear_diff);
                        UART0_SendString("cm)\r\n");
                        
                        /* Simple decision logic based on safety and alignment */
                        if (rear_too_close)
                        {
                            /* Safety first - move forward if too close to rear vehicle */
                            UART0_SendString("SAFETY: Moving forward - rear distance critical\r\n");
                            AutoParking_SendData(CONTROL_FORWARD, 15);
                        }
                        else if (front_too_close)
                        {
                            /* Safety first - move backward if too close to front vehicle */
                            UART0_SendString("SAFETY: Moving backward - front distance critical\r\n");
                            AutoParking_SendData(CONTROL_BACKWARD, 15);
                        }
                        else if (front_diff > 3)
                        {
                            /* Front sensors differ - need to align with front vehicle */
                            if (front_left > front_right)
                            {
                                /* Turn right to align (front left sensor detects more distance) */
                                UART0_SendString("ALIGNMENT: Forward-right to align with front vehicle\r\n");
                                AutoParking_SendData(CONTROL_FORWARD_RIGHT, 10);
                            }
                            else
                            {
                                /* Turn left to align (front right sensor detects more distance) */
                                UART0_SendString("ALIGNMENT: Forward-left to align with front vehicle\r\n");
                                AutoParking_SendData(CONTROL_FORWARD_LEFT, 10);
                            }
                        }
                        else if (rear_diff > 4)
                        {
                            /* Rear sensors differ - need to align with rear vehicle */
                            if (rear_left > rear_right)
                            {
                                /* Need to turn left to align with rear vehicle */
                                UART0_SendString("ALIGNMENT: Reverse-right to align with rear vehicle\r\n");
                                AutoParking_SendData(CONTROL_REVERSE_RIGHT, 10);
                            }
                            else
                            {
                                /* Need to turn right to align with rear vehicle */
                                UART0_SendString("ALIGNMENT: Reverse-left to align with rear vehicle\r\n");
                                AutoParking_SendData(CONTROL_REVERSE_LEFT, 10);
                            }
                        }
                        else
                        {
                            UART0_SendString("Parking complete! Position is acceptable.\r\n");
                            UART0_SendString("Final front sensors: ");
                            UART0_SendNumber(front_left);
                            UART0_SendString("cm, ");
                            UART0_SendNumber(front_right);
                            UART0_SendString("cm\r\n");
                            UART0_SendString("Final rear sensors: ");
                            UART0_SendNumber(rear_left);
                            UART0_SendString("cm, ");
                            UART0_SendNumber(rear_right);
                            UART0_SendString("cm\r\n");
                            ParkingStatus = PARKING_STATE_FINISHED;
                            AutoParking_SendData(CONTROL_STOP, 0);
                        }
                    }
                    break;

                default:
                    UART0_SendString("Error: Invalid alignment phase!\r\n");
                    ParkingStatus = PARKING_STATE_IDLE;
                    AlignmentPhase = 0;
                    AutoParking_SendData(CONTROL_STOP, 0);
                    break;
            }
            break;


        
        case PARKING_STATE_FINISHED:
            ParkingStatus = PARKING_STATE_IDLE;
            UART0_SendString("Auto-parking completed!\r\n");
            AutoParking_SendData(CONTROL_STOP, 0);
            ParkingActive = FALSE;
            AlignmentPhase = 0;
            break;

        default:
            UART0_SendString("Error: Unexpected state!\r\n");
            ParkingStatus = PARKING_STATE_IDLE;
            AutoParking_SendData(CONTROL_STOP, 0);
            break;

    }
    return ;

}

/*
 * Service Name: AutoParking_GetState
 * Sync/Async: Synchronous
 * Reentrancy: Reentrant
 * Parameters (in): None
 * Parameters (inout): None
 * Parameters (out): None
 * Return value: ParkingState - current state of the auto-parking system
 * Description: to get the current state of the Auto Parking System
 */
ParkingState AutoParking_GetState(void)
{
    return ParkingStatus;
}

/*
 * Service Name: AutoParking_SendData
 * Sync/Async: Synchronous
 * Reentrancy: Reentrant
 * Parameters (in): control - control command to send
 *                  value - value associated with the control command
 * Parameters (inout): None
 * Parameters (out): None
 * Return value: None
 * Description: to send data to the vehicle control system via UART
 */
void AutoParking_SendData(uint16 control, uint16 value)
{
    UART0_SendString("CMD:");
    UART0_SendNumber(control);
    UART0_SendString(",");
    UART0_SendNumber(value);
    UART0_SendString("#\r\n");

    UART0_SendString("ACK#\r\n");
}
