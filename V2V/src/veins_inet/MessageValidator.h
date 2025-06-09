#ifndef __MESSAGEVALIDATOR_H
#define __MESSAGEVALIDATOR_H

#include "veins_inet/VeinsInetSampleMessage_m.h"
#include "veins_inet/VeinsInetSampleApplication.h"
#include <set>
#include <string>

class MessageValidator {
public:
    /**
     * Service Name: V2V Message Validation Service
     * Sync/Async: Synchronous
     * Reentrancy: Thread-safe
     * Parameters:
     *   - message: const VeinsInetSampleMessage* - Message to validate
     * Return value: bool - true if message is valid, false otherwise
     * Description: Validates V2V message integrity including coordinate bounds,
     * road ID format, car ID format, and message completeness.
     */
    static bool validateMessage(const VeinsInetSampleMessage* message);
    
    /**
     * Service Name: Duplicate Message Detection Service
     * Sync/Async: Synchronous  
     * Reentrancy: Thread-safe
     * Parameters:
     *   - messageId: const std::string& - Unique message identifier
     *   - seenMessages: std::set<std::string>& - Set of processed messages
     * Return value: bool - true if duplicate, false if new message
     * Description: Checks if message has been previously processed to prevent loops.
     */
    static bool isDuplicateMessage(const std::string& messageId, std::set<std::string>& seenMessages);
    
    /**
     * Service Name: Self Message Detection Service
     * Sync/Async: Synchronous
     * Reentrancy: Thread-safe
     * Parameters:
     *   - app: VeinsInetSampleApplication* - Application instance
     *   - payload: const VeinsInetSampleMessage* - Message payload
     * Return value: bool - true if self-sent message, false otherwise
     * Description: Checks if received message was sent by the same vehicle.
     */
    static bool isSelfSentMessage(VeinsInetSampleApplication* app, const VeinsInetSampleMessage* payload);

private:
    static bool isValidCoordinate(double x, double y);
    static bool isValidRoadId(const std::string& roadId);
    static bool isValidCarId(const std::string& carId);
};

#endif
