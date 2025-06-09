#include "veins_inet/MessageValidator.h"
#include <cmath>
#include <regex>

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
bool MessageValidator::validateMessage(const VeinsInetSampleMessage* message)
{
    if (!message) {
        return false;
    }
    
    // Validate coordinates
    if (!isValidCoordinate(message->getAccidentX(), message->getAccidentY())) {
        return false;
    }
    
    // Validate road ID
    if (!isValidRoadId(message->getRoadId())) {
        return false;
    }
    
    // Validate car ID
    if (!isValidCarId(message->getCarId())) {
        return false;
    }
    
    return true;
}

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
bool MessageValidator::isDuplicateMessage(const std::string& messageId, std::set<std::string>& seenMessages)
{
    if (seenMessages.find(messageId) != seenMessages.end()) {
        return true; // Duplicate found
    }
    
    // Mark as seen
    seenMessages.insert(messageId);
    return false; // New message
}

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
bool MessageValidator::isSelfSentMessage(VeinsInetSampleApplication* app, const VeinsInetSampleMessage* payload)
{
    return std::string(payload->getCarId()) == std::string(app->getMobility()->getExternalId());
}

bool MessageValidator::isValidCoordinate(double x, double y)
{
    // Check for reasonable coordinate bounds (adjust based on your simulation area)
    return (x >= -10000.0 && x <= 10000.0 && y >= -10000.0 && y <= 10000.0);
}

bool MessageValidator::isValidRoadId(const std::string& roadId)
{
    // Check if road ID is not empty and has reasonable length
    return !roadId.empty() && roadId.length() < 100;
}

bool MessageValidator::isValidCarId(const std::string& carId)
{
    // Check if car ID is not empty and follows expected format
    return !carId.empty() && carId.length() < 50;
}
