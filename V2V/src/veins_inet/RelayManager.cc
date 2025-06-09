#include "veins_inet/RelayManager.h"
#include <cmath>
#include <cstdlib>

// Static member definitions
const double RelayManager::MIN_RELAY_DISTANCE = 100.0;
const int RelayManager::RELAY_PROBABILITY = 50;
const double RelayManager::MIN_DELAY = 0.2;
const double RelayManager::MAX_DELAY = 0.9;

/**
 * Service Name: Message Relay Decision Service
 * Sync/Async: Synchronous
 * Reentrancy: Thread-safe
 * Parameters:
 *   - app: VeinsInetSampleApplication* - Application instance
 *   - payload: const VeinsInetSampleMessage* - Message to potentially relay
 *   - messageId: const std::string& - Unique message identifier
 * Return value: bool - true if message should be relayed, false otherwise
 * Description: Determines if a message should be relayed based on distance
 * and probability constraints to prevent network flooding.
 */
bool RelayManager::shouldRelay(VeinsInetSampleApplication* app, const VeinsInetSampleMessage* payload, const std::string& messageId)
{
    // Check probability filter first (fastest check)
    if (!passesProbabilityFilter()) {
        return false;
    }
    
    // Check distance constraint
    if (!isDistanceValidForRelay(app, payload)) {
        return false;
    }
    
    return true;
}

/**
 * Service Name: Distance-Based Relay Filter
 * Sync/Async: Synchronous
 * Reentrancy: Thread-safe
 * Parameters: VeinsInetSampleApplication and Message payload
 * Return value: true if distance allows relay else false 
 * Description: Checks if vehicle is far enough from accident to make relay useful.
 */
bool RelayManager::isDistanceValidForRelay(VeinsInetSampleApplication* app, const VeinsInetSampleMessage* payload)
{
    auto currentPos = app->getMobility()->getCurrentPosition();
    double distanceFromAccident = sqrt(pow(currentPos.x - payload->getAccidentX(), 2) + pow(currentPos.y - payload->getAccidentY(), 2));
    
    return distanceFromAccident >= MIN_RELAY_DISTANCE;
}

/**
 * Service Name: Probabilistic Relay Filter
 * Sync/Async: Synchronous
 * Reentrancy: Non-reentrant 
 * Parameters: None
 * Return value: true if probability allows relay else false
 * Description: Apply 50% probability filter to reduce network congestion
 */
bool RelayManager::passesProbabilityFilter()
{
    return (rand() % 100) >= RELAY_PROBABILITY;
}

/**
 * Service Name: Relay Delay Calculator
 * Sync/Async: Synchronous
 * Reentrancy: Non-reentrant 
 * Parameters: None
 * Return value:Delay in seconds (0.2 to 0.9)
 * Description: Calculates random delay to prevent message collision storms.
 */
double RelayManager::calculateRelayDelay()
{
    return MIN_DELAY + (rand() % 8) * 0.1; // 0.2, 0.3, 0.4 UNTIL 0.9 seconds
}
