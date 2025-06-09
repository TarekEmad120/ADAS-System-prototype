#ifndef __RELAYMANAGER_H
#define __RELAYMANAGER_H

#include "veins_inet/VeinsInetSampleMessage_m.h"
#include "veins_inet/VeinsInetSampleApplication.h"
#include <set>
#include <string>

class RelayManager {
public:
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
    static bool shouldRelay(VeinsInetSampleApplication* app, const VeinsInetSampleMessage* payload, const std::string& messageId);
    
    /**
     * Service Name: Distance-Based Relay Filter
     * Sync/Async: Synchronous
     * Reentrancy: Thread-safe
     * Parameters:
     *   - app: VeinsInetSampleApplication* - Application instance
     *   - payload: const VeinsInetSampleMessage* - Message payload
     * Return value: bool - true if distance allows relay, false otherwise
     * Description: Checks if vehicle is far enough from accident to make relay useful.
     */
    static bool isDistanceValidForRelay(VeinsInetSampleApplication* app, const VeinsInetSampleMessage* payload);
    
    /**
     * Service Name: Probabilistic Relay Filter
     * Sync/Async: Synchronous
     * Reentrancy: Non-reentrant (uses random)
     * Parameters: None
     * Return value: bool - true if probability allows relay, false otherwise
     * Description: Applies 50% probability filter to reduce network congestion.
     */
    static bool passesProbabilityFilter();
    
    /**
     * Service Name: Relay Delay Calculator
     * Sync/Async: Synchronous
     * Reentrancy: Non-reentrant (uses random)
     * Parameters: None
     * Return value: double - Delay in seconds (0.2 to 0.9)
     * Description: Calculates random delay to prevent message collision storms.
     */
    static double calculateRelayDelay();

private:
    static const double MIN_RELAY_DISTANCE; // 100.0 meters
    static const int RELAY_PROBABILITY;     // 50%
    static const double MIN_DELAY;          // 0.2 seconds
    static const double MAX_DELAY;          // 0.9 seconds
};

#endif
