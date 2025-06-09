#ifndef __MESSAGEIDGENERATOR_H
#define __MESSAGEIDGENERATOR_H

#include "veins_inet/VeinsInetSampleMessage_m.h"
#include <string>

class MessageIdGenerator {
public:
    /**
     * Service Name: Unique Message ID Generation Service
     * Sync/Async: Synchronous
     * Reentrancy: Thread-safe
     * Parameters:
     *   - payload: const VeinsInetSampleMessage* - Message payload
     * Return value: std::string - Unique message identifier
     * Description: Creates unique message ID based on car ID and accident coordinates
     * for duplicate detection and message tracking.
     */
    static std::string generateMessageId(const VeinsInetSampleMessage* payload);
    
    /**
     * Service Name: Relay Message ID 
     * Sync/Async: Synchronous
     * Reentrancy: Thread-safe
     * Parameters: baseMessageId, Original message ID
     * Return value:Relay-specific message identifier
     * Description: Creates relay-specific ID to track relayed messages separately.
     */
    static std::string generateRelayId(const std::string& baseMessageId);
    
private:
    static const std::string RELAY_SUFFIX;
};

#endif
