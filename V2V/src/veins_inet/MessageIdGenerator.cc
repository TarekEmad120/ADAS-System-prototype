#include "veins_inet/MessageIdGenerator.h"

// Static member definition
const std::string MessageIdGenerator::RELAY_SUFFIX = "_relayed";


std::string MessageIdGenerator::generateMessageId(const VeinsInetSampleMessage* payload)
{
    return std::string(payload->getCarId()) + "_" + 
           std::to_string(payload->getAccidentX()) + "_" + 
           std::to_string(payload->getAccidentY());
}


std::string MessageIdGenerator::generateRelayId(const std::string& baseMessageId)
{
    return baseMessageId + RELAY_SUFFIX;
}
