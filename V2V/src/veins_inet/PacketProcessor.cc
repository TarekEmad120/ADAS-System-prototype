#include "veins_inet/PacketProcessor.h"
#include "veins_inet/VeinsInetSampleApplication.h"
#include "veins_inet/LogHandler.h"
#include "veins_inet/PacketSender.h"
#include "veins_inet/VeinsInetSampleMessage_m.h"
#include "veins_inet/MessageValidator.h"
#include "veins_inet/RelayManager.h"
#include "veins_inet/MessageIdGenerator.h"
#include "veins_inet/VehicleDisplayManager.h"

#include "inet/common/packet/Packet.h"
#include <cmath>

using namespace inet;

/**
 * Service Name: V2V Packet Reception and Processing Service
 * Sync/Async: Synchronous with asynchronous relay
 * Reentrancy: Thread-safe with duplicate detection
 * Parameters:
 *   - app: VeinsInetSampleApplication* - Application instance
 *   - pk: std::shared_ptr<inet::Packet> - Received packet
 *   - seenMessages: std::set<std::string>& - Set of processed messages
 * Return value: void
 * Description: Processes incoming V2V packets with validation, duplicate detection,
 * logging, and intelligent relay management for accident notification propagation.
 */
void PacketProcessor::processPacket(VeinsInetSampleApplication* app, std::shared_ptr<inet::Packet> pk, std::set<std::string>& seenMessages)
{
    auto payload = pk->peekAtFront<VeinsInetSampleMessage>();
    
    // Check if this message is from the same car (to avoid self-reception)
    if (MessageValidator::isSelfSentMessage(app, payload.get())) {
        EV_INFO << "🚫 Ignoring self-sent message from car " << payload->getCarId() << endl;
        return;
    }

    // Create unique message ID based on car ID and accident coordinates
    std::string messageId = MessageIdGenerator::generateMessageId(payload.get());
    
    // Check if we've already seen this message
    if (MessageValidator::isDuplicateMessage(messageId, seenMessages)) {
        EV_INFO << " Car " << app->getMobility()->getExternalId() 
                << " already processed message " << messageId << ", ignoring" << endl;
        return;
    }

    // Determine message type from packet name
    std::string messageType = std::string(pk->getName());
    std::string messageIcon = (messageType == "accident") ? "🚨" : "🔄";
    

    // Create log message for received packet
    std::string logMessage = "📨 RECEIVED PACKET:\n";
    logMessage += " Receiving car ID: " + std::string(app->getMobility()->getExternalId()) + "\n";
    logMessage += " Original accident car ID: " + std::string(payload->getCarId()) + "\n";
    logMessage += " Accident location: (" + std::to_string(payload->getAccidentX()) + ", " + std::to_string(payload->getAccidentY()) + ")\n";
    logMessage += " Road ID: " + std::string(payload->getRoadId()) + "\n";
    logMessage += " Simulation time: " + std::to_string(simTime().dbl()) + "\n";
    logMessage += "========================================";
    
    // Write to log file
    LogHandler::writeToLog(logMessage);

    // Print detailed received message information to console
    EV_INFO << "========================================" << endl;
    EV_INFO << " Receiving car ID: " << app->getMobility()->getExternalId() << endl;
    EV_INFO << " Original accident car ID: " << payload->getCarId() << endl;
    EV_INFO << " Accident location: (" << payload->getAccidentX() << ", " << payload->getAccidentY() << ")" << endl;
    EV_INFO << " Road ID: " << payload->getRoadId() << endl;
    EV_INFO << " Time: " << simTime() << endl;    EV_INFO << "========================================" << endl;
    
    // Set vehicle color to green to indicate message received
    VehicleDisplayManager::setVehicleColor(app, VehicleDisplayManager::COLOR_GREEN);

    // Change route towards accident location
    VehicleDisplayManager::changeVehicleRoute(app, payload->getRoadId(), 999.9);
    
    // Setup relay logic with controlled frequency
    auto relayCallback = [app, payload, messageId, &seenMessages]() {
        // Check if we have not already relayed this specific message
        std::string relayId = MessageIdGenerator::generateRelayId(messageId);
        if (seenMessages.find(relayId) != seenMessages.end()) {
            return;
        }
        
        // Check if message should be relayed (distance and probability filters)
        if (!RelayManager::shouldRelay(app, payload.get(), messageId)) {
            EV_INFO << "🚫 Car " << app->getMobility()->getExternalId() 
                    << " skipping relay due to filters" << endl;
            return;
        }
        
        // Mark as relayed
        seenMessages.insert(relayId);
        
        // Create log message for relay
        std::string relayLogMessage = "🔄 RELAYING MESSAGE:\n";
        relayLogMessage += " Relay car: " + std::string(app->getMobility()->getExternalId()) + "\n";
        relayLogMessage += " Original accident car: " + std::string(payload->getCarId()) + "\n";
        relayLogMessage += " Accident location: (" + std::to_string(payload->getAccidentX()) + ", " + std::to_string(payload->getAccidentY()) + ")\n";
        relayLogMessage += " Simulation time: " + std::to_string(simTime().dbl()) + "\n";
        relayLogMessage += "----------------------------------------";
        
        // Write relay log to file
        LogHandler::writeToLog(relayLogMessage);

        // When forwarding, also print relay information
        EV_INFO << " RELAYING MESSAGE from car " << payload->getCarId() << " to other vehicles" << endl;
        std::cout << " Car " << app->getMobility()->getExternalId() << " relaying accident message from car " 
                  << payload->getCarId() << " at time " << simTime() << std::endl;

        auto packet = app->callCreatePacket("relay");
        packet->insertAtBack(payload);
        
        // Debug: Verify packet name before sending
        EV_INFO << " RELAY DEG: Create packet named '" << packet->getName()  << endl;
        std::cout << " RELAY DEG: Car " << app->getMobility()->getExternalId() 
                  << " created relay packet named '" << packet->getName() <<  std::endl;
        
        PacketSender::sendPacket(app, std::move(packet));    };
    
    // Calculate random delay to prevent message collision storms
    double relayDelay = RelayManager::calculateRelayDelay();
    app->getTimerManager().create(veins::TimerSpecification(relayCallback).oneshotIn(SimTime(relayDelay, SIMTIME_S)));
}
