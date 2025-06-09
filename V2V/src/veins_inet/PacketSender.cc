#include "veins_inet/PacketSender.h"
#include "veins_inet/VeinsInetSampleApplication.h"
#include "veins_inet/LogHandler.h"
#include "veins_inet/VeinsInetSampleMessage_m.h"

#include "inet/common/packet/Packet.h"

using namespace inet;

void PacketSender::sendPacket(VeinsInetSampleApplication* app, std::unique_ptr<inet::Packet> pk)
{
    // Extract message details for logging
    auto payload = pk->peekAtFront<VeinsInetSampleMessage>();
    
    // Create log message
    std::string logMessage = "📤 SENDING PACKET:\n";
    logMessage += " From car: " + std::string(app->getMobility()->getExternalId()) + "\n";
    logMessage += " Original accident car: " + std::string(payload->getCarId()) + "\n";
    logMessage += " Accident location: (" + std::to_string(payload->getAccidentX()) + ", " + std::to_string(payload->getAccidentY()) + ")\n";
    logMessage += " Road ID: " + std::string(payload->getRoadId()) + "\n";
    logMessage += " Packet name: " + std::string(pk->getName()) + "\n";
    logMessage += " Simulation time: " + std::to_string(simTime().dbl()) + "\n";
    logMessage += "----------------------------------------";
    
    // Write to log file
    LogHandler::writeToLog(logMessage);
    
    EV_INFO << "📤 SENDING PACKET:" << endl;
    EV_INFO << " From car: " << app->getMobility()->getExternalId() << endl;
    EV_INFO << " Original accident car: " << payload->getCarId() << endl;
    EV_INFO << " Accident location: (" << payload->getAccidentX() << ", " << payload->getAccidentY() << ")" << endl;
    EV_INFO << " Road ID: " << payload->getRoadId() << endl;
    EV_INFO << " Packet name: " << pk->getName() << endl;
    EV_INFO << " Simulation time: " << simTime() << endl;
    EV_INFO << "----------------------------------------" << endl;
      // Also print to console
    std::cout << "📤 Car " << app->getMobility()->getExternalId() << " sending " << pk->getName() 
              << " message from car " << payload->getCarId() << " at time " << simTime() << std::endl;
    
    // Call the base class method to actually send the packet
    app->callBaseSendPacket(std::move(pk));
}
