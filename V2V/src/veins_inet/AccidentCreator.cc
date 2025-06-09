#include "veins_inet/AccidentCreator.h"
#include "veins_inet/VeinsInetSampleApplication.h"
#include "veins_inet/LogHandler.h"
#include "veins_inet/PacketSender.h"
#include "veins_inet/VeinsInetSampleMessage_m.h"

#include "inet/common/packet/Packet.h"

using namespace inet;

/**
 * Service Name: Create Accident
 * Sync/Async: Synchronous 
 * Reentrancy: Non-reentrant 
 * Parameters:app, accidentType
 * Return value: void
 * Description: Produces accident scenario of a specific vehicle and broadcasts directly
* emergency warning to all nearby vehicles in the V2V network. This safety feature that could save lives:
 1. VISUAL INDICATION: Changes vehicle display to red color for graphical illustration of accident
 2. VEHICLE CONTROL: Brakes the vehicle immediately by adjusting the speed to 0
 3. MESSAGE CREATION: Creates informative accident message containing:
 Vehicle ID and index
 Precise GPS coordinates (X, Y position)
 Road ID and timestamp
 Accident type classification
 4. LOGGING: Creates comprehensive log records for accident analysis and debugging
 5. BROADCAST: Alerts emergency packet to all vehicles in communication range
 6. STATE MANAGEMENT: Informs vehicle that it has transmitted its own accident report

Required for real-time accident detection, emergency response coordination, and
vehicle safety warning systems in the V2V network. Enables rapid
distribution of essential safety information to prevent secondary accidents
 */
void AccidentCreator::createAccident(VeinsInetSampleApplication* app, const std::string& accidentType)
{
    // Change visual appearance to red (accident)
    app->getParentModule()->getDisplayString().setTagArg("i", 1, "red");

    // Stop the vehicle
    app->getTraciVehicle()->setSpeed(0);

    // Create the accident message with all details
    auto payload = makeShared<VeinsInetSampleMessage>();
    payload->setChunkLength(B(100));
    payload->setRoadId(app->getTraciVehicle()->getRoadId().c_str());
    
    // Set car ID and accident location
    payload->setCarId(app->getMobility()->getExternalId().c_str());
    auto position = app->getMobility()->getCurrentPosition();
    payload->setAccidentX(position.x);
    payload->setAccidentY(position.y);
    
    app->callTimestampPayload(payload);

    // Create log message for accident
    std::string accidentLogMessage = "\n" + std::string(50, '=') + "\n";
    accidentLogMessage += "🚨 " + accidentType + " ACCIDENT DETECTED:\n";
    accidentLogMessage += " Accident car ID: " + std::string(payload->getCarId()) + "\n";
    accidentLogMessage += " Car index: " + std::to_string(app->getParentModule()->getIndex()) + "\n";
    accidentLogMessage += " Accident location: (" + std::to_string(payload->getAccidentX()) + ", " + std::to_string(payload->getAccidentY()) + ")\n";
    accidentLogMessage += " Road ID: " + std::string(payload->getRoadId()) + "\n";
    accidentLogMessage += " Simulation time: " + std::to_string(simTime().dbl()) + "\n";
    accidentLogMessage += " STATUS: Vehicle stopped, sending accident message\n";
    accidentLogMessage += std::string(50, '#');
    
    // Write to log file
    LogHandler::writeToLog(accidentLogMessage);

    // Print detailed accident message
    EV_INFO << "🚨 " << accidentType << " ACCIDENT DETECTED!" << endl;
    EV_INFO << " Sending accident message from car: " << payload->getCarId() << endl;
    EV_INFO << " Accident location: (" << payload->getAccidentX() << ", " << payload->getAccidentY() << ")" << endl;
    EV_INFO << " Road ID: " << payload->getRoadId() << endl;
    EV_INFO << " Time: " << simTime() << endl;
    EV_INFO << " Car Index: " << app->getParentModule()->getIndex() << endl;
    
    // Also print to console
    std::cout << " " << accidentType << " ACCIDENT at time " << simTime() 
              << " - Car " << payload->getCarId() << " (index " << app->getParentModule()->getIndex() 
              << ") at (" << payload->getAccidentX() << ", " << payload->getAccidentY() << ")" << std::endl;

    auto packet = app->callCreatePacket("accident");
    packet->insertAtBack(payload);
    PacketSender::sendPacket(app, std::move(packet));
    
    // Mark that this car has sent its own accident message
    app->hasSentOwnAccident = true;
}
