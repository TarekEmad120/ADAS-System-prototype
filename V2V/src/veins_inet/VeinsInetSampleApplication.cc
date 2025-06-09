

#include "veins_inet/VeinsInetSampleApplication.h"

#include "inet/common/ModuleAccess.h"
#include "inet/common/packet/Packet.h"
#include "inet/common/TagBase_m.h"
#include "inet/common/TimeTag_m.h"
#include "inet/networklayer/common/L3AddressResolver.h"
#include "inet/networklayer/common/L3AddressTag_m.h"
#include "inet/transportlayer/contract/udp/UdpControlInfo_m.h"
#include <cmath>

#include "veins_inet/VeinsInetSampleMessage_m.h"

using namespace inet;

Define_Module(VeinsInetSampleApplication);

/**
 * Service Name: VeinsInetSampleApplication Constructor
 * Sync/Async: Synchronous
 * Reentrancy: Thread-safe
 * Parameters: None
 * Return value: None (Constructor)
 * Description: Default constructor for the VeinsInetSampleApplication class.
 * Initializes a new instance of the V2V communication application that handles
 * vehicle-to-vehicle message exchange, accident detection, and emergency warning
 * protocols within the VEINS/INET framework simulation environment.
 */
VeinsInetSampleApplication::VeinsInetSampleApplication()
{
}

/**
 * Service Name: Application Startup Handler
 * Sync/Async: Synchronous
 * Reentrancy: Non-reentrant
 * Parameters: None
 * Return value: bool - Returns true if application starts successfully, false otherwise
 * Description: Initializes the V2V communication application and sets up accident scenarios
 * for specific vehicles in the simulation. Configures timer-based accident triggers for
 * vehicles at predefined simulation times:
 * - Vehicle[0]: Creates first accident at t=20s, resumes after 30s
 * - Vehicle[2]: Creates second accident at t=35s, resumes after 25s  
 * - Vehicle[4]: Creates third accident at t=50s, resumes after 20s
 * Also initializes the logging system for recording V2V communication events and
 * accident scenarios throughout the simulation lifecycle.
 */
bool VeinsInetSampleApplication::startApplication()
{
    // Initialize log file if this is the first car to start
    LogHandler::initializeLogFile();

    int carIndex = getParentModule()->getIndex();
    
    // host[0] should stop at t=20s (First accident)
    if (carIndex == 0) {
        auto callback = [this]() {
            AccidentCreator::createAccident(this, "FIRST");
            
            
            auto resumeCallback = [this]() {
                getTraciVehicle()->setSpeed(-1);
                EV_INFO << "🟢 Car " << getMobility()->getExternalId() << " resuming normal speed" << endl;
                std::cout << "🟢 Car " << getMobility()->getExternalId() << " resuming normal speed at time " << simTime() << std::endl;
            };
            getTimerManager().create(veins::TimerSpecification(resumeCallback).oneshotIn(SimTime(30, SIMTIME_S)));
        };
        getTimerManager().create(veins::TimerSpecification(callback).oneshotAt(SimTime(20, SIMTIME_S)));
    }
    
    // host[2] should stop at t=35s (Second accident)
    else if (carIndex == 2) {
        auto callback = [this]() {
            AccidentCreator::createAccident(this, "SECOND");
            
            // host should continue after 25s
            auto resumeCallback = [this]() {
                getTraciVehicle()->setSpeed(-1);
                EV_INFO << "🟢 Car " << getMobility()->getExternalId() << " resuming normal speed" << endl;
                std::cout << "🟢 Car " << getMobility()->getExternalId() << " resuming normal speed at time " << simTime() << std::endl;
            };
            getTimerManager().create(veins::TimerSpecification(resumeCallback).oneshotIn(SimTime(25, SIMTIME_S)));
        };
        getTimerManager().create(veins::TimerSpecification(callback).oneshotAt(SimTime(35, SIMTIME_S)));
    }
    
    // host[4] should stop at t=50s (Third accident)
    else if (carIndex == 4) {
        auto callback = [this]() {
            AccidentCreator::createAccident(this, "THIRD");
            
            // host should continue after 20s
            auto resumeCallback = [this]() {
                getTraciVehicle()->setSpeed(-1);
                EV_INFO << "🟢 Car " << getMobility()->getExternalId() << " resuming normal speed" << endl;
                std::cout << "🟢 Car " << getMobility()->getExternalId() << " resuming normal speed at time " << simTime() << std::endl;
            };
            getTimerManager().create(veins::TimerSpecification(resumeCallback).oneshotIn(SimTime(20, SIMTIME_S)));
        };
        getTimerManager().create(veins::TimerSpecification(callback).oneshotAt(SimTime(50, SIMTIME_S)));
    }

    return true;
}

/**
 * Service Name: Application Shutdown Handler
 * Sync/Async: Synchronous
 * Reentrancy: Non-reentrant
 * Parameters: None
 * Return value: bool - Returns true if application stops successfully, false otherwise
 * Description: Gracefully terminates the V2V communication application by calling the
 * parent class stopApplication() method. Ensures proper cleanup of network connections,
 * timer cancellations, and resource deallocation before the vehicle leaves the simulation.
 * This method is automatically invoked when a vehicle exits the simulation area or
 * when the simulation terminates.
 */
bool VeinsInetSampleApplication::stopApplication()
{
    return ApplicationLifecycle::stopApplication();
}

/**
 * Service Name: VeinsInetSampleApplication Destructor
 * Sync/Async: Synchronous
 * Reentrancy: Thread-safe
 * Parameters: None
 * Return value: None (Destructor)
 * Description: Destructor for the VeinsInetSampleApplication class that performs
 * critical cleanup operations. Closes the log file handler to ensure all V2V
 * communication logs, accident records, and performance metrics are properly
 * written to disk before the application instance is destroyed. Essential for
 * maintaining data integrity and preventing log file corruption in multi-vehicle
 * simulation scenarios.
 */
VeinsInetSampleApplication::~VeinsInetSampleApplication()
{
    LogHandler::closeLogFile();
}

/**
 * Service Name: V2V Packet Transmission Service
 * Sync/Async: Asynchronous
 * Reentrancy: Thread-safe
 * Parameters: 
 *   - pk: std::unique_ptr<inet::Packet> - Smart pointer to the packet to be transmitted
 * Return value: void
 * Description: Transmits V2V communication packets containing accident warnings, emergency
 * messages, or periodic beacons to neighboring vehicles. Utilizes the PacketSender utility
 * class to handle UDP broadcast transmission with appropriate addressing and routing.
 * The function transfers ownership of the packet through move semantics to ensure
 * efficient memory management. Critical for real-time accident notification and
 * vehicle safety coordination in the V2V network.
 */
void VeinsInetSampleApplication::sendPacket(std::unique_ptr<inet::Packet> pk)
{
    PacketSender::sendPacket(this, std::move(pk));
}

/**
 * Service Name: V2V Packet Reception and Processing Service
 * Sync/Async: Synchronous
 * Reentrancy: Thread-safe with duplicate detection
 * Parameters:
 *   - pk: std::shared_ptr<inet::Packet> - Shared pointer to the received packet
 * Return value: void
 * Description: Processes incoming V2V communication packets received from neighboring
 * vehicles. Implements duplicate message detection using the seenMessages container
 * to prevent message loops and redundant processing. Handles various message types
 * including accident warnings, emergency notifications, and status updates.
 * Delegates packet parsing and response generation to the PacketProcessor utility
 * class while maintaining message history for loop prevention and performance
 * optimization in dense vehicle networks.
 */
void VeinsInetSampleApplication::processPacket(std::shared_ptr<inet::Packet> pk)
{
    PacketProcessor::processPacket(this, pk, seenMessages);
}
