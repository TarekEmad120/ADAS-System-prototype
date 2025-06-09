#include "veins_inet/LogHandler.h"
#include "omnetpp.h"

using namespace omnetpp;

// Static member definitions
std::ofstream LogHandler::logFile;
bool LogHandler::fileInitialized = false;

/**
 * Service Name: V2V Communication Log Writer
 * Sync/Async: Synchronous
 * Reentrancy: Thread-safe with static file management
 * Parameters: The message to write to the file
 * Return value: void
 * Description: Writes V2V communication events, accident notifications, and system
 * messages to "V2V_accident_messages.txt".
 */
void LogHandler::writeToLog(const std::string& message)
{
    // File should already be initialized in startApplication
    if (!logFile.is_open() && !fileInitialized) {
        logFile.open("V2V_accident_messages.txt", std::ios::out | std::ios::trunc);
        if (logFile.is_open()) {
            logFile << "=== V2V Accident Communication Log ===" << std::endl;
            logFile << "Simulation started at: " << simTime() << std::endl;
            logFile << "----------------------------------------------" << std::endl;
            fileInitialized = true;
        }
    }
    
    if (logFile.is_open()) {
        logFile << message << std::endl;
        logFile.flush(); // Ensure immediate write to file
    }
}

void LogHandler::initializeLogFile()
{
    if (!fileInitialized) {
        // Clear previous log and start fresh for new simulation
        logFile.open("V2V_accident_messages.txt", std::ios::out | std::ios::trunc);
        if (logFile.is_open()) {
            logFile << "=== V2V Accident Communication Log ===" << std::endl;
            logFile << "Simulation started at: " << simTime() << std::endl;
            logFile << "----------------------------------------------" << std::endl;
            logFile.flush();
            fileInitialized = true;
        }
    }
}

void LogHandler::closeLogFile()
{
    static bool simulationEnded = false;
    
    if (logFile.is_open() && !simulationEnded) {
        // Check if this is the last vehicle by looking at simulation time
        if (simTime() >= SimTime(59, SIMTIME_S)) { // Close to end of 60s simulation
            logFile << std::endl;
            logFile << "----------------------------------------------" << std::endl;
            logFile << "Simulation ended at: " << simTime() << std::endl;
            logFile << "=== End of V2V Accident Communication Log ===" << std::endl;
            logFile.close();
            fileInitialized = false;
            simulationEnded = true;
        }
    }
}
