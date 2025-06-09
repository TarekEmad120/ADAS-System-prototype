#ifndef __VEHICLEDISPLAYMANAGER_H
#define __VEHICLEDISPLAYMANAGER_H

#include "veins_inet/VeinsInetSampleApplication.h"
#include <string>

class VehicleDisplayManager {
public:
    /**
     * Service Name: Vehicle Visual State Update Service
     * Sync/Async: Synchronous
     * Reentrancy: Thread-safe
     * Parameters:
     *   - app: VeinsInetSampleApplication* - Application instance
     *   - color: const std::string& - Color to set ("red", "green", "blue", etc.)
     * Return value: void
     * Description: Updates vehicle visual appearance in the simulation display
     * to indicate different states (accident, message received, normal).
     */
    static void setVehicleColor(VeinsInetSampleApplication* app, const std::string& color);
    
    /**
     * Service Name: Vehicle Route Change Service
     * Sync/Async: Synchronous
     * Reentrancy: Thread-safe
     * Parameters:
     *   - app: VeinsInetSampleApplication* - Application instance
     *   - roadId: const std::string& - Target road identifier
     *   - position: double - Position on the road (default 999.9 for end)
     * Return value: void
     * Description: Changes vehicle route to redirect towards accident location
     * for emergency response or avoidance maneuvers.
     */
    static void changeVehicleRoute(VeinsInetSampleApplication* app, const std::string& roadId, double position = 999.9);

    // Color constants
    static const std::string COLOR_RED;     // Accident
    static const std::string COLOR_GREEN;   // Message received
    static const std::string COLOR_BLUE;    // Normal state
    static const std::string COLOR_YELLOW;  // Warning state
};

#endif
