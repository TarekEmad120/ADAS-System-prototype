#include "veins_inet/VehicleDisplayManager.h"

// Static member definitions
const std::string VehicleDisplayManager::COLOR_RED = "red";
const std::string VehicleDisplayManager::COLOR_GREEN = "green";
const std::string VehicleDisplayManager::COLOR_BLUE = "blue";
const std::string VehicleDisplayManager::COLOR_YELLOW = "yellow";

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
void VehicleDisplayManager::setVehicleColor(VeinsInetSampleApplication* app, const std::string& color)
{
    app->getParentModule()->getDisplayString().setTagArg("i", 1, color.c_str());
}

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
void VehicleDisplayManager::changeVehicleRoute(VeinsInetSampleApplication* app, const std::string& roadId, double position)
{
    app->getTraciVehicle()->changeRoute(roadId, position);
}
