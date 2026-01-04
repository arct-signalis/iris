#ifndef STATES_H
#define STATES_H

#include <Arduino.h>
#include "config.h"
#include "telemetry.h"

// Forward declarations to avoid circular dependencies
struct UnifiedCollectorBuffer;
struct FlightData;

// ========== FLIGHT STATE MANAGEMENT ====================
enum FlightState
{
  DEBUG_STATE,
  SENSORS_CALIBRATING,
  GROUND_IDLE,
  ASCENDING_ENGINE,
  ASCENDING_NO_ENGINE,
  DESCENDING,
  LANDED
};

struct FlightStateData
{
  FlightState currentState;
  uint32_t startTime;
  uint32_t launchDetectionStart;
  uint32_t engineCutoffStart;
  float previousAltitude;
  uint32_t lastAltitudeTime;

  uint8_t descendingCount;      // Count consecutive descending readings
  uint32_t landingConfirmStart; // Start time for landing confirmation
  float landingAltitude;        // Altitude when landing detection started
  uint32_t lastEchoTime;
};

extern UnifiedCollectorBuffer collectorBuffer;
extern FlightData flightData;
extern FlightStateData stateData;

// Function declarations
void changeState(FlightState newState);
void collectTelemetry();
void debugLoop(TelemetryType type);
void groundIdleLoop();
void ascendingNoEngineLoop();
void ascendingEngineLoop();
void descendingLoop();
void landedLoop();

#endif