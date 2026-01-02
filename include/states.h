#ifndef STATES_H
#define STATES_H

#include <Arduino.h>

#include "settings.h"
#include "structs.h"
#include "sensors.h"
#include "unifiedCollector.h"
#include "telemetry.h"

extern UnifiedCollectorBuffer collectorBuffer;
extern FlightData flightData;
extern FlightStateData stateData;

void changeState(FlightState newState);
void collectTelemetry();
void debugLoop(TelemetryType type);
void groundIdleLoop();
void ascendingNoEngineLoop();
void ascendingEngineLoop();
void descendingLoop();
void landedLoop();

#endif