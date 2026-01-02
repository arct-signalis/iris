#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <Arduino.h>
#include "structs.h"
#include "settings.h"

String formatTestTelemetry(const FlightData& data, TelemetryType type = ALL);

#endif