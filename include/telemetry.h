#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <Arduino.h>
#include "config.h"
#include "structs.h"

// ========== TELEMETRY MANAGEMENT ====================
enum TelemetryType
{
  NONE,
  ACCEL,
  GYRO,
  MAG,
  ENV,
  GPS_SENSOR,
  ATTITUDE,
  ALL
};

String formatTestTelemetry(const FlightData *data, TelemetryType type);

#endif