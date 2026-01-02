#ifndef UNIFIED_COLLECTOR_H
#define UNIFIED_COLLECTOR_H

#include <Arduino.h>
#include "structs.h"
#include "settings.h"
#include "sensors.h"
#include "sdCards.h"

extern UnifiedCollectorBuffer collectorBuffer;
extern FlightData flightData;

void flushBuffer(uint8_t card = SD_CS_2);
void writeToBuffer(uint8_t sensorType, const void* data, uint8_t dataSize, uint32_t timestamp);
void updateSensors();

#endif