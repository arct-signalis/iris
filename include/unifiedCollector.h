#ifndef UNIFIED_COLLECTOR_H
#define UNIFIED_COLLECTOR_H

#include <Arduino.h>
#include "config.h"
#include "structs.h"

void flushBuffer(uint8_t card = SD_CS_2);
void writeToBuffer(uint8_t sensorType, const void *data, uint8_t dataSize, uint32_t timestamp);
void updateSensors();

#endif