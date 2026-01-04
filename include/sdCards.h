#ifndef SD_CARDS_H
#define SD_CARDS_H

#include <Arduino.h>
#include <SD.h>

#include "config.h"
#include "structs.h"

bool initSDCard(uint8_t chipSelect);
bool deleteFlightDataFile(uint8_t chipSelect);
void testSDCard(uint8_t card);
bool writeBufferToSD(const uint8_t *buffer, uint16_t bufferSize, uint8_t chipSelect);
void readBufferFromSD(uint8_t chipSelect);

#endif