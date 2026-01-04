#ifndef SD_CARDS_H
#define SD_CARDS_H

#include <SD.h>

#include "config.h"

// SD Card selection enum for readable code
enum SDCard {
  SD_CARD_1,
  SD_CARD_2
};

// Global file objects for fast writing - kept open for performance
File flightDataFile1;  // SD Card 1 file handle
File flightDataFile2;  // SD Card 2 file handle
SDCard currentActiveCard = SD_CARD_2;  // Track which card is currently active
uint16_t writeCounter = 0;  // Counter for periodic flushing

// Switch to specified SD card using enum - unified function
void useCard(SDCard card) {
  SD.end();
  
  uint8_t chipSelect = (card == SD_CARD_1) ? SD_CS_1 : SD_CS_2;
  
  if (!SD.begin(chipSelect)) {
    Serial.print("Failed to switch to SD Card ");
    Serial.println(card == SD_CARD_1 ? "1" : "2");
  }
}

// Initialize specified SD card for fast writing
bool initializeSDCardForFastWrite(SDCard card) {
  Serial.print("Initializing SD Card ");
  Serial.print(card == SD_CARD_1 ? "1" : "2");
  Serial.println(" for high-speed writing...");
  
  useCard(card);
  
  File* targetFile = (card == SD_CARD_1) ? &flightDataFile1 : &flightDataFile2;
  *targetFile = SD.open(SD_FILENAME, FILE_WRITE);
  
  if (!(*targetFile)) {
    Serial.print("Error: Cannot open flight_data.bin on SD Card ");
    Serial.println(card == SD_CARD_1 ? "1" : "2");
    return false;
  }
  
  Serial.print("SD Card ");
  Serial.print(card == SD_CARD_1 ? "1" : "2");
  Serial.println(" ready for fast writing");
  
  currentActiveCard = card;
  writeCounter = 0;
  
  return true;
}

// Safely close specified SD card file
void closeSDCardFile(SDCard card) {
  File* targetFile = (card == SD_CARD_1) ? &flightDataFile1 : &flightDataFile2;
  
  if (*targetFile) {
    targetFile->flush();
    targetFile->close();
    Serial.print("SD Card ");
    Serial.print(card == SD_CARD_1 ? "1" : "2");
    Serial.println(" file closed safely");
  }
}

// Safely delete flight data file from specified SD card
bool deleteFlightDataFile(SDCard card) {
  useCard(card);
  
  if (SD.exists(SD_FILENAME)) {
    if (SD.remove(SD_FILENAME)) {
      Serial.print("Deleted flight_data.bin from SD Card ");
      Serial.println(card == SD_CARD_1 ? "1" : "2");
      return true;
    } else {
      Serial.print("Error: Failed to delete flight_data.bin from SD Card ");
      Serial.println(card == SD_CARD_1 ? "1" : "2");
      return false;
    }
  } else {
    Serial.print("flight_data.bin not found on SD Card ");
    Serial.println(card == SD_CARD_1 ? "1" : "2");
    return true;  // File doesn't exist, consider it "deleted"
  }
}

// Test specified SD card functionality
void testSDCard(SDCard card) {
  useCard(card);
  Serial.print("  -> SD Card ");
  Serial.print(card == SD_CARD_1 ? "1" : "2");
  Serial.println(" initialized.");

  if (SD.exists(SD_FILENAME)) {
    Serial.print("Card ");
    Serial.print(card == SD_CARD_1 ? "1" : "2");
    Serial.print(" - File exists. Reading ");
    Serial.println(SD_FILENAME);

    File file = SD.open(SD_FILENAME, FILE_READ);
    if (file) {
      Serial.print("Card ");
      Serial.print(card == SD_CARD_1 ? "1" : "2");
      Serial.println(" - File contents:");
      while (file.available()) {
        Serial.write(file.read());
      }
      file.close();
      Serial.print("\n--- End of Card ");
      Serial.print(card == SD_CARD_1 ? "1" : "2");
      Serial.println(" file ---");
    } else {
      Serial.print("Card ");
      Serial.print(card == SD_CARD_1 ? "1" : "2");
      Serial.println(" - Error opening file for reading.");
    }
  } else {
    Serial.print("Card ");
    Serial.print(card == SD_CARD_1 ? "1" : "2");
    Serial.print(" - File not found. Creating ");
    Serial.println(SD_FILENAME);

    File file = SD.open(SD_FILENAME, FILE_WRITE);
    if (file) {
      file.print("Hello from Card ");
      file.print(card == SD_CARD_1 ? "1" : "2");
      file.println("!");
      file.close();
      Serial.print("Card ");
      Serial.print(card == SD_CARD_1 ? "1" : "2");
      Serial.println(" - File created and written successfully!");
    } else {
      Serial.print("Card ");
      Serial.print(card == SD_CARD_1 ? "1" : "2");
      Serial.println(" - Error creating file.");
    }
  }
  Serial.println("");
}

// Write binary buffer to specified SD card - MAXIMUM SPEED (files kept open)
bool writeBufferToSD(const uint8_t* buffer, uint16_t bufferSize, SDCard card = SD_CARD_2) {
  // Get the appropriate file handle (no card switching needed!)
  File* targetFile = (card == SD_CARD_1) ? &flightDataFile1 : &flightDataFile2;
  
  // Check if file is open and valid
  if (!(*targetFile)) {
    #ifdef DEBUG_SD_VERBOSE
    Serial.print("Error: Flight data file not open on SD Card");
    #endif
    return false;
  }
  
  // Switch to correct card only if different from current
  if (currentActiveCard != card) {
    useCard(card);
    currentActiveCard = card;
  }
  
  // Ultra-fast binary write - no open/close overhead!
  uint8_t header[2];
  memcpy(header, &bufferSize, sizeof(uint16_t));
  
  // Direct write to open file handle
  size_t headerWritten = targetFile->write(header, sizeof(uint16_t));
  size_t dataWritten = targetFile->write(buffer, bufferSize);
  
  // Periodic flush for safety (every 10 writes or adjust as needed)
  writeCounter++;
  if (writeCounter >= 10) {
    targetFile->flush();  // Ensure data is written to SD card
    writeCounter = 0;
  }
  
  #ifdef DEBUG_SD_VERBOSE
  if (headerWritten == sizeof(uint16_t) && dataWritten == bufferSize) {
    Serial.print("SD");
    Serial.print(card == SD_CARD_1 ? "1" : "2");
    Serial.print(": ");
    Serial.print(bufferSize);
    Serial.println("B");
  }
  #endif
  
  return (headerWritten == sizeof(uint16_t) && dataWritten == bufferSize);
}

// Read and decode buffer data from specified SD card to Serial - CHUNKED for large files
void readBufferFromSD(SDCard card = SD_CARD_2) {
  // Switch to specified SD card
  useCard(card);
  
  File file = SD.open(SD_FILENAME, FILE_READ);
  if (!file) {
    Serial.print("Error: Cannot open flight_data.bin for reading on SD Card ");
    Serial.println(card == SD_CARD_1 ? "1" : "2");
    return;
  }
  
  // Get total file size first
  uint32_t totalFileSize = file.size();
  Serial.print("=== Flight Data Dump from SD Card ");
  Serial.print(card == SD_CARD_1 ? "1" : "2");
  Serial.print(" (");
  Serial.print(totalFileSize);
  Serial.println(" bytes) ===");
  
  if (totalFileSize == 0) {
    Serial.println("File is empty");
    file.close();
    return;
  }
  
  uint32_t totalBytesRead = 0;
  uint16_t chunkNumber = 1;
  
  // Process file in manageable chunks to avoid memory issues
  while (file.available() && totalBytesRead < totalFileSize) {
    // Read buffer size header
    uint16_t bufferSize;
    if (file.readBytes((char*)&bufferSize, sizeof(uint16_t)) != sizeof(uint16_t)) {
      Serial.println("Error: Cannot read buffer size header");
      break;  // End of valid data or corrupted file
    }
    totalBytesRead += sizeof(uint16_t);
    
    Serial.print("Chunk #");
    Serial.print(chunkNumber++);
    Serial.print(": ");
    Serial.print(bufferSize);
    Serial.print(" bytes (");
    Serial.print(totalBytesRead);
    Serial.print("/");
    Serial.print(totalFileSize);
    Serial.println(")");
    
    // Validate buffer size to prevent memory issues
    if (bufferSize > UNIFIED_BUFFER_SIZE || bufferSize == 0) {
      Serial.print("Error: Invalid buffer size: ");
      Serial.println(bufferSize);
      break;
    }
    
    // Process this buffer chunk sensor by sensor
    uint16_t chunkBytesRead = 0;
    while (chunkBytesRead < bufferSize && file.available()) {
      // Read sensor type
      uint8_t sensorType = file.read();
      chunkBytesRead++;
      totalBytesRead++;
      
      // Read timestamp
      uint32_t timestamp;
      if (file.readBytes((char*)&timestamp, 4) != 4) {
        Serial.println("Error: Cannot read timestamp");
        break;
      }
      chunkBytesRead += 4;
      totalBytesRead += 4;
      
      // Print sensor info
      Serial.print(timestamp);
      Serial.print(",");
      
      // Decode sensor data based on type
      switch (sensorType) {
        case SENSOR_ACCEL: {
          Serial.print("ACCEL,");
          float values[3];
          if (file.readBytes((char*)values, 12) == 12) {
            Serial.print(values[0], 3); Serial.print(",");
            Serial.print(values[1], 3); Serial.print(",");
            Serial.print(values[2], 3);
            chunkBytesRead += 12;
            totalBytesRead += 12;
          } else {
            Serial.print("ERROR_READING_ACCEL");
          }
          break;
        }
        
        case SENSOR_GYRO: {
          Serial.print("GYRO,");
          float values[3];
          if (file.readBytes((char*)values, 12) == 12) {
            Serial.print(values[0], 3); Serial.print(",");
            Serial.print(values[1], 3); Serial.print(",");
            Serial.print(values[2], 3);
            chunkBytesRead += 12;
            totalBytesRead += 12;
          } else {
            Serial.print("ERROR_READING_GYRO");
          }
          break;
        }
        
        case SENSOR_MAG: {
          Serial.print("MAG,");
          float values[4];
          if (file.readBytes((char*)values, 16) == 16) {
            Serial.print(values[0], 3); Serial.print(",");
            Serial.print(values[1], 3); Serial.print(",");
            Serial.print(values[2], 3); Serial.print(",");
            Serial.print(values[3], 3);
            chunkBytesRead += 16;
            totalBytesRead += 16;
          } else {
            Serial.print("ERROR_READING_MAG");
          }
          break;
        }
        
        case SENSOR_BARO: {
          Serial.print("BARO,");
          float values[3];
          if (file.readBytes((char*)values, 12) == 12) {
            Serial.print(values[0], 3); Serial.print(",");  // temperature
            Serial.print(values[1], 3); Serial.print(",");  // pressure
            Serial.print(values[2], 3);                     // altitude
            chunkBytesRead += 12;
            totalBytesRead += 12;
          } else {
            Serial.print("ERROR_READING_BARO");
          }
          break;
        }
        
        case SENSOR_GPS: {
          Serial.print("GPS,");
          struct {
            double lat, lng;
            float hdop, speed, course;
            uint16_t sats;
          } gpsData;
          
          if (file.readBytes((char*)&gpsData, 26) == 26) {
            Serial.print(gpsData.lat, 6); Serial.print(",");
            Serial.print(gpsData.lng, 6); Serial.print(",");
            Serial.print(gpsData.hdop, 2); Serial.print(",");
            Serial.print(gpsData.sats); Serial.print(",");
            Serial.print(gpsData.speed, 2); Serial.print(",");
            Serial.print(gpsData.course, 2);
            chunkBytesRead += 26;
            totalBytesRead += 26;
          } else {
            Serial.print("ERROR_READING_GPS");
          }
          break;
        }
        
        default:
          Serial.print("UNKNOWN_SENSOR_TYPE_");
          Serial.print(sensorType);
          // Skip to next sensor - this is dangerous without knowing size
          break;
      }
      Serial.println();
      
      // Safety check to prevent infinite loops
      if (totalBytesRead >= totalFileSize) {
        break;
      }
    }
    
    // Progress update for large files
    if (totalFileSize > 10000) {  // Show progress for files > 10KB
      Serial.print("Progress: ");
      Serial.print((totalBytesRead * 100) / totalFileSize);
      Serial.println("%");
    }
  }
  
  file.close();
  Serial.print("=== End of Flight Data (");
  Serial.print(totalBytesRead);
  Serial.print(" bytes processed) ===");
}

#endif