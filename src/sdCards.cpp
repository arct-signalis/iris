#include "sdCards.h"

void useCard(uint8_t chipSelect) {
  SD.end();
  if (!SD.begin(chipSelect)) {
    Serial.print("Failed to switch to SD Card ");
    Serial.println(chipSelect == SD_CS_1 ? "1" : "2");
  }
}

bool deleteFlightDataFile(uint8_t chipSelect) {
  SD.end();
  if (!SD.begin(chipSelect)) {
    Serial.print("Failed to initialize SD Card ");
    Serial.println(chipSelect == SD_CS_1 ? "1" : "2");
    return false;
  }
  
  if (SD.exists(SD_FILENAME)) {
    if (SD.remove(SD_FILENAME)) {
      Serial.print("Deleted ");
      Serial.print(SD_FILENAME);
      Serial.print(" from SD Card ");
      Serial.println(chipSelect == SD_CS_1 ? "1" : "2");
      return true;
    } else {
      Serial.print("Error: Failed to delete ");
      Serial.print(SD_FILENAME);
      Serial.print(" from SD Card ");
      Serial.println(chipSelect == SD_CS_1 ? "1" : "2");
      return false;
    }
  } else {
    Serial.print(SD_FILENAME);
    Serial.print(" not found on SD Card ");
    Serial.println(chipSelect == SD_CS_1 ? "1" : "2");
    return true;
  }
}

void testSDCard(uint8_t card) {
  useCard(card);
  Serial.print("  -> SD Card ");
  Serial.println(" initialized.");

  if (SD.exists(SD_FILENAME)) {
    Serial.print("Card ");
    Serial.print(" - File exists. Reading ");
    Serial.println(SD_FILENAME);

    File file = SD.open(SD_FILENAME, FILE_READ);
    if (file) {
      Serial.print("Card ");
      Serial.println(" - File contents:");
      while (file.available()) {
        Serial.write(file.read());
      }
      file.close();
      Serial.print("\n--- End of Card ");
      Serial.println(" file ---");
    } else {
      Serial.print("Card ");
      Serial.println(" - Error opening file for reading.");
    }
  } else {
    Serial.print("Card ");
    Serial.print(" - File not found. Creating ");
    Serial.println(SD_FILENAME);

    File file = SD.open(SD_FILENAME, FILE_WRITE);
    if (file) {
      file.print("Hello from Card ");
      file.print(card == SD_CS_1 ? "1" : "2");
      file.println("!");
      file.close();
      Serial.print("Card ");
      Serial.println(" - File created and written successfully!");
    } else {
      Serial.print("Card ");
      Serial.println(" - Error creating file.");
    }
  }
  Serial.println("");
}

bool writeBufferToSD(const uint8_t* buffer, uint16_t bufferSize, uint8_t chipSelect) {
  // Switch to correct SD card
  SD.end();
  if (!SD.begin(chipSelect)) {
    Serial.print("Failed to initialize SD Card ");
    Serial.println(chipSelect == SD_CS_1 ? "1" : "2");
    return false;
  }
  
  // Open file in append mode - will create if doesn't exist
  File file = SD.open(SD_FILENAME, FILE_WRITE);
  if (!file) {
    Serial.print("Error: Cannot open ");
    Serial.print(SD_FILENAME);
    Serial.print(" on SD Card ");
    Serial.println(chipSelect == SD_CS_1 ? "1" : "2");
    return false;
  }
  
  // Write buffer size header + data
  uint8_t header[2];
  memcpy(header, &bufferSize, sizeof(uint16_t));
  
  size_t headerWritten = file.write(header, sizeof(uint16_t));
  size_t dataWritten = file.write(buffer, bufferSize);
  
  file.close();
  
  #ifdef DEBUG_SD_VERBOSE
  if (headerWritten == sizeof(uint16_t) && dataWritten == bufferSize) {
    Serial.print("SD");
    Serial.print(": ");
    Serial.print(bufferSize);
    Serial.println("B");
  }
  #endif
  
  return (headerWritten == sizeof(uint16_t) && dataWritten == bufferSize);
}

void readBufferFromSD(uint8_t chipSelect) {
  SD.end();
  if (!SD.begin(chipSelect)) {
    Serial.print("Failed to initialize SD Card ");
    Serial.println(chipSelect == SD_CS_1 ? "1" : "2");
    return;
  }
  
  File file = SD.open(SD_FILENAME, FILE_READ);
  if (!file) {
    Serial.print("Error: Cannot open ");
    Serial.print(SD_FILENAME);
    Serial.print(" for reading on SD Card ");
    Serial.println(chipSelect == SD_CS_1 ? "1" : "2");
    return;
  }
  
  // Get total file size first
  uint32_t totalFileSize = file.size();
  Serial.print("=== Flight Data Dump from SD Card ");
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
          float values[3];
          if (file.readBytes((char*)values, 12) == 12) {
            Serial.print(values[0], 3); Serial.print(",");
            Serial.print(values[1], 3); Serial.print(",");
            Serial.print(values[2], 3);
            chunkBytesRead += 12;
            totalBytesRead += 12;
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
          Serial.println(" - BUFFER CORRUPTED, STOPPING READ");
          file.close();
          return;  // Stop reading to prevent further corruption
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
