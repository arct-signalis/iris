#include "sdCards.h"

// Helper function to get card number string
const char *getCardNumber(uint8_t chipSelect)
{
  return (chipSelect == SD_CS_1) ? "1" : "2";
}

// Helper function to initialize SD card with error handling
bool initSDCard(uint8_t chipSelect)
{
  SD.end();
  if (!SD.begin(chipSelect))
  {
    Serial.print("Failed to initialize SD Card ");
    Serial.println(getCardNumber(chipSelect));
    return false;
  }
  return true;
}

bool deleteFlightDataFile(uint8_t chipSelect)
{
  if (!initSDCard(chipSelect))
    return false;

  if (!SD.exists(SD_FILENAME))
  {
    Serial.print(SD_FILENAME);
    Serial.print(" not found on SD Card ");
    Serial.println(getCardNumber(chipSelect));
    return true;
  }

  bool success = SD.remove(SD_FILENAME);
  Serial.print(success ? "Deleted " : "Error: Failed to delete ");
  Serial.print(SD_FILENAME);
  Serial.print(" from SD Card ");
  Serial.println(getCardNumber(chipSelect));
  return success;
}

void testSDCard(uint8_t card)
{
  if (initSDCard(card) && SD.exists(SD_FILENAME))
  {
    Serial.print("Card ");
    Serial.print(" - File exists. Reading ");
    Serial.println(SD_FILENAME);

    File file = SD.open(SD_FILENAME, FILE_READ);
    if (file)
    {
      Serial.print("Card ");
      Serial.println(" - File contents:");
      while (file.available())
      {
        Serial.write(file.read());
      }
      file.close();
      Serial.print("\n--- End of Card ");
      Serial.println(" file ---");
    }
    else
    {
      Serial.print("Card ");
      Serial.println(" - Error opening file for reading.");
    }
  }
  else
  {
    Serial.print("Card ");
    Serial.print(" - File not found. Creating ");
    Serial.println(SD_FILENAME);

    File file = SD.open(SD_FILENAME, FILE_WRITE);
    if (file)
    {
      file.print("Hello from Card ");
      file.print(card == SD_CS_1 ? "1" : "2");
      file.println("!");
      file.close();
      Serial.print("Card ");
      Serial.println(" - File created and written successfully!");
    }
    else
    {
      Serial.print("Card ");
      Serial.println(" - Error creating file.");
    }
  }
}

bool writeBufferToSD(const uint8_t *buffer, uint16_t bufferSize, uint8_t chipSelect)
{
  if (!initSDCard(chipSelect))
    return false;

  File file = SD.open(SD_FILENAME, FILE_WRITE);
  if (!file)
  {
    Serial.print("Error: Cannot open ");
    Serial.print(SD_FILENAME);
    Serial.print(" on SD Card ");
    Serial.println(getCardNumber(chipSelect));
    return false;
  }

  uint8_t header[2];
  memcpy(header, &bufferSize, sizeof(uint16_t));

  size_t headerWritten = file.write(header, sizeof(uint16_t));
  size_t dataWritten = file.write(buffer, bufferSize);
  file.close();

#ifdef DEBUG_SD_VERBOSE
  if (headerWritten == sizeof(uint16_t) && dataWritten == bufferSize)
  {
    Serial.print("SD: ");
    Serial.print(bufferSize);
    Serial.println("B");
  }
#endif

  return (headerWritten == sizeof(uint16_t) && dataWritten == bufferSize);
}

bool readSensorData(File &file, uint8_t sensorType, uint16_t &chunkBytesRead, uint32_t &totalBytesRead)
{
  switch (sensorType)
  {
  case SENSOR_ACCEL:
  {
    Serial.print("ACCEL,");
    AccelerometerData data;
    if (file.readBytes((char *)&data, sizeof(data)) == sizeof(data))
    {
      Serial.print(data.x, 3);
      Serial.print(",");
      Serial.print(data.y, 3);
      Serial.print(",");
      Serial.print(data.z, 3);
      Serial.print(",");
      Serial.print(data.totalAcceleration, 3);
      chunkBytesRead += sizeof(data);
      totalBytesRead += sizeof(data);
      return true;
    }
    Serial.print("ERROR_READING_ACCEL");
    return false;
  }

  case SENSOR_GYRO:
  {
    Serial.print("GYRO,");
    GyroscopeData data;
    if (file.readBytes((char *)&data, sizeof(data)) == sizeof(data))
    {
      Serial.print(data.x, 3);
      Serial.print(",");
      Serial.print(data.y, 3);
      Serial.print(",");
      Serial.print(data.z, 3);
      Serial.print(",");
      Serial.print(data.angularRate, 3);
      chunkBytesRead += sizeof(data);
      totalBytesRead += sizeof(data);
      return true;
    }
    Serial.print("ERROR_READING_GYRO");
    return false;
  }

  case SENSOR_MAG:
  {
    Serial.print("MAG,");
    MagnetometerData data;
    if (file.readBytes((char *)&data, sizeof(data)) == sizeof(data))
    {
      Serial.print(data.x, 3);
      Serial.print(",");
      Serial.print(data.y, 3);
      Serial.print(",");
      Serial.print(data.z, 3);
      Serial.print(",");
      Serial.print(data.magneticFieldStrength, 3);
      chunkBytesRead += sizeof(data);
      totalBytesRead += sizeof(data);
      return true;
    }
    Serial.print("ERROR_READING_MAG");
    return false;
  }

  case SENSOR_BARO:
  {
    Serial.print("BARO,");
    EnvironmentalData data;
    if (file.readBytes((char *)&data, sizeof(data)) == sizeof(data))
    {
      Serial.print(data.temperature, 3);
      Serial.print(",");
      Serial.print(data.pressure, 3);
      Serial.print(",");
      Serial.print(data.altitudeAboveLaunchPad, 3);
      Serial.print(",");
      Serial.print(data.airDensity, 3);
      chunkBytesRead += sizeof(data);
      totalBytesRead += sizeof(data);
      return true;
    }
    Serial.print("ERROR_READING_BARO");
    return false;
  }

  case SENSOR_GPS:
  {
    Serial.print("GPS,");
    GPSData data;
    if (file.readBytes((char *)&data, sizeof(data)) == sizeof(data))
    {
      Serial.print(data.latitude, 6);
      Serial.print(",");
      Serial.print(data.longitude, 6);
      Serial.print(",");
      Serial.print(data.hdop, 2);
      Serial.print(",");
      Serial.print(data.satellites);
      Serial.print(",");
      Serial.print(data.speed, 2);
      Serial.print(",");
      Serial.print(data.course, 2);
      chunkBytesRead += sizeof(data);
      totalBytesRead += sizeof(data);
      return true;
    }
    Serial.print("ERROR_READING_GPS");
    return false;
  }

  case SENSOR_ATTITUDE:
  {
    Serial.print("ATTITUDE,");
    AttitudeData data;
    if (file.readBytes((char *)&data, sizeof(data)) == sizeof(data))
    {
      Serial.print(data.roll, 3);
      Serial.print(",");
      Serial.print(data.pitch, 3);
      Serial.print(",");
      Serial.print(data.yaw, 3);
      Serial.print(",");
      Serial.print(data.offVert, 3);
      chunkBytesRead += sizeof(data);
      totalBytesRead += sizeof(data);
      return true;
    }
    Serial.print("ERROR_READING_ATTITUDE");
    return false;
  }

  default:
    Serial.print("UNKNOWN_SENSOR_TYPE_");
    Serial.print(sensorType);
    Serial.println(" - BUFFER CORRUPTED, STOPPING READ");
    return false;
  }
}

bool processDataChunk(File &file, uint16_t bufferSize, uint32_t &totalBytesRead, uint16_t chunkNumber)
{
  Serial.print("Chunk #");
  Serial.print(chunkNumber);
  Serial.print(": ");
  Serial.print(bufferSize);
  Serial.print(" bytes");
  Serial.print(" (");
  Serial.print(totalBytesRead);
  Serial.print("/");
  Serial.print(file.size());
  Serial.println(")");

  if (bufferSize > UNIFIED_BUFFER_SIZE || bufferSize == 0)
  {
    Serial.print("Error: Invalid buffer size: ");
    Serial.println(bufferSize);
    return false;
  }

  uint16_t chunkBytesRead = 0;
  while (chunkBytesRead < bufferSize && file.available())
  {
    uint8_t sensorType = file.read();
    chunkBytesRead++;
    totalBytesRead++;

    uint32_t timestamp;
    if (file.readBytes((char *)&timestamp, 4) != 4)
    {
      Serial.println("Error: Cannot read timestamp");
      return false;
    }
    chunkBytesRead += 4;
    totalBytesRead += 4;

    Serial.print(timestamp);
    Serial.print(",");

    if (!readSensorData(file, sensorType, chunkBytesRead, totalBytesRead))
      return false;
    Serial.println();

    if (totalBytesRead >= file.size())
      break;
  }
  return true;
}

void readBufferFromSD(uint8_t chipSelect)
{
  if (!initSDCard(chipSelect))
    return;

  File file = SD.open(SD_FILENAME, FILE_READ);
  if (!file)
  {
    Serial.print("Error: Cannot open ");
    Serial.print(SD_FILENAME);
    Serial.print(" for reading on SD Card ");
    Serial.println(getCardNumber(chipSelect));
    return;
  }

  uint32_t totalFileSize = file.size();
  Serial.print("=== Flight Data Dump from SD Card (");
  Serial.print(totalFileSize);
  Serial.println(" bytes) ===");

  if (totalFileSize == 0)
  {
    Serial.println("File is empty");
    file.close();
    return;
  }

  uint32_t totalBytesRead = 0;
  uint16_t chunkNumber = 1;

  while (file.available() && totalBytesRead < totalFileSize)
  {
    uint16_t bufferSize;
    if (file.readBytes((char *)&bufferSize, sizeof(uint16_t)) != sizeof(uint16_t))
    {
      Serial.println("Error: Cannot read buffer size header");
      break;
    }
    totalBytesRead += sizeof(uint16_t);

    if (!processDataChunk(file, bufferSize, totalBytesRead, chunkNumber++))
      break;

    if (totalFileSize > 10000)
    {
      Serial.print("Progress: ");
      Serial.print((totalBytesRead * 100) / totalFileSize);
      Serial.println("%");
    }
  }

  file.close();
  Serial.print("=== End of Flight Data (");
  Serial.print(totalBytesRead);
  Serial.println(" bytes processed) ===");
}
