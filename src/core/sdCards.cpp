#include "sdCards.h"

const char *getCardNumber(uint8_t chipSelect)  {
  return (chipSelect == SD_CS_1) ? "1" : "2";
} 

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

void readFile(const char *filename, uint8_t chipSelect)
{
  if (!initSDCard(chipSelect))
    return;

  File file = SD.open(filename, FILE_READ);
  if (!file)
    return;

  while (file.available())
    Serial.write(file.read());

  Serial.println("--- End of File ---");
  file.close();
}

bool deleteFile(const char *filename, uint8_t chipSelect)
{
  if (!initSDCard(chipSelect))
    return false;

  if (!SD.exists(filename))
    return true; // File doesn't exist, consider it deleted

  return SD.remove(filename);
}

void initCSVFile(uint8_t chipSelect)
{
#ifdef CSV_FORMAT_ENABLED
  if (!initSDCard(chipSelect))
    return;

  File csvFile = SD.open(SD_FILENAME, FILE_WRITE);
  if (!csvFile)
    return;

  csvFile.println("millis_ms_uint32,flightState_enum_int,accelX_g_float,accelY_g_float,accelZ_g_float,totalAccel_g_float,gyroX_degs_float,gyroY_degs_float,gyroZ_degs_float,angularRate_degs_float,magX_uT_float,magY_uT_float,magZ_uT_float,magFieldStrength_uT_float,pitch_deg_float,roll_deg_float,yaw_deg_float,offVert_deg_float,temperature_C_float,pressure_kPa_float,altitudeAboveLaunchPad_m_float,airDensity_kgm3_float,latitude_deg_float,longitude_deg_float,hdop_dimensionless_float,speed_kmh_float,course_deg_float,satellites_count_uint8");
  
  csvFile.close();
#endif
}

bool writeBufferToSD(const uint8_t *buffer, uint16_t bufferSize, uint8_t chipSelect)
{
#ifdef CSV_FORMAT_ENABLED
  return writeBufferToCSV(buffer, bufferSize, chipSelect);
#else
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
#endif
}

bool readSensorDataBufferFromSD(File &file, uint8_t sensorType, uint16_t &chunkBytesRead, uint32_t &totalBytesRead)
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
      Serial.print(data.latitude, 3);
      Serial.print(",");
      Serial.print(data.longitude, 3);
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

bool processBufferChunkFromSD(File &file, uint16_t bufferSize, uint32_t &totalBytesRead, uint16_t chunkNumber)
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

    if (!readSensorDataBufferFromSD(file, sensorType, chunkBytesRead, totalBytesRead))
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

    if (!processBufferChunkFromSD(file, bufferSize, totalBytesRead, chunkNumber++))
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

void writeCSVRow(File &csvFile, uint32_t timestamp, const CSVRow &row)
{
  csvFile.print(timestamp);
  csvFile.print(",0,");

  if (row.hasAccel)
  {
    csvFile.print(row.accel.x, 3);
    csvFile.print(",");
    csvFile.print(row.accel.y, 3);
    csvFile.print(",");
    csvFile.print(row.accel.z, 3);
    csvFile.print(",");
    csvFile.print(row.accel.totalAcceleration, 3);
  }
  else
    csvFile.print(",,,");

  csvFile.print(",");

  if (row.hasGyro)
  {
    csvFile.print(row.gyro.x, 3);
    csvFile.print(",");
    csvFile.print(row.gyro.y, 3);
    csvFile.print(",");
    csvFile.print(row.gyro.z, 3);
    csvFile.print(",");
    csvFile.print(row.gyro.angularRate, 3);
  }
  else
    csvFile.print(",,,");

  csvFile.print(",");

  if (row.hasMag)
  {
    csvFile.print(row.mag.x, 3);
    csvFile.print(",");
    csvFile.print(row.mag.y, 3);
    csvFile.print(",");
    csvFile.print(row.mag.z, 3);
    csvFile.print(",");
    csvFile.print(row.mag.magneticFieldStrength, 3);
  }
  else
    csvFile.print(",,,");

  csvFile.print(",");

  if (row.hasAttitude)
  {
    csvFile.print(row.attitude.pitch, 3);
    csvFile.print(",");
    csvFile.print(row.attitude.roll, 3);
    csvFile.print(",");
    csvFile.print(row.attitude.yaw, 3);
    csvFile.print(",");
    csvFile.print(row.attitude.offVert, 3);
  }
  else
    csvFile.print(",,,");

  csvFile.print(",");

  if (row.hasBaro)
  {
    csvFile.print(row.env.temperature, 3);
    csvFile.print(",");
    csvFile.print(row.env.pressure, 3);
    csvFile.print(",");
    csvFile.print(row.env.altitudeAboveLaunchPad, 3);
    csvFile.print(",");
    csvFile.print(row.env.airDensity, 3);
  }
  else
    csvFile.print(",,,");

  csvFile.print(",");

  if (row.hasGPS)
  {
    csvFile.print(row.gps.latitude, 3);
    csvFile.print(",");
    csvFile.print(row.gps.longitude, 3);
    csvFile.print(",");
    csvFile.print(row.gps.hdop, 3);
    csvFile.print(",");
    csvFile.print(row.gps.speed, 3);
    csvFile.print(",");
    csvFile.print(row.gps.course, 3);
    csvFile.print(",");
    csvFile.print(row.gps.satellites);
  }
  else
    csvFile.print(",,,,,");

  csvFile.println();
}

bool writeBufferToCSV(const uint8_t *buffer, uint16_t bufferSize, uint8_t chipSelect)
{
  if (!initSDCard(chipSelect))
    return false;

  File csvFile = SD.open(SD_FILENAME, FILE_WRITE);
  if (!csvFile)
    return false;

  // Single row to hold current data being assembled
  CSVRow currentRow;
  uint32_t currentTimestamp = 0;
  bool hasCurrentRow = false;

  // Parse buffer sequentially
  uint16_t bufferPos = 0;
  while (bufferPos < bufferSize)
  {
    uint8_t sensorType = buffer[bufferPos++];

    uint32_t timestamp;
    memcpy(&timestamp, &buffer[bufferPos], sizeof(timestamp));
    bufferPos += sizeof(timestamp);

    // If timestamp changed, write current row and start new one
    if (hasCurrentRow && timestamp != currentTimestamp)
    {
      writeCSVRow(csvFile, currentTimestamp, currentRow);
      currentRow = CSVRow(); // Reset row
    }

    currentTimestamp = timestamp;
    hasCurrentRow = true;

    // Add sensor data to current row
    switch (sensorType)
    {
    case SENSOR_ACCEL:
      memcpy(&currentRow.accel, &buffer[bufferPos], sizeof(AccelerometerData));
      currentRow.hasAccel = true;
      bufferPos += sizeof(AccelerometerData);
      break;

    case SENSOR_GYRO:
      memcpy(&currentRow.gyro, &buffer[bufferPos], sizeof(GyroscopeData));
      currentRow.hasGyro = true;
      bufferPos += sizeof(GyroscopeData);
      break;

    case SENSOR_MAG:
      memcpy(&currentRow.mag, &buffer[bufferPos], sizeof(MagnetometerData));
      currentRow.hasMag = true;
      bufferPos += sizeof(MagnetometerData);
      break;

    case SENSOR_ATTITUDE:
      memcpy(&currentRow.attitude, &buffer[bufferPos], sizeof(AttitudeData));
      currentRow.hasAttitude = true;
      bufferPos += sizeof(AttitudeData);
      break;

    case SENSOR_BARO:
      memcpy(&currentRow.env, &buffer[bufferPos], sizeof(EnvironmentalData));
      currentRow.hasBaro = true;
      bufferPos += sizeof(EnvironmentalData);
      break;

    case SENSOR_GPS:
      memcpy(&currentRow.gps, &buffer[bufferPos], sizeof(GPSData));
      currentRow.hasGPS = true;
      bufferPos += sizeof(GPSData);
      break;

    default:
      csvFile.close();
      return false;
    }
  }

  // Write final row if exists
  if (hasCurrentRow)
  {
    writeCSVRow(csvFile, currentTimestamp, currentRow);
  }

  csvFile.close();
  return true;
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