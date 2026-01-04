#include "calibration.h"

#include <Arduino_BMI270_BMM150.h>
#include <Arduino_LPS22HB.h>
#include "utils.h"
#include "structs.h"

AccelerometerCalibration sensorAccelCalib;
GyroscopeCalibration sensorGyroCalib;
MagnetometerCalibration sensorMagCalib;

AccelerometerData applyCalibratedAccelerometer(AccelerometerData data)
{
  data.x = (data.x - sensorAccelCalib.biasX) * sensorAccelCalib.scaleX;
  data.y = (data.y - sensorAccelCalib.biasY) * sensorAccelCalib.scaleY;
  data.z = (data.z - sensorAccelCalib.biasZ) * sensorAccelCalib.scaleZ;
  return data;
}

GyroscopeData applyCalibratedGyroscope(GyroscopeData data)
{
  data.x -= sensorGyroCalib.biasX;
  data.y -= sensorGyroCalib.biasY;
  data.z -= sensorGyroCalib.biasZ;
  return data;
}

MagnetometerData applyCalibratedMagnetometer(MagnetometerData data)
{
  // Apply hard iron correction (offset)
  data.x -= sensorMagCalib.hardIronX;
  data.y -= sensorMagCalib.hardIronY;
  data.z -= sensorMagCalib.hardIronZ;

  // Apply soft iron correction (scale and cross-axis)
  float tempX = data.x;
  float tempY = data.y;
  float tempZ = data.z;

  data.x = sensorMagCalib.softIronXX * tempX +
           sensorMagCalib.softIronXY * tempY +
           sensorMagCalib.softIronXZ * tempZ;
  data.y = sensorMagCalib.softIronYX * tempX +
           sensorMagCalib.softIronYY * tempY +
           sensorMagCalib.softIronYZ * tempZ;
  data.z = sensorMagCalib.softIronZX * tempX +
           sensorMagCalib.softIronZY * tempY +
           sensorMagCalib.softIronZZ * tempZ;
  return data;
}

// ========== CALIBRATION FUNCTIONS ==========
void calibrateAccelerometer()
{
  Serial.println("Accelerometer calibration - place flat...");

  float sumX = 0, sumY = 0, sumZ = 0;
  uint16_t samples = 0;

  for (uint16_t i = 0; i < CALIBRATION_SAMPLES; i++)
  {
    AccelerometerData data;
    if (IMU.accelerationAvailable())
    {
      IMU.readAcceleration(data.x, data.y, data.z);
      sumX += data.x;
      sumY += data.y;
      sumZ += data.z;
      samples++;
    }
    delay(CALIBRATION_DELAY_MS);
  }

  if (samples < CALIBRATION_SAMPLES * 0.8)
  {
    Serial.println("Accel calibration failed");
    return;
  }

  sensorAccelCalib.biasX = sumX / samples;
  sensorAccelCalib.biasY = sumY / samples;
  sensorAccelCalib.biasZ = (sumZ / samples) - 1.0;

  sensorAccelCalib.scaleX = 1.0;
  sensorAccelCalib.scaleY = 1.0;
  sensorAccelCalib.scaleZ = 1.0;

  Serial.println("Accel calibration done");
}

void calibrateGyroscope()
{
  Serial.println("Gyroscope calibration - keep still...");

  float sumX = 0, sumY = 0, sumZ = 0;
  uint16_t samples = 0;

  for (uint16_t i = 0; i < CALIBRATION_SAMPLES; i++)
  {
    GyroscopeData data;
    if (IMU.gyroscopeAvailable())
    {
      IMU.readGyroscope(data.x, data.y, data.z);
      sumX += data.x;
      sumY += data.y;
      sumZ += data.z;
      samples++;
    }
    delay(CALIBRATION_DELAY_MS);
  }

  if (samples < CALIBRATION_SAMPLES * 0.8)
  {
    Serial.println("Gyro calibration failed");
    return;
  }

  sensorGyroCalib.biasX = sumX / samples;
  sensorGyroCalib.biasY = sumY / samples;
  sensorGyroCalib.biasZ = sumZ / samples;

  Serial.println("Gyro calibration done");
}

void calibrateMagnetometer()
{
  Serial.println("Magnetometer calibration - rotate device in all directions...");
  Serial.println("Press any key when done or wait 30 seconds");

  // Turn on LED during calibration
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  float minX = 10000, maxX = -10000;
  float minY = 10000, maxY = -10000;
  float minZ = 10000, maxZ = -10000;

  uint32_t startTime = GET_TIME_MS();

  while (GET_TIME_MS() - startTime < MAGNETOMETER_CALIBRATION_TIME && !Serial.available())
  {
    MagnetometerData data;
    if (IMU.magneticFieldAvailable())
    {
      IMU.readMagneticField(data.x, data.y, data.z);

      // Track min/max values
      if (data.x < minX)
        minX = data.x;
      if (data.x > maxX)
        maxX = data.x;
      if (data.y < minY)
        minY = data.y;
      if (data.y > maxY)
        maxY = data.y;
      if (data.z < minZ)
        minZ = data.z;
      if (data.z > maxZ)
        maxZ = data.z;
    }
    delay(10);
  }

  // Turn off LED when done
  digitalWrite(LED_BUILTIN, LOW);

  // Calculate hard iron correction (offset)
  sensorMagCalib.hardIronX = (maxX + minX) / 2.0;
  sensorMagCalib.hardIronY = (maxY + minY) / 2.0;
  sensorMagCalib.hardIronZ = (maxZ + minZ) / 2.0;

  // Calculate soft iron correction (scale factors)
  float avgRange = ((maxX - minX) + (maxY - minY) + (maxZ - minZ)) / 3.0;
  sensorMagCalib.softIronXX = avgRange / (maxX - minX);
  sensorMagCalib.softIronYY = avgRange / (maxY - minY);
  sensorMagCalib.softIronZZ = avgRange / (maxZ - minZ);

  // Set cross-axis terms to zero (simple calibration)
  sensorMagCalib.softIronXY = 0.0;
  sensorMagCalib.softIronXZ = 0.0;
  sensorMagCalib.softIronYX = 0.0;
  sensorMagCalib.softIronYZ = 0.0;
  sensorMagCalib.softIronZX = 0.0;
  sensorMagCalib.softIronZY = 0.0;

  Serial.println("Magnetometer calibration done");
}

void calibrateBarometer()
{
  Serial.println("Barometer calibration - setting launch pad baseline...");

  float sumPressure = 0;
  uint16_t samples = 0;

  for (uint16_t i = 0; i < CALIBRATION_SAMPLES; i++)
  {
    float pressure = BARO.readPressure();
    if (!isnanf(pressure))
    {
      sumPressure += pressure;
      samples++;
    }
    delay(CALIBRATION_DELAY_MS);
  }

  if (samples < CALIBRATION_SAMPLES * 0.8)
  {
    Serial.println("Baro calibration failed");
    return;
  }

  float avgPressure = sumPressure / samples;

  // Calculate sea level altitude from average pressure
  extern FlightData flightData;
  flightData.launchAltitude = SEA_LEVEL_ALTITUDE * (1.0f - powf(avgPressure / STANDARD_SEA_LEVEL_PRESSURE_KPA, BAROMETRIC_EXPONENT));

  Serial.print("Launch altitude baseline: ");
  Serial.print(flightData.launchAltitude);
  Serial.println("m above sea level");
  Serial.println("Baro calibration done");
}

void calibrateAllSensors()
{
#ifdef CALIBRATION_ENABLED
  Serial.println("Starting calibration...");
  calibrateAccelerometer();
  calibrateGyroscope();
  calibrateBarometer();
#ifdef MAGNETOMETER_CALIBRATION_ENABLED
  calibrateMagnetometer();
#endif
#else
  Serial.println("Calibration disabled");
#endif

  return;
}