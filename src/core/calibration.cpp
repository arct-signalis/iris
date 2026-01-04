#include "calibration.h"

#include <Arduino_BMI270_BMM150.h>
#include <Arduino_LPS22HB.h>
#include "utils.h"
#include "structs.h"

SensorCalibration sensorCalibration;

AccelerometerData applyCalibratedAccelerometer(AccelerometerData data)
{
  if (sensorCalibration.isCalibrated)
  {
    data.x = (data.x - sensorCalibration.accel.biasX) * sensorCalibration.accel.scaleX;
    data.y = (data.y - sensorCalibration.accel.biasY) * sensorCalibration.accel.scaleY;
    data.z = (data.z - sensorCalibration.accel.biasZ) * sensorCalibration.accel.scaleZ;
  }
  return data;
}

GyroscopeData applyCalibratedGyroscope(GyroscopeData data)
{
  if (sensorCalibration.isCalibrated)
  {
    data.x -= sensorCalibration.gyro.biasX;
    data.y -= sensorCalibration.gyro.biasY;
    data.z -= sensorCalibration.gyro.biasZ;
  }
  return data;
}

MagnetometerData applyCalibratedMagnetometer(MagnetometerData data)
{
  if (sensorCalibration.isCalibrated)
  {
    // Apply hard iron correction (offset)
    data.x -= sensorCalibration.mag.hardIronX;
    data.y -= sensorCalibration.mag.hardIronY;
    data.z -= sensorCalibration.mag.hardIronZ;

    // Apply soft iron correction (scale and cross-axis)
    float tempX = data.x;
    float tempY = data.y;
    float tempZ = data.z;

    data.x = sensorCalibration.mag.softIronXX * tempX +
             sensorCalibration.mag.softIronXY * tempY +
             sensorCalibration.mag.softIronXZ * tempZ;
    data.y = sensorCalibration.mag.softIronYX * tempX +
             sensorCalibration.mag.softIronYY * tempY +
             sensorCalibration.mag.softIronYZ * tempZ;
    data.z = sensorCalibration.mag.softIronZX * tempX +
             sensorCalibration.mag.softIronZY * tempY +
             sensorCalibration.mag.softIronZZ * tempZ;
  }
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

  sensorCalibration.accel.biasX = sumX / samples;
  sensorCalibration.accel.biasY = sumY / samples;
  sensorCalibration.accel.biasZ = (sumZ / samples) - 1.0;

  sensorCalibration.accel.scaleX = 1.0;
  sensorCalibration.accel.scaleY = 1.0;
  sensorCalibration.accel.scaleZ = 1.0;

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

  sensorCalibration.gyro.biasX = sumX / samples;
  sensorCalibration.gyro.biasY = sumY / samples;
  sensorCalibration.gyro.biasZ = sumZ / samples;

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
  sensorCalibration.mag.hardIronX = (maxX + minX) / 2.0;
  sensorCalibration.mag.hardIronY = (maxY + minY) / 2.0;
  sensorCalibration.mag.hardIronZ = (maxZ + minZ) / 2.0;

  // Calculate soft iron correction (scale factors)
  float avgRange = ((maxX - minX) + (maxY - minY) + (maxZ - minZ)) / 3.0;
  sensorCalibration.mag.softIronXX = avgRange / (maxX - minX);
  sensorCalibration.mag.softIronYY = avgRange / (maxY - minY);
  sensorCalibration.mag.softIronZZ = avgRange / (maxZ - minZ);

  // Set cross-axis terms to zero (simple calibration)
  sensorCalibration.mag.softIronXY = 0.0;
  sensorCalibration.mag.softIronXZ = 0.0;
  sensorCalibration.mag.softIronYX = 0.0;
  sensorCalibration.mag.softIronYZ = 0.0;
  sensorCalibration.mag.softIronZX = 0.0;
  sensorCalibration.mag.softIronZY = 0.0;

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
  Serial.println("Starting calibration...");
  calibrateAccelerometer();
  calibrateGyroscope();
  calibrateBarometer();
#ifdef MAGNETOMETER_CALIBRATION_ENABLED
  calibrateMagnetometer();
#endif

  return;
}