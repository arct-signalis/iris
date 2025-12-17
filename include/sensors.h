#ifndef SENSORS_H
#define SENSORS_H

#include <Arduino.h>
#include <Wire.h>
#include <MPU9250.h>
#include <Adafruit_BMP280.h>
#include <TinyGPSPlus.h>
#include <HardwareSerial.h>
#include "structs.h"
#include "settings.h"

// External references
extern MPU9250 mpu;
extern Adafruit_BMP280 bmp;
extern TinyGPSPlus gps;
extern HardwareSerial gpsSerial;

// ========== SENSOR READING FUNCTIONS ==========

AccelerometerData readAccelerometer() {
  AccelerometerData data;
  
  // MPU9250 library handles calibration internally
  data.x = mpu.getAccX() * EARTH_GRAVITY_MS2;
  data.y = mpu.getAccY() * EARTH_GRAVITY_MS2;
  data.z = mpu.getAccZ() * EARTH_GRAVITY_MS2;
  
  return data;
}

GyroscopeData readGyroscope() {
  GyroscopeData data;
  
  // MPU9250 library handles calibration internally
  data.x = mpu.getGyroX();
  data.y = mpu.getGyroY();
  data.z = mpu.getGyroZ();
  
  return data;
}

MagnetometerData readMagnetometer() {
  MagnetometerData data;

  mpu.update();
  
  data.x = mpu.getMagX();
  data.y = mpu.getMagY();
  data.z = mpu.getMagZ();
  
  // Use MPU9250's built-in YAW calculation - it's already a tilt-compensated compass!
  // The library calculates proper heading from quaternions and magnetic declination
  data.heading = mpu.getYaw();
  if (data.heading < 0) 
    data.heading += 360.0; // Normalize to 0-360°
  
  return data;
}

EnvironmentalData readEnvironmental() {
  EnvironmentalData data;
  
  data.temperature = bmp.readTemperature();
  data.pressure = bmp.readPressure();
  data.altitude = bmp.readAltitude(STANDARD_SEA_LEVEL_PRESSURE_HPA); 
  
  if (isnan(data.temperature)) data.temperature = 0.0;
  if (isnan(data.pressure)) data.pressure = 0.0;
  if (isnan(data.altitude)) data.altitude = 0.0;
  
  return data;
}

GPSData readGPS() {
  GPSData data;
  data.latitude = gps.location.isValid() ? gps.location.lat() : 0.0;
  data.longitude = gps.location.isValid() ? gps.location.lng() : 0.0;
  data.hdop = gps.hdop.isValid() ? gps.hdop.hdop() : 0.0;
  data.satellites = gps.satellites.isValid() ? gps.satellites.value() : 0;
  data.speed = gps.speed.isValid() ? gps.speed.kmph() : 0.0;
  data.course = gps.course.isValid() ? gps.course.deg() : 0.0;
  return data;
}

// ========== SENSOR INITIALIZATION ==========

bool initializeSensors() {
  Serial.println("Initializing sensors...");
  
  // Setup GPS PPS pin
  pinMode(GPS_PPS_PIN, INPUT);
  
  if (!mpu.setup(MPU9250_ADDRESS)) {
    Serial.println("MPU9250 connection failed!");
    return false;
  }
  Serial.println("MPU9250 initialized");
  
  if (!bmp.begin(IRIS_BMP280_ADDRESS)) {
    Serial.println("BMP280 connection failed - continuing without environmental data");
  } else {
    Serial.println("BMP280 initialized");
  }
  
  return true;
}

void setupSerial() {
  Serial.begin(SERIAL_BAUD_RATE);
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  gpsSerial.begin(GPS_BAUD_RATE, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
}

// =============== SENSOR LOOP ===============

void processGPSData() {
  while (gpsSerial.available() > 0) {
    if (gps.encode(gpsSerial.read())) {
      // GPS data successfully parsed - ready for use in telemetry
    }
  }
}

FlightData collectTelemetryData() {
  FlightData data;
  data.timestamp = millis();
  data.packetId = 0;
  data.accel = readAccelerometer();
  data.gyro = readGyroscope();
  data.mag = readMagnetometer();
  data.env = readEnvironmental();
  data.gps = readGPS();
  return data;
}

#endif