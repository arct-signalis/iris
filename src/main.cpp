// IRIS Flight Controller - Sensor Telemetry System

#include <Arduino.h>
#include <Wire.h>
#include <MPU9250.h>
#include <Adafruit_BMP280.h>
#include <TinyGPSPlus.h>
#include <HardwareSerial.h>

// Include modular headers
#include "settings.h"
#include "structs.h"
#include "sensors.h"
#include "calibration.h"
#include "telemetry.h"

// Hardware instances
MPU9250 mpu;
Adafruit_BMP280 bmp;
TinyGPSPlus gps;
HardwareSerial gpsSerial(GPS_UART_PORT);

// Global variables
FlightData telemetryData;

void setup() {
  // Initialize serial communication
  setupSerial();
  
  // Initialize sensors with error handling
  if (!initializeSensors()) {
    Serial.println("Critical sensor initialization failure!");
    while(1);
  }
  
#ifdef ENABLE_SENSOR_CALIBRATION
  // Calibrate all sensors
  calibrateSensors();
#else
  Serial.println("Sensor calibration disabled - using raw values");
#endif
  
  // System ready message
  printSystemReady();
}

void loop() {
  static uint16_t packetCounter = 0;
  
  // Process incoming GPS data
  processGPSData();
  
  // Update sensor readings and send telemetry
  if (mpu.update()) {
    telemetryData = collectTelemetryData();
    telemetryData.packetId = packetCounter++;
    Serial.println(formatTelemetry(telemetryData));
  }
  
  delay(TELEMETRY_LOOP_DELAY_MS);
}