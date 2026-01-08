// IRIS Flight Controller - Sensor Telemetry System for Nano 33 BLE Sense Rev2

#include <Arduino.h>
#include <Arduino_BMI270_BMM150.h> // Built-in IMU (accel, gyro, mag)
#include <Arduino_LPS22HB.h>       // Built-in barometric pressure sensor
#include <TinyGPSPlus.h>           // External GPS module

#include "config.h"
#include "structs.h"
#include "sensors.h"
#include "sdCards.h"
#include "states.h"
#include "unifiedCollector.h"
#include "telemetry.h"
#include "utils.h"
#include "lora.h"

UnifiedCollectorBuffer collectorBuffer;
FlightData flightData;
FlightStateData stateData;

void setup()
{
  // Begin Serial
  Serial.begin(SERIAL_BAUD_RATE);
  while (!Serial)
    delay(10);
  Serial.println("Nano 33 BLE Sense Rev2");

  // Initialize LoRa module
  loraInit();

  // Initialize Sensors
  changeState(SENSORS_CALIBRATING);
  if (!initializeSensors())
  {
    Serial.println("Sensor initialization failure!");
    while (1)
      ;
  }

  deleteFile(SD_FILENAME, SD_CS_2);
#ifdef CSV_FORMAT_ENABLED
  initCSVFile(SD_CS_2);
#endif

#ifdef DEBUG_STATE_ENABLED
  Serial.println("Debug mode enabled");
  changeState(DEBUG_STATE);
#endif
  stateData.startTime = GET_TIME_MS();
}

void loop()
{
  switch (stateData.currentState)
  {
  case DEBUG_STATE:
    debugLoop(ALL);
    break;
  case GROUND_IDLE:
    groundIdleLoop();
    break;
  case ASCENDING_NO_ENGINE:
    ascendingNoEngineLoop();
    break;
  case ASCENDING_ENGINE:
    ascendingEngineLoop();
    break;
  case DESCENDING:
    descendingLoop();
    break;
  case LANDED:
    landedLoop();
    break;
  case SENSORS_CALIBRATING:
    break;
  default:
    break;
  }
}