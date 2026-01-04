#include "states.h"
#include "config.h"
#include "sensors.h"
#include "unifiedCollector.h"
#include "telemetry.h"
#include "sdCards.h"
#include "utils.h"

void changeState(FlightState newState)
{
  Serial.print("STATE: ");
  switch (newState)
  {
  case DEBUG_STATE:
    Serial.println("Debug");
    break;
  case SENSORS_CALIBRATING:
    Serial.println("Sensors Calibrating");
    break;
  case GROUND_IDLE:
    Serial.println("Ground Idle");
    break;
  case ASCENDING_ENGINE:
    Serial.println("Ascending (Engine)");
    break;
  case ASCENDING_NO_ENGINE:
    Serial.println("Ascending (No Engine)");
    break;
  case DESCENDING:
    Serial.println("Descending");
    break;
  case LANDED:
    Serial.println("Landed");
    break;
  }
  stateData.currentState = newState;
}

void collectTelemetry()
{
  updateSensors();
  if (collectorBuffer.bufferIndex > UNIFIED_BUFFER_FLUSH_THRESHOLD)
    flushBuffer();
}

void debugLoop(TelemetryType type)
{
  // Collect telemetry for 1 minute and print current state every 5 seconds
  collectTelemetry();
  if (GET_TIME_MS() - stateData.lastEchoTime >= SECONDS_TO_MILLIS(5))
  {
    Serial.println(formatTestTelemetry(&flightData, type));
    stateData.lastEchoTime = GET_TIME_MS();
  }
  if (GET_TIME_MS() - stateData.startTime >= SECONDS_TO_MILLIS(60))
  {
    Serial.println("1 minute telemetry collection complete");
    changeState(LANDED);
  }
}

void groundIdleLoop()
{
  // Wait for launch detection (acceleration > threshold)
  if (readAccelerometer(&flightData.accel))
  {
    float totalAccel = VECTOR_LENGTH(flightData.accel.x, flightData.accel.y, flightData.accel.z);
    if (totalAccel > LAUNCH_THRESHOLD)
    {
      if (stateData.launchDetectionStart == 0)
      {
        // Start the 0.1 second timer
        stateData.launchDetectionStart = GET_TIME_MS();
      }
      else
      {
        // Check if 0.1 seconds have passed
        if (GET_TIME_MS() - stateData.launchDetectionStart >= SECONDS_TO_MILLIS(0.1))
        {
          Serial.println("Launch detected!");
          changeState(ASCENDING_ENGINE);
        }
      }
    }
    else
    {
      stateData.launchDetectionStart = 0;
    }
  }
}

void ascendingEngineLoop()
{
  collectTelemetry();

  // Check for engine cutoff
  float totalAccel = VECTOR_LENGTH(flightData.accel.x, flightData.accel.y, flightData.accel.z);

  if (totalAccel <= ENGINE_CUTOFF_THRESHOLD)
  {
    if (stateData.engineCutoffStart == 0)
    {
      // Start the 0.1 second timer
      stateData.engineCutoffStart = GET_TIME_MS();
    }
    else
    {
      // Check if 0.1 seconds have passed
      if (GET_TIME_MS() - stateData.engineCutoffStart >= SECONDS_TO_MILLIS(0.1))
      {
        Serial.println("Engine cutoff detected!");
        changeState(ASCENDING_NO_ENGINE);
      }
    }
  }
  else
  {
    // Reset timer if acceleration goes above threshold
    stateData.engineCutoffStart = 0;
  }
}

void ascendingNoEngineLoop()
{
  collectTelemetry();

  // Check for apogee every 100ms (faster detection for brief apogee)
  if (GET_TIME_MS() - stateData.lastAltitudeTime >= SECONDS_TO_MILLIS(0.1))
  {
    float currentAltitude = flightData.env.altitudeAboveLaunchPad;

    // Improved apogee detection - require consecutive descending readings
    if (currentAltitude < stateData.previousAltitude)
    {
      stateData.descendingCount++;
      if (stateData.descendingCount >= APOGEE_DESCENT_COUNT)
      {
        Serial.println("Apogee reached - descending!");
        changeState(DESCENDING);
        return;
      }
    }
    else
    {
      stateData.descendingCount = 0; // Reset counter if altitude increases
    }

    stateData.previousAltitude = currentAltitude;
    stateData.lastAltitudeTime = GET_TIME_MS();
  }
}

void descendingLoop()
{
  collectTelemetry();

  if (flightData.env.altitudeAboveLaunchPad < LANDING_ALTITUDE_THRESHOLD)
  {
    if (stateData.landingConfirmStart == 0)
    {
      // Start landing confirmation timer
      stateData.landingConfirmStart = millis();
      stateData.landingAltitude = flightData.env.altitudeAboveLaunchPad;
      Serial.println("Potential landing detected - confirming...");
    }
    else
    {
      // Check if confirmation period has passed
      if (millis() - stateData.landingConfirmStart >= LANDING_CONFIRM_TIME)
      {
        Serial.println("Landing confirmed!");
        Serial.print("Final AGL: ");
        Serial.print(flightData.env.altitudeAboveLaunchPad);
        Serial.println("m");
        changeState(LANDED);
      }
    }
  }
  else
  {
    // Reset confirmation if altitude goes above threshold
    stateData.landingConfirmStart = 0;
  }
}

void landedLoop()
{
  readBufferFromSD(SD_CS_2); // Use default SD card 2
  while (1) {}
}