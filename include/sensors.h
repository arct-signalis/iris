#ifndef SENSORS_H
#define SENSORS_H

#include <Arduino.h>
#include <Arduino_BMI270_BMM150.h>  
#include <Arduino_LPS22HB.h>        
#include <TinyGPSPlus.h>

#include "structs.h"
#include "settings.h"

bool readAccelerometer(AccelerometerData& data);
bool readGyroscope(GyroscopeData& data);
bool readMagnetometer(MagnetometerData& data);
bool readEnvironmental(EnvironmentalData& data);
bool readGPS(GPSData& data);
void processGPSData();

bool initializeSensors();

// AttitudeData calculateAttitude(const AccelerometerData& accel, const MagnetometerData& mag);

#endif