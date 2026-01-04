#ifndef SENSORS_H
#define SENSORS_H

#include <Arduino.h>
#include <Arduino_BMI270_BMM150.h>
#include <Arduino_LPS22HB.h>
#include <TinyGPSPlus.h>

#include "structs.h"
#include "config.h"
#include "calibration.h"

extern SensorCalibration sensorCalibration;

bool readAccelerometer(AccelerometerData *data);
bool readGyroscope(GyroscopeData *data);
bool readMagnetometer(MagnetometerData *data);
void calculateAttitude(AttitudeData *attitude, AccelerometerData accel, MagnetometerData mag);
bool readEnvironmental(EnvironmentalData *data);
bool readGPS(GPSData *data);
void processGPSData();

bool initializeSensors();

#endif