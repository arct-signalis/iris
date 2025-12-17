#ifndef CALIBRATION_H
#define CALIBRATION_H

#include <Arduino.h>
#include "settings.h"
#include <MPU9250.h>

// External references
extern MPU9250 mpu;

// ========== MPU9250 NATIVE CALIBRATION FUNCTIONS ==========

void printCalibrationResults() {
  Serial.println("========== CALIBRATION RESULTS ==========");
  
  // Print accelerometer bias (convert from internal units to m/s²)
  Serial.println("Accel bias [m/s²]: ");
  Serial.print("  X: "); Serial.print(mpu.getAccBiasX() * 1000.0f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY * 9.80665f, 3);
  Serial.print("  Y: "); Serial.print(mpu.getAccBiasY() * 1000.0f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY * 9.80665f, 3);
  Serial.print("  Z: "); Serial.println(mpu.getAccBiasZ() * 1000.0f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY * 9.80665f, 3);
  
  // Print gyroscope bias (convert from internal units to °/s)
  Serial.println("Gyro bias [°/s]: ");
  Serial.print("  X: "); Serial.print(mpu.getGyroBiasX() / (float)MPU9250::CALIB_GYRO_SENSITIVITY, 3);
  Serial.print("  Y: "); Serial.print(mpu.getGyroBiasY() / (float)MPU9250::CALIB_GYRO_SENSITIVITY, 3);
  Serial.print("  Z: "); Serial.println(mpu.getGyroBiasZ() / (float)MPU9250::CALIB_GYRO_SENSITIVITY, 3);
  
#ifdef CALIBRATE_MAGNETOMETER
  // Print magnetometer bias and scale
  Serial.println("Mag bias [mG]: ");
  Serial.print("  X: "); Serial.print(mpu.getMagBiasX(), 1);
  Serial.print("  Y: "); Serial.print(mpu.getMagBiasY(), 1);
  Serial.print("  Z: "); Serial.println(mpu.getMagBiasZ(), 1);
  
  Serial.println("Mag scale: ");
  Serial.print("  X: "); Serial.print(mpu.getMagScaleX(), 3);
  Serial.print("  Y: "); Serial.print(mpu.getMagScaleY(), 3);
  Serial.print("  Z: "); Serial.println(mpu.getMagScaleZ(), 3);
#endif
  
  Serial.println("=========================================");
}

void calibrateSensors() {
  Serial.println("Starting MPU9250 native calibration...");
  
  // Enable verbose output for calibration feedback
  mpu.verbose(true);
  
  // Calibrate accelerometer and gyroscope using MPU9250 built-in functions
  Serial.println("Accel/Gyro calibration will start in 3 seconds...");
  Serial.println("Please leave the device still on a flat surface!");
  delay(3000);
  
  mpu.calibrateAccelGyro();
  
#ifdef CALIBRATE_MAGNETOMETER
  // Calibrate magnetometer using MPU9250 built-in function
  Serial.println("Magnetometer calibration will start in 3 seconds...");
  Serial.println("Please wave device in a figure-eight pattern until done!");
  delay(3000);
  
  mpu.calibrateMag();
#else
  Serial.println("Magnetometer calibration DISABLED");
#endif
  
  // Print calibration results
  printCalibrationResults();
  
  // Disable verbose output
  mpu.verbose(false);
  
  Serial.println("MPU9250 calibration complete!");
}

#endif
