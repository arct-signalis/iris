#ifndef SETTINGS_H
#define SETTINGS_H

// ========== PHYSICAL CONSTANTS ==========
#define EARTH_GRAVITY_MS2 9.8          // Earth's gravity acceleration (m/s²)
#define STANDARD_SEA_LEVEL_PRESSURE_HPA 1013.25  // Standard atmospheric pressure at sea level (hPa)

// ========== ARRAY INDICES ==========
#define AXIS_X 0                       // X-axis array index
#define AXIS_Y 1                       // Y-axis array index
#define AXIS_Z 2                       // Z-axis array index

// ========== CALIBRATION PARAMETERS ==========
#define CALIBRATION_SAMPLES 1000       // Number of samples for calibration
#define CALIBRATION_DELAY_MS 10        // Delay between calibration samples (ms)
#define CALIBRATION_PROGRESS_INTERVAL 100  // Progress indicator interval

// ========== MAGNETOMETER CALIBRATION ==========
#define MAG_CALIBRATION_DURATION_MS 30000    // Magnetometer calibration duration (30 seconds)
#define MAG_CALIBRATION_PROGRESS_MS 3000     // Progress indicator every 3 seconds

// ========== COMMUNICATION SETTINGS ==========
#define SERIAL_BAUD_RATE 115200        // Serial monitor baud rate
#define GPS_BAUD_RATE 9600             // GPS module baud rate
#define GPS_UART_PORT 2                // GPS UART port number (UART2)

// ========== SYSTEM STATUS ==========
#define STATUS_READY 1                 // System ready status
#define STATUS_ERROR 0                 // System error status

// ========== HARDWARE ADDRESSES ==========
#define MPU9250_ADDRESS 0x68           // MPU9250 I2C address
#define IRIS_BMP280_ADDRESS 0x76       // BMP280 I2C address (SDO to GND)

// ========== PIN ASSIGNMENTS ==========
#define I2C_SDA_PIN 21                 // I2C SDA pin
#define I2C_SCL_PIN 22                 // I2C SCL pin
#define GPS_RX_PIN 16                  // GPS RX pin (ESP32 TX)
#define GPS_TX_PIN 17                  // GPS TX pin (ESP32 RX)
#define GPS_PPS_PIN 2                  // GPS PPS (Pulse Per Second) pin

// ========== TIMING PARAMETERS ==========
#define TELEMETRY_LOOP_DELAY_MS 100    // Main loop delay for telemetry output (ms)

// ========== CONDITIONAL COMPILATION ==========
// #define CALIBRATE_MAGNETOMETER       // Uncomment to enable magnetometer calibration
#define ENABLE_SENSOR_CALIBRATION    // Comment this line to disable all calibration

#endif