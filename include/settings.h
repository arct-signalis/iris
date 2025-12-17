#ifndef SETTINGS_H
#define SETTINGS_H

// ========== PHYSICAL CONSTANTS ==========
#define EARTH_GRAVITY_MS2 9.80665      // Earth's gravity acceleration (m/s²)
#define STANDARD_SEA_LEVEL_PRESSURE_KPA 101.325  // Standard atmospheric pressure at sea level (kPa)

// ========== COMMUNICATION SETTINGS ==========
#define SERIAL_BAUD_RATE 115200        // Serial monitor baud rate

// ========== SENSOR SETTINGS ==========
#define IMU_SAMPLE_RATE 104            // Hz - BMI270 sample rate
#define MAG_SAMPLE_RATE 20             // Hz - BMM150 sample rate  
#define PRESSURE_SAMPLE_RATE 75        // Hz - LPS22HB sample rate
#define HUMIDITY_SAMPLE_RATE 1         // Hz - HS300x sample rate (slow sensor)

// ========== GPS SETTINGS ==========
#define GPS_BAUD_RATE 9600             // GPS module baud rate
#define GPS_RX_PIN 2                   // GPS TX connects to Arduino pin D2
#define GPS_TX_PIN 3                   // GPS RX connects to Arduino pin D3
#define GPS_PPS_PIN 4                  // GPS PPS connects to Arduino pin D4

// ========== TIMING PARAMETERS ==========
#define TELEMETRY_LOOP_DELAY_MS 100    // Main loop delay for telemetry output (ms)
#define SENSOR_UPDATE_INTERVAL_MS 10   // How often to read fast sensors (ms)
#define SLOW_SENSOR_INTERVAL_MS 1000   // How often to read slow sensors like humidity (ms)

// ========== DATA PROCESSING ==========
#define MOVING_AVERAGE_SAMPLES 5       // Number of samples for moving average filter
#define CALIBRATION_SAMPLES 1000       // Number of samples for calibration
#define CALIBRATION_DELAY_MS 5         // Delay between calibration samples (ms)

// ========== BLUETOOTH SETTINGS ==========
#define BLE_DEVICE_NAME "IRIS2_Sensor"
#define BLE_SERVICE_UUID "12345678-1234-1234-1234-123456789abc"
#define BLE_CHARACTERISTIC_UUID "87654321-4321-4321-4321-cba987654321"

// ========== CONDITIONAL COMPILATION ==========
#define ENABLE_SENSOR_CALIBRATION      // Enable IMU calibration on startup
#define ENABLE_GPS                     // Enable GPS functionality
#define ENABLE_SERIAL_OUTPUT           // Enable serial telemetry output

// ========== SYSTEM STATUS ==========
#define STATUS_READY 1                 // System ready status
#define STATUS_ERROR 0                 // System error status

#endif