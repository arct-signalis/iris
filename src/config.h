#ifndef CONFIG_H
#define CONFIG_H

// Debug Settings
#define DEBUG_STATE_ENABLED // Uncomment to enable debug mode
// #define DEBUG_SD_VERBOSE                   // Uncomment to enable verbose SD card write debugging

// Calibration Settings
// #define GPS_WAIT_FOR_SATELLITES            // Uncomment to enable waiting for GPS lock before proceeding
// #define CALIBRATION_ENABLED                // Comment out to disable sensor calibration
// #define MAGNETOMETER_CALIBRATION_ENABLED   // Comment out to disable magnetometer calibration (requires device movement)
#define CALIBRATION_SAMPLES 1000            // Number of samples for calibration
#define CALIBRATION_DELAY_MS 10             // Delay between calibration samples
#define MAGNETOMETER_CALIBRATION_TIME 30000 // Magnetometer calibration time in milliseconds
#define MAGNETOMETER_LED_GLOW               // Turn on built-in LED during magnetometer calibration

// States detection thresholds
#define LAUNCH_THRESHOLD 1.53f                    // Launch detection threshold in g (15m/s² / 9.8m/s²)
#define ENGINE_CUTOFF_THRESHOLD 0.204f            // Engine cutoff threshold in g (2m/s² / 9.8m/s²)
#define APOGEE_DESCENT_COUNT 5                    // Require 5 consecutive descending readings
#define LANDING_ALTITUDE_THRESHOLD 2.0f           // Landing altitude threshold (m AGL)
#define LANDING_CONFIRM_TIME SECONDS_TO_MILLIS(3) // Confirm landing for 3 seconds

// Communication Settings
#define SERIAL_BAUD_RATE 115200
#define GPS_BAUD_RATE 9600

// GPS
#define GPS_RX_PIN 9
#define GPS_TX_PIN 8
// #define GPS_PPS_PIN -

// SD
#define SD_CS_1 10
#define SD_CS_2 2
#define SD_FILENAME "test.txt"
#define O_READ 0x01
#define O_WRITE 0x02

// Buffer Settings
#define UNIFIED_BUFFER_SIZE 16384                                   // 16KB buffer - adjust based on available memory
#define UNIFIED_BUFFER_FLUSH_THRESHOLD (UNIFIED_BUFFER_SIZE * 0.90) // Flush when 90% full

#define SENSOR_ACCEL 0x01
#define SENSOR_GYRO 0x02
#define SENSOR_MAG 0x03
#define SENSOR_BARO 0x04
#define SENSOR_GPS 0x05
#define SENSOR_ATTITUDE 0x06

// Physical Constants
#define STANDARD_SEA_LEVEL_PRESSURE_KPA 101.325 // Standard atmospheric pressure at sea level (kPa)
#define SEA_LEVEL_ALTITUDE 44330.0              // Constant for barometric altitude calculation (m)
#define BAROMETRIC_EXPONENT 0.1903              // Exponent for barometric altitude formula
#define DECLINATION 5.54                        // Magnetic declination for your location (degrees)
#define DRY_AIR_GAS_CONSTANT 287.05f            // Specific gas constant for dry air (J/kg·K)
#define CELSIUS_TO_KELVIN 273.15f               // Convert Celsius to Kelvin
#define KPA_TO_PASCAL 100.0f                    // Convert kPa to Pascal
#endif