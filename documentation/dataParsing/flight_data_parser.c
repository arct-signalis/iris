#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>

// Sensor type definitions
#define SENSOR_ACCEL 0x01
#define SENSOR_GYRO 0x02
#define SENSOR_MAG 0x03
#define SENSOR_BARO 0x04
#define SENSOR_GPS 0x05
#define SENSOR_ATTITUDE 0x06

#define UNIFIED_BUFFER_SIZE 16384

// Data structures
typedef struct {
    float x, y, z;
    float totalAcceleration;
} AccelerometerData;

typedef struct {
    float x, y, z;
    float angularRate;
} GyroscopeData;

typedef struct {
    float x, y, z;
    float magneticFieldStrength;
} MagnetometerData;

typedef struct {
    float temperature;            // °C
    float pressure;               // kPa
    float altitudeAboveLaunchPad; // m (relative to launch pad)
    float airDensity;             // kg/m³
} EnvironmentalData;

typedef struct {
    float latitude;     // degrees
    float longitude;    // degrees
    float hdop;         // horizontal dilution of precision
    float speed;        // km/h
    float course;       // degrees
    uint8_t satellites; // number of satellites
} GPSData;

typedef struct {
    float pitch;
    float roll;
    float yaw;     // heading
    float offVert; // degrees off vertical
} AttitudeData;

// Function to get sensor type name
const char* getSensorTypeName(uint8_t sensorType) {
    switch (sensorType) {
        case SENSOR_ACCEL: return "ACCEL";
        case SENSOR_GYRO: return "GYRO";
        case SENSOR_MAG: return "MAG";
        case SENSOR_BARO: return "BARO";
        case SENSOR_GPS: return "GPS";
        case SENSOR_ATTITUDE: return "ATTITUDE";
        default: return "UNKNOWN";
    }
}

// Function to write CSV header
void writeCSVHeader(FILE *csvFile) {
    fprintf(csvFile, "timestamp,datatype,raw_data\n");
}

// Function to read and parse sensor data
int readSensorData(FILE *file, uint8_t sensorType, uint32_t timestamp, FILE *csvFile) {
    switch (sensorType) {
        case SENSOR_ACCEL: {
            AccelerometerData data;
            if (fread(&data, sizeof(data), 1, file) != 1) {
                printf("Error reading accelerometer data\n");
                return 0;
            }
            fprintf(csvFile, "%u,ACCEL,%.3f, %.3f, %.3f, %.3f\n",
                   timestamp, data.x, data.y, data.z, data.totalAcceleration);
            return 1;
        }
        
        case SENSOR_GYRO: {
            GyroscopeData data;
            if (fread(&data, sizeof(data), 1, file) != 1) {
                printf("Error reading gyroscope data\n");
                return 0;
            }
            fprintf(csvFile, "%u,GYRO,%.3f, %.3f, %.3f, %.3f\n",
                   timestamp, data.x, data.y, data.z, data.angularRate);
            return 1;
        }
        
        case SENSOR_MAG: {
            MagnetometerData data;
            if (fread(&data, sizeof(data), 1, file) != 1) {
                printf("Error reading magnetometer data\n");
                return 0;
            }
            fprintf(csvFile, "%u,MAG,%.3f, %.3f, %.3f, %.3f\n",
                   timestamp, data.x, data.y, data.z, data.magneticFieldStrength);
            return 1;
        }
        
        case SENSOR_BARO: {
            EnvironmentalData data;
            if (fread(&data, sizeof(data), 1, file) != 1) {
                printf("Error reading environmental data\n");
                return 0;
            }
            fprintf(csvFile, "%u,BARO,%.3f, %.3f, %.3f, %.3f\n",
                   timestamp, data.temperature, data.pressure, data.altitudeAboveLaunchPad, data.airDensity);
            return 1;
        }
        
        case SENSOR_GPS: {
            GPSData data;
            if (fread(&data, sizeof(data), 1, file) != 1) {
                printf("Error reading GPS data\n");
                return 0;
            }
            fprintf(csvFile, "%u,GPS,%.6f, %.6f, %.2f, %u, %.2f, %.2f\n",
                   timestamp, data.latitude, data.longitude, data.hdop, data.satellites, data.speed, data.course);
            return 1;
        }
        
        case SENSOR_ATTITUDE: {
            AttitudeData data;
            if (fread(&data, sizeof(data), 1, file) != 1) {
                printf("Error reading attitude data\n");
                return 0;
            }
            fprintf(csvFile, "%u,ATTITUDE,%.3f, %.3f, %.3f, %.3f\n",
                   timestamp, data.roll, data.pitch, data.yaw, data.offVert);
            return 1;
        }
        
        default:
            printf("Unknown sensor type: %u\n", sensorType);
            return 0;
    }
}

// Function to process a data chunk
int processDataChunk(FILE *file, uint16_t bufferSize, long *totalBytesRead, 
                    long fileSize, uint16_t chunkNumber, FILE *csvFile) {
    printf("Processing chunk #%u: %u bytes (%ld/%ld)\n", 
           chunkNumber, bufferSize, *totalBytesRead, fileSize);
    
    if (bufferSize > UNIFIED_BUFFER_SIZE || bufferSize == 0) {
        printf("Error: Invalid buffer size: %u\n", bufferSize);
        return 0;
    }
    
    uint16_t chunkBytesRead = 0;
    while (chunkBytesRead < bufferSize && !feof(file)) {
        // Read sensor type
        uint8_t sensorType;
        if (fread(&sensorType, 1, 1, file) != 1) {
            printf("Error reading sensor type\n");
            return 0;
        }
        chunkBytesRead++;
        (*totalBytesRead)++;
        
        // Read timestamp
        uint32_t timestamp;
        if (fread(&timestamp, sizeof(uint32_t), 1, file) != 1) {
            printf("Error reading timestamp\n");
            return 0;
        }
        chunkBytesRead += sizeof(uint32_t);
        (*totalBytesRead) += sizeof(uint32_t);
        
        // Read sensor data
        if (!readSensorData(file, sensorType, timestamp, csvFile)) {
            return 0;
        }
        
        // Update bytes read based on sensor type
        switch (sensorType) {
            case SENSOR_ACCEL:
                chunkBytesRead += sizeof(AccelerometerData);
                (*totalBytesRead) += sizeof(AccelerometerData);
                break;
            case SENSOR_GYRO:
                chunkBytesRead += sizeof(GyroscopeData);
                (*totalBytesRead) += sizeof(GyroscopeData);
                break;
            case SENSOR_MAG:
                chunkBytesRead += sizeof(MagnetometerData);
                (*totalBytesRead) += sizeof(MagnetometerData);
                break;
            case SENSOR_BARO:
                chunkBytesRead += sizeof(EnvironmentalData);
                (*totalBytesRead) += sizeof(EnvironmentalData);
                break;
            case SENSOR_GPS:
                chunkBytesRead += sizeof(GPSData);
                (*totalBytesRead) += sizeof(GPSData);
                break;
            case SENSOR_ATTITUDE:
                chunkBytesRead += sizeof(AttitudeData);
                (*totalBytesRead) += sizeof(AttitudeData);
                break;
            default:
                printf("Unknown sensor type, cannot determine size\n");
                return 0;
        }
        
        if (*totalBytesRead >= fileSize) {
            break;
        }
    }
    
    return 1;
}

// Main function to parse flight data
void parseFlightData(const char *inputFile, const char *outputFile) {
    FILE *file = fopen(inputFile, "rb");
    if (!file) {
        printf("Error: Cannot open input file '%s'\n", inputFile);
        return;
    }
    
    FILE *csvFile = fopen(outputFile, "w");
    if (!csvFile) {
        printf("Error: Cannot create output file '%s'\n", outputFile);
        fclose(file);
        return;
    }
    
    // Write CSV header
    writeCSVHeader(csvFile);
    
    // Get file size
    fseek(file, 0, SEEK_END);
    long fileSize = ftell(file);
    fseek(file, 0, SEEK_SET);
    
    printf("=== Flight Data Parser ===\n");
    printf("Input file: %s (%ld bytes)\n", inputFile, fileSize);
    printf("Output file: %s\n", outputFile);
    
    if (fileSize == 0) {
        printf("Input file is empty\n");
        fclose(file);
        fclose(csvFile);
        return;
    }
    
    long totalBytesRead = 0;
    uint16_t chunkNumber = 1;
    
    while (!feof(file) && totalBytesRead < fileSize) {
        // Read buffer size header
        uint16_t bufferSize;
        if (fread(&bufferSize, sizeof(uint16_t), 1, file) != 1) {
            printf("Error reading buffer size header\n");
            break;
        }
        totalBytesRead += sizeof(uint16_t);
        
        // Process the chunk
        if (!processDataChunk(file, bufferSize, &totalBytesRead, fileSize, chunkNumber++, csvFile)) {
            break;
        }
        
        // Show progress for large files
        if (fileSize > 10000) {
            printf("Progress: %ld%%\n", (totalBytesRead * 100) / fileSize);
        }
    }
    
    fclose(file);
    fclose(csvFile);
    
    printf("=== Parsing Complete ===\n");
    printf("Total bytes processed: %ld\n", totalBytesRead);
    printf("CSV file created: %s\n", outputFile);
}

int main(int argc, char *argv[]) {
    const char *inputFile = "TEST.txt";
    const char *outputFile = "test.csv";
    
    // Allow command line arguments to override default file names
    if (argc >= 2) {
        inputFile = argv[1];
    }
    if (argc >= 3) {
        outputFile = argv[2];
    }
    
    printf("Flight Data Parser - Converting binary data to CSV\n");
    printf("Input: %s -> Output: %s\n\n", inputFile, outputFile);
    
    parseFlightData(inputFile, outputFile);
    
    return 0;
}