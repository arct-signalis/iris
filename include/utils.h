#ifndef UTILS_H
#define UTILS_H

// Time Utilities
#define GET_TIME_MS() ((uint32_t)millis())  // Consistent uint32_t time across project
#define SECONDS_TO_MILLIS(s) ((s) * 1000)   // Convert seconds to milliseconds
#define MILLIS_TO_SECONDS(ms) ((ms) / 1000) // Convert milliseconds to seconds

// MATH OPTIMIZATION MACROS
#define VECTOR_LENGTH(x, y, z) sqrtf((x) * (x) + (y) * (y) + (z) * (z)) // 3D vector magnitude using C float math
#define DOUBLE_TO_FLOAT(x) ((float)(x))                                 // Macro for converting double to float

#endif