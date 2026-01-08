#ifndef LORA_H
#define LORA_H

#include "structs.h"

#pragma pack(push, 1)
struct FlightPacket {
  char rocket_name[16];
  float altitude;
  float gyro[3];
  float lat;
  float lon;
  float temperature;
};
#pragma pack(pop)

extern FlightData flightData;

void printParameters(struct Configuration configuration);
void printModuleInformation(struct ModuleInformation moduleInformation);
void loraInit();
void loraSend();

#endif