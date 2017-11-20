#ifndef _PACKET_PARSER_H_
#define _PACKET_PARSER_H_

uint8_t readPacket(Adafruit_BLE *ble, uint16_t timeout);
float parsefloat(uint8_t *buffer);
void printHex(const uint8_t * data, const uint32_t numBytes);
void initialize_bluetooth(Adafruit_BluefruitLE_UART& ble, const char* device_name);

#endif
