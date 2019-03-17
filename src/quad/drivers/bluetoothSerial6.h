#ifndef __BLUETOOTHSERIAL6_H
#define __BLUETOOTHSERIAL6_H

void bluetoothSerial6Init(void);
void bluetoothSerial6Write(uint8_t ch);
uint8_t bluetoothSerial6Read(void);
void bluetoothSerial6Print(const char *str);

#endif	// __BLUETOOTHSERIAL6_H
