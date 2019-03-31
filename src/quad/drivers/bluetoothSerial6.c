
#include <stdio.h>
#include <stdint.h>
#include "serial.h"

static serialPort_t *bluetoothSerial6Port;

void bluetoothSerial6Init(void)
{
	serialPortConfig_t *rxSerialTestPortConfig = findSerialPortConfig(FUNCTION_RX_SERIAL);
	if (!rxSerialTestPortConfig)
		return;
	
	/* gpsPortConfig->identifier = SERIAL_PORT_USART6 */
	bluetoothSerial6Port = openSerialPort(rxSerialTestPortConfig->identifier, FUNCTION_RX_SERIAL, NULL, 1382400, MODE_RXTX, SERIAL_NOT_INVERTED);
//	bluetoothSerial6Port = openSerialPort(rxSerialTestPortConfig->identifier, FUNCTION_RX_SERIAL, NULL, 9600, MODE_RXTX, SERIAL_NOT_INVERTED);
//	bluetoothSerial6Port = openSerialPort(rxSerialTestPortConfig->identifier, FUNCTION_RX_SERIAL, NULL, 115200, MODE_RXTX, SERIAL_NOT_INVERTED);
	if (!bluetoothSerial6Port)
		return;
	
//	printf("%s, %d\r\n", __FUNCTION__, __LINE__);
}

void bluetoothSerial6Write(uint8_t ch)
{
	serialWrite(bluetoothSerial6Port, ch);
}

uint8_t bluetoothSerial6Read(void)
{
	return serialRead(bluetoothSerial6Port);
}

void bluetoothSerial6Print(const char *str)
{
	serialPrint(bluetoothSerial6Port, str);
}
