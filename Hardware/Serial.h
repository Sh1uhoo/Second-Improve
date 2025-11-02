#ifndef __SERIAL_H
#define __SERIAL_H

extern char Serial_RxData[];
extern uint8_t Serial_RxFlag;

void Serial_Init(void);
void Serial_sendByte(uint8_t Byte);
uint8_t Serial_GetRxFlag(void);

#endif
