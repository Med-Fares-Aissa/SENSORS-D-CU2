/*
 * PollingRoutines.h
 *
 *  Created on: Aug 8, 2024
 *      Author: emanu
 */

#ifndef INC_POLLINGROUTINES_H_
#define INC_POLLINGROUTINES_H_
#ifdef __cpluslus
extern "C" {
}
#endif


#define UART_BUF_SIZE 16

void PollingInit();
void PollingRoutine();
unsigned short modbus_CRC16(unsigned char *ptr, unsigned char len);
void processModbusData(uint8_t* buffer);

#ifdef __cplusplus
#endif


#endif /* INC_POLLINGROUTINES_H_ */
