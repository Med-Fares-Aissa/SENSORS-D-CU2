/*
 * PollingRoutines.c
 *
 *  Created on: Aug 8, 2024
 *      Author: emanu
 */
#include "main.h"
#include "PollingRoutines.h"
#include "string.h"
#include "cmsis_os.h"
#include "semphr.h"

#define MODBUS_BUFFER_SIZE 6  //Dimensione del pacchetto Modbus
#define UART_MSG_LENGTH 15   // Lunghezza della sequenza di dati


extern UART_HandleTypeDef huart6;
extern UART_HandleTypeDef huart1;
extern osSemaphoreId binarySemUartMsgHandle;
extern osSemaphoreId binarySemUartMsg2Handle;
extern osSemaphoreId binarySemUartMsg3Handle;

uint8_t uartMsgBuf[UART_BUF_SIZE];
uint8_t uartMsgBuf2[UART_BUF_SIZE];
uint8_t uartMsgBuf3[UART_BUF_SIZE];
uint8_t concatenatedBuf[UART_BUF_SIZE];
uint8_t modbusRxBuffer[MODBUS_BUFFER_SIZE];
uint8_t receivedData[UART_MSG_LENGTH];
uint8_t rxIndex = 0;
uint8_t msgRdyFlag = 0;
uint8_t msgRdyFlag2 = 0;
uint8_t msgRdyFlag3 = 0;
uint8_t bufferSwitch = 0;
uint8_t modbusFrameComplete = 0;
//uint8_t uartMsgData[2];
uint8_t recvIndex=0;
uint8_t sendIndex=1;
int16_t humidity;
int16_t temperature;
int16_t voc_index;
int16_t nox_index;
extern float humidity_scaled;
extern float temperature_scaled;
extern float voc_scaled;
extern float nox_scaled;

//INTERRUPT PER LA RICEZIONE DATI CON TIMER
void PollingInit()
{
	//HAL_UART_Receive_IT(&huart6, uartMsgData, 1);
}

//FLAGS PER L'INVIO DEI DATI A TOUCHGFX
void PollingRoutine()
{
	if(msgRdyFlag)
	{
		xSemaphoreGive(binarySemUartMsgHandle);
		msgRdyFlag = 0;
	}
	if(msgRdyFlag2)
	{
		xSemaphoreGive(binarySemUartMsg2Handle);
		msgRdyFlag2 = 0;
	}
	if(msgRdyFlag3)
	{
		xSemaphoreGive(binarySemUartMsg3Handle);
		msgRdyFlag3 = 0;
	}
}

//FUNZIONE DI RICEZIONE DEI DATI DAI VARI DISPOSITIVI
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
#if 0
    if (huart == &huart6) {

        receivedData[rxIndex++] = uartMsgData[0];
        if (rxIndex == 1 && receivedData[0] == 0x7E) {
            rxIndex = 1;
        }

        if (rxIndex > 1 && receivedData[rxIndex - 1] == 0x7E) {
            msgRdyFlag = 1;
        } else if (rxIndex == UART_MSG_LENGTH) {
            if (receivedData[rxIndex - 1] == 0x7E) {
                msgRdyFlag = 1;
            } else {
                rxIndex = 0;
            }
        }

        HAL_UART_Receive_IT(&huart6, uartMsgData, 1);

		if (msgRdyFlag)
		{
			msgRdyFlag = 0;
			rxIndex = 0;

			// Byte 0 - Start byte
			// Byte 1 - Device address byte
			// Byte 2 - Command identifier byte
			// Byte 3 - Error code byte
			// Byte 4/Byte 5 - Relative Humidity (RH)
			// Byte 6/Byte 7 - Temperature (in °C)
			// Byte 8/Byte 9 -  VOC Index
			// Byte 10/Byte 11 - NOx Index
			// Byte 13 - Reserved addition data byte
			// Byte 14 - Reserved additional data byte
			// Byte 15 - End byte

			if (receivedData[0] == 0x7E && receivedData[1] == 0x00
					&& receivedData[2] == 0x03 && receivedData[3] == 0x00) {
				humidity = (receivedData[4] << 8) | receivedData[5]; // Humidity
				temperature = (receivedData[6] << 8) | receivedData[7]; // Temperature
				voc_index = (receivedData[8] << 8) | receivedData[9]; // VOC Index
				nox_index = (receivedData[10] << 8) | receivedData[11]; // NOx Index

				//Scaled values
				humidity_scaled = humidity / 100.0f;  // % RH
				temperature_scaled = temperature / 200.0f;  // °C
				voc_scaled = voc_index / 10.0f;  // VOC Index
				nox_scaled = nox_index / 10.0f;  // NOx Index
			}
		}
    }
#endif
}












