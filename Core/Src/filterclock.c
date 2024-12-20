/*
 * filterclock.c
 *
 *  Created on: Aug 26, 2024
 *      Author: emanu
 */
#include "main.h"
#include "PollingRoutines.h"
#include "string.h"
#include "cmsis_os.h"
#include "semphr.h"

extern UART_HandleTypeDef huart6;
extern UART_HandleTypeDef huart1;
extern osSemaphoreId tim2SemHandle;
extern TIM_HandleTypeDef htim2;

extern uint8_t tim2Flag;

void TimInit()
{
	HAL_TIM_Base_Start_IT(&htim2);
}

void TimerRoutine()
{
	if(tim2Flag)
	{
		xSemaphoreGive(tim2SemHandle);
		tim2Flag = 0;
	}
}

void TimStop()
{
	HAL_TIM_Base_Stop_IT(&htim2);
}





