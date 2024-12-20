/*
 * sdsave.h
 *
 *  Created on: Oct 10, 2024
 *      Author: emanu
 */

#ifndef INC_SDSAVE_H_
#define INC_SDSAVE_H_

#include "stm32f7xx_hal.h"
#include "main.h"
#include "string.h"
#include "stdio.h"
#include "stdbool.h"
#include "semphr.h"
#include "stdlib.h"


void GetCurrentTime(uint8_t *timeBuffer);
void setEventType(int i, char *eventType);

#endif /* INC_SDSAVE_H_ */
