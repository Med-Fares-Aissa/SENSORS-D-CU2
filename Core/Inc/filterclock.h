/*
 * filterclock.h
 *
 *  Created on: Aug 26, 2024
 *      Author: emanu
 */

#ifndef INC_FILTERCLOCK_H_
#define INC_FILTERCLOCK_H_

#ifdef __cpluslus
extern "C" {
}
#endif

void TimInit();
void TimerRoutine();
void TimStop();

#ifdef __cplusplus
#endif

#endif /* INC_FILTERCLOCK_H_ */
