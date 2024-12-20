/*
 * sdsave.c
 *
 *  Created on: Oct 10, 2024
 *      Author: emanu
 */

#include "sdsave.h"

#define EVENT_SIZE 14

extern uint8_t hours;
extern uint8_t minutes;
extern uint8_t seconds;
extern uint8_t mioEvento[EVENT_SIZE];

void GetCurrentTime(uint8_t *timeBuffer) {

    timeBuffer[0] = hours;   // Ore
    timeBuffer[1] = minutes; // Minuti
    timeBuffer[2] = seconds; // Secondi
}

void setEventType(int i, char *eventType) {

    const char *eventTypes[] = {
        "Motore ON",
        "Motore OFF",
        "Velocità Standard",
        "Velocità Massima",
        "Modalità Emergenza ON",
		"Modalità Emergenza OFF",
        "Pressione Fuori 1 (>-5)",
        "Fine Pressione Fuori",
        "Pressione Fuori 2 (<-30)",
        "Allarme Filtro ON",
		"Allarme Filtro OFF",
		"Batteria sotto 10%",
		"Batteria sopra 70%",
		"Batteria Completamente Carica",
    };

    if (i < 0 || i > 14) {
        i = 14;
    }

    strcpy(eventType, eventTypes[i]);

    mioEvento[i] = 0;
}
