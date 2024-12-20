#include <gui/model/Model.hpp>
#include <gui/model/ModelListener.hpp>
#include <main.h>

extern osSemaphoreId sensordatasemHandle;

extern float humidity_scaled;
extern float temperature_scaled;
extern float voc_scaled;
extern float nox_scaled;

Model::Model() : modelListener(0)
{
}

void Model::tick()
{
	if (sensordatasemHandle != NULL) {

		// If semaphore is available, take it and call uartMsgRdy2()
		if (xSemaphoreTake(sensordatasemHandle, (TickType_t) 10) == pdTRUE) {
			updatedataonscreen();
		}
	}
}

void Model::updatedataonscreen()
{
	modelListener->updatedata();
}
