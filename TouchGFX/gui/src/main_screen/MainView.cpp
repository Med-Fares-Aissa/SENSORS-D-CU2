#include <gui/main_screen/MainView.hpp>

extern float humidity_scaled;
extern float temperature_scaled;
extern float voc_scaled;
extern float nox_scaled;

MainView::MainView() :
    tickCounter(0)
{
}

void MainView::setupScreen()
{
    MainViewBase::setupScreen();
    digitalHours = digitalClock.getCurrentHour();
    digitalMinutes = digitalClock.getCurrentMinute();
    digitalSeconds = digitalClock.getCurrentSecond();

    analogHours = analogClock.getCurrentHour();
    analogMinutes = analogClock.getCurrentMinute();
    analogSeconds = analogClock.getCurrentSecond();
}

void MainView::tearDownScreen()
{
    MainViewBase::tearDownScreen();
}

void MainView::handleTickEvent()
{
    tickCounter++;

    if (tickCounter % 60 == 0)
    {
        if (++digitalSeconds >= 60)
        {
            digitalSeconds = 0;
            if (++digitalMinutes >= 60)
            {
                digitalMinutes = 0;
                if (++digitalHours >= 24)
                {
                    digitalHours = 0;
                }
            }
        }

        if (++analogSeconds >= 60)
        {
            analogSeconds = 0;
            if (++analogMinutes >= 60)
            {
                analogMinutes = 0;
                if (++analogHours >= 24)
                {
                    analogHours = 0;
                }
            }
        }

        // Update the clocks
        digitalClock.setTime24Hour(digitalHours, digitalMinutes, digitalSeconds);
        analogClock.setTime24Hour(analogHours, analogMinutes, analogSeconds);
    }
}

void MainView::updatedata()
{
	Unicode::snprintf(HumitidyBuffer, HUMITIDY_SIZE, "%0.2f", humidity_scaled);
	Unicode::snprintf(TemperatureBuffer, TEMPERATURE_SIZE, "%0.2f", temperature_scaled);
	Unicode::snprintf(VocBuffer, VOC_SIZE, "%0.2f", voc_scaled);
	Unicode::snprintf(NoxBuffer, NOX_SIZE, "%0.2f", nox_scaled);

	Humitidy.invalidate();  // This assumes the text area displays humidity
	Temperature.invalidate(); // This assumes the text area displays temperature
	Voc.invalidate();  // This assumes the text area displays VOC
	Nox.invalidate();  // This assumes the text area displays NOx
}
