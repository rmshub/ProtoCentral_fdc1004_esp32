#include <stdio.h>
#include "fdc1004.hpp"

#define UPPER_BOUND 0x4000 // max readout capacitance
#define LOWER_BOUND (-1 * UPPER_BOUND)
#define CHANNEL 0            // channel to be read
#define MEASURMENT 0         // measurment channel
#define SDA_IO 18 // gpio number for I2C master data
#define SCL_IO 19 // gpio number for I2C master clock

int capdac = 0;
char result[100];
FDC1004 FDC(FDC1004_100HZ, SDA_IO, SCL_IO);
static const char *TAG = "main";

extern "C" void app_main(void)
{
    FDC.begin();

    for (;;)
    {
        FDC.configureMeasurementSingle(MEASURMENT, CHANNEL, capdac);
        FDC.triggerSingleMeasurement(MEASURMENT, FDC1004_100HZ);

        // wait for completion
        vTaskDelay(15 / portTICK_RATE_MS);
        uint16_t value[2];
        if (!FDC.readMeasurement(MEASURMENT, value))
        {
            int16_t msb = (int16_t)value[0];
            int32_t capacitance = ((int32_t)457) * ((int32_t)msb); // in attofarads
            capacitance /= 1000;                                   // in femtofarads
            capacitance += ((int32_t)3028) * ((int32_t)capdac);

            ESP_LOGE(TAG, "%.4f pf", (((float)capacitance / 1000)));

            if (msb > UPPER_BOUND) // adjust capdac accordingly
            {
                if (capdac < FDC1004_CAPDAC_MAX)
                {
                    capdac++;
                }
            }
            else if (msb < LOWER_BOUND)
            {
                if (capdac > 0)
                {
                    capdac--;
                }
            }
        }
    }
}