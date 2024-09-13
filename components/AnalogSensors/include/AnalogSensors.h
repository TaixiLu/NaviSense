#pragma once
#include "driver/adc.h"
#include "esp_adc/adc_continuous.h"
#include "driver/temperature_sensor.h"
#include "ADC.h"
#include "DataFilter.h"

#define ADC_BAT_RATIO 3.272

class AnalogSensors
{
public:
    AnalogSensors();
    ~AnalogSensors();

    float get_temperature();

    float get_bat_V();

private:
    temperature_sensor_handle_t temp_handle;
    ADC1_DMA *adc_dma;
    adc_channel_t bat_ADC_CH;

    KFP bat_KFP;
};
