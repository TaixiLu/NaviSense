
#include "AnalogSensors.h"
#include "esp_log.h"
#include "math.h"

const char *SENSOR_TAG = "AnalogSensors:";

#ifdef bat_ADC_EN
#define PIN_bat_ADC GPIO_NUM_10
#endif

AnalogSensors::AnalogSensors()
{
    float tsens_out;
    ESP_LOGI(SENSOR_TAG, "INITING");
    temperature_sensor_config_t temp_sensor = {
        .range_min = -10,
        .range_max = 80,
    };
    ESP_ERROR_CHECK(temperature_sensor_install(&temp_sensor, &temp_handle));
    ESP_ERROR_CHECK(temperature_sensor_enable(temp_handle));

#ifdef bat_ADC_EN
    adc_unit_t unit;
    std::set<adc_channel_t> chs;
    ESP_ERROR_CHECK(adc_continuous_io_to_channel(PIN_bat_ADC, &unit, &bat_ADC_CH));
    chs.insert(bat_ADC_CH);
    adc_dma = new ADC1_DMA(chs);

    KFP_init(&bat_KFP, 0.1, 0.1);
    get_bat_V();
#endif
}
AnalogSensors::~AnalogSensors()
{
#ifdef bat_ADC_EN
    delete adc_dma;
#endif
}

float AnalogSensors::get_temperature()
{
    float tsens_out;
    temperature_sensor_get_celsius(temp_handle, &tsens_out);
    return tsens_out;
}
#ifdef bat_ADC_EN
float AnalogSensors::get_bat_V()
{
    ESP_LOGI(SENSOR_TAG, "bat_V");
    float rtn = adc_dma->getmV(bat_ADC_CH) / 1000 * ADC_BAT_RATIO;
    if (rtn < 0)
        return 0;
    rtn = kalmanFilter(&bat_KFP, rtn);
    ESP_LOGI(SENSOR_TAG, "bat_V: %f", rtn);
    return rtn;
}
#endif