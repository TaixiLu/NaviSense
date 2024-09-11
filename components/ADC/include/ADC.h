#pragma once
#include <string.h>
#include <stdio.h>
#include "sdkconfig.h"
#include "esp_adc/adc_continuous.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "driver/gpio.h"
#include <set>
#include <map>

class ADC1_DMA
{
public:
  ADC1_DMA(std::set<adc_channel_t> channels);
  ~ADC1_DMA();
  float getmV(adc_channel_t ch);
  float getRaw(adc_channel_t ch);

private:
  std::set<adc_channel_t> chs;
  std::map<adc_channel_t, float> ch_buf;
  adc_continuous_handle_t adc_continuous_handle = NULL;
  adc_cali_handle_t cali_handle;
};