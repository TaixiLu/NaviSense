#include "ADC.h"
#include <math.h>
#include "esp_log.h"

const char *ADC_TAG = "ADC:";
#define CH_READ_LEN SOC_ADC_DIGI_DATA_BYTES_PER_CONV
#define ADC_READ_LEN (CH_READ_LEN * chs.size())

ADC1_DMA::ADC1_DMA(std::set<adc_channel_t> channels)
{
  // adc_unit_t unit;
  // for (std::set<gpio_num_t>::iterator io = gpios.begin(); io != gpios.end(); ++io)
  // {
  //   adc_channel_t ch;
  //   esp_err_t err = adc_continuous_io_to_channel(*io, &unit, &ch);
  //   if (ESP_OK == err && unit == ADC_UNIT_1)
  //     chs.insert(ch);
  // }
  chs = channels;

  adc_continuous_handle_cfg_t adc_config = {
      .max_store_buf_size = ADC_READ_LEN,
      .conv_frame_size = ADC_READ_LEN,
  };
  ESP_ERROR_CHECK(adc_continuous_new_handle(&adc_config, &adc_continuous_handle));

  adc_continuous_config_t dig_cfg = {
      .pattern_num = chs.size(),
      .sample_freq_hz = 10 * 1000,
      .conv_mode = ADC_CONV_SINGLE_UNIT_1,
      .format = ADC_DIGI_OUTPUT_FORMAT_TYPE2,
  };

  adc_digi_pattern_config_t adc_pattern[SOC_ADC_PATT_LEN_MAX] = {0};
  int i = 0;
  for (std::set<adc_channel_t>::iterator ch = chs.begin(); ch != chs.end(); ++ch)
  {
    adc_pattern[i].atten = ADC_ATTEN_DB_11;
    adc_pattern[i].channel = *ch;
    adc_pattern[i].unit = ADC_UNIT_1;
    adc_pattern[i].bit_width = SOC_ADC_DIGI_MAX_BITWIDTH;

    ch_buf[*ch] = NAN;

    ESP_LOGI(ADC_TAG, "adc_pattern[%d].atten is :%x", i, adc_pattern[i].atten);
    ESP_LOGI(ADC_TAG, "adc_pattern[%d].channel is :%x", i, adc_pattern[i].channel);
    ESP_LOGI(ADC_TAG, "adc_pattern[%d].unit is :%x", i, adc_pattern[i].unit);
    i++;
  }
  adc_cali_curve_fitting_config_t cali_config = {
      .unit_id = (adc_unit_t)adc_pattern[0].unit,
      .atten = (adc_atten_t)adc_pattern[0].atten,
      .bitwidth = (adc_bitwidth_t)adc_pattern[0].bit_width,
  };
  ESP_ERROR_CHECK(adc_cali_create_scheme_curve_fitting(&cali_config, &cali_handle));
  dig_cfg.adc_pattern = adc_pattern;
  ESP_ERROR_CHECK(adc_continuous_config(adc_continuous_handle, &dig_cfg));
  ESP_ERROR_CHECK(adc_continuous_start(adc_continuous_handle));
}

ADC1_DMA::~ADC1_DMA()
{
  ESP_ERROR_CHECK(adc_cali_delete_scheme_curve_fitting(cali_handle));
  ESP_ERROR_CHECK(adc_continuous_stop(adc_continuous_handle));
  ESP_ERROR_CHECK(adc_continuous_deinit(adc_continuous_handle));
}

float ADC1_DMA::getmV(adc_channel_t ch)
{
  // float rtn = getRaw(ch) * 3.55 * 1100 / 4096;
  int rtn = 0;
  float raw = getRaw(ch);
  if (raw == -1)
    return -1;
  adc_cali_raw_to_voltage(cali_handle, raw, &rtn);
  // ESP_LOGI(ADC_TAG, "Channel: %d, mV: %f", ch, rtn);
  return (float)rtn;
}
float ADC1_DMA::getRaw(adc_channel_t ch)
{
  uint8_t result[ADC_READ_LEN] = {0};
  uint32_t ret_num = 0;
  if (ESP_OK == adc_continuous_read(adc_continuous_handle,
                                    result, ADC_READ_LEN, &ret_num, 0))
  {
    // ESP_LOGI(ADC_TAG, "ret_num is %d", (int)ret_num);
    std::map<uint8_t, int> sum;
    std::map<uint8_t, int> sum_num;
    for (int i = 0; i < ret_num; i += SOC_ADC_DIGI_RESULT_BYTES)
    {
      adc_digi_output_data_t *p = (adc_digi_output_data_t *)&result[i];
      if (p->type2.channel < SOC_ADC_CHANNEL_NUM(ADC_UNIT_1))
      {
        if (p->type2.data == 0xfff)
          return -1;
        // ESP_LOGI(ADC_TAG, "Channel: %d, Value: %x", p->type2.channel, p->type2.data);

        if (sum_num.find(p->type2.channel) == sum_num.end())
        {
          sum_num[p->type2.channel] = 1;
          sum[p->type2.channel] = p->type2.data;
        }
        else
        {
          sum_num[p->type2.channel]++;
          sum[p->type2.channel] += p->type2.data;
        }
      }
      else
        ESP_LOGI(ADC_TAG, "Invalid data");
    }
    for (const auto &pair : sum_num)
    {
      ch_buf[(adc_channel_t)pair.first] = (float)sum[pair.first] / pair.second;
      // ESP_LOGI(ADC_TAG, "ch:%d n:%d, vol:%f", pair.first, pair.second, ch_buf[(adc_channel_t)pair.first]);
    }
  }

  return ch_buf[ch];
}