#include "General_Battery.h"
#include "esp_log.h"

static const char *BatteryTAG = "General_Battery";

#define V_PERCENT_RATIO 0.01
#define COULOMB_REGRESSION_RATIO 0.01
#define V_VARIANCE_LIMIT 0.000050

General_Battery::General_Battery()
{
  KFP_init(&voltage_KFP, 0.01, 0.05);
  voltage_KFP.out = -1;
  voltage_variance = varianceInit(50);
}
void General_Battery::set_bat_info(Battery_info bat_info_src)
{
  bat_info = bat_info_src;
  if (bat_info.capacity <= 0)
    bat_info.capacity = 1;
  current_charge_graph = bat_info.capacity * bat_info.Cnum_charge_graph;
  current_load_graph = bat_info.capacity * bat_info.Cnum_load_graph;
  // ESP_LOGI(BatteryTAG, "current_load_graph:%f = capacity:%f Cnum_load_graph:%f ",
  //          current_load_graph, (float)bat_info.capacity, bat_info.Cnum_load_graph);
}

void General_Battery::heartbeat(float volt, float current)
{
  if (isnan(current))
    current_ma = 0;
  else
    current_ma = current;
  if (voltage_KFP.out == -1)
  { // maually init KFP
    voltage_KFP.out = volt / bat_info.serial;
    avg_voltage_v = voltage_KFP.out;
  }
  else
    avg_voltage_v = kalmanFilter(&voltage_KFP, volt / bat_info.serial);
  variancePush(voltage_variance, avg_voltage_v);
  float voltage_variance_val = varianceGet(voltage_variance);
  // ESP_LOGI(BatteryTAG, "current:%f current_ma:%f avg_voltage_v:%f serial:%d ", current, current_ma, avg_voltage_v, bat_info.serial);

  float V_percent_tmp = get_V_percentage();
  if (!isnan(current)) //  update the coulombmeter when the current is not NAN
  {
    float Dt = coulombmeter_timer.elapsed() / 1000.0;
    // 电压过低或变化过大则重新计算avg_capacity_mah_init
    if (Dt > 10000 || avg_voltage_v < 2.8 || avg_voltage_v >= 4.25 || capacity_mah_init == 0)
    {
      if (Dt > 10000 && voltage_variance_val > V_VARIANCE_LIMIT)
      {
        percent = get_V_percentage();
        // ESP_LOGI(BatteryTAG, "V: %f voltage_variance_val: %fpercent:%f ",
        //          avg_voltage_v, voltage_variance_val, percent);
        return;
      }
      // ESP_LOGI(BatteryTAG, "current:%f current_ma:%f avg_voltage_v:%f serial:%d ", current, current_ma, avg_voltage_v, bat_info.serial);
      capacity_mah_init = bat_info.capacity * V_percent_tmp / 100;
      used_capacity_mah = 0;
    }
    else
      used_capacity_mah += Dt * current_ma / 3600 / 1000;
    coulombmeter_timer.start();

    float coulomb_Percentage = (capacity_mah_init + used_capacity_mah) /
                               bat_info.capacity * 100;
    // ESP_LOGI(BatteryTAG, "coulomb_Percentage:%f ", coulomb_Percentage);
    if (coulomb_Percentage < 0)
    {
      coulomb_Percentage = 0;
      capacity_mah_init = 1;
      used_capacity_mah = -1;
    }
    else if (coulomb_Percentage > 100)
    {
      coulomb_Percentage = 100;
      capacity_mah_init = bat_info.capacity;
      used_capacity_mah = 0;
    }
    float percent_tmp = V_percent_tmp * V_PERCENT_RATIO +
                        coulomb_Percentage * (1 - V_PERCENT_RATIO);
    if (!((percent_tmp < percent && current_ma > 0) || (percent_tmp > percent && current_ma < 0)) // 充电不能掉电量，放电不能涨电量
        || 0 == percent || fabs(percent_tmp - percent) > 20)
      percent = percent_tmp;
    // ESP_LOGI(BatteryTAG, "V: %f voltage_variance_val: %f    V_percentage:%f coulomb_Percentage:%f    percent:%f percent_tmp:%f ",
    //          avg_voltage_v, voltage_variance_val, V_percent_tmp, coulomb_Percentage, percent, percent_tmp);
    // if ((fabs(V_percent_tmp - coulomb_Percentage) > 10 && voltage_variance_val < V_VARIANCE_LIMIT))
    // {
    //   capacity_mah_init = bat_info.capacity * V_percent_tmp / 100;
    //   used_capacity_mah = 0;
    // }
    // else
    capacity_mah_init = (bat_info.capacity * percent / 100 - used_capacity_mah) * COULOMB_REGRESSION_RATIO +
                        capacity_mah_init * (1 - COULOMB_REGRESSION_RATIO);
  }
  else
  {
    percent = V_percent_tmp;
  }
}

inline float General_Battery::get_V_percentage()
{
  if (current_ma <= current_load_graph)
    return get_V_percentage(bat_info.volt_perc_graph_load);
  else if (current_ma < 0)
  {
    float tmp_ratio = current_ma / current_load_graph;
    return (1 - tmp_ratio) * get_V_percentage(bat_info.volt_perc_graph) +
           tmp_ratio * get_V_percentage(bat_info.volt_perc_graph_load);
  }
  else if (current_ma == 0)
    return get_V_percentage(bat_info.volt_perc_graph);
  else if (current_ma >= current_charge_graph)
    return get_V_percentage(bat_info.volt_perc_graph_charge);
  else // if (current_ma > 0)
  {
    float tmp_ratio = current_ma / current_charge_graph;
    return (1 - tmp_ratio) * get_V_percentage(bat_info.volt_perc_graph) +
           tmp_ratio * get_V_percentage(bat_info.volt_perc_graph_charge);
  }
}
inline float General_Battery::get_V_percentage(vector<float> graph)
{
  float V_percent_tmp = 0;
  if (graph.size() > 1)
  {
    float perc_per_step = 100 / (graph.size() - 1);
    if (avg_voltage_v < graph[graph.size() - 1])
    {
      int upper_step = 0;
      for (int i = 0; i < graph.size(); i++)
      {
        if (graph[i] > avg_voltage_v)
        {
          upper_step = i;
          break;
        }
      }
      if (upper_step > 0)
      {
        float volt_upper = graph[upper_step];
        float volt_lower = graph[upper_step - 1];
        V_percent_tmp = perc_per_step * (upper_step - 1 +
                                         (avg_voltage_v - volt_lower) /
                                             (volt_upper - volt_lower));
        // ESP_LOGI(BatteryTAG, "V_percent_tmp:%f perc_per_step:%f avg_voltage_v:%f volt_upper:%f volt_lower:%f ",
        //          V_percent_tmp, perc_per_step, avg_voltage_v, volt_upper, volt_lower);
      }
    }
    else
    {
      V_percent_tmp = 100;
      // ESP_LOGI(BatteryTAG, "V_percent_tmp:%f",
      //          V_percent_tmp);
    }
  }
  return V_percent_tmp;
}

float General_Battery::get_percentage()
{
  // ESP_LOGI(BatteryTAG, "percent:%f\n", percent);
  return percent;
}
float General_Battery::get_remained_hour()
{
  return (bat_info.capacity * percent / 100) / current_ma;
}
float General_Battery::get_coulombmeter_mah()
{
  return used_capacity_mah;
}
Battery_info General_Battery::get_bat_info()
{
  return bat_info;
}