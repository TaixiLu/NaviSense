#pragma once
#include "DataFilter.h"
#include "Differential_Timer.h"
#include "Battery_info.h"

class General_Battery
{
public:
  General_Battery();

  void heartbeat(float volt, float current = NAN);

  void set_bat_info(Battery_info bat_info_src);

  float get_percentage(); //%
  float get_remained_hour();
  float get_coulombmeter_mah();
  Battery_info get_bat_info();

private:
  Battery_info bat_info;
  float current_load_graph = -1;  // ma
  float current_charge_graph = 1; // ma

  inline float get_V_percentage(vector<float> graph);
  inline float get_V_percentage();

  float percent = 0;

  float current_ma = 0;
  float avg_voltage_v = 0;

  float used_capacity_mah = 0;
  float capacity_mah_init = 0;
  Timer_micro coulombmeter_timer;

  KFP voltage_KFP;

  variance *voltage_variance;
};
