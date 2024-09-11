#include "PID.h"
#include "esp_log.h"

PID::PID(PID_Parameter src_para)
{
  set_parameter(src_para);
}
PID::PID()
{
  parameter.PID_P = 1;
  parameter.PID_I = 0;
  parameter.PID_D = 0;
  parameter.PID_I_MIN = -INFINITY;
  parameter.PID_I_MAX = 0;
  parameter.PID_D_MAX = 0;
  KFP_init(&D_KFP, 0.001, 0.543);
}
void PID::set_parameter(PID_Parameter src_para)
{
  parameter = src_para;
  KFP_init(&D_KFP, parameter.PID_D_Q, parameter.PID_D_R);
}
PID_Parameter PID::get_parameter()
{
  return parameter;
}
float PID::PID_CAL(double delta_src)
{
  // P
  float tmp_P = delta_src * parameter.PID_P;

  float tmp_D = 0;
  uint64_t interval_int = interval_timer.elapsed();
  interval_timer.start();
  float interval;
  if (TIMER_MICRO_DEFAULT == interval_int)
    interval = 1;
  else
  {
    interval = interval_int / 1000.0;
    // D
    tmp_D = kalmanFilter(&D_KFP, (delta_src - buf.D_buf) * parameter.PID_D / interval);
    if (tmp_D > parameter.PID_D_MAX)
      tmp_D = parameter.PID_D_MAX;
    else if (tmp_D < -parameter.PID_D_MAX)
      tmp_D = -parameter.PID_D_MAX;
    buf.D_buf = delta_src;
  }

  // I
  double tmp_I = buf.I_buf + delta_src * interval * parameter.PID_I;
  if (tmp_I > parameter.PID_I_MAX)
    tmp_I = parameter.PID_I_MAX;
  else if (tmp_I < (parameter.PID_I_MAX * -1))
    tmp_I = -parameter.PID_I_MAX;
  if (tmp_I < parameter.PID_I_MIN)
    tmp_I = parameter.PID_I_MIN;
  buf.I_buf = tmp_I;

  // ESP_LOGI("PID", "delta_src: %02.4f P: %02.4f I: %02.4f D: %02.4f  OUT:%02.4f",
  //          delta_src, tmp_P, (float)tmp_I, tmp_D, (tmp_P + tmp_I + tmp_D));
  return (tmp_P + tmp_I + tmp_D);
}

void PID::clear_buf()
{
  buf.D_buf = 0;
  buf.I_buf = 0;
  interval_timer.clear();
  KFP_init(&D_KFP, parameter.PID_D_Q, parameter.PID_D_R);
}
void PID::set_I_buff(double I_src)
{
  if (I_src > parameter.PID_I_MAX)
    I_src = parameter.PID_I_MAX;
  else if (I_src < (parameter.PID_I_MAX * -1))
    I_src = -parameter.PID_I_MAX;
  if (I_src < parameter.PID_I_MIN)
    I_src = parameter.PID_I_MIN;
  buf.I_buf = I_src;
}