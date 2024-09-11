#pragma once

#include "Timer.h"
#include "math.h"
#include "DataFilter.h"

typedef struct
{
  double I_buf = 0;
  float D_buf = 0;
} PID_buf;

typedef struct
{
  float PID_P = 0;
  double PID_I = 0;
  float PID_D = 0;
  float PID_I_MIN = -INFINITY; // 非对称限制！！！
  float PID_I_MAX = 0;
  float PID_D_MAX = 0;
  float PID_D_Q = 0.01; // 过程噪声协方差，越大越信任输入
  float PID_D_R = 0.2;  // 观测噪声协方差，越大收敛越慢
} PID_Parameter;

class PID
{
public:
  PID(PID_Parameter src_para);
  PID();
  void set_parameter(PID_Parameter src_para);
  PID_Parameter get_parameter();
  float PID_CAL(double delta_src);
  void set_I_buff(double I_src);
  void clear_buf();

private:
  PID_Parameter parameter;
  PID_buf buf;
  Timer_micro interval_timer;

  KFP D_KFP;
};