#include <string.h>
#include "DataFilter.h"

// 以高度为例 定义卡尔曼结构体并初始化参数
static KFP KFP_default = {0.02, 0, 0, 0, 0.001, 0.543};
void KFP_init(KFP *src, float reliability = 0.001, float convergence = 0.543)
{
  src->LastP = KFP_default.LastP;
  src->Now_P = KFP_default.Now_P;
  src->out = KFP_default.out;
  src->Kg = KFP_default.Kg;
  src->Q = reliability;
  src->R = convergence;
}
/**
 *卡尔曼滤波器
 *@param KFP *kfp 卡尔曼结构体参数
 *   float input 需要滤波的参数的测量值（即传感器的采集值）
 *@return 滤波后的参数（最优值）
 */
float kalmanFilter(KFP *kfp, float input)
{
  if (isnan(kfp->out))
  {
    kfp->out = 0;
    printf("Kfp Out is NAN, set 0\n");
  }
  // 预测协方差方程：k时刻系统估算协方差 = k-1时刻的系统协方差 + 过程噪声协方差
  kfp->Now_P = kfp->LastP + kfp->Q;
  // 卡尔曼增益方程：卡尔曼增益 = k时刻系统估算协方差 / （k时刻系统估算协方差 + 观测噪声协方差）
  kfp->Kg = kfp->Now_P / (kfp->Now_P + kfp->R);
  // 更新最优值方程：k时刻状态变量的最优值 = 状态变量的预测值 + 卡尔曼增益 * （测量值 - 状态变量的预测值）
  kfp->out = kfp->out + kfp->Kg * (input - kfp->out); // 因为这一次的预测值就是上一次的输出值
  // 更新协方差方程: 本次的系统协方差付给 kfp->LastP 为下一次运算准备。
  kfp->LastP = (1 - kfp->Kg) * kfp->Now_P;
  return kfp->out;
}
variance *varianceInit(uint8_t size)
{
  variance *rtn = new variance;
  rtn->varianceSize = size;
  rtn->varianceBuff = new float[size];
  memset(rtn->varianceBuff, 0, size * sizeof(float));
  rtn->variancePointer = 0;
  rtn->varianceSum = 0;
  return rtn;
}

bool variancePush(variance *src, float data)
{
  if (src->varianceSize < 2)
    return false;
  src->varianceSum -= src->varianceBuff[src->variancePointer];
  src->varianceSum += data;
  src->varianceBuff[src->variancePointer] = data;
  src->variancePointer++;
  if (src->variancePointer >= src->varianceSize)
    src->variancePointer = 0;
  return true;
}
float varianceGet(variance *src)
{
  if (src->varianceSize < 2)
    return 0;
  float avg = src->varianceSum / src->varianceSize;
  float sqrSum = 0;
  for (uint8_t i = 0; i < src->varianceSize; i++)
  {
    float tmp = src->varianceBuff[i] - avg;
    sqrSum += tmp * tmp;
  }
  return sqrSum / src->varianceSize;
}

float standard_varianceGet(variance *src)
{
  return sqrt(varianceGet(src));
}
