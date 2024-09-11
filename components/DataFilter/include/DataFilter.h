#pragma once
#include <stdio.h>
#include "math.h"

typedef struct KFP_t
{
  float LastP; // 上次估算协方差 初始化值为0.02
  float Now_P; // 当前估算协方差 初始化值为0
  float out;   // 卡尔曼滤波器输出 初始化值为0
  float Kg;    // 卡尔曼增益 初始化值为0
  float Q;     // 过程噪声协方差 初始化值为0.001，越大越信任输入
  float R;     // 观测噪声协方差 初始化值为0.543，越大收敛越慢
} KFP;         // Kalman Filter parameter

typedef struct variance_t
{
  float *varianceBuff;
  uint8_t varianceSize = 0;
  uint8_t variancePointer = 0;
  float varianceSum = 0;
} variance;

void KFP_init(KFP *src, float reliability, float convergence);
float kalmanFilter(KFP *kfp, float input);
variance *varianceInit(uint8_t size);
bool variancePush(variance *src, float data);
float varianceGet(variance *src);