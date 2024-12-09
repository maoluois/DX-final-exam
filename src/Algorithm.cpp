#include "hero_chassis_controller/Algorithm.h"

// 构造函数，初始化前一个输出值
Filter::Filter() : prev_output_(0.0) {}

// 低通滤波函数，输入原始数据和系数，返回滤波后的数据
double Filter::lowPassFilter(double input, double alpha)
{
  double output = alpha * input + (1.0 - alpha) * prev_output_;
  prev_output_ = output;
  return output;
}