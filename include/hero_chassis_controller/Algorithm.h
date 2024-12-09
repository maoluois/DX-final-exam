#ifndef FILTER_H
#define FILTER_H

class Filter
{
public:
  // 构造函数，初始化前一个输出值
  Filter();

  // 低通滤波函数，输入原始数据和系数，返回滤波后的数据
  double lowPassFilter(double input, double alpha);

private:
  double prev_output_;  // 前一个输出值
};

#endif // FILTER_H