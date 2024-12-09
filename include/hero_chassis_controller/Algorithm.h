#ifndef ALGORITHM_H
#define ALGORITHM_H
#include <vector>
#include <cmath>

double normalizeAngle(double angle);
double radians2degrees(double radians);

class Filter
{
public:
  // 构造函数，初始化前一个输出值
  Filter() : prev_output_(0.0) {}
  // 低通滤波函数，输入原始数据和系数，返回滤波后的数据
  double lowPassFilter(double input, double alpha);

private:
  double prev_output_;  // 前一个输出值
};

class Kinematics
{
public:
  static std::vector<double> inverseKinematics(double vx, double vy, double wz, double wheel_base, double track_width, double r);

  static std::vector<double> forwardKinematics(double w1, double w2, double w3, double w4, double wheel_base, double track_width, double r);

};


#endif // ALGORITHM_H


