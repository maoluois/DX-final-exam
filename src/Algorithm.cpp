#include "hero_chassis_controller/Algorithm.h"
#include <iostream>


// 低通滤波函数，输入原始数据和系数，返回滤波后的数据
double Filter::lowPassFilter(double input, double alpha)
{
  double output = alpha * input + (1.0 - alpha) * prev_output_;
  prev_output_ = output;
  return output;
}

// 逆运动学
std::vector<double> Kinematics::inverseKinematics(double vx, double vy, double wz, double wheel_base, double track_width, double r)
{
  double L = wheel_base / 2;
  double W = track_width / 2;

  std::vector<double> wheel_speeds(4);
  wheel_speeds[0] = (vx - vy - (L + W) * wz) / r;
  wheel_speeds[1] = (vx + vy + (L + W) * wz) / r;
  wheel_speeds[2] = (vx + vy - (L + W) * wz) / r;
  wheel_speeds[3] = (vx - vy + (L + W) * wz) / r;
  // std::cout << "w1: " << wheel_speeds[0] << " w2: " << wheel_speeds[1] << " w3: " << wheel_speeds[2] << " w4: " << wheel_speeds[3] << std::endl;

  return wheel_speeds;

}

// 正运动学
std::vector<double> Kinematics::forwardKinematics(double w1, double w2, double w3, double w4, double wheel_base, double track_width, double r)
{
  double L = wheel_base / 2;
  double W = track_width / 2;

  std::vector<double> baselink(3);
  double vx = r / 4 * (w1 + w2 + w3 + w4);
  double vy = r / 4 * (-w1 + w2 + w3 - w4);
  double wz = r / (4 * (L + W)) * (-w1 + w2 - w3 + w4);
  // std::cout << "vx: " << vx << " vy: " << vy << " wz: " << wz << std::endl;

  baselink[0] = vx;
  baselink[1] = vy;
  baselink[2] = wz;

  return baselink;
}

// 角度标准化
double normalizeAngle(double angle)
{
  // std::cout << "Normalizing angle: before = " << angle << std::endl;
  while (angle > M_PI) angle -= 2 * M_PI;
  while (angle < -M_PI) angle += 2 * M_PI;
  // std::cout << "Normalizing angle: after = " << angle << std::endl;
  return angle;
}

// 弧度制转角度制
double radians2degrees(double radians)
{
  return radians * 180.0 / M_PI;
}