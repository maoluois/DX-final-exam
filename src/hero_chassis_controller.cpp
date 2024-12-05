//
// Created by qiayuan on 2/6/21.
//

#include "hero_chassis_controller/hero_chassis_controller.h"
#include <pluginlib/class_list_macros.hpp>

namespace hero_chassis_controller {
bool HeroChassisController::init(hardware_interface::EffortJointInterface* effort_joint_interface,
                                   ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh)
{
  front_left_joint_ = effort_joint_interface->getHandle("left_front_wheel_joint");
  front_right_joint_ = effort_joint_interface->getHandle("right_front_wheel_joint");
  back_left_joint_ = effort_joint_interface->getHandle("left_back_wheel_joint");
  back_right_joint_ = effort_joint_interface->getHandle("right_back_wheel_joint");

  // 从参数服务器加载PID参数
  if (!loadPIDParams(controller_nh, pid_front_left_, "front_left_wheel_joint"))
    return false;
  if (!loadPIDParams(controller_nh, pid_front_right_, "front_right_wheel_joint"))
    return false;
  if (!loadPIDParams(controller_nh, pid_back_left_, "back_left_wheel_joint"))
    return false;
  if (!loadPIDParams(controller_nh, pid_back_right_, "back_right_wheel_joint"))
    return false;

  // 设置目标速度
  target_velocity_1_ = 1.0;
  target_velocity_2_ = 1.0;
  target_velocity_3_ = 1.0;
  target_velocity_4_ = 1.0;
  return true;
}

void HeroChassisController::update(const ros::Time& time, const ros::Duration& period)
{
  // 获取当前速度
  double current_velocity_1 = front_left_joint_.getVelocity();
  double current_velocity_2 = front_right_joint_.getVelocity();
  double current_velocity_3 = back_left_joint_.getVelocity();
  double current_velocity_4 = back_right_joint_.getVelocity();

  // PID计算
  double control_effort_front_left = pid_front_left_.computeCommand(target_velocity_1_ - current_velocity_1, period);
  double control_effort_front_right = pid_front_right_.computeCommand(target_velocity_2_ - current_velocity_2, period);
  double control_effort_back_left = pid_back_left_.computeCommand(target_velocity_3_ - current_velocity_3, period);
  double control_effort_back_right = pid_back_right_.computeCommand(target_velocity_4_ - current_velocity_4, period);

  // 设置控制命令
  front_left_joint_.setCommand(control_effort_front_left);
  front_right_joint_.setCommand(control_effort_front_right);
  back_left_joint_.setCommand(control_effort_back_left);
  back_right_joint_.setCommand(control_effort_back_right);



  // double tau = 0.2;  // torque
  //                    // NOTE: DON'T COPY THESE NAIVE TESTING CODES !!! USE INVERSE KINEMATICS !!!
  //                    // NOTE: DON'T COPY THESE NAIVE TESTING CODES !!! USE INVERSE KINEMATICS !!!
  //                    // NOTE: DON'T COPY THESE NAIVE TESTING CODES !!! USE INVERSE KINEMATICS !!!
  // static double cmd_[6][4] = { { tau, tau, tau, tau },                      //  forward
  //                              { -2 * tau, -2 * tau, -2 * tau, -2 * tau },  //  backward
  //                              { -tau, tau, tau, -tau },                    //  left
  //                              { 2 * tau, -2 * tau, -2 * tau, 2 * tau },    //  right
  //                              { 2 * tau, -2 * tau, 2 * tau, -2 * tau },    //  clockwise
  //                              { -tau, tau, -tau, tau } };                  //  counterclockwise
  // if ((time - last_change_).toSec() > 2)
  // {
  //   state_ = (state_ + 1) % 6;
  //   last_change_ = time;
  // }
  // front_left_joint_.setCommand(cmd_[state_][0]);
  // front_right_joint_.setCommand(cmd_[state_][1]);
  // back_left_joint_.setCommand(cmd_[state_][2]);
  // back_right_joint_.setCommand(cmd_[state_][3]);
}

PLUGINLIB_EXPORT_CLASS(hero_chassis_controller::HeroChassisController, controller_interface::ControllerBase)
}  // namespace hero_chassis_controller
