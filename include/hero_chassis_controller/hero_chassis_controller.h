//
// Created by qiayuan on 2/6/21.
//

#ifndef HERO_CHASSIS_CONTROLLER_H
#define HERO_CHASSIS_CONTROLLER_H

#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <control_toolbox/pid.h>

namespace hero_chassis_controller {

class HeroChassisController : public controller_interface::Controller<hardware_interface::EffortJointInterface>
{
public:
  HeroChassisController() = default;
  ~HeroChassisController() override = default;

  bool init(hardware_interface::EffortJointInterface* effort_joint_interface, ros::NodeHandle& root_nh,
            ros::NodeHandle& controller_nh) override;

  void update(const ros::Time& time, const ros::Duration& period) override;


  hardware_interface::JointHandle front_left_joint_, front_right_joint_, back_left_joint_, back_right_joint_;

private:
  int state_{};
  ros::Time last_change_;

  control_toolbox::Pid pid_front_left_;
  control_toolbox::Pid pid_front_right_;
  control_toolbox::Pid pid_back_left_;
  control_toolbox::Pid pid_back_right_;

  double target_velocity_1_ = 0;
  double target_velocity_2_ = 0;
  double target_velocity_3_ = 0;
  double target_velocity_4_ = 0;

  double i_clamp_max = 10;
  double i_clamp_min = -10;


  // 从参数服务器加载PID参数
  bool loadPIDParams(ros::NodeHandle& controller_nh, control_toolbox::Pid& pid, const std::string& prefix);
};
}  //  namespace hero_chassis_controller

#endif  // HERO_CHASSIS_CONTROLLER_H
