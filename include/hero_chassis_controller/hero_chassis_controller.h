//
// Created by qiayuan on 2/6/21.
//

#ifndef HERO_CHASSIS_CONTROLLER_H
#define HERO_CHASSIS_CONTROLLER_H

#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <control_toolbox/pid.h>
#include <std_msgs/Float64.h>
#include <hero_chassis_controller/Algorithm.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
// #include "hero_chassis_controller/WheelParams.h"

namespace hero_chassis_controller
{
class HeroChassisController : public controller_interface::Controller<hardware_interface::EffortJointInterface>
{
public:
  HeroChassisController();
  ~HeroChassisController() override = default;

  bool init(hardware_interface::EffortJointInterface* effort_joint_interface, ros::NodeHandle& root_nh,
            ros::NodeHandle& controller_nh) override;

  void update(const ros::Time& time, const ros::Duration& period) override;

  hardware_interface::JointHandle front_left_joint_, front_right_joint_, back_left_joint_, back_right_joint_;

private:
  bool loadParams(ros::NodeHandle& controller_nh, control_toolbox::Pid& pid, double& target_velocity, const std::string& prefix);
  void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& cmd_vel_msg);

  control_toolbox::Pid pid_front_left_;
  control_toolbox::Pid pid_front_right_;
  control_toolbox::Pid pid_back_left_;
  control_toolbox::Pid pid_back_right_;

  Filter fliter_front_left_;
  Filter fliter_front_right_;
  Filter fliter_back_left_;
  Filter fliter_back_right_;
  double alpha_lowPassFilter_ = 0.3;

  double target_velocity_1_ = 0;
  double target_velocity_2_ = 0;
  double target_velocity_3_ = 0;
  double target_velocity_4_ = 0;

  double i_clamp_max = 10;
  double i_clamp_min = -10;

  double wheel_radius = 0.07625;
  double wheel_base_ = 0.4;
  double track_width_ = 0.4;

  ros::Subscriber cmd_vel_sub_;
  ros::Publisher odom_pub_;
  ros::Time last_time_;

  nav_msgs::Odometry odom;
  // ??? 不能用静态变量
  tf2_ros::TransformBroadcaster odom_broadcaster;
  tf2_ros::Buffer tfBuffer_;
  // tf_listener_ 要在类成员初始化列表中初始化，生命周期要比类成员变量长，在Init函数中初始化会被销毁
  tf2_ros::TransformListener tf_listener_;
  tf2::Quaternion q;
  geometry_msgs::Quaternion odom_quat;
  geometry_msgs::TransformStamped odom_trans;
  std::vector<double> baselink;
  std::vector<double> wheel_speeds;


  double x_ = 0.0;
  double y_ = 0.0;
  double theta_ = 0.0;
  double vx = 0.0;
  double vy = 0.0;
  double vtheta = 0.0;

  std::string velocity_mode_;

};
  // 从参数服务器加载PID参数
};  //  namespace hero_chassis_controller

#endif  // HERO_CHASSIS_CONTROLLER_H
