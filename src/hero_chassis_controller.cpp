#include "hero_chassis_controller/hero_chassis_controller.h"
#include <pluginlib/class_list_macros.hpp>
#include <cmath>


namespace hero_chassis_controller {

bool HeroChassisController::init(hardware_interface::EffortJointInterface* effort_joint_interface,
                                 ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh)
{
  std::cout << "******************************init*************************************" << std::endl;
  front_left_joint_ = effort_joint_interface->getHandle("left_front_wheel_joint");
  front_right_joint_ = effort_joint_interface->getHandle("right_front_wheel_joint");
  back_left_joint_ = effort_joint_interface->getHandle("left_back_wheel_joint");
  back_right_joint_ = effort_joint_interface->getHandle("right_back_wheel_joint");

  // 从参数服务器加载PID参数和目标速度
  if (!loadParams(controller_nh, pid_front_left_, target_velocity_1_, "front_left"))
    return false;
  if (!loadParams(controller_nh, pid_front_right_, target_velocity_2_, "front_right"))
    return false;
  if (!loadParams(controller_nh, pid_back_left_, target_velocity_3_, "back_left"))
    return false;
  if (!loadParams(controller_nh, pid_back_right_, target_velocity_4_, "back_right"))
    return false;

  // 从参数服务器加载轴距和轮距，如果没有配置文件，则使用默认值
  if (!controller_nh.getParam("wheel_base", wheel_base_))
  {
    wheel_base_ = 0.4; // 默认值
    ROS_WARN("Using default wheel_base: %f", wheel_base_);
  }
  if (!controller_nh.getParam("track_width", track_width_))
  {
    track_width_ = 0.4; // 默认值
    ROS_WARN("Using default track_width: %f", track_width_);
  }

  // 初始化低通滤波器
  fliter_front_left_ = Filter();
  fliter_front_right_ = Filter();
  fliter_back_left_ = Filter();
  fliter_back_right_ = Filter();

  // 订阅/cmd_vel话题
  cmd_vel_sub_ = root_nh.subscribe("/cmd_vel", 10, &HeroChassisController::cmdVelCallback, this);

  // // 发布里程计信息
  odom_pub_ = root_nh.advertise<nav_msgs::Odometry>("odom", 10);

  // 初始化里程计信息
  x_ = 0.0;
  y_ = 0.0;
  theta_ = 0.0;

  last_time_ = ros::Time::now();

  ROS_INFO("Successfully init controller with target velocities: FL=%f, FR=%f, BL=%f, BR=%f",
           target_velocity_1_, target_velocity_2_, target_velocity_3_, target_velocity_4_);
  return true;
}

void HeroChassisController::update(const ros::Time& time, const ros::Duration& period)
{
  // 低通滤波器后的速度
  double current_velocity_1 = fliter_front_left_.lowPassFilter(front_left_joint_.getVelocity(), alpha_lowPassFilter_);
  double current_velocity_2 = fliter_front_right_.lowPassFilter(front_right_joint_.getVelocity(), alpha_lowPassFilter_);
  double current_velocity_3 = fliter_back_left_.lowPassFilter(back_left_joint_.getVelocity(), alpha_lowPassFilter_);
  double current_velocity_4 = fliter_back_right_.lowPassFilter(back_right_joint_.getVelocity(), alpha_lowPassFilter_);

  // // 获取当前速度
  // double current_velocity_1 = front_left_joint_.getVelocity();
  // double current_velocity_2 = front_right_joint_.getVelocity();
  // double current_velocity_3 = back_left_joint_.getVelocity();
  // double current_velocity_4 = back_right_joint_.getVelocity();

  // PID计算
  double front_left_v = pid_front_left_.computeCommand(target_velocity_1_ - current_velocity_1, period);
  double front_right_v = pid_front_right_.computeCommand(target_velocity_2_ - current_velocity_2, period);
  double back_left_v = pid_back_left_.computeCommand(target_velocity_3_ - current_velocity_3, period);
  double back_right_v = pid_back_right_.computeCommand(target_velocity_4_ - current_velocity_4, period);

  // 设置控制命令
  front_left_joint_.setCommand(front_left_v);
  front_right_joint_.setCommand(front_right_v);
  back_left_joint_.setCommand(back_left_v);
  back_right_joint_.setCommand(back_right_v);

  // 正运动学解算
  baselink = Kinematics::forwardKinematics(front_left_v, front_right_v, back_left_v, back_right_v, wheel_base_, track_width_, wheel_radius);
  double dt = (time - last_time_).toSec();
  double delta_x = (baselink[0] * cos(theta_) - baselink[1] * sin(theta_)) * dt;
  // std::cout << baselink[0] << " " << baselink[1] << " " << baselink[2] << std::endl;
  double delta_y = (baselink[0] * sin(theta_) + baselink[1] * cos(theta_)) * dt;
  double delta_theta = baselink[2] * dt;

  x_ += delta_x;
  y_ += delta_y;

  // 标准化角度
  theta_ = normalizeAngle(theta_ + delta_theta);

  // 弧度转角度
  // double Degree = radians2degrees(theta_);

  // std::cout << "x: " << x_ << " y: " << y_ << " theta: " << Degree << std::endl;

  // 设置odom的名称和时间戳
  q.setRPY(0.0, 0.0, theta_);  // 设置roll, pitch, yaw（这里roll和pitch为0）
  geometry_msgs::Quaternion odom_quat = tf2::toMsg(q);  // 将四元数转换为 ROS 消息类型
  odom.header.stamp = time;
  odom.header.frame_id = "odom";

  // 设置odom的位置
  odom.pose.pose.position.x = x_;
  odom.pose.pose.position.y = y_;
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation = odom_quat;

  odom_pub_.publish(odom);

  odom_trans.header.stamp = time;
  odom_trans.header.frame_id = "odom";
  odom_trans.child_frame_id = "base_link";

  odom_trans.transform.translation.x = x_;
  odom_trans.transform.translation.y = y_;
  odom_trans.transform.translation.z = 0.0;
  odom_trans.transform.rotation = odom_quat;

  // send the transform
  odom_broadcaster.sendTransform(odom_trans);
  last_time_ = time;
}

void HeroChassisController::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
  double vx = msg->linear.x;
  double vy = msg->linear.y;
  double wz = msg->angular.z;

  // std::cout << "vx: " << vx << " vy: " << vy << " wz: " << wz << std::endl;

  double L = wheel_base_;
  double W = track_width_;
  double r = wheel_radius;

  // 逆运动学计算目标速度
  wheel_speeds = Kinematics::inverseKinematics(vx, vy, wz, L, W, r);
  target_velocity_1_ = wheel_speeds[0];
  target_velocity_2_ = wheel_speeds[1];
  target_velocity_3_ = wheel_speeds[2];
  target_velocity_4_ = wheel_speeds[3];

  // std::cout<< "target_velocity_1_: " << target_velocity_1_ << " target_velocity_2_: " << target_velocity_2_ << " target_velocity_3_: " << target_velocity_3_ << " target_velocity_4_: " << target_velocity_4_ << std::endl;
}

bool HeroChassisController::loadParams(ros::NodeHandle& controller_nh, control_toolbox::Pid& pid, double& target_velocity, const std::string& prefix)
{
  double p, i, d;
  if (!controller_nh.getParam(prefix + "/p", p) ||
      !controller_nh.getParam(prefix + "/i", i) ||
      !controller_nh.getParam(prefix + "/d", d) ||
      !controller_nh.getParam(prefix + "/target_velocity", target_velocity))
  {
    ROS_ERROR_STREAM("Cannot find required parameter '" << prefix << "'");
    return false;
  }
  ROS_INFO("Loaded parameters for %s: P=%f, I=%f, D=%f, Target Velocity=%f", prefix.c_str(), p, i, d, target_velocity);
  pid.initPid(p, i, d, i_clamp_max, i_clamp_min);
  return true;
}

PLUGINLIB_EXPORT_CLASS(hero_chassis_controller::HeroChassisController, controller_interface::ControllerBase)

}  // namespace hero_chassis_controller