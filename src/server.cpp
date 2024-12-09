#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include "hero_chassis_controller/HeroChassisControllerConfig.h"

// 回调函数，用于处理动态参数调整
void reconfigureCallback(hero_chassis_controller::HeroChassisControllerConfig &config, uint32_t level)
{
  ROS_INFO("Reconfigure Request: P=%f, I=%f, D=%f, target_v=%f",
           config.p_front_left, config.i_front_left, config.d_front_left, config.target_v_front_left);

  ROS_INFO("Reconfigure Request: P=%f, I=%f, D=%f, target_v=%f",
           config.p_front_right, config.i_front_right, config.d_front_right, config.target_v_front_right);

  ROS_INFO("Reconfigure Request: P=%f, I=%f, D=%f, target_v=%f",
           config.p_back_left, config.i_back_left, config.d_back_left, config.target_v_back_left);

  ROS_INFO("Reconfigure Request: P=%f, I=%f, D=%f, target_v=%f",
           config.p_back_right, config.i_back_right, config.d_back_right, config.target_v_back_right);

  // 更新参数服务器中的参数
  ros::param::set("controller/hero_chassis_controller/front_left/p", config.p_front_left);
  ros::param::set("controller/hero_chassis_controller/front_left/i", config.i_front_left);
  ros::param::set("controller/hero_chassis_controller/front_left/d", config.d_front_left);
  ros::param::set("controller/hero_chassis_controller/front_left/target_velocity", config.target_v_front_left);

  ros::param::set("controller/hero_chassis_controller/front_right/p", config.p_front_right);
  ros::param::set("controller/hero_chassis_controller/front_right/i", config.i_front_right);
  ros::param::set("controller/hero_chassis_controller/front_right/d", config.d_front_right);
  ros::param::set("controller/hero_chassis_controller/front_right/target_velocity", config.target_v_front_right);

  ros::param::set("controller/hero_chassis_controller/back_left/p", config.p_back_left);
  ros::param::set("controller/hero_chassis_controller/back_left/i", config.i_back_left);
  ros::param::set("controller/hero_chassis_controller/back_left/d", config.d_back_left);
  ros::param::set("controller/hero_chassis_controller/back_left/target_velocity", config.target_v_back_left);

  ros::param::set("controller/hero_chassis_controller/back_right/p", config.p_back_right);
  ros::param::set("controller/hero_chassis_controller/back_right/i", config.i_back_right);
  ros::param::set("controller/hero_chassis_controller/back_right/d", config.d_back_right);
  ros::param::set("controller/hero_chassis_controller/back_right/target_velocity", config.target_v_back_right);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "dynamic_reconfigure_node");
  ros::NodeHandle nh;

  // 创建动态调参服务器和回调函数
  dynamic_reconfigure::Server<hero_chassis_controller::HeroChassisControllerConfig> server;
  dynamic_reconfigure::Server<hero_chassis_controller::HeroChassisControllerConfig>::CallbackType f;

  f = boost::bind(&reconfigureCallback, _1, _2);
  server.setCallback(f);

  ROS_INFO("Spinning node");
  ros::spin();
  return 0;
}