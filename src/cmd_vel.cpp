//
// Created by luomao on 24-12-9.
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "cmd_vel");
  ros::NodeHandle nh;

  ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  ros::Rate rate(100);

  while (ros::ok())
  {
    geometry_msgs::Twist cmd_vel_msg;
    cmd_vel_msg.linear.x = 0.0;
    cmd_vel_msg.angular.z = 0.0;

    cmd_vel_pub.publish(cmd_vel_msg);
    rate.sleep();
  }

  return 0;
}