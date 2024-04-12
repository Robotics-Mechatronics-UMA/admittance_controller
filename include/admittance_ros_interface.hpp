#pragma once

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Float32MultiArray.h>

#include <include/admittance_class.hpp>

class admittance_ros_interface
{
private:

    ros::NodeHandle nh_;

    ros::Subcriber wrench_sub;
    ros::Publisher vel_pub;

    void ForceSensorCallback(const geometry_msgs::TwistConstPtr& msg);

pulic:
    admittance_ros_interface();
    ~admittance_ros_interface();

};
