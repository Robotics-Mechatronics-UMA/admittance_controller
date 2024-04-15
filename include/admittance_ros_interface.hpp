#pragma once

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Float32MultiArray.h>
#include <memory>

#include <admittance_model.hpp>

class admittance_ros_interface
{
private:

    std::shared_ptr<Controller> admittance_controller;
    ros::NodeHandle nh_;

    ros::Subscriber wrench_sub;
    ros::Publisher vel_pub;



public:
    admittance_ros_interface();
    ~admittance_ros_interface();
    void ForceSensorCallback(const geometry_msgs::TwistConstPtr& msg);

};
