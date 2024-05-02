#pragma once

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Float32MultiArray.h>
#include <dynamic_reconfigure/server.h>
#include <admittance_controller/admittanceConfig.h>
#include <admittance_model.hpp>

class admittance_ros_interface
{
private:

    std::shared_ptr<Controller> admittance_controller;
    ros::NodeHandle nh_;

    ros::Subscriber wrench_sub;
    ros::Publisher vel_pub; 

public:

    admittance_ros_interface(double mx, double my, double mz, double bx, double by, double bz);
    ~admittance_ros_interface();
    void ForceSensorCallback(const geometry_msgs::TwistConstPtr& msg);
    void DrCallback(admittance_controller::admittanceConfig &config, uint32_t level);

};
