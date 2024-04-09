#pragma once

#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Float64MultiArray.h>
#include <eigen3/Eigen/Dense>
#include <dynamic_reconfigure/server.h>
#include <robot_delta/ParamsConfig.h> 

class Controller {
private:
    ros::NodeHandle nh;
    ros::Subscriber force_sensor_subscriber;
    ros::Publisher motor_publisher;

    Eigen::Matrix<double, 6, 6> Mass;
    Eigen::Matrix<double, 6, 6> Damping;
    Eigen::Matrix<double, 6, 6> Stiffness;
    Eigen::Matrix<double, 6, 1> Vel;
    Eigen::VectorXd Wrench;
    double Dt;

    dynamic_reconfigure::Server<robot_delta::ParamsConfig> server;

public:
    Controller(const Eigen::Matrix<double, 6, 6>&,
                const Eigen::Matrix<double, 6, 6>&,
                const Eigen::Matrix<double, 6, 6>&);
    void ForceSensorCallback(const geometry_msgs::Vector3ConstPtr&);
    void DynamicReconfigureCallback(robot_delta::ParamsConfig &config, uint32_t level);
    Eigen::Matrix<double, 6, 1> AdmittanceController();
};
