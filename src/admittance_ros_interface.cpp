#include "include/admittance_class.hpp"
#include "include/admittance_ros_interface.hpp"


admittance_ros_interface::admittance_ros_interface()
{
    wrench_sub = nh.subscribe("/force", 10,&admittance_ros_interface::ForceSensorCallback, this);
    vel_pub = nh.advertise("/motor", 10);

}
admittance_ros_interface::~admittance_ros_interface()   
{
}

void Controller::ForceSensorCallback(const geometry_msgs::TwistConstPtr& msg) {
    // Next update using Twist
    Wrench << wrench_msg.linear.x,wrench_msg.linear.y,wrench_msg.linear.z,
            wrench_msg.angular.x,wrench_msg.angular.y,wrench_msg.angular.z;

    // Calculate velocity desired using an admittancec controller
    Eigen::Matrix<double, 6, 1> Vel_pub = AdmittanceController();

}
