#include "include/admittance_class.hpp"
#include "include/admittance_ros_interface.hpp"


admittance_ros_interface::admittance_ros_interface()
{
    admittance_controller = std::make_shared<admittance_class>();

    wrench_sub = nh.subscribe("/force", 10,&admittance_ros_interface::ForceSensorCallback, this);
    vel_pub = nh.advertise("/motor", 10);

}
admittance_ros_interface::~admittance_ros_interface()   
{
}

void Controller::ForceSensorCallback(const geometry_msgs::TwistConstPtr& msg) {
    // Update Wrench matrix
    Wrench << wrench_msg.linear.x,wrench_msg.linear.y,wrench_msg.linear.z,
            wrench_msg.angular.x,wrench_msg.angular.y,wrench_msg.angular.z;

    // Calculate velocity desired using an admittancec controller
    Eigen::Matrix<double, 6, 1> vel_desired = admittance_controller->AdmittanceController();

    //Create a Twist mesage to publish the velocity
    geometry_msgs::Twist vel_msg;

    vel_msg.linear.x = vel_desired(0);
    vel_msg.linear.y = vel_desired(1);
    vel_msg.linear.z = vel_desired(2);
    vel_msg.angular.x = vel_desired(3);
    vel_msg.angular.y = vel_desired(4);
    vel_msg.angular.z = vel_desired(5);

    //Publish velocity mesage
    vel_pub.publish(vel_msg);

}
