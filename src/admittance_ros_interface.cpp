#include "admittance_model.hpp"
#include "admittance_ros_interface.hpp"


admittance_ros_interface::admittance_ros_interface(double mx, double my, double mz, double bx, double by, double bz)
{
    admittance_controller = std::make_shared<Controller>(mx, my, mz, bx, by, bz);

    wrench_sub = nh_.subscribe("/Force", 10,&admittance_ros_interface::ForceSensorCallback, this);
    vel_pub = nh_.advertise<geometry_msgs::Twist>("/Vel", 10);

}
admittance_ros_interface::~admittance_ros_interface(){}

void admittance_ros_interface::ForceSensorCallback(const geometry_msgs::TwistConstPtr& msg)
{
    // Update Wrench matrix
    Eigen::Matrix<double, 6, 1> wrench = admittance_controller->getWrench();
    wrench << msg->linear.x,msg->linear.y,msg->linear.z,
            msg->angular.x,msg->angular.y,msg->angular.z;

    admittance_controller->setWrench(wrench);


    // Calculate velocity desired using an admittancec controller
    Eigen::Matrix<double, 6, 1> vel_desired = admittance_controller->AdmittanceController();

    //Create a Twist mesage to publish the velocity
    geometry_msgs::Twist vel_msg;

    //Vel max = 0,02m/s
    if (vel_desired(0)>0.02)
    {
        vel_msg.linear.x = 0.02;
    }
    else
    {
        vel_msg.linear.x = vel_desired(0);

    }
    if (vel_desired(1)>0.02)
    {
        vel_msg.linear.y = 0.02;
    }
    else
    {
        vel_msg.linear.y = vel_desired(1);

    }
    if (vel_desired(2)>0.02)
    {
        vel_msg.linear.z = 0.02;
    }
    else
    {
        vel_msg.linear.z = vel_desired(2);

    }
    vel_msg.angular.x = vel_desired(3);
    vel_msg.angular.y = vel_desired(4);
    vel_msg.angular.z = vel_desired(5);

    //Publish velocity mesage
    vel_pub.publish(vel_msg);

}
