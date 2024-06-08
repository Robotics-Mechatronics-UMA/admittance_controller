#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <eigen3/Eigen/Dense>


void cmdVelEECallback(const geometry_msgs::TwistConstPtr& msg)
{
    Eigen::Matrix<double, 6, 1> Vel;
    Vel << msg->linear.x,msg->linear.y,msg->linear.z,
            msg->angular.x,msg->angular.y,msg->angular.z;

}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "delta_ros_interface_node");

    ros::NodeHandle nh;
    
    ros::Subscriber cmd_vel_ee_sub_ = nh.subscribe("/cmd_vel_ee", 10, &cmdVelEECallback);



    ros::spin();


    return 0;
}
