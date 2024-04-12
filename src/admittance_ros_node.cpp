#include "include/admittance_ros_interface.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "admittance_ros_node");

    ros::NodeHandle nh;


    ros::spin();

    return 0;
}