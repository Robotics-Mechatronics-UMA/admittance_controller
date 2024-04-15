#include "admittance_ros_interface.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "admittance_ros_node");

    ros::NodeHandle nh("~");

    //Controller variables
    double mx,my,mz,bx,by,bz;
    nh.param<double>("mx", mx, 30);
    nh.param<double>("my", my, 30);
    nh.param<double>("mz", mz, 30);
    nh.param<double>("bx", bx, 20);
    nh.param<double>("by", by, 20);
    nh.param<double>("bz", bz, 20);

    //Create an object admittance_controller
    admittance_ros_interface admittance_controller(mx, my, mz, bx, by, bz);

    
    ros::spin();

    return 0;
}
