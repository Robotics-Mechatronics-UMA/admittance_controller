#include "admittance_ros_interface.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "admittance_ros_node");

    ros::NodeHandle nh("~");

    //Controller variables
    double mx,my,mz,bx,by,bz;
    nh.param<double>("mx", mx, 30.0);
    nh.param<double>("my", my, 30.0);
    nh.param<double>("mz", mz, 30.0);
    nh.param<double>("bx", bx, 20.0);
    nh.param<double>("by", by, 20.0);
    nh.param<double>("bz", bz, 20.0);

    //Create an object admittance_controller
    admittance_ros_interface admittance_controller(mx, my, mz, bx, by, bz);

    //Dynamic_reconfigure server
    dynamic_reconfigure::Server<admittance_controller::admittanceConfig> server;
    dynamic_reconfigure::Server<admittance_controller::admittanceConfig>::CallbackType f;
    f = boost::bind(&admittance_ros_interface::DrCallback,&admittance_controller, _1, _2);
    server.setCallback(f);

    
    ros::spin();

    return 0;
}
