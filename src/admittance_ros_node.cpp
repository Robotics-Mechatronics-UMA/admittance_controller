#include "admittance_ros_interface.hpp"
#include "integer_time.hpp"

//Time variable for integer
ros::Time int_time; //(Global variable)


int main(int argc, char** argv)
{
    ros::init(argc, argv, "admittance_ros_node");

    ros::NodeHandle nh("~");
    double m = 30.0;
    double b = 20.0;

    //Controller variables
    std::vector<double> mass = {m,m,m,m,m,m};
    std::vector<double> damping = {b,b,b,0.0,0.0,0.0};

    //Create an object admittance_controller
    admittance_ros_interface admittance_controller(mass, damping);

    //Dynamic_reconfigure server
    dynamic_reconfigure::Server<admittance_controller::admittanceConfig> server;
    dynamic_reconfigure::Server<admittance_controller::admittanceConfig>::CallbackType f;
    f = boost::bind(&admittance_ros_interface::DrCallback,&admittance_controller, _1, _2);
    server.setCallback(f);


    int_time = ros::Time::now();
    
    ros::spin();

    return 0;
}
