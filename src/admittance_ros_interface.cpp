#include "admittance_model.hpp"
#include "admittance_ros_interface.hpp"
#include "integer_time.hpp"


admittance_ros_interface::admittance_ros_interface(const std::vector<double>& mass,const std::vector<double>& damping)
{
    admittance_controller = std::make_shared<Controller>(mass, damping);

    wrench_sub = nh_.subscribe("/Force", 10,&admittance_ros_interface::ForceSensorCallback, this);
    vel_pub = nh_.advertise<geometry_msgs::Vector3>("/cmd_vel_ee", 10);

}
admittance_ros_interface::~admittance_ros_interface(){}

void admittance_ros_interface::ForceSensorCallback(const geometry_msgs::Vector3ConstPtr& W_msg)
{
    // Update Wrench matrix
    Eigen::Matrix<double, 6, 1> wrench = admittance_controller->getWrench();
    wrench << W_msg->x,W_msg->y,W_msg->z,0.0,0.0,0.0;

    admittance_controller->setWrench(wrench);


    ros::Duration dt = ros::Time::now() - int_time;

    int_time = ros::Time::now();

    double Dt = dt.toSec();
    admittance_controller->setDt(Dt);
    

    // Calculate increased velocity using an admittance controller
    Eigen::Matrix<double, 6, 1> vel_desired = admittance_controller->AdmittanceController();


    //Create a vector mesage to publish the velocity
    // std_msgs::Float32MultiArray vel_msg;
    geometry_msgs::Vector3 vel_msg;

    //Vel max = 0,02m/s
    double max_vel = admittance_controller->getVel_max();
    
    //Compare ve_desired with max_vel and -max_vel 
    vel_msg.x = (vel_desired(0) >= 0.0) ? std::min(vel_desired(0), max_vel) : std::max(vel_desired(0), -max_vel);
    vel_msg.y = (vel_desired(1) >= 0.0) ? std::min(vel_desired(1), max_vel) : std::max(vel_desired(1), -max_vel);
    vel_msg.z = (vel_desired(2) >= 0.0) ? std::min(vel_desired(2), max_vel) : std::max(vel_desired(2), -max_vel);

    // vel_msg.angular.x = (vel_desired(3) >= 0.0) ? std::min(vel_desired(3), max_vel) : std::max(vel_desired(3), -max_vel);
    // vel_msg.angular.y = (vel_desired(4) >= 0.0) ? std::min(vel_desired(4), max_vel) : std::max(vel_desired(4), -max_vel);
    // vel_msg.angular.z = (vel_desired(5) >= 0.0) ? std::min(vel_desired(5), max_vel) : std::max(vel_desired(5), -max_vel);

    //Publish velocity mesage
    vel_pub.publish(vel_msg);

}

//Dynamic_reconfigure callback function
void admittance_ros_interface::DrCallback(admittance_controller::admittanceConfig &config, uint32_t level)
{
    Eigen::Matrix<double, 6,6> mass_cfg;
    Eigen::Matrix<double, 6,6> damping_cfg;
    double vel_cfg;
    mass_cfg <<config.mx,0,0,0,0,0,
                0,config.my,0,0,0,0,
                0,0,config.mz,0,0,0,
                0,0,0,config.mrx,0,0,
                0,0,0,0,config.mry,0,
                0,0,0,0,0,config.mrz;
    admittance_controller->setMass(mass_cfg);

    damping_cfg <<config.bx,0,0,0,0,0,
    0,config.by,0,0,0,0,
    0,0,config.bz,0,0,0,
    0,0,0,config.brx,0,0,
    0,0,0,0,config.bry,0,
    0,0,0,0,0,config.brz;

    admittance_controller->setDamping(damping_cfg);

    vel_cfg = config.v_max;
    admittance_controller->setVel_max(vel_cfg);
}

