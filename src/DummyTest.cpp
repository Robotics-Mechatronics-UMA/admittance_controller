#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <cstdlib>
#include <ctime>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "force_sensor_node");
    ros::NodeHandle nh;
    ros::Publisher force_publisher = nh.advertise<geometry_msgs::Twist>("/force", 10);

    ros::Rate loop_rate(100); //Frecuency of publication (100Hz)

    // Initialize time variable
    ros::Time start_time = ros::Time::now();

    geometry_msgs::Twist msg; 
    msg.angular.x = 0.0;
    msg.angular.y = 0.0;
    msg.angular.z = 0.0;

    while (ros::ok())
    {
        // Publish a 30N force for 1 minute
        if ((ros::Time::now() - start_time).toSec() < 1)
        {
            msg.linear.x = 30.0;
            msg.linear.y = 30.0;
            msg.linear.z = 30.0;


            force_publisher.publish(msg);
        }
        // Publish no force for 1 minute
        else if ((ros::Time::now() - start_time).toSec() < 2)
        {
            msg.linear.x = 0.0;
            msg.linear.y = 0.0;
            msg.linear.z = 0.0;


            force_publisher.publish(msg);
        }
        // Reset timer
        else
        {
            start_time = ros::Time::now();
        }

    }

    return 0;
}
