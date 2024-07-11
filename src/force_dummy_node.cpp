#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <cstdlib>
#include <ctime>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "dummy_test_node");
    ros::NodeHandle nh;
    ros::Publisher force_publisher = nh.advertise<geometry_msgs::Vector3>("/Force", 10);

    ros::Rate loop_rate(100); //Frecuency of publication (100Hz)

    // Initialize time variable
    ros::Time start_time = ros::Time::now();

    geometry_msgs::Vector3 msg; 

    //Generates a square signal between 10 and -10 N 
    while (ros::ok())
    {
        // Publish a 30N force for 10 seconds
        if ((ros::Time::now() - start_time).toSec() < 10)
        {
            msg.x = 10.0;
            msg.y = 10.0;
            msg.z = 10.0;


            force_publisher.publish(msg);
        }
        // Publish no force for 10 seconds
        else if ((ros::Time::now() - start_time).toSec() < 20)
        {
            msg.x = -10.0;
            msg.y = -10.0;
            msg.z = -10.0;


            force_publisher.publish(msg);
        }
        // Reset timer
        else
        {
            start_time = ros::Time::now();
        }
        loop_rate.sleep();

    }

    return 0;
}
