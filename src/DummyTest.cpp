#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <cstdlib>
#include <ctime>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "force_sensor_node");
    ros::NodeHandle nh;
    ros::Publisher force_publisher = nh.advertise<geometry_msgs::Twist>("/force", 10);

    ros::Rate loop_rate(100); // Frecuencia de publicación en Hz

    // Inicializar la variable de tiempo
    ros::Time start_time = ros::Time::now();

    geometry_msgs::Twist msg;
    msg.angular.x = 0.0;
    msg.angular.y = 0.0;
    msg.angular.z = 0.0;

    while (ros::ok())
    {
        // Publicar una fuerza de 30N durante 1 segundo
        if ((ros::Time::now() - start_time).toSec() < 1)
        {
            msg.linear.x = 30.0;
            msg.linear.y = 30.0;
            msg.linear.z = 30.0;


            force_publisher.publish(msg);
        }
        // Publicar una fuerza de 0N durante 1 segundo
        else if ((ros::Time::now() - start_time).toSec() < 2)
        {
            msg.linear.x = 0.0;
            msg.linear.y = 0.0;
            msg.linear.z = 0.0;


            force_publisher.publish(msg);
        }
        // Reiniciar el temporizador
        else
        {
            start_time = ros::Time::now();
        }

    }

    return 0;
}
