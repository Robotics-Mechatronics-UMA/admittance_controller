#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <cstdlib>
#include <ctime>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "force_sensor_node");
    ros::NodeHandle nh;
    ros::Publisher force_publisher = nh.advertise<geometry_msgs::Vector3>("/ForceSensor", 1);

    ros::Rate loop_rate(100); // Frecuencia de publicación en Hz

    // Inicializar la variable de tiempo
    ros::Time start_time = ros::Time::now();

    geometry_msgs::Vector3 force_msg;

    while (ros::ok())
    {
        // Publicar una fuerza de 30N durante 1 segundo
        if ((ros::Time::now() - start_time).toSec() < 1)
        {
            force_msg.x = 30.0;
            force_msg.y = 30.0;
            force_msg.z = 30.0;

            force_publisher.publish(force_msg);
        }
        // Publicar una fuerza de 0N durante 1 segundo
        else if ((ros::Time::now() - start_time).toSec() < 2)
        {
            force_msg.x = 0.0;
            force_msg.y = 0.0;
            force_msg.z = 0.0;

            force_publisher.publish(force_msg);
        }
        // Reiniciar el temporizador
        else
        {
            start_time = ros::Time::now();
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}