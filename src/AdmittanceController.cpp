// You can't include a library like this (not local folders!)
#include "/home/victor/ROS_ws/src/robot_delta/include/AdmittanceController.hpp"

// Constructor
Controller::Controller(const Eigen::Matrix<double, 6, 6>& mass,
                        const Eigen::Matrix<double, 6, 6>& damping,
                        const Eigen::Matrix<double, 6, 6>& stiffness)
    : Mass(mass), Damping(damping), Stiffness(stiffness), Dt(0.01) {

    Wrench = Eigen::VectorXd::Zero(6);
    Vel = Eigen::Matrix<double, 6, 1>::Zero();

    ros::NodeHandle nh;

    // Crear el servidor de configuración dinámica
    dynamic_reconfigure::Server<robot_delta::ParamsConfig>::CallbackType f;
    f = boost::bind(&Controller::DynamicReconfigureCallback, this, _1, _2);
    server.setCallback(f);

    force_sensor_subscriber = nh.subscribe("/ForceSensor", 1, &Controller::ForceSensorCallback, this);
    motor_publisher = nh.advertise<geometry_msgs::Vector3>("/Motor", 1);
}


// Método de callback para la reconfiguración dinámica
void Controller::DynamicReconfigureCallback(robot_delta::ParamsConfig &config, uint32_t level) {
    double m = config.mass;
    double b = config.damping;

    // Condición de estabilidad
    if (m > sqrt(2) * b) {
        // Actualizar el valor de las matrices de masa y amortiguación
        for (int i = 0; i < 3; i++) {
            Mass(i, i) = m;
            Damping(i, i) = b;
        } 
    }
}

// Método de callback para el sensor de fuerza
void Controller::ForceSensorCallback(const geometry_msgs::Vector3ConstPtr& F_msg) {
    // Actualizar los valores de la fuerza en cada eje
    Wrench << F_msg->x, F_msg->y, F_msg->z, 0.0, 0.0, 0.0;

    // Calcular la velocidad deseada usando el control de admitancia
    Eigen::Matrix<double, 6, 1> Vel_pub = AdmittanceController();

    // Crear el mensaje para publicar la velocidad
    geometry_msgs::Vector3 vel_msg;

    //Velocidad maxima del efector final 0.02 m/s
    for (int i = 0; i < 3; i++) {
        switch (i) {
            case 0: // x
                if (Vel_pub[i] >= 0.02) {
                    vel_msg.x = 0.02;
                } else {
                    vel_msg.x = Vel_pub[i];
                }
                break;
            case 1: // y
                if (Vel_pub[i] >= 0.02) {
                    vel_msg.y = 0.02;
                } else {
                    vel_msg.y = Vel_pub[i];
                }
                break;
            case 2: // z
                if (Vel_pub[i] >= 0.02) {
                    vel_msg.z = 0.02;
                } else {
                    vel_msg.z = Vel_pub[i];
                }
                break;
            default:
                break;
        }
    }

    // Publicar el mensaje de velocidad
    motor_publisher.publish(vel_msg);

    // Mostrar información sobre la velocidad publicada
    ROS_INFO_STREAM("VELOCIDAD PUBLICADA: " << vel_msg.x << " en x, " << vel_msg.y << " en y, " << vel_msg.z << " en z");
}


// Método para calcular la velocidad deseada usando el control de admitancia
Eigen::Matrix<double, 6, 1> Controller::AdmittanceController() {
    Eigen::Matrix<double, 6, 1> Vel_desired = (Mass.fullPivLu().solve(Wrench - Damping * Vel)) * Dt;
    Vel = Vel_desired;

    return Vel_desired;
}

// Main
int main(int argc, char** argv) {
    // Inicializar el nodo
    ros::init(argc, argv, "admittance_controller_node");

    //Declaramos las variables de nuestro controlador
    Eigen::Matrix<double ,6,6> Mass;
    Eigen::Matrix<double ,6,6> Damping;
    Eigen::Matrix<double ,6,6> Stiffness;

    //Inicializamos los valores de nuestras matrices Mass, Damping, STiffness
    Mass <<30,0,0,0,0,0,
            0,30,0,0,0,0,
            0,0,30,0,0,0,
            0,0,0,0,0,0,
            0,0,0,0,0,0,
            0,0,0,0,0,0;
    //Garantía de estabilidad en sistemas de segundo orden Mass<=sqrt(2)*Damping
    Damping <<20,0,0,0,0,0,
            0,20,0,0,0,0,
            0,0,20,0,0,0,
            0,0,0,0,0,0,
            0,0,0,0,0,0,
            0,0,0,0,0,0;
    //Simplificamos el sistema colocando el coeficiente del muelle a 0
    //Para otro manipulador le dariamos un valor
    Stiffness.setZero();

    // Creamos el objeto Controller
    Controller controller(Mass, Damping, Stiffness);

    //Bucle de control
    ros::Rate loop_rate(100); 
    while(ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

